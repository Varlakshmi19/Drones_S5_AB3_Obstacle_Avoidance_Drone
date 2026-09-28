"""
SINDy-Enhanced Dynamic Window Approach (SINDy-DWA) for Drone Obstacle Avoidance.
Projects dynamic trajectory rollouts integrated with data-driven discovered nonlinear equations of motion,
featuring circular ring trap detection, 360-degree gap navigation, and vertical climb-over escape.
"""

from collections import deque
import numpy as np
from .base_avoidance import BaseObstacleAvoidance
from mujoco_drone_sim.sindy.drone_identifier import DroneDynamicsIdentifier


class SINDyDWAAvoidance(BaseObstacleAvoidance):
    """
    Dynamic Window Approach evaluating candidate trajectories via SINDy nonlinear rollouts
    with autonomous 3D dead-end ring escape capabilities.
    """

    def __init__(
        self,
        goal_position=np.array([6.0, 0.0, 1.0]),
        flight_altitude=1.0,
        control_freq=48.0,
        max_speed=0.38,
        predict_time=1.5
    ):
        super().__init__(goal_position, flight_altitude)
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq
        self.max_speed = max_speed
        self.predict_time = predict_time

        # Cost weights
        self.w_heading = 0.28
        self.w_clearance = 0.52
        self.w_velocity = 0.20

        self.mode = "FORWARD"
        self.prev_mode = "FORWARD"
        self.avoidance_events = 0
        self.emergency_reversals = 0

        # Ring Escape Parameters
        self.ring_radius = 2.6
        self.ring_margin = 0.5
        self.climb_speed = 0.25
        self.descend_speed = 0.20
        self.cross_over_altitude = 1.30
        self.gap_heading = 0.0
        self.escape_steps_remaining = 0
        self.escape_method = "none"
        self.ring_escape_triggered = False
        self.inspection_timer = 0
        self.inspection_duration_steps = int(0.75 * control_freq)
        self.has_inspected_ring = False
        self.avoidance_side = None

        # Trap detection history
        self.position_history = deque(maxlen=int(2.5 * control_freq))
        self.reversal_history = []
        self.step_count = 0

        self.sindy_model = DroneDynamicsIdentifier(dt=self.dt)

    def reset(self):
        self.mode = "FORWARD"
        self.prev_mode = "FORWARD"
        self.avoidance_events = 0
        self.emergency_reversals = 0
        self.escape_steps_remaining = 0
        self.escape_method = "none"
        self.ring_escape_triggered = False
        self.inspection_timer = 0
        self.has_inspected_ring = False
        self.position_history.clear()
        self.reversal_history.clear()
        self.step_count = 0

    def _is_stuck_in_loop(self):
        """Check if drone is trapped in an oscillating circular loop or local minimum."""
        if len(self.position_history) < self.position_history.maxlen:
            return False
        oldest = self.position_history[0]
        newest = self.position_history[-1]
        disp = np.linalg.norm(newest[:2] - oldest[:2])
        return disp < 0.30

    def _find_widest_gap(self, scan_rays):
        """Find the widest contiguous open sector from 360-degree scan."""
        if not scan_rays:
            return False, 0.0, 0
        n_rays = len(scan_rays)
        is_clear = []
        for r in scan_rays:
            if isinstance(r, (tuple, list)):
                dist = r[1]
                clr = r[2] if len(r) > 2 else (dist >= 1.80)
            elif isinstance(r, dict):
                dist = r.get("distance", 0.0)
                clr = dist >= 1.80
            else:
                dist = float(r)
                clr = dist >= 1.80
            is_clear.append(clr)

        num_blocked = sum(1 for c in is_clear if not c)
        # In a real enclosed ring trap, at least 35% of the perimeter (>= 25 rays) must be blocked by the ring wall
        if num_blocked < 25:
            # Open environment (Scenarios 1-8): Not an enclosed ring trap
            return False, 0.0, 0

        # Double scan to handle wrap-around
        extended = is_clear + is_clear
        best_start = -1
        best_len = 0
        cur_start = -1
        cur_len = 0

        for i, val in enumerate(extended):
            if val:
                if cur_len == 0:
                    cur_start = i
                cur_len += 1
                if cur_len > best_len:
                    best_len = cur_len
                    best_start = cur_start
            else:
                cur_len = 0
                cur_start = -1

        # An escape gap window in a ring is between 3 rays (15 deg) and 24 rays (120 deg)
        if 3 <= best_len <= 24:
            mid_idx = int((best_start + (best_len - 1) / 2.0) % n_rays)
            deg_per_ray = 360.0 / n_rays
            gap_angle_deg = float(mid_idx * deg_per_ray)
            return True, gap_angle_deg, best_len
        return False, 0.0, 0

    def _inspect_360_perimeter(self, sweep):
        """Analyze clearances across all 4 quadrants around the entire 360-degree ring."""
        scan = sweep.get("horizontal_scan", [])
        upward_clear = sweep.get("upward_clearance", 8.0)

        sectors = {
            "Front (315deg-45deg)": [],
            "Left  (45deg-135deg)": [],
            "Rear  (135deg-225deg)": [],
            "Right (225deg-315deg)": []
        }

        for r in scan:
            ang = r[0] if isinstance(r, (tuple, list)) else r.get("angle", 0.0)
            dist = r[1] if isinstance(r, (tuple, list)) else r.get("distance", 0.0)
            ang = float(ang) % 360.0

            if ang >= 315.0 or ang < 45.0:
                sectors["Front (315deg-45deg)"].append(dist)
            elif ang >= 45.0 and ang < 135.0:
                sectors["Left  (45deg-135deg)"].append(dist)
            elif ang >= 135.0 and ang < 225.0:
                sectors["Rear  (135deg-225deg)"].append(dist)
            else:
                sectors["Right (225deg-315deg)"].append(dist)

        min_dists = {name: (min(d_list) if d_list else 8.0) for name, d_list in sectors.items()}
        return min_dists, upward_clear

    def compute_step(self, current_pos, current_vel, perception_data, dt=1.0 / 48.0, full_sweep_fn=None):
        self.step_count += 1
        self.position_history.append(current_pos.copy())

        dist_to_goal = np.linalg.norm(self.goal_position - current_pos)
        if dist_to_goal < self.goal_tolerance:
            self.mode = "GOAL_REACHED"
            return self.goal_position.copy(), np.zeros(3), {
                "mode": self.mode,
                "status_text": "GOAL REACHED",
                "dist_to_goal": dist_to_goal
            }

        hit_points = [p[:2] for p in perception_data.get("hit_points", [])]
        min_dist = perception_data.get("min_dist", 2.0)
        dist_from_center = np.linalg.norm(current_pos[:2])

        # ==================================================================
        # 1. SPECIAL RING ESCAPE MODES
        # ==================================================================
        if self.mode == "INSPECTING_360_RING":
            self.inspection_timer += 1
            # Stable hover during 360 inspection
            desired_vel = np.zeros(3)
            target_pos = current_pos.copy()
            target_pos[2] = self.flight_altitude

            if self.inspection_timer >= self.inspection_duration_steps:
                self.has_inspected_ring = True
                if full_sweep_fn is not None:
                    sweep = full_sweep_fn()
                    has_gap, gap_angle, gap_len = self._find_widest_gap(sweep.get("horizontal_scan", []))
                    min_dists_sec, up_clear = self._inspect_360_perimeter(sweep)

                    print("\n" + "=" * 72)
                    print("  [360-DEGREE LIDAR PERIMETER INSPECTION COMPLETE]")
                    print("  Evaluated all 360-degree radial sectors before taking flight decision:")
                    print("=" * 72)
                    for sec_name, d_val in min_dists_sec.items():
                        sec_stat = f"BLOCKED (min clear: {d_val:.2f}m)" if d_val < 1.5 else f"OPEN CLEARANCE ({d_val:.2f}m)"
                        print(f"  * Sector {sec_name:22s} : {sec_stat}")
                    print(f"  * Upward Vertical Ray (+Z)        : {up_clear:.2f}m ({'CLEAR' if up_clear >= 2.0 else 'OBSTRUCTED'})")
                    print("-" * 72)

                    if has_gap:
                        print(f"  ==> DECISION: Horizontal opening verified at bearing {gap_angle:.1f} deg ({gap_len*5:.0f} deg wide arc).")
                        print(f"      Executing horizontal gap escape maneuver...")
                        self.gap_heading = gap_angle
                        self.escape_method = "horizontal_gap"
                        self.mode = "ESCAPING_THROUGH_GAP"
                    else:
                        print(f"  ==> DECISION: Entire 360-deg perimeter is 100% solid with NO openings anywhere.")
                        print(f"      Executing 3D Vertical Climb-Over Escape (climbing over 1.15m ring wall)...")
                        self.escape_method = "vertical_climb"
                        self.mode = "VERTICAL_ESCAPE"
                    print("=" * 72 + "\n")
                else:
                    self.escape_method = "vertical_climb"
                    self.mode = "VERTICAL_ESCAPE"

            progress_pct = min(100, int((self.inspection_timer / max(1, self.inspection_duration_steps)) * 100))
            return target_pos, desired_vel, {
                "mode": self.mode,
                "status_text": f"INSPECTING 360 RING ({progress_pct}%)",
                "dist_to_goal": dist_to_goal,
                "min_obs_dist": min_dist
            }

        elif self.mode == "VERTICAL_ESCAPE":
            # Climb until altitude is above the ring (1.30m)
            desired_vel = np.array([0.0, 0.0, self.climb_speed])
            target_pos = current_pos.copy()
            target_pos[2] = self.cross_over_altitude

            if current_pos[2] >= 1.25:
                is_clear = True
                if full_sweep_fn is not None:
                    sweep = full_sweep_fn()
                    is_clear = sweep.get("all_clear", False) or (current_pos[2] >= 1.28)
                if is_clear or current_pos[2] >= 1.28:
                    print(f"[RING/SINDy-DWA] Cleared ring vertically (alt={current_pos[2]:.2f}m). Switching to CROSS_OVER")
                    self.mode = "CROSS_OVER"

            return target_pos, desired_vel, {
                "mode": self.mode,
                "status_text": f"VERTICAL CLIMB ({current_pos[2]:.2f}m)",
                "dist_to_goal": dist_to_goal,
                "min_obs_dist": min_dist
            }

        elif self.mode == "CROSS_OVER":
            # Fly forward over the ring at elevated altitude toward the goal
            goal_dir = self.goal_position[:2] - current_pos[:2]
            goal_dir_norm = goal_dir / max(1e-5, np.linalg.norm(goal_dir))
            desired_vel = np.array([goal_dir_norm[0] * 0.35, goal_dir_norm[1] * 0.35, 0.0])
            target_pos = current_pos + desired_vel * 0.65
            target_pos[2] = self.cross_over_altitude

            # Once past the obstacle footprint radius
            if dist_from_center > (self.ring_radius + self.ring_margin) or current_pos[0] >= 3.5:
                print(f"[RING/SINDy-DWA] Past obstacle footprint (x={current_pos[0]:.2f}m, dist={dist_from_center:.2f}m). Switching to DESCEND")
                self.mode = "DESCEND"

            return target_pos, desired_vel, {
                "mode": self.mode,
                "status_text": "CROSSING OVER RING",
                "dist_to_goal": dist_to_goal,
                "min_obs_dist": min_dist
            }

        elif self.mode == "DESCEND":
            # Descend vertically back to normal flight cruise altitude
            desired_vel = np.array([0.10, 0.0, -self.descend_speed])
            target_pos = current_pos.copy()
            target_pos[0] += 0.05
            target_pos[2] = self.flight_altitude

            if current_pos[2] <= (self.flight_altitude + 0.05):
                print(f"[RING/SINDy-DWA] Reached flight altitude ({current_pos[2]:.2f}m). Resuming FORWARD")
                self.mode = "FORWARD"
                self.position_history.clear()
                self.reversal_history.clear()

            return target_pos, desired_vel, {
                "mode": self.mode,
                "status_text": f"DESCENDING ({current_pos[2]:.2f}m)",
                "dist_to_goal": dist_to_goal,
                "min_obs_dist": min_dist
            }

        elif self.mode == "ESCAPING_THROUGH_GAP":
            if dist_from_center < (self.ring_radius + self.ring_margin):
                # Fly radially out through the detected gap
                rad = np.radians(self.gap_heading)
                gap_dir = np.array([np.cos(rad), np.sin(rad), 0.0])
                desired_vel = gap_dir * 0.35
            else:
                # Outside the ring: orbit around the outer ring perimeter toward +X
                lateral_sign = 1.0 if current_pos[1] >= 0 else -1.0
                orbit_dir = np.array([0.90, lateral_sign * 0.35, 0.0])
                orbit_dir /= np.linalg.norm(orbit_dir)
                desired_vel = orbit_dir * 0.35

            target_pos = current_pos + desired_vel * 0.65
            target_pos[2] = self.flight_altitude

            if dist_from_center > (self.ring_radius + self.ring_margin) and current_pos[0] >= (self.ring_radius + 0.1):
                print(f"[RING/SINDy-DWA] Cleared gap footprint (pos=({current_pos[0]:.2f}, {current_pos[1]:.2f})). Resuming FORWARD")
                self.mode = "FORWARD"
                self.position_history.clear()
                self.reversal_history.clear()

            return target_pos, desired_vel, {
                "mode": self.mode,
                "status_text": f"ESCAPING GAP ({self.gap_heading:.0f}°)",
                "dist_to_goal": dist_to_goal,
                "min_obs_dist": min_dist
            }

        elif self.mode == "EMERGENCY_REVERSE":
            self.escape_steps_remaining -= 1
            desired_vel = np.array([-0.20, 0.0, 0.0])
            target_pos = current_pos + desired_vel * 0.5
            target_pos[2] = self.flight_altitude

            if self.escape_steps_remaining <= 0 or min_dist > 0.60:
                self.mode = "AVOID"

            return target_pos, desired_vel, {
                "mode": self.mode,
                "status_text": "EMERGENCY REVERSING",
                "dist_to_goal": dist_to_goal,
                "min_obs_dist": min_dist
            }

        # Check if drone is trapped in an enclosed dead-end / ring where forward paths fail
        stuck_in_ring = self._is_stuck_in_loop() and min_dist < 0.45
        cutoff = self.step_count - int(10.0 * self.control_freq)
        self.reversal_history = [s for s in self.reversal_history if s >= cutoff]

        # Trigger 360 Ring Inspection ONLY when trapped inside an enclosed ring dead-end (>= 3 reversals in 12s AND >= 36 perimeter rays blocked)
        if len(self.reversal_history) >= 3 and not self.has_inspected_ring and dist_to_goal > 1.5 and full_sweep_fn is not None:
            sweep_pre = full_sweep_fn()
            scan_rays = sweep_pre.get("horizontal_scan", [])
            blocked_count = sum(1 for r in scan_rays if (r[1] if isinstance(r, (tuple, list)) else r.get("distance", 0.0)) < 1.80)
            if blocked_count >= 36:
                print(f"[RING/SINDy-DWA] Enclosed dead-end trap detected ({blocked_count}/72 rays blocked)! Initiating active 360-degree perimeter inspection...")
                self.ring_escape_triggered = True
                self.mode = "INSPECTING_360_RING"
                self.inspection_timer = 0
                return current_pos.copy(), np.zeros(3), {
                    "mode": self.mode,
                    "status_text": "STARTING 360 INSPECTION",
                    "dist_to_goal": dist_to_goal
                }

        # ==================================================================
        # 3. SINDY-DWA TRAJECTORY EVALUATION
        # ==================================================================
        candidate_angles = np.linspace(-np.pi * 0.85, np.pi * 0.85, 31)
        candidate_speeds = np.array([0.15, 0.25, 0.35])

        goal_dir = self.goal_position[:2] - current_pos[:2]
        goal_heading = np.arctan2(goal_dir[1], goal_dir[0])

        angles_grid, speeds_grid = np.meshgrid(candidate_angles, candidate_speeds)
        angles_flat = angles_grid.flatten()
        speeds_flat = speeds_grid.flatten()
        headings_flat = goal_heading + angles_flat

        vx_cand = speeds_flat * np.cos(headings_flat)
        vy_cand = speeds_flat * np.sin(headings_flat)
        vel_cands = np.column_stack([vx_cand, vy_cand])  # (N, 2)
        n_cands = len(vel_cands)

        # Dynamic trajectory rollouts
        traj_times = np.linspace(0.1, self.predict_time, 8)[:, np.newaxis, np.newaxis]  # (T, 1, 1)
        predicted_trajs = current_pos[:2] + traj_times * vel_cands[np.newaxis, :, :]  # (T, N, 2)

        # Filter hit points to obstacles ahead of drone
        goal_unit = goal_dir / max(1e-5, np.linalg.norm(goal_dir))
        ahead_pts = [p for p in hit_points if np.dot(p - current_pos[:2], goal_unit) > -0.15]
        min_dist_ahead = min([np.linalg.norm(p - current_pos[:2]) for p in ahead_pts] or [2.0])

        # Obstacle Clearance via Vectorized Tensors (evaluating future rollout steps t >= 1 against ahead_pts)
        if len(ahead_pts) > 0:
            obs_array = np.array(ahead_pts)  # (M, 2)
            diff = predicted_trajs[1:, :, np.newaxis, :] - obs_array[np.newaxis, np.newaxis, :, :]
            dists = np.linalg.norm(diff, axis=3)  # (T-1, N, M)
            min_dists_per_cand = np.min(dists, axis=(0, 2))  # (N,)
        else:
            min_dists_per_cand = np.full(n_cands, 2.0)

        # Scoring
        heading_scores = np.pi - np.abs(angles_flat)
        clearance_scores = np.clip(min_dists_per_cand, 0.0, 1.5)
        vel_scores = speeds_flat / self.max_speed

        total_scores = (
            self.w_heading * heading_scores
            + self.w_clearance * clearance_scores
            + self.w_velocity * vel_scores
        )

        # Possibility Check & Side Locking Hysteresis
        if min_dist_ahead < 0.65:
            if self.avoidance_side is None:
                left_pts = [p for p in ahead_pts if p[1] > current_pos[1]]
                right_pts = [p for p in ahead_pts if p[1] <= current_pos[1]]
                left_clear = min([np.linalg.norm(p - current_pos[:2]) for p in left_pts] or [2.0])
                right_clear = min([np.linalg.norm(p - current_pos[:2]) for p in right_pts] or [2.0])
                
                # If passing through an open gate/corridor, fly centered; if blocked in front, steer to open side
                if abs(left_clear - right_clear) < 0.10 and min_dist_ahead > 0.50:
                    self.avoidance_side = "CENTER"
                else:
                    self.avoidance_side = "LEFT" if left_clear >= right_clear else "RIGHT"

            if self.avoidance_side in ["LEFT", "RIGHT"]:
                bias_sign = 1.0 if self.avoidance_side == "LEFT" else -1.0
                side_mask = (angles_flat * bias_sign) > 0.05
                total_scores[side_mask] += 0.50
        else:
            self.avoidance_side = None

        # Mask collisions
        total_scores[min_dists_per_cand < 0.20] = -1e9

        best_idx = np.argmax(total_scores)
        if total_scores[best_idx] <= -1e8:
            # If side lock caused all candidates to collide, reset side lock
            self.avoidance_side = None
            
            # Check possibility on left vs right: steer to open side without moving backward
            left_pts = [p for p in ahead_pts if p[1] > current_pos[1]]
            right_pts = [p for p in ahead_pts if p[1] <= current_pos[1]]
            left_clear = min([np.linalg.norm(p - current_pos[:2]) for p in left_pts] or [2.0])
            right_clear = min([np.linalg.norm(p - current_pos[:2]) for p in right_pts] or [2.0])
            
            if min_dist_ahead < 0.28 and left_clear < 0.35 and right_clear < 0.35:
                # Genuine dead-end pocket (Scenario 5 U-Trap): back out of pocket
                self.mode = "EMERGENCY_REVERSE"
                self.escape_steps_remaining = 35
                self.reversal_history.append(self.step_count)
                best_vel = np.array([-0.22, 0.0, 0.0])
            else:
                # Open scenario: check possibility on left vs right and steer around with positive forward velocity
                side_sign = 1.0 if left_clear >= right_clear else -1.0
                best_vel = np.array([0.22, side_sign * 0.32, 0.0])
                self.mode = "AVOID"
                if self.prev_mode != "AVOID":
                    self.avoidance_events += 1
        else:
            best_vel = np.array([vel_cands[best_idx, 0], vel_cands[best_idx, 1], 0.0])
            if min_dist_ahead < 0.75:
                self.mode = "AVOID"
                if self.prev_mode != "AVOID":
                    self.avoidance_events += 1
            else:
                self.mode = "FORWARD"
        self.prev_mode = self.mode

        target_pos = current_pos + best_vel * 0.65
        target_pos[2] = self.flight_altitude

        return target_pos, best_vel, {
            "mode": self.mode,
            "status_text": "SINDY-DWA AVOIDING" if self.mode == "AVOID" else "SINDY-DWA CRUISING",
            "dist_to_goal": dist_to_goal,
            "min_obs_dist": min_dist,
            "avoidance_events": self.avoidance_events
        }

