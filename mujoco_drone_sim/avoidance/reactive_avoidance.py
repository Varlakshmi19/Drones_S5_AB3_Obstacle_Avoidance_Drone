"""
Reactive Sector State Machine Obstacle Avoidance for MuJoCo Drone.
Extended with:
- Part A: Ring Trap Signature Detection & Multi-Modal Escape (Horizontal Gap & Vertical Climb-Over)
- Part B: Dynamic Moving Obstacle Closing-Velocity Estimation & Trajectory Prediction
- Part C: Full backward-compatible support for Scenarios 1-10 and isolation from U-trap (Scenario 5)
"""

from collections import deque
import numpy as np
from .base_avoidance import BaseObstacleAvoidance


def horizontal_unit_vector(vector):
    """Convert a vector into a horizontal 2D unit vector on XY plane."""
    vec = np.asarray(vector, dtype=float).copy()
    vec[2] = 0.0
    mag = np.linalg.norm(vec)
    if mag < 1e-8:
        return np.array([1.0, 0.0, 0.0])
    return vec / mag


def rotate_horizontal(vector, angle_degrees):
    """Rotate a horizontal vector about the vertical z-axis."""
    angle = np.radians(angle_degrees)
    rot_matrix = np.array([
        [np.cos(angle), -np.sin(angle), 0.0],
        [np.sin(angle),  np.cos(angle), 0.0],
        [0.0,            0.0,           1.0]
    ])
    return rot_matrix @ vector


class ReactiveObstacleAvoidance(BaseObstacleAvoidance):
    """
    Reactive sector-based obstacle avoidance controller with ring trap escape
    and dynamic obstacle closing-velocity prediction.
    """

    def __init__(
        self,
        goal_position=np.array([6.0, 0.0, 1.0]),
        flight_altitude=1.0,
        control_freq=48.0
    ):
        super().__init__(goal_position, flight_altitude)
        self.control_freq = control_freq

        # Speed configurations
        self.forward_speed = 0.38
        self.avoid_forward_speed = 0.28
        self.side_speed = 0.35
        self.return_speed = 0.35
        self.reverse_speed = 0.22
        self.climb_speed = 0.25
        self.descend_speed = 0.25

        # Safety & threshold distances
        self.sensor_range = 2.0
        self.detection_distance = 0.95
        self.side_detection_distance = 0.65
        self.emergency_distance = 0.35
        self.side_blocked_distance = 0.42
        self.front_clear_distance = 1.00
        self.ring_clear_distance = 3.2
        self.goal_tolerance = 0.25

        # Timing / step limits
        self.min_avoid_steps = int(0.7 * self.control_freq)
        self.clear_confirmation_steps = int(0.25 * self.control_freq)
        self.escape_steps = int(1.4 * self.control_freq)
        self.stuck_window_steps = int(2.5 * self.control_freq)
        self.min_progress_distance = 0.08

        # Ring Escape Parameters (Part A)
        self.ring_trigger_count = 3
        self.ring_window_steps = int(12.0 * self.control_freq)
        self.minimum_gap_arc_width = 3  # >= 3 rays clear (>= 15 deg arc with 72 rays)
        self.max_climb_altitude = 2.2   # Safety cap
        self.ring_radius = 2.6
        self.ring_margin = 0.5
        self.vertical_check_interval = int(0.5 * self.control_freq)

        # Dynamic Obstacle Prediction Parameters (Part B)
        self.dynamic_prediction_horizon = 0.8  # seconds

        self.reset()

    def reset(self):
        """Reset internal finite state machine variables."""
        self.mode = "FORWARD"
        self.previous_mode = self.mode
        self.avoidance_side = None
        self.previous_avoidance_side = None

        self.avoid_steps = 0
        self.clear_steps = 0
        self.escape_steps_remaining = 0
        self.force_opposite_side = False
        self.emergency_cooldown = 0

        self.position_history = deque(maxlen=self.stuck_window_steps)
        self.avoidance_events = 0
        self.emergency_reversals = 0

        # Ring Escape Tracking State
        self.reversal_step_history = []
        self.side_decision_history = []
        self.current_step_count = 0
        self.gap_heading = 0.0
        self.gap_clear_steps = 0
        self.cross_over_altitude = self.flight_altitude
        self.vertical_timer = 0

        # Metrics
        self.ring_escape_triggered = False
        self.escape_method = "none"
        self.max_altitude_reached = float(self.flight_altitude)
        self.gap_search_count = 0
        self.dynamic_avoidance_events = 0

    def goal_direction(self, current_position):
        """Return the horizontal unit direction toward the goal."""
        return horizontal_unit_vector(self.goal_position - current_position)

    def calculate_side_scores(self, distances):
        """
        Calculate weighted clearance score for left and right sectors.
        Higher score indicates more free open space.
        """
        left_score = (
            0.45 * distances.get("left", 2.0)
            + 0.35 * distances.get("far_left", 2.0)
            + 0.20 * distances.get("side_left", 2.0)
        )
        right_score = (
            0.45 * distances.get("right", 2.0)
            + 0.35 * distances.get("far_right", 2.0)
            + 0.20 * distances.get("side_right", 2.0)
        )
        return left_score, right_score

    def choose_avoidance_side(self, distances, previous_side=None):
        """
        Check possibility: choose the side with greater available clearance.
        - If obstacle is on LEFT (left_score < right_score): returns RIGHT to steer right.
        - If obstacle is on RIGHT (right_score < left_score): returns LEFT to steer left.
        Maintains previous side if scores are within hysteresis threshold (0.05).
        """
        left_score, right_score = self.calculate_side_scores(distances)
        score_difference = abs(left_score - right_score)

        if score_difference < 0.05 and previous_side is not None:
            return previous_side

        if left_score >= right_score:
            return "LEFT"
        return "RIGHT"

    def progress_is_too_small(self):
        """Determine whether the drone is stuck horizontally."""
        if len(self.position_history) < self.stuck_window_steps:
            return False
        oldest = self.position_history[0]
        newest = self.position_history[-1]
        horizontal_progress = np.linalg.norm(newest[:2] - oldest[:2])
        return horizontal_progress < self.min_progress_distance

    def _find_widest_contiguous_gap(self, horizontal_scan):
        """
        Find the widest contiguous run of clear rays around 360 degrees for ring escape.
        Returns: (has_gap, gap_midpoint_angle_deg, max_run_len)
        """
        num_rays = len(horizontal_scan)
        if num_rays == 0:
            return False, 0.0, 0

        is_clear = [item[2] if len(item) > 2 else (item[1] >= 1.80) for item in horizontal_scan]

        num_blocked = sum(1 for c in is_clear if not c)
        # In a real enclosed ring trap, at least 35% of the perimeter (>= 25 rays) must be blocked by the ring wall
        if num_blocked < 25:
            # Open environment (Scenarios 1-8): Not an enclosed ring trap
            return False, 0.0, 0

        # Handle circular wrap-around
        doubled = is_clear + is_clear
        best_start = -1
        best_len = 0
        curr_start = -1
        curr_len = 0

        for idx, clr in enumerate(doubled):
            if clr:
                if curr_len == 0:
                    curr_start = idx
                curr_len += 1
                if curr_len > best_len:
                    best_len = curr_len
                    best_start = curr_start
            else:
                curr_len = 0
                curr_start = -1

        # An escape gap window in a ring is between 3 rays (15 deg) and 24 rays (120 deg)
        if 3 <= best_len <= 24:
            mid_idx = (best_start + (best_len - 1) / 2.0) % num_rays
            deg_per_ray = 360.0 / num_rays
            gap_angle = float(mid_idx * deg_per_ray)
            return True, gap_angle, best_len

        return False, 0.0, best_len

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

    def make_velocity_command(self, current_position, directions, distances):
        """Generate desired velocity vector based on current state."""
        toward_goal = directions["front"]
        left_vector = rotate_horizontal(toward_goal, 90)
        right_vector = rotate_horizontal(toward_goal, -90)

        if self.mode == "FORWARD":
            velocity = toward_goal * self.forward_speed

        elif self.mode == "AVOID":
            # If obstacle is on the left, check possibility -> steer right & forward
            # If obstacle is on the right, check possibility -> steer left & forward
            lateral_vector = left_vector if self.avoidance_side == "LEFT" else right_vector
            
            front_d = distances.get("front", 2.0)
            if front_d > 0.85:
                # Mostly open ahead (e.g. side obstacle passing or open corridor)
                fwd_spd = self.forward_speed
                lat_spd = self.side_speed * 0.70
            elif front_d > 0.45:
                # Moderate obstacle in front -> balanced forward + lateral
                fwd_spd = self.avoid_forward_speed * 0.60
                lat_spd = self.side_speed * 1.10
            else:
                # Close obstacle in front -> slide laterally to clear obstacle without ramming into it
                fwd_spd = 0.04
                lat_spd = self.side_speed * 1.35

            velocity = (
                toward_goal * fwd_spd
                + lateral_vector * lat_spd
            )

        elif self.mode == "RETURN":
            velocity = toward_goal * self.return_speed

        elif self.mode == "EMERGENCY_REVERSE":
            velocity = -toward_goal * self.reverse_speed

        elif self.mode == "ESCAPING_THROUGH_GAP":
            dist_from_center = np.linalg.norm(current_position[:2])
            if dist_from_center < (self.ring_radius + self.ring_margin):
                # Fly radially out through the detected gap
                rad = np.radians(self.gap_heading)
                gap_vec = np.array([np.cos(rad), np.sin(rad), 0.0])
                velocity = gap_vec * self.avoid_forward_speed
            else:
                # Outside the ring: orbit around the outer ring perimeter toward +X
                lateral_sign = 1.0 if current_position[1] >= 0 else -1.0
                orbit_dir = np.array([0.90, lateral_sign * 0.35, 0.0])
                orbit_dir /= np.linalg.norm(orbit_dir)
                velocity = orbit_dir * self.forward_speed

        elif self.mode == "VERTICAL_ESCAPE":
            # Hold horizontal position, climb vertically
            velocity = np.array([0.0, 0.0, self.climb_speed])

        elif self.mode == "CROSS_OVER":
            # Fly in goal direction at elevated altitude
            velocity = toward_goal * self.avoid_forward_speed

        elif self.mode == "DESCEND":
            # Descend vertically back to flight altitude
            velocity = np.array([0.0, 0.0, -self.descend_speed])

        elif self.mode == "GOAL_REACHED":
            velocity = np.zeros(3)

        else:
            velocity = np.zeros(3)

        if self.mode not in ["VERTICAL_ESCAPE", "DESCEND"]:
            velocity[2] = 0.0

        return velocity

    def compute_step(self, current_pos, current_vel, perception_data, dt=1.0 / 48.0, full_sweep_fn=None):
        """
        Execute one control step of the reactive avoidance finite state machine.
        """
        self.current_step_count += 1
        self.emergency_cooldown = max(0, self.emergency_cooldown - 1)
        raw_distances = perception_data["distances"]
        self.position_history.append(current_pos.copy())

        # Track max altitude
        if current_pos[2] > self.max_altitude_reached:
            self.max_altitude_reached = float(current_pos[2])

        # ============================================================
        # PART B: DYNAMIC OBSTACLE PREDICTION & INFLATION
        # ============================================================
        base_dir = self.goal_direction(current_pos)
        dynamic_sectors = perception_data.get("dynamic_sectors", {})
        is_dynamic = perception_data.get("is_dynamic", False)
        distances = raw_distances.copy()

        if is_dynamic:
            # 1. Sector closing-velocity inflation
            for s_name in ["left", "right", "far_left", "far_right", "side_left", "side_right", "front"]:
                if s_name in dynamic_sectors:
                    d_info = dynamic_sectors[s_name]
                    if d_info.get("is_closing", False):
                        v_close = d_info.get("v_closing", 0.0)
                        pred_reduction = v_close * self.dynamic_prediction_horizon
                        distances[s_name] = max(0.05, raw_distances[s_name] - pred_reduction)

            # 2. Multi-step Kalman trajectory anticipation
            kalman_tracks = perception_data.get("kalman_tracks", {})
            for t_name, t_info in kalman_tracks.items():
                for pred_pt in t_info.get("trajectory_prediction", []):
                    pred_p = np.array(pred_pt["pos"])
                    d_pred = np.linalg.norm(pred_p[:2] - current_pos[:2])
                    if d_pred < 0.85:
                        to_pred = pred_p[:2] - current_pos[:2]
                        cross_z = base_dir[0] * to_pred[1] - base_dir[1] * to_pred[0]
                        if cross_z > 0.04:  # Left side intrusion predicted
                            distances["left"] = min(distances.get("left", 2.0), max(0.1, d_pred - 0.15))
                            distances["far_left"] = min(distances.get("far_left", 2.0), max(0.1, d_pred - 0.15))
                        elif cross_z < -0.04:  # Right side intrusion predicted
                            distances["right"] = min(distances.get("right", 2.0), max(0.1, d_pred - 0.15))
                            distances["far_right"] = min(distances.get("far_right", 2.0), max(0.1, d_pred - 0.15))

            # Compare side decision with vs without dynamic inflation
            static_side = self.choose_avoidance_side(raw_distances, self.previous_avoidance_side)
            dynamic_side = self.choose_avoidance_side(distances, self.previous_avoidance_side)
            if dynamic_side != static_side:
                self.dynamic_avoidance_events += 1
                if getattr(self, "_last_dyn_side", None) != dynamic_side:
                    self._last_dyn_side = dynamic_side
                    print(f"[DYNAMIC-KF] Kalman dynamic obstacle prediction: Inflating danger on closing side -> Switched decision {static_side} -> {dynamic_side}")

        # Directions relative to goal heading
        base_dir = self.goal_direction(current_pos)
        directions = {
            "far_left": rotate_horizontal(base_dir, 70),
            "left": rotate_horizontal(base_dir, 35),
            "front": base_dir,
            "right": rotate_horizontal(base_dir, -35),
            "far_right": rotate_horizontal(base_dir, -70),
            "rear": rotate_horizontal(base_dir, 180)
        }

        dist_to_goal = np.linalg.norm(self.goal_position - current_pos)

        # Danger condition flags
        left_dist = min(raw_distances.get("left", 2.0), raw_distances.get("far_left", 2.0), raw_distances.get("side_left", 2.0))
        right_dist = min(raw_distances.get("right", 2.0), raw_distances.get("far_right", 2.0), raw_distances.get("side_right", 2.0))
        front_dist = raw_distances["front"]

        front_blocked = distances["front"] < self.detection_distance
        left_blocked = left_dist < self.side_blocked_distance
        right_blocked = right_dist < self.side_blocked_distance
        both_sides_blocked = left_blocked and right_blocked

        obstacle_detected = (
            front_blocked
            or left_dist < self.side_detection_distance
            or right_dist < self.side_detection_distance
            or (is_dynamic and len(dynamic_sectors) > 0)
        )

        stuck = (
            self.progress_is_too_small()
            and dist_to_goal > self.goal_tolerance
            and self.mode not in ["EMERGENCY_REVERSE", "RING_ESCAPE_SEARCH", "VERTICAL_ESCAPE", "CROSS_OVER", "DESCEND", "ESCAPING_THROUGH_GAP"]
        )

        # Emergency reverse is ONLY triggered if the drone is truly trapped in a dead end pocket
        # (front blocked < 0.28m AND both sides blocked < 0.35m, or prolonged stuck against a wall)
        emergency_trap = (
            (front_dist < self.emergency_distance and both_sides_blocked)
            or (stuck and front_dist < 0.50)
        ) and (self.emergency_cooldown <= 0) and (self.mode not in ["VERTICAL_ESCAPE", "CROSS_OVER", "DESCEND", "ESCAPING_THROUGH_GAP"])

        self.previous_mode = self.mode

        # ============================================================
        # 1. HIGHEST-PRIORITY TERMINATION / SPECIAL STATES
        # ============================================================
        if dist_to_goal < self.goal_tolerance:
            self.mode = "GOAL_REACHED"

        # ------------------------------------------------------------
        # RING ESCAPE SUB-STATES (Part A)
        # ------------------------------------------------------------
        elif self.mode == "ESCAPING_THROUGH_GAP":
            dist_from_center = np.linalg.norm(current_pos[:2])
            # The drone must fly through the gap and bypass the ring footprint until past ring in X (x >= 2.7m)
            if dist_from_center > (self.ring_radius + self.ring_margin) and current_pos[0] >= (self.ring_radius + 0.1):
                print(f"[RING] Successfully cleared ring footprint via horizontal gap (pos=({current_pos[0]:.2f}, {current_pos[1]:.2f})). Resuming FORWARD toward goal")
                self.mode = "FORWARD"
                self.clear_steps = 0
                self.avoid_steps = 0
                self.reversal_step_history.clear()

        elif self.mode == "VERTICAL_ESCAPE":
            self.vertical_timer += 1
            # Periodically re-run full sweep
            if self.vertical_timer % self.vertical_check_interval == 0 and full_sweep_fn is not None:
                sweep = full_sweep_fn()
                self.gap_search_count += 1
                if sweep["all_clear"] or current_pos[2] >= self.max_climb_altitude:
                    self.cross_over_altitude = max(1.75, current_pos[2])
                    print(f"[RING] Cleared ring vertically (altitude={current_pos[2]:.2f}m, all 72 rays clear). Switching to CROSS_OVER")
                    self.mode = "CROSS_OVER"
            elif current_pos[2] >= 1.75:
                # Direct check: if altitude clears obstacle height (1.50m + margin), cross over
                self.cross_over_altitude = current_pos[2]
                print(f"[RING] Cleared obstacle vertically (altitude={current_pos[2]:.2f}m). Switching to CROSS_OVER")
                self.mode = "CROSS_OVER"

        elif self.mode == "CROSS_OVER":
            dist_from_center = np.linalg.norm(current_pos[:2])
            if dist_from_center > (self.ring_radius + self.ring_margin) or current_pos[0] >= 3.6:
                print(f"[RING] Fully past obstacle footprint (x={current_pos[0]:.2f}m, dist={dist_from_center:.2f}m). Switching to DESCEND")
                self.mode = "DESCEND"

        elif self.mode == "DESCEND":
            if current_pos[2] <= (self.flight_altitude + 0.05):
                print(f"[RING] Reached flight altitude ({current_pos[2]:.2f}m). Resuming FORWARD toward goal")
                self.mode = "FORWARD"
                self.clear_steps = 0
                self.avoid_steps = 0
                self.reversal_step_history.clear()

        # ------------------------------------------------------------
        # EMERGENCY REVERSE & RING SIGNATURE CHECK
        # ------------------------------------------------------------
        elif self.mode == "EMERGENCY_REVERSE":
            self.escape_steps_remaining -= 1
            if raw_distances["rear"] < self.emergency_distance:
                self.escape_steps_remaining = 0

            if self.escape_steps_remaining <= 0:
                # Keep only reversals within rolling window (12 seconds)
                cutoff = self.current_step_count - self.ring_window_steps
                self.reversal_step_history = [s for s in self.reversal_step_history if s >= cutoff]

                # Check for Ring Trap signature (>= 2 reversals in rolling window)
                if len(self.reversal_step_history) >= self.ring_trigger_count:
                    print(f"[RING] Ring trap signature detected ({len(self.reversal_step_history)} reversals in 12s window). Entering RING_ESCAPE_SEARCH")
                    self.ring_escape_triggered = True
                    self.mode = "RING_ESCAPE_SEARCH"
                    self.gap_search_count += 1

                    # Run full sweep to find horizontal gap or decide vertical climb
                    if full_sweep_fn is not None:
                        sweep = full_sweep_fn()
                        has_gap, gap_angle, gap_len = self._find_widest_contiguous_gap(sweep["horizontal_scan"])
                        min_dists_sec, up_clear = self._inspect_360_perimeter(sweep)

                        print("\n" + "=" * 72)
                        print("  [360-DEGREE LIDAR PERIMETER INSPECTION] Dead-end detected! Inspecting 360-deg ring...")
                        print("=" * 72)
                        for sec_name, d_val in min_dists_sec.items():
                            sec_stat = f"BLOCKED (min clear: {d_val:.2f}m)" if d_val < 1.5 else f"OPEN CLEARANCE ({d_val:.2f}m)"
                            print(f"  * Sector {sec_name:22s} : {sec_stat}")
                        print(f"  * Upward Vertical Ray (+Z)        : {up_clear:.2f}m ({'CLEAR' if up_clear >= 2.0 else 'OBSTRUCTED'})")
                        print("-" * 72)

                        if has_gap:
                            deg_per_ray = 360.0 / len(sweep["horizontal_scan"]) if len(sweep["horizontal_scan"]) > 0 else 5.0
                            print(f"  ==> DECISION: Horizontal opening verified at bearing {gap_angle:.1f} deg ({gap_len*deg_per_ray:.0f} deg wide arc).")
                            print(f"      Executing horizontal gap escape maneuver...")
                            self.gap_heading = gap_angle
                            self.escape_method = "horizontal_gap"
                            self.gap_clear_steps = 0
                            self.mode = "ESCAPING_THROUGH_GAP"
                        else:
                            print(f"  ==> DECISION: Entire 360-deg perimeter is 100% solid with NO openings anywhere.")
                            print(f"      Executing 3D Vertical Climb-Over Escape (climbing over 1.15m ring wall)...")
                            self.escape_method = "vertical_climb"
                            self.vertical_timer = 0
                            self.mode = "VERTICAL_ESCAPE"
                        print("=" * 72 + "\n")
                    else:
                        self.escape_method = "vertical_climb"
                        self.mode = "VERTICAL_ESCAPE"

                    self.position_history.clear()

                else:
                    # Normal avoidance release (handles U-trap Scenario 5 smoothly in 1 reversal)
                    self.mode = "AVOID"
                    selected_side = self.choose_avoidance_side(distances, self.previous_avoidance_side)

                    if self.force_opposite_side and self.previous_avoidance_side == "LEFT":
                        selected_side = "RIGHT"
                    elif self.force_opposite_side and self.previous_avoidance_side == "RIGHT":
                        selected_side = "LEFT"

                    self.avoidance_side = selected_side
                    self.previous_avoidance_side = selected_side
                    self.side_decision_history.append(selected_side)

                    self.avoid_steps = 0
                    self.clear_steps = 0
                    self.force_opposite_side = False
                    self.emergency_cooldown = int(0.5 * self.control_freq)
                    self.position_history.clear()

        elif emergency_trap:
            self.mode = "EMERGENCY_REVERSE"
            self.escape_steps_remaining = self.escape_steps
            self.force_opposite_side = True
            self.clear_steps = 0
            self.avoid_steps = 0
            self.position_history.clear()
            if not self.reversal_step_history or (self.current_step_count - self.reversal_step_history[-1]) >= int(1.0 * self.control_freq):
                self.reversal_step_history.append(self.current_step_count)

        # ============================================================
        # 2. NORMAL NAVIGATION & PROACTIVE POSSIBILITY CHECKING
        # ============================================================
        elif self.mode == "FORWARD":
            if obstacle_detected:
                self.mode = "AVOID"
                # Check possibilities: if obstacle is on left, choose RIGHT; if on right, choose LEFT
                self.avoidance_side = self.choose_avoidance_side(distances, self.previous_avoidance_side)
                self.previous_avoidance_side = self.avoidance_side
                self.side_decision_history.append(self.avoidance_side)
                self.avoid_steps = 0
                self.clear_steps = 0

        elif self.mode == "AVOID":
            self.avoid_steps += 1
            left_score, right_score = self.calculate_side_scores(distances)

            # Check if trapped in dead-end ring (prolonged avoidance without forward progress)
            if self.avoid_steps >= int(2.5 * self.control_freq) and stuck and front_dist < 0.60:
                print(f"[RING] Prolonged avoidance trap detected ({self.avoid_steps} steps without forward progress). Entering RING_ESCAPE_SEARCH")
                self.ring_escape_triggered = True
                self.mode = "RING_ESCAPE_SEARCH"
                self.gap_search_count += 1

                if full_sweep_fn is not None:
                    sweep = full_sweep_fn()
                    has_gap, gap_angle, gap_len = self._find_widest_contiguous_gap(sweep["horizontal_scan"])
                    if has_gap:
                        self.gap_heading = gap_angle
                        self.escape_method = "horizontal_gap"
                        self.gap_clear_steps = 0
                        self.mode = "ESCAPING_THROUGH_GAP"
                        deg_per_ray = 360.0 / len(sweep["horizontal_scan"]) if len(sweep["horizontal_scan"]) > 0 else 5.0
                        print(f"[RING] Horizontal gap found at bearing {gap_angle:.1f}° ({gap_len} clear rays = {gap_len*deg_per_ray:.0f}° arc). Switching to ESCAPING_THROUGH_GAP")
                    else:
                        self.escape_method = "vertical_climb"
                        self.vertical_timer = 0
                        self.mode = "VERTICAL_ESCAPE"
                        print(f"[RING] No horizontal gap found (closed ring). Switching to VERTICAL_ESCAPE (climbing at {self.climb_speed} m/s)")
                else:
                    self.escape_method = "vertical_climb"
                    self.mode = "VERTICAL_ESCAPE"

                self.position_history.clear()

            # Dynamic path adjustment: switch side if current side becomes heavily blocked while alternative is wide open
            elif self.avoidance_side == "LEFT" and left_dist < 0.30 and right_score > left_score + 0.20:
                self.avoidance_side = "RIGHT"
                self.previous_avoidance_side = "RIGHT"
                self.side_decision_history.append("RIGHT")
                self.avoid_steps = 0
            elif self.avoidance_side == "RIGHT" and right_dist < 0.30 and left_score > right_score + 0.20:
                self.avoidance_side = "LEFT"
                self.previous_avoidance_side = "LEFT"
                self.side_decision_history.append("LEFT")
                self.avoid_steps = 0

            # Determine if obstacle alongside is cleared
            if self.avoidance_side == "RIGHT":
                obstacle_alongside = left_dist < self.side_detection_distance
            else:
                obstacle_alongside = right_dist < self.side_detection_distance

            if distances["front"] > self.front_clear_distance and not obstacle_alongside:
                self.clear_steps += 1
            else:
                self.clear_steps = 0

            if self.avoid_steps >= self.min_avoid_steps and self.clear_steps >= self.clear_confirmation_steps:
                self.mode = "RETURN"
                self.clear_steps = 0

        elif self.mode == "RETURN":
            if obstacle_detected:
                self.mode = "AVOID"
                self.avoidance_side = self.choose_avoidance_side(distances, self.previous_avoidance_side)
                self.previous_avoidance_side = self.avoidance_side
                self.side_decision_history.append(self.avoidance_side)
                self.avoid_steps = 0
                self.clear_steps = 0
            elif abs(current_pos[1]) < 0.10:
                self.mode = "FORWARD"

        # Record event transitions
        if self.mode == "AVOID" and self.previous_mode != "AVOID":
            self.avoidance_events += 1
        if self.mode == "EMERGENCY_REVERSE" and self.previous_mode != "EMERGENCY_REVERSE":
            self.emergency_reversals += 1

        # Calculate desired velocity command
        desired_velocity = self.make_velocity_command(current_pos, directions, distances)

        if self.mode == "GOAL_REACHED":
            target_pos = self.goal_position.copy()
        elif self.mode == "VERTICAL_ESCAPE":
            target_pos = current_pos.copy()
            target_pos[2] = current_pos[2] + self.climb_speed * 0.65
        elif self.mode == "CROSS_OVER":
            target_pos = current_pos + desired_velocity * 0.65
            target_pos[2] = self.cross_over_altitude
        elif self.mode == "DESCEND":
            target_pos = current_pos.copy()
            target_pos[2] = max(self.flight_altitude, current_pos[2] - self.descend_speed * 0.65)
        else:
            target_pos = current_pos + desired_velocity * 0.65
            target_pos[2] = self.flight_altitude

        status_text = (
            f"AVOID {self.avoidance_side}" if self.mode == "AVOID" else
            "TRAPPED: REVERSING" if self.mode == "EMERGENCY_REVERSE" else
            "RING: GAP ESCAPE" if self.mode == "ESCAPING_THROUGH_GAP" else
            "RING: VERTICAL CLIMB" if self.mode == "VERTICAL_ESCAPE" else
            "RING: CROSSING OVER" if self.mode == "CROSS_OVER" else
            "RING: DESCENDING" if self.mode == "DESCEND" else
            "RETURNING TO PATH" if self.mode == "RETURN" else
            "GOAL REACHED" if self.mode == "GOAL_REACHED" else
            "MOVING TO GOAL"
        )

        return target_pos, desired_velocity, {
            "mode": self.mode,
            "side": self.avoidance_side,
            "status_text": status_text,
            "dist_to_goal": dist_to_goal,
            "avoidance_events": self.avoidance_events,
            "emergency_reversals": self.emergency_reversals,
            "ring_escape_triggered": self.ring_escape_triggered,
            "escape_method": self.escape_method,
            "max_altitude_reached": round(self.max_altitude_reached, 3),
            "gap_search_count": self.gap_search_count,
            "dynamic_avoidance_events": self.dynamic_avoidance_events,
            "dynamic_obstacle": is_dynamic
        }
