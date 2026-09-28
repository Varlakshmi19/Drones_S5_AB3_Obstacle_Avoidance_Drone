"""
Dynamic Window Approach (DWA) for Drone Obstacle Avoidance.
Projects circular arc trajectories and scores velocity candidates based on goal heading, obstacle clearance, and speed.
"""

import numpy as np
from .base_avoidance import BaseObstacleAvoidance


class DWAObstacleAvoidance(BaseObstacleAvoidance):
    """
    Dynamic Window Approach for reactive navigation.
    """

    def __init__(
        self,
        goal_position=np.array([6.0, 0.0, 1.0]),
        flight_altitude=1.0,
        max_speed=0.38,
        predict_time=1.5,
        dt_sim=0.1
    ):
        super().__init__(goal_position, flight_altitude)
        self.max_speed = max_speed
        self.predict_time = predict_time
        self.dt_sim = dt_sim

        # Cost weights
        self.w_heading = 0.25
        self.w_clearance = 0.55
        self.w_velocity = 0.20

        self.mode = "FORWARD"
        self.avoidance_events = 0
        self.prev_mode = "FORWARD"

    def reset(self):
        self.mode = "FORWARD"
        self.avoidance_events = 0
        self.prev_mode = "FORWARD"

    def compute_step(self, current_pos, current_vel, perception_data, dt=1.0/48.0):
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

        # Vectorized candidate generation (speed v, heading angle theta)
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
        vel_cands = np.column_stack([vx_cand, vy_cand]) # shape (N, 2)

        # Time horizon vector
        traj_times = np.linspace(0.1, self.predict_time, 8)[:, np.newaxis, np.newaxis] # shape (T, 1, 1)
        
        # Predicted positions: shape (T, N, 2)
        # current_pos[:2] (2,) + times (T, 1, 1) * vel_cands (1, N, 2)
        predicted_trajs = current_pos[:2] + traj_times * vel_cands[np.newaxis, :, :] # (T, N, 2)

        # Filter hit points to obstacles ahead of drone
        goal_unit = goal_dir / max(1e-5, np.linalg.norm(goal_dir))
        ahead_pts = [p for p in hit_points if np.dot(p - current_pos[:2], goal_unit) > -0.15]
        min_dist_ahead = min([np.linalg.norm(p - current_pos[:2]) for p in ahead_pts] or [2.0])

        # Compute clearance against obstacle hit points ahead (evaluating future predicted steps t >= 1)
        if len(ahead_pts) > 0:
            obs_array = np.array(ahead_pts) # shape (M, 2)
            diff = predicted_trajs[1:, :, np.newaxis, :] - obs_array[np.newaxis, np.newaxis, :, :]
            dists = np.linalg.norm(diff, axis=3) # shape (T-1, N, M)
            min_dists_per_cand = np.min(dists, axis=(0, 2)) # shape (N,)
        else:
            min_dists_per_cand = np.full(len(vel_cands), 2.0)

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
        if not hasattr(self, "avoidance_side"):
            self.avoidance_side = None

        if min_dist_ahead < 0.65:
            if self.avoidance_side is None:
                left_pts = [p for p in ahead_pts if p[1] > current_pos[1]]
                right_pts = [p for p in ahead_pts if p[1] <= current_pos[1]]
                left_clear = min([np.linalg.norm(p - current_pos[:2]) for p in left_pts] or [2.0])
                right_clear = min([np.linalg.norm(p - current_pos[:2]) for p in right_pts] or [2.0])
                
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

        # Mask out collisions (clearance < 0.20m)
        total_scores[min_dists_per_cand < 0.20] = -1e9

        best_idx = np.argmax(total_scores)
        if total_scores[best_idx] <= -1e8:
            # Check possibility on left vs right: steer to open side without moving backward
            left_pts = [p for p in ahead_pts if p[1] > current_pos[1]]
            right_pts = [p for p in ahead_pts if p[1] <= current_pos[1]]
            left_clear = min([np.linalg.norm(p - current_pos[:2]) for p in left_pts] or [2.0])
            right_clear = min([np.linalg.norm(p - current_pos[:2]) for p in right_pts] or [2.0])
            
            # If left is blocked, steer right; if right is blocked, steer left
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
            "status_text": "DWA AVOIDING" if self.mode == "AVOID" else "DWA CRUISING",
            "dist_to_goal": dist_to_goal,
            "min_obs_dist": min_dist,
            "avoidance_events": self.avoidance_events
        }
