"""
3D Volumetric Obstacle Avoidance for Drone in MuJoCo.
Allows vertical altitude adjustments (climbing over low obstacles or descending below overhangs)
as well as lateral bypass navigation.
"""

import numpy as np
from .base_avoidance import BaseObstacleAvoidance


class Avoidance3D(BaseObstacleAvoidance):
    """
    3D Obstacle Avoidance allowing altitude climbs or lateral avoidance maneuvers.
    """

    def __init__(
        self,
        goal_position=np.array([6.0, 0.0, 1.0]),
        flight_altitude=1.0,
        max_speed=0.35,
        max_climb_speed=0.25
    ):
        super().__init__(goal_position, flight_altitude)
        self.max_speed = max_speed
        self.max_climb_speed = max_climb_speed
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

        distances = perception_data.get("distances", {})
        front_dist = distances.get("front", 2.0)
        up_dist = distances.get("up", 2.0)
        down_dist = distances.get("down", 2.0)
        left_dist = distances.get("left", 2.0)
        right_dist = distances.get("right", 2.0)

        # Base 3D goal vector
        goal_vec = self.goal_position - current_pos
        d_goal = np.linalg.norm(goal_vec)
        unit_goal = goal_vec / d_goal if d_goal > 1e-5 else np.zeros(3)

        desired_vel = unit_goal * self.max_speed

        # Check for obstacles (front, left, right)
        obstacle_detected = front_dist < 0.85 or left_dist < 0.60 or right_dist < 0.60

        if obstacle_detected:
            self.mode = "AVOID_3D"
            if self.prev_mode != "AVOID_3D":
                self.avoidance_events += 1

            # Possibility check:
            # 1. If obstacle is on the left, steer right
            # 2. If obstacle is on the right, steer left
            # 3. If obstacle in front, climb over if upward path clear, otherwise steer to clearer side
            if left_dist < 0.60 and right_dist >= left_dist:
                desired_vel[1] = -self.max_speed * 0.7
                desired_vel[0] = self.max_speed * 0.8
            elif right_dist < 0.60 and left_dist > right_dist:
                desired_vel[1] = self.max_speed * 0.7
                desired_vel[0] = self.max_speed * 0.8
            elif up_dist > 1.2 and current_pos[2] < 1.8:
                # Climb over obstacle while maintaining forward momentum
                desired_vel[2] = self.max_climb_speed
                desired_vel[0] = self.max_speed * 0.7
            elif left_dist >= right_dist:
                desired_vel[1] = self.max_speed * 0.8
                desired_vel[0] = self.max_speed * 0.5
            else:
                desired_vel[1] = -self.max_speed * 0.8
                desired_vel[0] = self.max_speed * 0.5
        else:
            self.mode = "FORWARD"
            # Return smoothly to default altitude
            alt_err = self.flight_altitude - current_pos[2]
            desired_vel[2] = np.clip(alt_err * 0.8, -self.max_climb_speed, self.max_climb_speed)

        self.prev_mode = self.mode

        target_pos = current_pos + desired_vel * 0.65
        status_text = "3D CLIMB/AVOID" if self.mode == "AVOID_3D" else "3D CRUISING"

        return target_pos, desired_vel, {
            "mode": self.mode,
            "status_text": status_text,
            "dist_to_goal": dist_to_goal,
            "min_obs_dist": front_dist,
            "avoidance_events": self.avoidance_events
        }
