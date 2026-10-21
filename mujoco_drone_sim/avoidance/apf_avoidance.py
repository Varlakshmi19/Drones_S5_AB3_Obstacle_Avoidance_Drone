"""
Artificial Potential Field (APF) Obstacle Avoidance for MuJoCo Drone.
Combines attractive potential to goal with multi-ray repulsive potential fields
and a tangential vortex force for robust local-minima escape.
"""

import numpy as np
from .base_avoidance import BaseObstacleAvoidance


class APFObstacleAvoidance(BaseObstacleAvoidance):
    """
    Artificial Potential Field (APF) with curl-field escape mechanism.
    """

    def __init__(
        self,
        goal_position=np.array([6.0, 0.0, 1.0]),
        flight_altitude=1.0,
        k_att=1.0,
        k_rep=0.45,
        influence_radius=1.2,
        max_speed=0.40
    ):
        super().__init__(goal_position, flight_altitude)
        self.k_att = k_att
        self.k_rep = k_rep
        self.d0 = influence_radius
        self.max_speed = max_speed
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

        # 1. Attractive Force toward Goal (2D horizontal)
        goal_vec = self.goal_position[:2] - current_pos[:2]
        d_goal = np.linalg.norm(goal_vec)
        if d_goal > 1e-6:
            f_att = self.k_att * (goal_vec / d_goal)
        else:
            f_att = np.zeros(2)

        # 2. Repulsive Force from LiDAR Obstacle Points
        f_rep = np.zeros(2)
        min_obs_dist = 999.0
        rays = perception_data.get("rays", [])

        for ray in rays:
            dist = ray["distance"]
            if dist < min_obs_dist:
                min_obs_dist = dist

            if dist < self.d0 and ray["hit"]:
                # Vector from obstacle hit point to drone
                diff = current_pos[:2] - ray["end"][:2]
                r = np.linalg.norm(diff)
                if r > 1e-4:
                    unit_diff = diff / r
                    # Standard Khatib potential repulsion: (1/d - 1/d0) * (1/d^2)
                    magnitude = self.k_rep * (1.0 / dist - 1.0 / self.d0) * (1.0 / (dist ** 2))
                    f_rep += magnitude * unit_diff

                    # Tangential Vortex / Curl Force to prevent deadlock in front of obstacles
                    vortex_dir = np.array([-unit_diff[1], unit_diff[0]])  # 90 deg rotation
                    if np.dot(vortex_dir, goal_vec) < 0:
                        vortex_dir = -vortex_dir
                    f_rep += 0.35 * magnitude * vortex_dir

        # Total Resultant Force
        f_total = f_att + f_rep
        mag_total = np.linalg.norm(f_total)

        if min_obs_dist < 0.75:
            self.mode = "AVOID"
            if self.prev_mode != "AVOID":
                self.avoidance_events += 1
        else:
            self.mode = "FORWARD"
        self.prev_mode = self.mode

        if mag_total > 1e-6:
            desired_vel_2d = (f_total / mag_total) * min(self.max_speed, mag_total)
        else:
            desired_vel_2d = np.zeros(2)

        desired_vel = np.array([desired_vel_2d[0], desired_vel_2d[1], 0.0])
        target_pos = current_pos + desired_vel * 0.65
        target_pos[2] = self.flight_altitude

        status_text = "APF AVOIDING" if self.mode == "AVOID" else "APF TRACKING GOAL"

        return target_pos, desired_vel, {
            "mode": self.mode,
            "status_text": status_text,
            "dist_to_goal": dist_to_goal,
            "min_obs_dist": min_obs_dist,
            "avoidance_events": self.avoidance_events
        }
