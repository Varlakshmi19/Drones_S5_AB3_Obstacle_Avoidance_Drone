"""
Base Class for Obstacle Avoidance Strategies.
"""

from abc import ABC, abstractmethod
import numpy as np


class BaseObstacleAvoidance(ABC):
    """
    Abstract interface for obstacle avoidance algorithms.
    """

    def __init__(self, goal_position=np.array([6.0, 0.0, 1.0]), flight_altitude=1.0):
        self.goal_position = np.array(goal_position, dtype=float)
        self.flight_altitude = flight_altitude
        self.goal_tolerance = 0.25

    def reset(self):
        """Reset internal controller state."""
        pass

    @abstractmethod
    def compute_step(self, current_pos, current_vel, perception_data, dt):
        """
        Compute desired target position and target velocity for the flight controller.

        Args:
            current_pos: [x, y, z]
            current_vel: [vx, vy, vz]
            perception_data: output dictionary from LiDAR perception system
            dt: control timestep

        Returns:
            target_pos: [x, y, z]
            desired_vel: [vx, vy, vz]
            status_info: dict with mode, status_text, metrics
        """
        pass
