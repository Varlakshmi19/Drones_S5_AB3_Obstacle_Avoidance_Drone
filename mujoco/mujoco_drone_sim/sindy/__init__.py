"""
SINDy (Sparse Identification of Nonlinear Dynamics) Module for MuJoCo Drone Simulation.
Provides data-driven dynamic identification, trajectory forecasting, and equation discovery.
"""

from .sindy_core import SINDy, PolynomialLibrary, finite_difference, stlsq
from .drone_identifier import DroneDynamicsIdentifier
from .obstacle_predictor import SINDyObstaclePredictor

__all__ = [
    "SINDy",
    "PolynomialLibrary",
    "finite_difference",
    "stlsq",
    "DroneDynamicsIdentifier",
    "SINDyObstaclePredictor"
]
