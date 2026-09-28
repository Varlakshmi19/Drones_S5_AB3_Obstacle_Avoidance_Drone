"""
Root benchmark runner for MuJoCo Drone Obstacle Avoidance Simulation.
"""

import sys
import os

root_dir = os.path.dirname(os.path.abspath(__file__))
if root_dir not in sys.path:
    sys.path.insert(0, root_dir)

from mujoco_drone_sim.benchmark import run_benchmark_suite

if __name__ == "__main__":
    run_benchmark_suite()
