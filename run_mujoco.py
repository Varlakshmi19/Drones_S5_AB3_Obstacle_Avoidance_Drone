#!/usr/bin/env python3
"""
Root-level convenient entry point for running the MuJoCo Quadcopter Obstacle Avoidance Simulation.

Usage examples:
  python run_mujoco.py --dashboard
  python run_mujoco.py --scenario 1 --algo reactive --gui
  python run_mujoco.py --scenario 7 --algo sindy_dwa --gui
  python run_mujoco.py --benchmark
  python run_mujoco.py --editor
"""

import sys
import os

# Add mujoco_drone_sim directory to Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
sim_dir = os.path.join(current_dir, "mujoco_drone_sim")
if sim_dir not in sys.path:
    sys.path.insert(0, sim_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from mujoco_drone_sim.run_sim import main

if __name__ == "__main__":
    main()
