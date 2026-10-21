"""
Interactive and Headless Simulation Runner for MuJoCo Drone Obstacle Avoidance.

Usage:
  python run_sim.py --scenario 1 --algo reactive --gui
  python run_sim.py --scenario 5 --algo reactive
  python run_sim.py --scenario 8 --algo 3d --gui
  python run_sim.py --benchmark
  python run_sim.py --dashboard
"""

import sys
import os
import time
import argparse
import json
import numpy as np

# Ensure workspace root is in python path
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace_dir = os.path.abspath(os.path.join(current_dir, ".."))
if workspace_dir not in sys.path:
    sys.path.insert(0, workspace_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

import mujoco
from mujoco_drone_sim.simulation.drone_sim import DroneObstacleSimulation


def run_simulation(
    scenario_id=1,
    algo="reactive",
    gui=False,
    duration=60.0,
    realtime_sync=True,
    verbose=True,
    log_file=None,
    sub_scenario="closed_ring",
    seed=42,
    intruder_speed=1.0,
    custom_obstacles=None,
    obstacle_noise=True,
    obs_noise_std=0.08,
    enable_kalman=True,
    sensor_noise=0.06
):
    """
    Run single scenario flight simulation with optional MuJoCo 3D interactive viewer.
    """
    print("=" * 65)
    print(f"  MuJoCo Quadcopter Obstacle Avoidance Simulation")
    print(f"  Scenario: {scenario_id} | Algorithm: {algo.upper()} | GUI: {gui}")
    if custom_obstacles:
        print(f"  Custom Obstacles: {len(custom_obstacles)} configured via user arrangement")
    elif scenario_id == 10:
        print(f"  Sub-scenario: {sub_scenario} | Seed: {seed}")
    elif scenario_id in [7, 9]:
        print(f"  Obstacle Noise: {obstacle_noise} (std={obs_noise_std}) | Kalman Filter: {enable_kalman}")
        if scenario_id == 9:
            print(f"  Intruder Speed Multiplier: {intruder_speed}x")
    print("=" * 65)

    sim = DroneObstacleSimulation(
        scenario_id=scenario_id,
        algorithm=algo,
        sub_scenario=sub_scenario,
        seed=seed,
        intruder_speed_multiplier=intruder_speed,
        custom_obstacles=custom_obstacles,
        obstacle_noise=obstacle_noise,
        obs_noise_std=obs_noise_std,
        enable_kalman=enable_kalman,
        measurement_noise_std=sensor_noise
    )

    total_steps = int(duration * sim.control_freq)
    step_delay = 1.0 / sim.control_freq

    viewer = None
    if gui:
        try:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
            print("[INFO] Interactive MuJoCo 3D Viewer launched successfully!")
        except Exception as e:
            print(f"[WARNING] Could not launch passive GUI viewer ({e}). Running in headless console mode.")
            viewer = None

    start_time = time.time()
    last_print_time = start_time

    try:
        for step in range(total_steps):
            step_start = time.time()
            info = sim.step()

            # Render 3D frame if viewer is open
            if viewer is not None and viewer.is_running():
                viewer.sync()
            elif viewer is not None and not viewer.is_running():
                print("\n[INFO] Viewer window closed by user.")
                break

            # Print telemetry to console at 5Hz
            now = time.time()
            if verbose and (now - last_print_time >= 0.20 or info["result"] != "RUNNING"):
                pos = info["pos"]
                vel = info["vel"]
                dist_goal = info["dist_to_goal"]
                mode = info["mode"]
                distances = info["perception"]["distances"]
                front_d = distances.get("front", 2.0)
                left_d = distances.get("left", 2.0)
                right_d = distances.get("right", 2.0)

                print(
                    f"Time: {info['sim_time']:5.2f}s | "
                    f"Mode: {mode:16s} | "
                    f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) | "
                    f"Vel: {np.linalg.norm(vel):.2f}m/s | "
                    f"Front: {front_d:.2f}m | "
                    f"L: {left_d:.2f}m | "
                    f"R: {right_d:.2f}m | "
                    f"Goal Dist: {dist_goal:.2f}m"
                )
                last_print_time = now

            if info["result"] != "RUNNING":
                break

            # Real-time synchronization
            if realtime_sync and gui:
                elapsed_step = time.time() - step_start
                sleep_time = step_delay - elapsed_step
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[INFO] Simulation interrupted by user.")
    finally:
        if viewer is not None:
            viewer.close()

    summary = sim.get_summary()

    print("\n" + "=" * 65)
    print("  EXPERIMENT PERFORMANCE SUMMARY")
    print("=" * 65)
    print(f"  Result:                   {summary['result']}")
    print(f"  Completion Time:          {summary['completion_time']:.2f} s")
    print(f"  Control Steps:            {summary['control_steps']}")
    print(f"  Final Position:           {summary['final_position']}")
    print(f"  Path Length:              {summary['path_length']:.3f} m")
    print(f"  Collision Steps:          {summary['collision_steps']}")
    print(f"  Minimum Clearance:        {summary['min_clearance']:.3f} m")
    print(f"  Avoidance Events:         {summary['avoidance_events']}")
    print(f"  Emergency Reversals:      {summary['emergency_reversals']}")
    if summary.get("dynamic_obstacles_count", 0) > 0 or scenario_id in [7, 9]:
        print(f"  Obstacle Noise Enabled:   {summary.get('obstacle_noise_enabled', False)}")
        print(f"  Kalman Filter Enabled:    {summary.get('kalman_filter_enabled', False)}")
        print(f"  Raw Sensor Error (RMSE):  {summary.get('raw_sensor_rmse_pos', 0.0):.4f} m")
        print(f"  Kalman Est. Error (RMSE): {summary.get('kalman_rmse_pos', 0.0):.4f} m")
        print(f"  Kalman Noise Reduction:   {summary.get('kalman_noise_reduction_pct', 0.0):.1f} %")
    print("=" * 65 + "\n")

    if log_file:
        def make_serializable(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if isinstance(obj, (np.float32, np.float64, np.floating)):
                return float(obj)
            if isinstance(obj, (np.int32, np.int64, np.integer)):
                return int(obj)
            if isinstance(obj, dict):
                return {k: make_serializable(v) for k, v in obj.items()}
            if isinstance(obj, list):
                return [make_serializable(x) for x in obj]
            return obj

        summary_export = make_serializable(summary)
        summary_export["flight_history"] = make_serializable(sim.history)
        if hasattr(sim, "kalman_telemetry") and sim.kalman_telemetry:
            summary_export["kalman_telemetry"] = make_serializable(sim.kalman_telemetry)
        with open(log_file, "w") as f:
            json.dump(summary_export, f, indent=2)
        print(f"[INFO] Flight telemetry saved to {log_file}")

    return summary


def main():
    parser = argparse.ArgumentParser(description="MuJoCo Quadcopter Obstacle Avoidance Runner")
    parser.add_argument("--scenario", "-s", type=int, default=1, choices=range(1, 11),
                        help="Scenario ID (1-10): 1:Frontal, 2:Left, 3:Right, 4:Narrow, 5:Dead-end, 6:Forest, 7:Dynamic, 8:3D, 9:Intruder, 10:Ring")
    parser.add_argument("--sub-scenario", type=str, default="closed_ring", choices=["closed_ring", "ring_with_gap"],
                        help="Sub-scenario for Scenario 10: closed_ring or ring_with_gap")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for Scenario 10 gap orientation (default: 42)")
    parser.add_argument("--intruder-speed", type=float, default=1.0,
                        help="Speed multiplier for Scenario 9 lateral intruder (default: 1.0)")
    parser.add_argument("--algo", "-a", type=str, default="reactive", choices=["reactive", "apf", "dwa", "3d", "sindy_dwa", "neural", "brain"],
                        help="Avoidance algorithm: reactive, apf, dwa, 3d, sindy_dwa, neural")
    parser.add_argument("--sindy-train", action="store_true",
                        help="Run flight trajectory and identify symbolic equations of motion using SINDy")
    parser.add_argument("--gui", "-g", action="store_true",
                        help="Launch interactive MuJoCo 3D GUI viewer")
    parser.add_argument("--duration", "-d", type=float, default=60.0,
                        help="Maximum simulation duration in seconds (default: 60)")
    parser.add_argument("--no-sync", action="store_true",
                        help="Run faster than real-time (no FPS sync)")
    parser.add_argument("--benchmark", "-b", action="store_true",
                        help="Run full automated benchmark across all scenarios and algorithms")
    parser.add_argument("--dashboard", action="store_true",
                        help="Launch interactive Web Telemetry Dashboard")
    parser.add_argument("--custom-file", type=str, default=None,
                        help="Path to custom obstacles JSON arrangement file")
    parser.add_argument("--editor", action="store_true",
                        help="Launch interactive Mouse & Keyboard Arena Customizer GUI")
    parser.add_argument("--no-obs-noise", action="store_true",
                        help="Disable stochastic obstacle motion noise (use deterministic motion)")
    parser.add_argument("--obs-noise-std", type=float, default=0.08,
                        help="Obstacle stochastic process noise standard deviation (default: 0.08)")
    parser.add_argument("--no-kalman", action="store_true",
                        help="Disable Kalman filter dynamic obstacle tracking")
    parser.add_argument("--sensor-noise", type=float, default=0.06,
                        help="LiDAR measurement noise standard deviation (default: 0.06m)")
    parser.add_argument("--log", type=str, default=None,
                        help="Output JSON log filepath")

    args = parser.parse_args()

    if args.editor:
        from mujoco_drone_sim.interactive_arena import launch_arena_editor
        launch_arena_editor()
        return

    custom_obstacles = None
    if args.custom_file and os.path.exists(args.custom_file):
        with open(args.custom_file, "r") as f:
            custom_data = json.load(f)
            custom_obstacles = custom_data.get("obstacles", custom_data if isinstance(custom_data, list) else None)
        print(f"[INFO] Loaded {len(custom_obstacles) if custom_obstacles else 0} custom obstacles from {args.custom_file}")

    if args.sindy_train:
        from mujoco_drone_sim.sindy import DroneDynamicsIdentifier
        print("\n[INFO] Starting SINDy Quadcopter Dynamics Identification...")
        sim = DroneObstacleSimulation(scenario_id=args.scenario, algorithm="reactive", intruder_speed_multiplier=args.intruder_speed)
        log = []
        for _ in range(int(args.duration * sim.control_freq)):
            pos, quat, vel, angvel = sim.get_drone_state()
            info = sim.step()
            log.append({
                "time": sim.sim_time,
                "pos": pos,
                "vel": vel,
                "quat": quat,
                "angvel": angvel,
                "ctrl": sim.data.ctrl.copy()
            })
            if info["result"] != "RUNNING":
                break
        identifier = DroneDynamicsIdentifier(dt=1.0 / sim.control_freq)
        identifier.fit_from_telemetry(log)
        identifier.print_summary()

    elif args.dashboard:
        from mujoco_drone_sim.web_dashboard.app import start_dashboard
        start_dashboard()
    elif args.benchmark:
        from mujoco_drone_sim.benchmark import run_benchmark_suite
        run_benchmark_suite()
    else:
        run_simulation(
            scenario_id=args.scenario,
            algo=args.algo,
            gui=args.gui,
            duration=args.duration,
            realtime_sync=not args.no_sync,
            log_file=args.log,
            sub_scenario=args.sub_scenario,
            seed=args.seed,
            intruder_speed=args.intruder_speed,
            custom_obstacles=custom_obstacles,
            obstacle_noise=not args.no_obs_noise,
            obs_noise_std=args.obs_noise_std,
            enable_kalman=not args.no_kalman,
            sensor_noise=args.sensor_noise
        )


if __name__ == "__main__":
    main()
