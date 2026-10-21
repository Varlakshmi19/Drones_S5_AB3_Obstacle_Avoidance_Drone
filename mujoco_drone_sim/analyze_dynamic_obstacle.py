"""
Dynamic Obstacle & Drone Interaction Analysis Tool.

Analyzes:
1. Drone trajectory kinematics (position, velocity, speed, flight modes).
2. Dynamic obstacle stochastic behavior (true trajectory vs noisy measurements vs Kalman filtered state).
3. Relative separation distance, Closest Point of Approach (CPA), and safety margins.
4. Kalman Filter tracking accuracy (RMSE reduction %, covariance envelope, velocity estimation).
5. Generates high-resolution multi-panel plots and terminal report.

Usage:
  python analyze_dynamic_obstacle.py --scenario 7
  python analyze_dynamic_obstacle.py --scenario 9 --algo reactive --save-plot
  python analyze_dynamic_obstacle.py --scenario 7 --no-noise  # compare with deterministic baseline
"""

import sys
import os
import argparse
import numpy as np

# Ensure workspace root in path
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace_dir = os.path.abspath(os.path.join(current_dir, ".."))
for p in [workspace_dir, current_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

from mujoco_drone_sim.simulation.drone_sim import DroneObstacleSimulation


def run_dynamic_analysis(
    scenario_id=7,
    algo="reactive",
    duration=25.0,
    obstacle_noise=True,
    obs_noise_std=0.08,
    enable_kalman=True,
    sensor_noise=0.06,
    intruder_speed=1.0,
    save_plot=True,
    plot_file="dynamic_obstacle_analysis.png",
    show_plot=False
):
    print("=" * 70)
    print("  DYNAMIC OBSTACLE & DRONE KINEMATIC ANALYSIS")
    print(f"  Scenario: {scenario_id} | Algorithm: {algo.upper()}")
    print(f"  Obstacle Process Noise: {obstacle_noise} (std={obs_noise_std}m)")
    print(f"  Sensor Measurement Noise: {sensor_noise}m | Kalman Filter: {enable_kalman}")
    print("=" * 70)

    sim = DroneObstacleSimulation(
        scenario_id=scenario_id,
        algorithm=algo,
        intruder_speed_multiplier=intruder_speed,
        obstacle_noise=obstacle_noise,
        obs_noise_std=obs_noise_std,
        enable_kalman=enable_kalman,
        measurement_noise_std=sensor_noise
    )

    times = []
    drone_positions = []
    drone_velocities = []
    drone_speeds = []
    drone_modes = []

    obs_times = []
    obs_true_positions = []
    obs_noisy_positions = []
    obs_kalman_positions = []
    obs_kalman_velocities = []
    obs_kalman_stds = []
    relative_distances = []
    err_noisy_list = []
    err_kalman_list = []

    total_steps = int(duration * sim.control_freq)

    for step in range(total_steps):
        pos, quat, vel, angvel = sim.get_drone_state()
        info = sim.step()

        t = info["sim_time"]
        times.append(t)
        drone_positions.append(pos.copy())
        drone_velocities.append(vel.copy())
        drone_speeds.append(float(np.linalg.norm(vel)))
        drone_modes.append(info["mode"])

        # Record dynamic obstacle data if present
        perception = info.get("perception", {})
        kalman_tracks = perception.get("kalman_tracks", {})
        raw_meas = perception.get("raw_measurements", {})

        # If we have tracked dynamic obstacles
        if kalman_tracks and raw_meas:
            for obs_name, kf_info in kalman_tracks.items():
                if obs_name in raw_meas:
                    m = raw_meas[obs_name]
                    tp = np.array(m["true_pos"])
                    np_pos = np.array(m["noisy_pos"])
                    kp = np.array(kf_info["pos"])
                    kv = np.array(kf_info["vel"])
                    kstd = np.array(kf_info["pos_std"])

                    obs_times.append(t)
                    obs_true_positions.append(tp)
                    obs_noisy_positions.append(np_pos)
                    obs_kalman_positions.append(kp)
                    obs_kalman_velocities.append(kv)
                    obs_kalman_stds.append(kstd)

                    d_rel = float(np.linalg.norm(pos - tp))
                    relative_distances.append(d_rel)

                    err_noisy_list.append(float(np.linalg.norm(np_pos - tp)))
                    err_kalman_list.append(float(np.linalg.norm(kp - tp)))
                    break  # primary dynamic obstacle

        if info["result"] != "RUNNING":
            break

    summary = sim.get_summary()

    # Convert arrays
    times = np.array(times)
    drone_positions = np.array(drone_positions)
    drone_velocities = np.array(drone_velocities)
    drone_speeds = np.array(drone_speeds)

    obs_times = np.array(obs_times)
    obs_true_positions = np.array(obs_true_positions) if len(obs_true_positions) > 0 else np.zeros((0, 3))
    obs_noisy_positions = np.array(obs_noisy_positions) if len(obs_noisy_positions) > 0 else np.zeros((0, 3))
    obs_kalman_positions = np.array(obs_kalman_positions) if len(obs_kalman_positions) > 0 else np.zeros((0, 3))
    obs_kalman_velocities = np.array(obs_kalman_velocities) if len(obs_kalman_velocities) > 0 else np.zeros((0, 3))
    relative_distances = np.array(relative_distances) if len(relative_distances) > 0 else np.array([sim.min_clearance])

    # Compute key statistics
    min_distance = float(np.min(relative_distances)) if len(relative_distances) > 0 else sim.min_clearance
    cpa_idx = int(np.argmin(relative_distances)) if len(relative_distances) > 0 else 0
    cpa_time = float(obs_times[cpa_idx]) if len(obs_times) > cpa_idx else 0.0

    raw_rmse = float(np.sqrt(np.mean(np.square(err_noisy_list)))) if err_noisy_list else 0.0
    kalman_rmse = float(np.sqrt(np.mean(np.square(err_kalman_list)))) if err_kalman_list else 0.0
    reduction_pct = float((1.0 - kalman_rmse / raw_rmse) * 100.0) if raw_rmse > 1e-6 else 0.0

    print("\n" + "=" * 70)
    print("  QUANTITATIVE ANALYSIS RESULTS")
    print("=" * 70)
    print(f"  [Flight Kinematics]")
    print(f"    - Mission Outcome:        {summary['result']}")
    print(f"    - Completion Time:        {summary['completion_time']:.2f} s")
    print(f"    - Total Path Length:      {summary['path_length']:.3f} m")
    print(f"    - Average Flight Speed:   {float(np.mean(drone_speeds)):.2f} m/s")
    print(f"    - Peak Flight Speed:      {float(np.max(drone_speeds)):.2f} m/s")
    print(f"    - Avoidance Maneuvers:    {summary['avoidance_events']}")
    print(f"    - Physical Collisions:    {summary['collision_steps']}")
    print(f"")
    print(f"  [Obstacle & Interaction Metrics]")
    print(f"    - Closest Point of Appr.: {min_distance:.3f} m (at t = {cpa_time:.2f}s)")
    print(f"    - Safety Margin:          {min_distance - 0.30:.3f} m above emergency threshold (0.30m)")
    if len(obs_true_positions) > 0:
        y_travel = float(np.max(obs_true_positions[:, 1]) - np.min(obs_true_positions[:, 1]))
        print(f"    - Obstacle Travel Span:   {y_travel:.3f} m along lateral axis")
    print(f"")
    print(f"  [Kalman Filter Tracking Performance]")
    print(f"    - Raw Sensor RMSE:        {raw_rmse:.4f} m (noisy LiDAR observations)")
    print(f"    - Kalman Filter RMSE:     {kalman_rmse:.4f} m (state estimation error)")
    print(f"    - Noise Reduction:        {reduction_pct:.1f} % variance suppression")
    print("=" * 70 + "\n")

    # Generate Visualization if matplotlib is available
    if save_plot or show_plot:
        try:
            import matplotlib.pyplot as plt
            from matplotlib.gridspec import GridSpec

            fig = plt.figure(figsize=(16, 11), facecolor="#0e131f")
            gs = GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.25)

            # Styling colors
            bg_card = "#161c2d"
            text_color = "#f0f4fc"
            cyan = "#00e5ff"
            green = "#00e676"
            amber = "#ffab00"
            red = "#ff5252"
            purple = "#b388ff"

            def style_ax(ax, title, xlabel, ylabel):
                ax.set_facecolor(bg_card)
                ax.set_title(title, color=text_color, fontsize=12, fontweight="bold", pad=8)
                ax.set_xlabel(xlabel, color="#9bb0c1", fontsize=10)
                ax.set_ylabel(ylabel, color="#9bb0c1", fontsize=10)
                ax.tick_params(colors="#9bb0c1", labelsize=9)
                for spine in ax.spines.values():
                    spine.set_color("#2a364f")
                ax.grid(True, color="#232d42", linestyle="--", alpha=0.7)

            # ----------------------------------------------------
            # Subplot 1: 2D Spatial Trajectory (Top-down view)
            # ----------------------------------------------------
            ax1 = fig.add_subplot(gs[0, :])
            style_ax(ax1, "2D Arena Trajectory: Drone Evasion vs Dynamic Obstacle", "X Position (m) [Forward]", "Y Position (m) [Lateral]")

            # Plot Start & Goal
            ax1.scatter([sim.start_pos[0]], [sim.start_pos[1]], color=cyan, s=120, zorder=5, label="Start (0,0)")
            ax1.scatter([sim.goal_pos[0]], [sim.goal_pos[1]], color=green, s=140, marker="*", zorder=5, label="Goal (6,0)")

            # Drone flight path
            ax1.plot(drone_positions[:, 0], drone_positions[:, 1], color=cyan, linewidth=2.5, label="Drone Flight Path")

            # Obstacle trajectories
            if len(obs_true_positions) > 0:
                ax1.plot(obs_true_positions[:, 0], obs_true_positions[:, 1], color=amber, linewidth=3.0, linestyle="-", label="Obstacle True Trajectory")
                ax1.scatter(obs_noisy_positions[::3, 0], obs_noisy_positions[::3, 1], color=red, s=15, alpha=0.5, label="Noisy Sensor Detections")
                ax1.plot(obs_kalman_positions[:, 0], obs_kalman_positions[:, 1], color=purple, linewidth=2.0, linestyle="--", label="Kalman Filtered Path")

            # CPA point
            if len(obs_times) > cpa_idx:
                d_cpa = drone_positions[np.argmin(np.abs(times - cpa_time))]
                o_cpa = obs_true_positions[cpa_idx]
                ax1.plot([d_cpa[0], o_cpa[0]], [d_cpa[1], o_cpa[1]], color=red, linestyle=":", linewidth=2.0, label=f"CPA ({min_distance:.2f}m)")

            ax1.set_xlim(-0.5, 6.5)
            ax1.set_ylim(-1.6, 1.6)
            ax1.legend(loc="upper left", facecolor="#101524", edgecolor="#2a364f", labelcolor=text_color, fontsize=8)

            # ----------------------------------------------------
            # Subplot 2: Lateral Position Y(t) - True vs Noisy vs Kalman
            # ----------------------------------------------------
            ax2 = fig.add_subplot(gs[1, 0])
            style_ax(ax2, "Dynamic Obstacle Lateral Motion: True vs Noisy vs Kalman", "Time (seconds)", "Lateral Position Y (m)")

            if len(obs_times) > 0:
                ax2.plot(obs_times, obs_true_positions[:, 1], color=amber, linewidth=2.5, label="Ground Truth Y(t)")
                ax2.scatter(obs_times, obs_noisy_positions[:, 1], color=red, s=12, alpha=0.45, label="Noisy LiDAR Measurement")
                ax2.plot(obs_times, obs_kalman_positions[:, 1], color=cyan, linewidth=2.0, linestyle="--", label="Kalman Estimate $\\hat{Y}(t)$")

                # Covariance envelope 2-sigma
                if len(obs_kalman_stds) > 0:
                    stds = np.array(obs_kalman_stds)[:, 1]
                    upper = obs_kalman_positions[:, 1] + 2.0 * stds
                    lower = obs_kalman_positions[:, 1] - 2.0 * stds
                    ax2.fill_between(obs_times, lower, upper, color=cyan, alpha=0.15, label="$\\pm 2\\sigma$ Confidence Envelope")

                ax2.legend(loc="upper right", facecolor="#101524", edgecolor="#2a364f", labelcolor=text_color, fontsize=8)

            # ----------------------------------------------------
            # Subplot 3: Drone-Obstacle Relative Separation Distance
            # ----------------------------------------------------
            ax3 = fig.add_subplot(gs[1, 1])
            style_ax(ax3, "Separation Distance & Safety Clearance Margin", "Time (seconds)", "Distance to Obstacle (m)")

            if len(obs_times) > 0:
                ax3.plot(obs_times, relative_distances, color=green, linewidth=2.2, label="Relative Separation $d(t)$")
                ax3.axhline(0.75, color=amber, linestyle="--", linewidth=1.5, label="Danger Threshold (0.75m)")
                ax3.axhline(0.30, color=red, linestyle="--", linewidth=1.5, label="Emergency Limit (0.30m)")

                # Highlight minimum distance
                ax3.scatter([cpa_time], [min_distance], color=red, s=100, zorder=5)
                ax3.annotate(
                    f"Min Clearance: {min_distance:.2f}m",
                    xy=(cpa_time, min_distance),
                    xytext=(cpa_time + 0.5, min_distance + 0.35),
                    arrowprops=dict(arrowstyle="->", color=red, lw=1.5),
                    color=text_color,
                    fontsize=9,
                    fontweight="bold"
                )
                ax3.legend(loc="upper right", facecolor="#101524", edgecolor="#2a364f", labelcolor=text_color, fontsize=8)

            # ----------------------------------------------------
            # Subplot 4: Kalman Filter Error vs Sensor Noise Over Time
            # ----------------------------------------------------
            ax4 = fig.add_subplot(gs[2, 0])
            style_ax(ax4, f"Kalman Error Reduction ({reduction_pct:.1f}% Variance Suppression)", "Time (seconds)", "Estimation Error (m)")

            if len(obs_times) > 0:
                ax4.plot(obs_times, err_noisy_list, color=red, alpha=0.6, linewidth=1.2, label=f"Raw Sensor Error (RMSE: {raw_rmse:.3f}m)")
                ax4.plot(obs_times, err_kalman_list, color=green, linewidth=2.0, label=f"Kalman Filter Error (RMSE: {kalman_rmse:.3f}m)")
                ax4.legend(loc="upper right", facecolor="#101524", edgecolor="#2a364f", labelcolor=text_color, fontsize=8)

            # ----------------------------------------------------
            # Subplot 5: Drone Speed & Avoidance State Transitions
            # ----------------------------------------------------
            ax5 = fig.add_subplot(gs[2, 1])
            style_ax(ax5, "Drone Velocity Profile & Avoidance Mode Timeline", "Time (seconds)", "Forward Speed (m/s)")

            ax5.plot(times, drone_speeds, color=purple, linewidth=2.0, label="Drone Speed $\\|v(t)\\|$")
            ax5.axvline(cpa_time, color=amber, linestyle=":", label=f"Obstacle Evasion (t={cpa_time:.1f}s)")
            ax5.legend(loc="upper right", facecolor="#101524", edgecolor="#2a364f", labelcolor=text_color, fontsize=8)

            fig.suptitle("MUJOCO DRONE SIMULATION - DYNAMIC OBSTACLE & KALMAN TRACKING ANALYSIS",
                         color=text_color, fontsize=14, fontweight="bold", y=0.98)

            if save_plot:
                plt.savefig(plot_file, dpi=200, bbox_inches="tight")
                print(f"[INFO] Analysis figure saved successfully to: {plot_file}")

            if show_plot:
                plt.show()

            plt.close()

        except Exception as e:
            print(f"[WARNING] Could not plot figure: {e}")

    return {
        "summary": summary,
        "min_distance": min_distance,
        "cpa_time": cpa_time,
        "raw_rmse": raw_rmse,
        "kalman_rmse": kalman_rmse,
        "reduction_pct": reduction_pct
    }


def main():
    parser = argparse.ArgumentParser(description="Dynamic Obstacle & Drone Kinematic Analysis")
    parser.add_argument("--scenario", "-s", type=int, default=7, choices=[7, 9],
                        help="Dynamic scenario ID (7: Sliding Obstacle, 9: Rapid Lateral Intruder)")
    parser.add_argument("--algo", "-a", type=str, default="reactive",
                        choices=["reactive", "apf", "dwa", "3d", "sindy_dwa"],
                        help="Avoidance algorithm to evaluate")
    parser.add_argument("--duration", "-d", type=float, default=20.0,
                        help="Simulation analysis duration in seconds")
    parser.add_argument("--no-noise", action="store_true",
                        help="Disable obstacle process noise (use purely deterministic motion)")
    parser.add_argument("--obs-noise-std", type=float, default=0.08,
                        help="Obstacle process disturbance standard deviation")
    parser.add_argument("--no-kalman", action="store_true",
                        help="Disable Kalman filter dynamic obstacle tracking")
    parser.add_argument("--sensor-noise", type=float, default=0.06,
                        help="LiDAR measurement noise standard deviation")
    parser.add_argument("--intruder-speed", type=float, default=1.0,
                        help="Dynamic obstacle speed multiplier")
    parser.add_argument("--save-plot", action="store_true", default=True,
                        help="Save high-resolution analysis figure to disk")
    parser.add_argument("--plot-file", type=str, default="dynamic_obstacle_analysis.png",
                        help="Output path for analysis figure")
    parser.add_argument("--show", action="store_true",
                        help="Display interactive plot window")

    args = parser.parse_args()

    run_dynamic_analysis(
        scenario_id=args.scenario,
        algo=args.algo,
        duration=args.duration,
        obstacle_noise=not args.no_noise,
        obs_noise_std=args.obs_noise_std,
        enable_kalman=not args.no_kalman,
        sensor_noise=args.sensor_noise,
        intruder_speed=args.intruder_speed,
        save_plot=args.save_plot,
        plot_file=args.plot_file,
        show_plot=args.show
    )


if __name__ == "__main__":
    main()
