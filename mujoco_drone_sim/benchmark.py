"""
Comprehensive Benchmark Evaluation Suite for MuJoCo Drone Obstacle Avoidance.
Evaluates 4 algorithms across 8 benchmark scenarios and exports comparative metrics.
"""

import sys
import os
import time
import json
import numpy as np

# Ensure workspace root is in python path
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace_dir = os.path.abspath(os.path.join(current_dir, ".."))
if workspace_dir not in sys.path:
    sys.path.insert(0, workspace_dir)

from mujoco_drone_sim.simulation.drone_sim import DroneObstacleSimulation

SCENARIO_NAMES = {
    1: "Frontal Obstacle",
    2: "Offset Left",
    3: "Offset Right",
    4: "Narrow Corridor",
    5: "Dead-End U-Trap",
    6: "Forest Maze",
    7: "Dynamic Slider",
    8: "3D Cluttered",
    9: "Dynamic Intruder",
    10: "Dead-End Ring"
}


def run_benchmark_suite(export_md=True, output_file="BENCHMARK_RESULTS.json"):
    """
    Run full matrix benchmark across all scenarios and algorithms.
    """
    algorithms = ["reactive", "sindy_dwa", "apf", "dwa", "3d"]
    scenarios = list(range(1, 11))

    results = []

    print("\n" + "=" * 80)
    print("  STARTING AUTOMATED MUJOCO DRONE OBSTACLE AVOIDANCE BENCHMARK")
    print(f"  Evaluating {len(algorithms)} Algorithms across Benchmark Scenarios")
    print("=" * 80 + "\n")

    t_suite_start = time.time()

    for algo in algorithms:
        print(f"\n--- ALGORITHM: {algo.upper()} ---")
        # Run scenarios 1 to 10 for reactive and sindy_dwa, and 1 to 8 for others
        algo_scenarios = scenarios if algo in ["reactive", "sindy_dwa"] else list(range(1, 9))
        for sc in algo_scenarios:
            t0 = time.time()
            try:
                sub_sc = "closed_ring" if sc == 10 else "default"
                sim = DroneObstacleSimulation(scenario_id=sc, sub_scenario=sub_sc, algorithm=algo)
                for _ in range(60 * 48):
                    info = sim.step()
                    if info["result"] != "RUNNING":
                        break
                summary = sim.get_summary()
                summary["scenario_name"] = SCENARIO_NAMES[sc]
                summary["calc_time"] = round(time.time() - t0, 3)
                results.append(summary)

                status_mark = "OK" if summary["result"] == "SUCCESS" else "WARN"
                print(
                    f"[{status_mark}] Scenario {sc:2d} ({SCENARIO_NAMES[sc]:16s}) | "
                    f"Result: {summary['result']:15s} | "
                    f"Time: {summary['completion_time']:5.2f}s | "
                    f"Collisions: {summary['collision_steps']:2d} | "
                    f"Path: {summary['path_length']:5.2f}m | "
                    f"Clearance: {summary['min_clearance']:4.2f}m",
                    flush=True
                )
            except Exception as e:
                print(f"[ERR] Scenario {sc} ({SCENARIO_NAMES[sc]}): {e}", flush=True)

    # Also evaluate Scenario 10 ring_with_gap for reactive
    print("\n--- EXTENDED EVALUATION: SCENARIO 10 (RING WITH GAP) ---")
    for seed in [42, 100, 2023, 777, 999]:
        t0 = time.time()
        sim = DroneObstacleSimulation(scenario_id=10, sub_scenario="ring_with_gap", seed=seed, algorithm="reactive")
        for _ in range(60 * 48):
            info = sim.step()
            if info["result"] != "RUNNING":
                break
        summary = sim.get_summary()
        summary["scenario_name"] = f"Ring Gap (seed {seed})"
        summary["calc_time"] = round(time.time() - t0, 3)
        results.append(summary)
        status_mark = "OK" if summary["result"] == "SUCCESS" else "WARN"
        print(
            f"[{status_mark}] Seed {seed:4d} ({summary['scenario_name']:20s}) | "
            f"Result: {summary['result']:15s} | "
            f"Time: {summary['completion_time']:5.2f}s | "
            f"Collisions: {summary['collision_steps']:2d} | "
            f"Method: {summary.get('escape_method', 'none'):15s} | "
            f"Path: {summary['path_length']:5.2f}m",
            flush=True
        )

    total_duration = time.time() - t_suite_start
    print("\n" + "=" * 80)
    print(f"  BENCHMARK COMPLETE in {total_duration:.2f}s")
    print("=" * 80)

    # Print Formatted Markdown Table
    md_table = generate_markdown_report(results)
    print("\n" + md_table)

    # Save to JSON
    with open(output_file, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n[INFO] Complete JSON benchmark results saved to {output_file}")

    if export_md:
        with open("RESULTS.md", "w") as f:
            f.write(md_table)
        print("[INFO] Markdown report updated in RESULTS.md")

    return results


def generate_markdown_report(results):
    """Generate structured markdown summary table and performance analysis."""
    lines = [
        "# MuJoCo Drone Obstacle Avoidance: Benchmark Performance Evaluation",
        "",
        "Evaluation across 10 benchmark scenarios comparing obstacle avoidance strategies:",
        "- **Reactive**: Finite State Machine with 6-sector virtual LiDAR rangefinding, 360° gap escape & vertical climb-over state machine, and closing-velocity dynamic prediction.",
        "- **APF**: Artificial Potential Fields (gradient-based repulsive obstacle & attractive goal fields).",
        "- **DWA**: Vectorized Dynamic Window Approach with forward trajectory arc sampling.",
        "- **3D Avoidance**: 3D Volumetric avoidance with dynamic altitude climbing and overhang descent.",
        "",
        "## Detailed Results Table",
        "",
        "| Scenario | Scenario Name | Algorithm | Result | Flight Time (s) | Path Length (m) | Collisions | Min Clear (m) |",
        "| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |"
    ]

    for r in results:
        res_tag = "**SUCCESS**" if r["result"] == "SUCCESS" else r["result"]
        lines.append(
            f"| {r['scenario_id']} | {r['scenario_name']} | {r['algorithm'].upper()} | {res_tag} | "
            f"{r['completion_time']} | {r['path_length']} | {r['collision_steps']} | {r['min_clearance']} |"
        )

    # Summary Metrics per algorithm
    lines.extend([
        "",
        "## Performance Metrics by Algorithm",
        "",
        "| Algorithm | Success Rate | Mean Flight Time (s) | Mean Path Length (m) | Total Collision Steps |",
        "| :--- | :--- | :--- | :--- | :--- |"
    ])

    algos = ["reactive", "sindy_dwa", "apf", "dwa", "3d"]
    for a in algos:
        a_runs = [r for r in results if r["algorithm"] == a]
        if not a_runs:
            continue
        success_count = sum(1 for r in a_runs if r["result"] == "SUCCESS")
        success_rate = (success_count / len(a_runs)) * 100.0
        success_runs = [r for r in a_runs if r["result"] == "SUCCESS"]

        mean_time = np.mean([r["completion_time"] for r in success_runs]) if success_runs else 0.0
        mean_path = np.mean([r["path_length"] for r in success_runs]) if success_runs else 0.0
        total_cols = sum(r["collision_steps"] for r in a_runs)

        lines.append(
            f"| **{a.upper()}** | {success_rate:.1f}% ({success_count}/{len(a_runs)}) | {mean_time:.2f} | {mean_path:.2f} | {total_cols} |"
        )

    lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    run_benchmark_suite()
