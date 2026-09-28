"""
Interactive Web Telemetry Dashboard for MuJoCo Quadcopter Obstacle Avoidance Simulation.
Pure Python standard library HTTP server - Zero external web framework dependencies required.
"""

import sys
import os
import json
import threading
import time
import webbrowser
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import numpy as np

# Ensure workspace root is in python path
current_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.abspath(os.path.join(current_dir, ".."))
workspace_dir = os.path.abspath(os.path.join(current_dir, "../.."))
for p in [workspace_dir, pkg_dir, current_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

from mujoco_drone_sim.simulation.drone_sim import DroneObstacleSimulation
from mujoco_drone_sim.simulation.scenarios import SCENARIOS, get_scenario_info
from mujoco_drone_sim.benchmark import SCENARIO_NAMES

HOST = "127.0.0.1"
PORT = 8080


class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        static_dir = os.path.join(current_dir, "static")
        super().__init__(*args, directory=static_dir, **kwargs)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)

        if path == "/api/info":
            self.send_json({
                "scenarios": [
                    {"id": sc_id, "name": sc_data["name"], "desc": sc_data["description"], "difficulty": sc_data["difficulty"]}
                    for sc_id, sc_data in SCENARIOS.items()
                ],
                "algorithms": [
                    {"id": "reactive", "name": "Reactive Sector State Machine", "desc": "LiDAR rangefinding with hysteresis side switching, dynamic closing-velocity evasion, and 360° ring escape."},
                    {"id": "apf", "name": "Artificial Potential Field (APF)", "desc": "Continuous gradient navigation with attractive goal fields and repulsive obstacle fields."},
                    {"id": "dwa", "name": "Dynamic Window Approach (DWA)", "desc": "Vectorized forward trajectory simulation evaluating heading, clearance, and velocity."},
                    {"id": "3d", "name": "3D Volumetric Avoidance", "desc": "Full 3D spatial avoidance allowing vertical climbs over low obstacles and underpass maneuvers."},
                    {"id": "sindy_dwa", "name": "SINDy-Enhanced DWA", "desc": "Dynamic Window Approach with forward trajectory rollouts integrated with data-driven discovered nonlinear equations of motion."}
                ]
            })

        elif path == "/api/run":
            scenario_id = int(query.get("scenario", [1])[0])
            algo = query.get("algo", ["reactive"])[0]
            sub_scenario = query.get("sub_scenario", ["closed_ring"])[0]
            seed = int(query.get("seed", [42])[0])
            intruder_speed = float(query.get("intruder_speed", [1.0])[0])

            result = self.run_simulation_trace(scenario_id, algo, sub_scenario=sub_scenario, seed=seed, intruder_speed=intruder_speed)
            self.send_json(result)

        elif path == "/api/benchmark":
            benchmark_file = os.path.join(workspace_dir, "BENCHMARK_RESULTS.json")
            if os.path.exists(benchmark_file):
                with open(benchmark_file, "r") as f:
                    data = json.load(f)
                self.send_json({"status": "success", "results": data})
            else:
                self.send_json({"status": "error", "message": "Benchmark not yet generated."})

        else:
            super().do_GET()

    def get_scenario_desc(self, sc_id):
        return get_scenario_info(sc_id).get("description", "")

    def run_simulation_trace(self, scenario_id, algo, sub_scenario="closed_ring", seed=42, intruder_speed=1.0):
        try:
            sim = DroneObstacleSimulation(
                scenario_id=scenario_id,
                algorithm=algo,
                sub_scenario=sub_scenario,
                seed=seed,
                intruder_speed_multiplier=intruder_speed
            )
            frames = []

            # Extract obstacle visual geometries for 2D arena rendering
            obstacles_info = []
            for i in range(sim.model.nbody):
                b_name = sim.model.body(i).name
                if b_name.startswith("obs"):
                    pos = sim.model.body_pos[i].tolist()
                    obstacles_info.append({"name": b_name, "pos": pos})

            max_steps = 60 * 48
            for step in range(max_steps):
                info = sim.step()

                # Sample frames at 12 Hz for smooth browser playback
                if step % 4 == 0 or info["result"] != "RUNNING":
                    pos = [round(float(x), 3) for x in info["pos"]]
                    vel = [round(float(x), 3) for x in info["vel"]]
                    
                    # LiDAR rays (clean start/end)
                    rays = []
                    for r in info["perception"].get("rays", []):
                        rays.append({
                            "name": r["name"],
                            "start": [round(float(x), 3) for x in r["start"][:2]],
                            "end": [round(float(x), 3) for x in r["end"][:2]],
                            "dist": round(float(r["distance"]), 3),
                            "color": r["color"]
                        })

                    frames.append({
                        "t": round(info["sim_time"], 2),
                        "pos": pos,
                        "vel": vel,
                        "speed": round(float(np.linalg.norm(vel)), 2),
                        "altitude": pos[2],
                        "mode": info["mode"],
                        "status_text": info["status_text"],
                        "min_dist": round(float(info["min_dist"]), 2),
                        "dist_to_goal": round(float(info["dist_to_goal"]), 2),
                        "rays": rays,
                        "is_collision": info["is_collision"]
                    })

                if info["result"] != "RUNNING":
                    break

            summary = sim.get_summary()
            summary["scenario_name"] = SCENARIO_NAMES.get(scenario_id, f"Scenario {scenario_id}")

            return {
                "status": "success",
                "summary": summary,
                "goal_pos": sim.goal_pos.tolist(),
                "start_pos": sim.start_pos.tolist(),
                "obstacles": obstacles_info,
                "frames": frames
            }

        except Exception as e:
            return {"status": "error", "message": str(e)}

    def send_json(self, data):
        body = json.dumps(data).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)


def start_dashboard(host=HOST, port=PORT, auto_open=True):
    """Start web dashboard server and open in default web browser."""
    server_address = (host, port)
    httpd = HTTPServer(server_address, DashboardHandler)
    url = f"http://{host}:{port}/"
    print("\n" + "=" * 70)
    print(f"  MuJoCo Quadcopter Telemetry Dashboard Running!")
    print(f"  URL: {url}")
    print(f"  Press Ctrl+C to stop the dashboard server.")
    print("=" * 70 + "\n")

    if auto_open:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] Dashboard server stopped.")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    start_dashboard()
