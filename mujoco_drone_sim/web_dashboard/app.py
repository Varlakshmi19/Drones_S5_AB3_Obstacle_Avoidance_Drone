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
try:
    from http.server import ThreadingHTTPServer as HTTPServerClass, SimpleHTTPRequestHandler
except ImportError:
    from http.server import HTTPServer as HTTPServerClass, SimpleHTTPRequestHandler
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
            obs_noise = query.get("obs_noise", ["1"])[0] in ["1", "true", "True"]
            enable_kalman = query.get("enable_kalman", ["1"])[0] in ["1", "true", "True"]

            result = self.run_simulation_trace(
                scenario_id, algo,
                sub_scenario=sub_scenario,
                seed=seed,
                intruder_speed=intruder_speed,
                obstacle_noise=obs_noise,
                enable_kalman=enable_kalman
            )
            self.send_json(result)

        elif path == "/api/benchmark":
            benchmark_file = os.path.join(workspace_dir, "BENCHMARK_RESULTS.json")
            if os.path.exists(benchmark_file):
                with open(benchmark_file, "r") as f:
                    data = json.load(f)
                self.send_json({"status": "success", "results": data})
            else:
                self.send_json({"status": "error", "message": "Benchmark not yet generated."})

        elif path == "/api/load_custom":
            custom_file = os.path.join(workspace_dir, "custom_arena.json")
            if os.path.exists(custom_file):
                with open(custom_file, "r") as f:
                    data = json.load(f)
                self.send_json({"status": "success", "data": data})
            else:
                self.send_json({"status": "error", "message": "No custom_arena.json file found."})

        else:
            super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/api/run_custom":
            content_length = int(self.headers.get("Content-Length", 0))
            post_body = self.rfile.read(content_length)
            payload = json.loads(post_body.decode("utf-8"))
            custom_obs = payload.get("obstacles", [])
            algo = payload.get("algo", "reactive")
            obs_noise = payload.get("obstacle_noise", True)
            enable_kalman = payload.get("enable_kalman", True)

            result = self.run_simulation_trace(
                scenario_id=1,
                algo=algo,
                custom_obstacles=custom_obs,
                obstacle_noise=obs_noise,
                enable_kalman=enable_kalman
            )
            self.send_json(result)

        elif path == "/api/save_custom":
            content_length = int(self.headers.get("Content-Length", 0))
            post_body = self.rfile.read(content_length)
            with open(os.path.join(workspace_dir, "custom_arena.json"), "wb") as f:
                f.write(post_body)
            self.send_json({"status": "success", "message": "Custom arena layout saved to custom_arena.json"})

        else:
            self.send_response(404)
            self.end_headers()

    def get_scenario_desc(self, sc_id):
        return get_scenario_info(sc_id).get("description", "")

    def run_simulation_trace(
        self,
        scenario_id,
        algo,
        sub_scenario="closed_ring",
        seed=42,
        intruder_speed=1.0,
        custom_obstacles=None,
        obstacle_noise=True,
        enable_kalman=True
    ):
        try:
            sim = DroneObstacleSimulation(
                scenario_id=scenario_id,
                algorithm=algo,
                sub_scenario=sub_scenario,
                seed=seed,
                intruder_speed_multiplier=intruder_speed,
                custom_obstacles=custom_obstacles,
                obstacle_noise=obstacle_noise,
                enable_kalman=enable_kalman
            )
            frames = []

            # Extract obstacle visual geometries for 2D arena rendering
            obstacles_info = []
            if custom_obstacles:
                obstacles_info = custom_obstacles
            else:
                for i in range(sim.model.nbody):
                    b_name = sim.model.body(i).name
                    if b_name.startswith("obs"):
                        pos = sim.model.body_pos[i].tolist()
                        is_dyn = any(j_name.startswith(b_name) for _, j_name in sim.dynamic_joints)
                        obstacles_info.append({
                            "name": b_name,
                            "pos": pos,
                            "type": "cylinder" if "cylinder" in b_name else "box",
                            "is_dynamic": is_dyn
                        })

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

                    # Dynamic obstacles live positions
                    dyn_obs_frame = []
                    for j_id, j_name in sim.dynamic_joints:
                        q_pos = float(sim.data.qpos[sim.model.jnt_qposadr[j_id]])
                        b_id = sim.model.jnt_bodyid[j_id]
                        b_pos = sim.data.xpos[b_id].tolist()
                        dyn_obs_frame.append({
                            "name": j_name,
                            "body_name": sim.model.body(b_id).name,
                            "pos": [round(x, 3) for x in b_pos],
                            "slide_pos": round(q_pos, 3)
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
                        "is_collision": info["is_collision"],
                        "dyn_obstacles": dyn_obs_frame,
                        "kalman_tracks": info.get("kalman_tracks", {})
                    })

                if info["result"] != "RUNNING":
                    break

            summary = sim.get_summary()
            summary["scenario_name"] = "Custom Arena Layout" if custom_obstacles else SCENARIO_NAMES.get(scenario_id, f"Scenario {scenario_id}")

            return {
                "status": "success",
                "summary": summary,
                "goal_pos": sim.goal_pos.tolist(),
                "start_pos": sim.start_pos.tolist(),
                "obstacles": obstacles_info,
                "kalman_telemetry": sim.kalman_telemetry[::2] if hasattr(sim, "kalman_telemetry") else [],
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
    httpd = HTTPServerClass(server_address, DashboardHandler)
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
