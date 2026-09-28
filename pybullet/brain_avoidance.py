"""
Brain-driven obstacle avoidance in gym-pybullet-drones.

Same drone, scenarios, PID controller and metrics as obstacle_avoidance.py,
but the FORWARD/AVOID/RETURN/EMERGENCY_REVERSE state machine is replaced by
the Adaptive Neural Brain (brain/neural_brain.py).

Usage
-----
    python brain_avoidance.py                    # scenario 1, GUI
    python brain_avoidance.py --scenario 5       # pick a scenario
    python brain_avoidance.py --all --headless   # run 1-6, print table
    python brain_avoidance.py --scenario 4 --log # save logs/brain_s4.npz (for SINDy)
"""

import argparse
import json
import os
import subprocess
import sys
import time
from collections import deque

import numpy as np
import pybullet as p

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.utils import sync

from brain import AdaptiveNeuralBrain, BrainConfig, RAY_ANGLES_DEG

# Scenario geometry is shared with the original controller so both are
# evaluated on exactly the same obstacles.
from obstacle_avoidance import (
    SCENARIOS,
    create_test_scenario,
    update_dynamic_obstacles,
    DYNAMIC_OBSTACLES
)


# ============================================================
# SETTINGS (kept identical to obstacle_avoidance.py)
# ============================================================
PYB_FREQ = 240
CONTROL_FREQ = 48
DT = 1.0 / CONTROL_FREQ
DURATION_SEC = 60

FLIGHT_ALTITUDE = 1.0
GOAL_POSITION = np.array([6.0, 0.0, FLIGHT_ALTITUDE])
GOAL_TOLERANCE = 0.25

SENSOR_RANGE = 1.6
RAY_START_OFFSET = 0.10       # start rays outside the drone body

LOOKAHEAD = 0.65              # same "point slightly ahead" rule as original

LOG_DIR = "logs"


def start_position(scenario):
    return np.array([0.0, 0.0, FLIGHT_ALTITUDE])


# ============================================================
# SENSING
# ============================================================
def cast_rays(position, world_angles, draw=False):
    """Horizontal ray fan. Returns distance per angle (from drone centre)."""
    dirs = np.stack([np.cos(world_angles), np.sin(world_angles),
                     np.zeros_like(world_angles)], axis=1)
    starts = position[None, :] + dirs * RAY_START_OFFSET
    ends = starts + dirs * SENSOR_RANGE
    results = p.rayTestBatch(starts.tolist(), ends.tolist())

    distances = np.empty(len(world_angles))
    for i, r in enumerate(results):
        hit_id, frac = r[0], r[2]
        d = SENSOR_RANGE if hit_id == -1 else frac * SENSOR_RANGE
        distances[i] = min(d + RAY_START_OFFSET, SENSOR_RANGE)
        if draw:
            colour = [1, 0, 0] if d < 0.6 else [0, 0.8, 0]
            p.addUserDebugLine(starts[i], starts[i] + dirs[i] * d,
                               colour, lineWidth=2, lifeTime=0.08)
    return distances


def draw_goal():
    p.addUserDebugText("GOAL", GOAL_POSITION + np.array([0, 0, 0.3]),
                       [0, 0, 1], textSize=1.4)
    for d in (np.array([0.2, 0, 0]), np.array([0, 0.2, 0])):
        p.addUserDebugLine(GOAL_POSITION - d, GOAL_POSITION + d, [0, 0, 1], lineWidth=4)


# ============================================================
# MAIN LOOP
# ============================================================
def run(scenario, gui=True, log=False, duration=DURATION_SEC, config=None, quiet=False):

    start = start_position(scenario)

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=np.array([start]),
        initial_rpys=np.array([[0.0, 0.0, 0.0]]),
        physics=Physics.PYB,
        neighbourhood_radius=10,
        pyb_freq=PYB_FREQ,
        ctrl_freq=CONTROL_FREQ,
        gui=gui,
        record=False,
        obstacles=False,
        user_debug_gui=False,
    )
    controller = DSLPIDControl(drone_model=DroneModel.CF2X)
    obstacle_ids = create_test_scenario(scenario)
    if gui:
        draw_goal()

    brain = AdaptiveNeuralBrain(GOAL_POSITION[:2], start[:2], DT, config or BrainConfig())

    action = np.zeros((1, 4))
    path_length = 0.0
    collision_steps = 0
    avoid_events = 0
    back_off_events = 0
    prev_mode = "GOAL_SEEK"
    prev_pos = None
    min_clearance = np.inf
    result = "TIME LIMIT REACHED"
    records = []

    wall_start = time.time()
    n_steps = int(duration * CONTROL_FREQ)

    for step in range(n_steps):

        # ---------------- dynamic obstacles ----------------
        update_dynamic_obstacles(step * DT)

        obs, _, terminated, truncated, _ = env.step(action)
        state = obs[0]
        pos, quat = state[0:3], state[3:7]
        rpy, vel, ang_vel = state[7:10], state[10:13], state[13:16]
        drone_id = env.DRONE_IDS[0]

        # ---------------- metrics ----------------
        if any(p.getContactPoints(bodyA=drone_id, bodyB=o) for o in obstacle_ids):
            collision_steps += 1
        if prev_pos is not None:
            path_length += float(np.linalg.norm(pos - prev_pos))
        prev_pos = pos.copy()

        # ---------------- brain ------------------
        world_angles = brain.ray_world_angles()
        distances = cast_rays(pos, world_angles, draw=gui)
        min_clearance = min(min_clearance, float(distances[:-1].min()))

        v_xy, done = brain.step(pos[:2], distances)
        mode = brain.last.get("mode", "GOAL")

        if mode == "AVOID" and prev_mode != "AVOID":
            avoid_events += 1
        if mode == "BACK_OFF" and prev_mode != "BACK_OFF":
            back_off_events += 1
        prev_mode = mode

        desired_vel = np.array([v_xy[0], v_xy[1], 0.0])
        if done:
            target_pos = GOAL_POSITION.copy()
        else:
            target_pos = pos + desired_vel * LOOKAHEAD
            target_alt = FLIGHT_ALTITUDE
            if scenario == 8:
                if 1.7 <= pos[0] <= 2.8:
                    target_alt = 1.35
                elif 3.0 <= pos[0] <= 3.9:
                    target_alt = 0.95
                elif 4.1 <= pos[0] <= 4.9:
                    target_alt = 1.35
            elif scenario == 10:
                if np.linalg.norm(pos[:2]) < 2.5:
                    target_alt = 1.50
            target_pos[2] = target_alt

        rpm, _, _ = controller.computeControl(
            control_timestep=DT,
            cur_pos=pos, cur_quat=quat, cur_vel=vel, cur_ang_vel=ang_vel,
            target_pos=target_pos, target_rpy=np.zeros(3), target_vel=desired_vel,
        )
        action[0, :] = rpm

        if log:
            b = brain.last
            records.append(np.concatenate([
                [step * DT], pos, quat, rpy, vel, ang_vel, rpm,
                desired_vel, target_pos,
                [b.get("I_L", 0), b.get("I_R", 0), b.get("o_L", 0), b.get("o_R", 0),
                 b.get("w_LL", 0), b.get("w_RR", 0), b.get("frustration", 0),
                 b.get("psi", 0), b.get("omega", 0), b.get("speed", 0)],
                distances,
            ]))

        # ---------------- display ----------------
        if gui:
            txt = f"{mode}  F={brain.frustration:.2f}  wL={brain.w_self[0]:.2f} wR={brain.w_self[1]:.2f}"
            p.addUserDebugText(txt, pos + np.array([0, 0, 0.32]), [0, 0, 0],
                               textSize=1.0, lifeTime=0.08)
            env.render()
            sync(step, wall_start, DT)

        if not quiet and step % 24 == 0:
            b = brain.last
            print(f"S{scenario} t={step*DT:5.1f}s | {mode:9s} | pos=({pos[0]:.2f},{pos[1]:.2f}) "
                  f"| front={b.get('d_front', 0):.2f} | IL={b.get('I_L', 0):.2f} IR={b.get('I_R', 0):.2f} "
                  f"| F={b.get('frustration', 0):.2f} | goal={b.get('goal_dist', 0):.2f}")

        if done:
            result = "SUCCESS"
            break
        if terminated or truncated:
            result = "SIMULATION TERMINATED"
            break

    sim_time = (step + 1) * DT
    env.close()

    summary = dict(
        scenario=scenario, result=result, control_steps=step + 1,
        sim_time_s=round(sim_time, 2),
        final_position=[round(float(v), 3) for v in pos],
        path_length_m=round(path_length, 3),
        collision_steps=collision_steps,
        avoidance_events=avoid_events,
        back_off_events=back_off_events,
        min_ray_clearance_m=round(min_clearance, 3),
    )

    if log:
        os.makedirs(LOG_DIR, exist_ok=True)
        cols = (["t", "x", "y", "z", "qx", "qy", "qz", "qw", "roll", "pitch", "yaw",
                 "vx", "vy", "vz", "wx", "wy", "wz", "rpm0", "rpm1", "rpm2", "rpm3",
                 "vcx", "vcy", "vcz", "tx", "ty", "tz",
                 "I_L", "I_R", "o_L", "o_R", "w_LL", "w_RR", "F", "psi", "omega", "speed"]
                + [f"ray_{int(a)}" for a in RAY_ANGLES_DEG])
        path = os.path.join(LOG_DIR, f"brain_s{scenario}.npz")
        np.savez(path, data=np.array(records), columns=np.array(cols),
                 dt=DT, controller="brain", scenario=scenario,
                 summary=json.dumps(summary))
        summary["log"] = path

    return summary


def print_summary(s):
    print("\n========================================")
    for k, v in s.items():
        print(f"{k.upper():22s}: {v}")
    print("========================================")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", type=int, default=1, choices=range(1, 11),
                    help="scenario number (1-10)")
    ap.add_argument("--all", action="store_true", help="run scenarios 1-10")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--log", action="store_true", help="save npz log for SINDy")
    ap.add_argument("--duration", type=float, default=DURATION_SEC)
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--json", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.all:
        # one subprocess per scenario -> clean PyBullet client every time
        rows = []
        for sc in range(1, 11):
            cmd = [sys.executable, __file__, "--scenario", str(sc), "--quiet", "--json",
                   "--duration", str(args.duration)]
            if args.headless:
                cmd.append("--headless")
            if args.log:
                cmd.append("--log")
            out = subprocess.run(cmd, capture_output=True, text=True)
            line = [l for l in out.stdout.splitlines() if l.startswith("{")]
            if not line:
                print(out.stdout[-2000:], out.stderr[-2000:])
                continue
            rows.append(json.loads(line[-1]))
        print("\n| Scenario | Name | Result | Steps | Time (s) | Path (m) | Collision steps | Avoid events | Back-offs | Min clearance (m) |")
        print("|:-:|:---|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|")
        for r in rows:
            sc_name = SCENARIOS.get(r['scenario'], {}).get('name', f"Scenario {r['scenario']}")
            print(f"| {r['scenario']} | {sc_name:24s} | {r['result']} | {r['control_steps']} | {r['sim_time_s']} | "
                  f"{r['path_length_m']} | {r['collision_steps']} | {r['avoidance_events']} | "
                  f"{r['back_off_events']} | {r['min_ray_clearance_m']} |")
        return

    info = SCENARIOS.get(args.scenario, {})
    if not args.quiet and not args.json:
        print("\n" + "=" * 60)
        print(f"BRAIN NAVIGATION - SCENARIO {args.scenario}: {info.get('name', '')}")
        print(f"Difficulty: {info.get('difficulty', '')}")
        print(f"Description: {info.get('description', '')}")
        print(f"Expected: {info.get('expected_behavior', '')}")
        print("=" * 60 + "\n")

    s = run(args.scenario, gui=not args.headless, log=args.log,
            duration=args.duration, quiet=args.quiet)
    if args.json:
        print(json.dumps(s))
    else:
        print_summary(s)


if __name__ == "__main__":
    main()
