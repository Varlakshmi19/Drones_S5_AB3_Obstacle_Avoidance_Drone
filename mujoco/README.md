# MuJoCo Autonomous Drone Obstacle Avoidance Simulation

[![MuJoCo 3.x](https://img.shields.io/badge/MuJoCo-3.13.0-blue.svg)](https://mujoco.org)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-green.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A high-fidelity port and major evolution of the **Autonomous Obstacle Avoidance Drone** project ([original PyBullet repository](https://github.com/Varlakshmi19/Drones_S5_AB3_Obstacle_Avoidance_Drone)) into the **MuJoCo physics engine**.

Featuring native multi-sector LiDAR raycasting with subtree exclusion, calibrated SE(3) cascaded PID flight control, **4 avoidance algorithms**, **10 benchmark scenarios**, multi-modal ring trap escape (horizontal gap navigation & vertical climb-over), dynamic closing-velocity obstacle evasion, real-time interactive 3D visualizer, automated benchmarking, and a modern web telemetry dashboard.

---

## Key Highlights & Improvements over PyBullet

| Feature | Original (PyBullet) | Port (MuJoCo) |
| :--- | :--- | :--- |
| **Physics Engine** | PyBullet (`gym-pybullet-drones`) | **MuJoCo 3.x** (High precision RK4 integrator, sub-stepped dynamics) |
| **Drone Dynamics** | URDF mesh | **Calibrated MJCF** (`drone.xml`) with exact $4\times4$ motor allocation mixer |
| **Perception** | Virtual raycasting (`p.rayTest`) | **Native `<rangefinder>` sensor array** + multi-directional 3D raycasting with subtree exclusion |
| **Algorithms** | Reactive State Machine | **4 Algorithms**: Reactive FSM (with Ring Escape & Dynamic Evasion), APF, Vectorized DWA, 3D Volumetric |
| **Scenarios** | 6 scenarios (1–6) | **10 scenarios** (1–6 original + Dynamic Slider + 3D Clutter + Lateral Intruder + Dead-End Ring) |
| **Execution Speed** | Real-time PyBullet GUI | **100× Faster-than-realtime** headless batch testing |
| **Visualization** | PyBullet OpenGL GUI | **Native MuJoCo Passive 3D Viewer** + **Interactive Web Dashboard** |

---

## Quick Start & Running Commands

### 1. Interactive 3D Flight Simulation (Native MuJoCo Viewer)
Run any scenario with the 3D interactive viewer:
```bash
# Scenario 1 (Frontal Obstacle) with Reactive Avoidance
python run_sim.py --scenario 1 --algo reactive --gui

# Scenario 5 (Dead-End U-Trap with Emergency Reverse)
python run_sim.py --scenario 5 --algo reactive --gui

# Scenario 9 (Dynamic Lateral Intruder at 1.0x Speed)
python run_sim.py --scenario 9 --algo reactive --intruder-speed 1.0 --gui

# Scenario 10 Variant A (Closed Ring: Vertical Climb-Over)
python run_sim.py --scenario 10 --sub-scenario closed_ring --algo reactive --gui

# Scenario 10 Variant B (Ring with Seeded Random Gap)
python run_sim.py --scenario 10 --sub-scenario ring_with_gap --seed 42 --algo reactive --gui
```

---

### 2. High-Speed Headless Flight & Terminal Telemetry
Run headless simulation with real-time console telemetry:
```bash
# Headless run for Scenario 10 closed_ring
python run_sim.py --scenario 10 --sub-scenario closed_ring --algo reactive

# Headless run for Scenario 10 ring_with_gap with specific seed
python run_sim.py --scenario 10 --sub-scenario ring_with_gap --seed 2023 --algo reactive

# Export flight path telemetry to JSON
python run_sim.py --scenario 10 --sub-scenario closed_ring --algo reactive --log flight_log.json
```

---

### 3. Launch Interactive Web Telemetry Dashboard
Launch the web dashboard on `http://localhost:8080/`:
```bash
python run_sim.py --dashboard
```
*Features:*
- Real-time 2D Flight Arena with live flight trace and color-coded LiDAR rays.
- LiDAR Sector Radar chart.
- Altitude and speed gauges.
- Flight state machine indicator (`FORWARD`, `AVOID`, `RETURN`, `EMERGENCY_REVERSE`, `RING_ESCAPE_SEARCH`, `ESCAPING_THROUGH_GAP`, `VERTICAL_ESCAPE`, `CROSS_OVER`, `DESCEND`, `GOAL_REACHED`).
- Interactive playback slider (1x, 2x, 4x speed).

---

### 4. Automated Benchmark Matrix Suite
Run the automated benchmark suite evaluating all scenarios and algorithms:
```bash
python benchmark.py
```
This automatically evaluates success rates, completion times, path lengths, clearances, and collision steps, updating [`RESULTS.md`](./RESULTS.md) and [`BENCHMARK_RESULTS.json`](./BENCHMARK_RESULTS.json).

---

## Avoidance Algorithms

1. **Reactive Sector State Machine (`reactive`)**
   - Direct port and major evolution of the reactive decision tree.
   - 8 horizontal sectors + 2 vertical sensors: `front (0°)`, `left (+35°)`, `far_left (+70°)`, `side_left (+90°)`, `right (-35°)`, `far_right (-70°)`, `side_right (-90°)`, `rear (180°)`, `up (+Z)`, `down (-Z)`.
   - **Ring Trap Signature Detection (Part A)**: Rolling window trap counter detects unresolvable dead-ends ($\ge 3$ reversals) and executes `RING_ESCAPE_SEARCH`:
     - `closed_ring`: Triggers `VERTICAL_ESCAPE` (climbing at $0.25\text{ m/s}$), transitions to `CROSS_OVER` once altitude clears ring ($z \ge 1.25\text{ m}$), transitions to `DESCEND` past the ring footprint ($dist > 3.1\text{ m}$), and resumes forward flight to the goal.
     - `ring_with_gap`: Performs 360° raycast (`full_sweep`), detects gap bearing without climbing, transitions to `ESCAPING_THROUGH_GAP`, bypasses the ring perimeter, and reaches the goal.
   - **Dynamic Obstacle Prediction (Part B)**: Computes relative closing velocity $\mathbf{v}_{\text{rel}} = \mathbf{v}_{\text{obs}} - \mathbf{v}_{\text{drone}}$ and closing speed $v_{\text{closing}} = -\mathbf{v}_{\text{rel}} \cdot \mathbf{u}_{\text{ray}}$. Dynamically inflates danger on closing sectors and biases avoidance away from the intruder's motion vector.
   - **Isolation from Scenario 5 (Part C)**: Scenario 5 (U-Trap) backs out smoothly in 1 reversal and never triggers ring escape search.

2. **Artificial Potential Field (`apf`)**
   - Continuous vector navigation with quadratic attractive goal field and inverse-distance repulsive obstacle fields.

3. **Dynamic Window Approach (`dwa`)**
   - Vectorized forward trajectory simulation evaluating candidate speeds and heading angles against clearance and goal progress.

4. **3D Volumetric Avoidance (`3d`)**
   - 3D spatial avoidance enabling vertical climbs over low obstacles and underpass maneuvers beneath high overhangs.

5. **SINDy-Enhanced DWA (`sindy_dwa`)**
   - Integrates **Sparse Identification of Nonlinear Dynamics (SINDy)** into DWA trajectory rollout evaluation.
   - Evaluates prospective candidate velocity arcs using learned symbolic equations of motion discovered from telemetry rather than kinematic point-mass assumptions.

---

## SINDy Data-Driven Equation Discovery

The project includes a minimal, dependency-light **SINDy** engine (`mujoco_drone_sim/sindy/`):
- **Translational Dynamics**: Identifies acceleration equations $\dot{\mathbf{v}} = \mathbf{\Theta}(\mathbf{v}, \text{Euler}, T_{\text{tot}}) \mathbf{\Xi}$ with $R^2 \ge 0.94 - 0.99$.
- **Rotational Dynamics**: Identifies angular acceleration equations $\dot{\boldsymbol{\omega}} = \mathbf{\Theta}(\boldsymbol{\omega}, \mathbf{u}_{\text{motors}}) \mathbf{\Xi}$ with $R^2 \ge 0.90 - 0.99$.
- **Moving Obstacle Forecasting**: Identifies online harmonic oscillator equations $\ddot{y} = -\omega^2 y$ for dynamic obstacles (e.g. Scenarios 7 & 9).

Run SINDy training and symbolic equation extraction directly from telemetry:
```bash
python run_sim.py --sindy-train --scenario 1
python run_sim.py --algo sindy_dwa --scenario 1 --gui
```

---

## Benchmark Scenarios

| Scenario ID | Name | Description |
| :--- | :--- | :--- |
| **1** | Direct Frontal | Single obstacle placed directly in the drone's line of sight |
| **2** | Offset Left | Obstacle shifted $+0.20\text{ m}$ left, testing rightward turn bias |
| **3** | Offset Right | Obstacle shifted $-0.20\text{ m}$ right, testing leftward turn bias |
| **4** | Narrow Passage | Twin obstacle corridor forming a tight slit |
| **5** | Dead-End U-Trap | Concave pocket forcing emergency reverse and lateral escape |
| **6** | Forest Maze | Staggered multi-cylinder environment requiring continuous multi-step avoidance |
| **7** | Dynamic Slider | Moving obstacle oscillating across the flight path |
| **8** | 3D Cluttered | Multi-level environment with low walls and high overhangs |
| **9** | Dynamic Intruder | $0.30\text{ m}$ cube sliding laterally between $y=-0.6\text{ m}$ and $+0.6\text{ m}$ across drone flight path |
| **10** | Dead-End Ring | Continuous circular ring surrounding start ($R=2.6\text{ m}$); supports `closed_ring` and `ring_with_gap` |

---

## CLI Options

| Argument | Description | Default |
| :--- | :--- | :--- |
| `--scenario`, `-s` | Scenario ID (`1` to `10`) | `1` |
| `--sub-scenario` | Scenario 10 variant (`closed_ring`, `ring_with_gap`) | `closed_ring` |
| `--seed` | Random seed for Scenario 10 gap orientation | `42` |
| `--intruder-speed` | Speed multiplier for Scenario 9 intruder | `1.0` |
| `--algo`, `-a` | Algorithm (`reactive`, `apf`, `dwa`, `3d`, `sindy_dwa`) | `reactive` |
| `--sindy-train` | Discover symbolic equations of motion from flight data | `False` |
| `--gui`, `-g` | Launch interactive MuJoCo 3D Viewer | `False` |
| `--duration`, `-d` | Max flight duration in seconds | `60.0` |
| `--no-sync` | Run at maximum headless CPU speed | `False` |
| `--benchmark`, `-b` | Run full automated benchmark suite | `False` |
| `--dashboard` | Launch Web Telemetry Dashboard | `False` |
| `--log` | Save flight trajectory to JSON file | `None` |
