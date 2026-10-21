# Autonomous Quadcopter Obstacle Avoidance Simulation in MuJoCo
### High-Fidelity Physics, 3D LiDAR Perception, Kalman Filter Tracking, and Non-Linear Avoidance

---

## Table of Contents
1. [Project Overview & Objectives](#1-project-overview--objectives)
2. [End-to-End System Architecture](#2-end-to-end-system-architecture)
3. [Physical Platform & Simulation Engine](#3-physical-platform--simulation-engine)
4. [Perception & Sensor Array](#4-perception--sensor-array)
5. [Kalman Filter Tracking & Trajectory Prediction](#5-kalman-filter-tracking--trajectory-prediction)
6. [Avoidance Algorithms & Decision Logic](#6-avoidance-algorithms--decision-logic)
7. [Benchmark Flight Scenarios (1 to 10)](#7-benchmark-flight-scenarios-1-to-10)
8. [Interactive Web Dashboard & Customizer](#8-interactive-web-dashboard--customizer)
9. [Command-Line Guide & Execution Options](#9-command-line-guide--execution-options)
10. [Mathematical & Algorithmic Formulations](#10-mathematical--algorithmic-formulations)
11. [Project Codebase Structure](#11-project-codebase-structure)

---

## 1. Project Overview & Objectives

This project implements an autonomous quadcopter obstacle avoidance system simulated inside the **MuJoCo physics engine**. It bridges the gap between theoretical path planning and realistic physical dynamics by simulating true rigid-body multi-rotor aerodynamics, motor delays, sensor measurement noise, and stochastic moving obstacles.

### Key Capabilities:
* **Physics Fidelity**: Native MuJoCo 6-DOF quadcopter model (Crazyflie 2.x scale, $m = 47.2\text{ g}$) simulated with Runge-Kutta 4th-order (RK4) integration at $500\text{ Hz}$ physics and $48\text{ Hz}$ control frequency.
* **Kalman Filter Tracking**: 6-DOF Constant-Velocity Kalman Filter tracks moving obstacles with noisy LiDAR range measurements, achieving **~50% noise variance reduction** and projecting future obstacle trajectories 1.0 second into the future.
* **Stochastic Dynamic Obstacles**: Moving obstacles do not follow predictable deterministic paths; they are driven by an **Ornstein-Uhlenbeck stochastic drift process** with random phase, frequency, and wandering drift.
* **6 Navigation Strategies**:
  1. **Reactive Sector State Machine** (with closing-velocity evasion and hysteresis side-switching)
* **Dead-End & Trap Solving**: Autonomous 360° LiDAR perimeter inspection that detects concave U-traps and circular dead-ends, initiating 3D vertical climb-overs or finding horizontal escape gaps.
* **Interactive Tooling**: Real-time Web Dashboard (`http://127.0.0.1:8080`) with mouse/keyboard obstacle placement, live LiDAR radar, Kalman trajectory curves, and desktop Tkinter visual arena editor.

---

## 2. End-to-End System Architecture

The software architecture follows a modular sense-estimate-plan-control feedback loop:

```
                  +----------------------------------------------+
                  |         MuJoCo 3.13 Physics World            |
                  |  - RK4 Numerical Integrator (dt = 0.002s)    |
                  |  - Dynamic Obstacles with OU Drift Process   |
                  +----------------------+-----------------------+
                                         |
                       [Rigid-body State & Sensor Rays]
                                         v
+--------------------------------------------------------------------------------+
|                           PERCEPTION SUBSYSTEM                                 |
|  * 8-Beam Horizontal LiDAR Array (Front, Left/Right 35°, 70°, 90°, Rear 180°)   |
|  * 2-Beam Vertical Array (Up +Z, Down -Z)                                      |
|  * Gaussian Sensor Noise Injection (sigma = 0.06m)                            |
+----------------------------------------+---------------------------------------+
                                         |
                        [Noisy Positional Hits & Range]
                                         v
+--------------------------------------------------------------------------------+
|                     KALMAN FILTER & TRACKING SUBSYSTEM                         |
|  * Multi-Obstacle 6-DOF State Estimator [x, y, z, vx, vy, vz]                  |
|  * Joseph-Form Stabilized Covariance Propagation                               |
|  * Closing-Velocity Calculation: v_rel = v_obs - v_drone                       |
|  * 1.0-Second Forward Trajectory Prediction Horizon                            |
+----------------------------------------+---------------------------------------+
                                         |
               [Filtered State, Projected Trajectory, Dynamic Sectors]
                                         v
+--------------------------------------------------------------------------------+
|                        AVOIDANCE & PLANNING SUBSYSTEM                          |
|  * Reactive State Machine / APF / DWA / SINDy-DWA / 3D Volumetric              |
|  * Dynamic Sector Inflation based on Kalman closing-velocity                   |
|  * Dead-End Reversal Monitor (3 reversals in 12s triggers 360° Scan)          |
|  * Output: Desired Position & Velocity Setpoint [x_des, y_des, z_des]          |
+----------------------------------------+---------------------------------------+
                                         |
                               [Desired Setpoint]
                                         v
+--------------------------------------------------------------------------------+
|                           CONTROL SUBSYSTEM (SE3)                              |
|  * Calibrated DSL-PID Attitude Controller                                      |
|  * Position PID -> Desired Acceleration Vector                                 |
|  * Orientation Extraction -> Desired Quaternion & Angular Velocities           |
|  * Motor Mixer -> 4 Rotor PWM Control Commands [u1, u2, u3, u4]                |
+----------------------------------------+---------------------------------------+
                                         |
                            [Actuator Forces & Torques]
                                         v
                         (Back to MuJoCo Physics World)
```

---

## 3. Physical Platform & Simulation Engine

### Drone Physical Parameters (Crazyflie 2.x Scale)
The quadcopter is modeled in `models/drone.xml` with high physical precision:
* **Total Mass**: $0.0472\text{ kg}$ ($47.2\text{ g}$)
* **Arm Length**: $0.040\text{ m}$ ($4.0\text{ cm}$)
* **Inertia Matrix**:
  $$I_{xx} = 1.4 \times 10^{-5}\text{ kg}\cdot\text{m}^2, \quad I_{yy} = 1.4 \times 10^{-5}\text{ kg}\cdot\text{m}^2, \quad I_{zz} = 2.17 \times 10^{-5}\text{ kg}\cdot\text{m}^2$$
* **Rotor Count**: 4 motors in an "X" configuration:
  * Motor 1: Front-Left (Counter-Clockwise)
  * Motor 2: Front-Right (Clockwise)
  * Motor 3: Rear-Left (Clockwise)
  * Motor 4: Rear-Right (Counter-Clockwise)
* **Maximum Thrust**: $0.155\text{ N}$ per rotor ($0.62\text{ N}$ total thrust $\approx 1.34\times$ thrust-to-weight ratio).
* **Torque-to-Thrust Ratio**: $k_m = 0.00596\text{ m}$.

### Physics Time-Stepping:
* **Physics Integration Step**: $\Delta t_{\text{physics}} = 0.002\text{ s}$ ($500\text{ Hz}$).
* **Control Step**: $\Delta t_{\text{control}} = \frac{1}{48}\text{ s} \approx 0.02083\text{ s}$ ($48\text{ Hz}$).
* Each control step runs **$10$ physics sub-steps** using Runge-Kutta 4th order (RK4) integration for numerical stability and realistic contact dynamics.

---

## 4. Perception & Sensor Array

The drone utilizes native MuJoCo raycasting (`mujoco.mj_ray`) configured to ignore the drone's own body geoms while reflecting off ground, curbs, and static/moving obstacles.

### Beam Layout:
| Sensor Name | Angle Relative to Forward | Purpose |
| :--- | :---: | :--- |
| **Front** | $0^\circ$ | Forward collision detection in line of sight |
| **Left** | $+35^\circ$ | Front-left flank detection (turns & corridor walls) |
| **Far Left** | $+70^\circ$ | Oblique lateral clearance |
| **Side Left** | $+90^\circ$ | Full lateral clearance while overtaking |
| **Right** | $-35^\circ$ | Front-right flank detection |
| **Far Right** | $-70^\circ$ | Oblique lateral clearance |
| **Side Right** | $-90^\circ$ | Full lateral clearance while overtaking |
| **Rear** | $180^\circ$ | Clearance monitoring during emergency reversals |
| **Up** | $+Z$ (Vertical) | Overflight clearance for 3D climb-overs |
| **Down** | $-Z$ (Vertical) | Ground proximity and altitude holding |

### Range & Safety Thresholds:
* **Maximum Range**: $2.00\text{ m}$ (readings beyond this register as clear space).
* **Caution Distance ($d_{\text{danger}}$)**: $0.75\text{ m}$ (beams turn **Orange**, triggers avoidance steering).
* **Emergency Distance ($d_{\text{emerg}}$)**: $0.30\text{ m}$ (beams turn **Red**, triggers immediate reverse thrust).

---

## 5. Kalman Filter Tracking & Trajectory Prediction

When obstacles move dynamically in real-world conditions, rangefinder hits exhibit measurement noise and jitter. Without filtering, raw velocity estimation amplifies noise, causing flight instability.

### State Space Formulation
The tracking system uses a 6-DOF Constant-Velocity (CV) Kalman Filter implemented in `perception/kalman_filter.py`:

$$\mathbf{x} = \begin{bmatrix} x & y & z & v_x & v_y & v_z \end{bmatrix}^T$$

#### State Transition Model:
$$\mathbf{x}_k = \mathbf{F} \mathbf{x}_{k-1} + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(0, \mathbf{Q})$$

$$\mathbf{F} = \begin{bmatrix} \mathbf{I}_{3\times3} & \Delta t \mathbf{I}_{3\times3} \\ \mathbf{0}_{3\times3} & \mathbf{I}_{3\times3} \end{bmatrix}$$

#### Measurement Model:
$$\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k, \quad \mathbf{v}_k \sim \mathcal{N}(0, \mathbf{R})$$

$$\mathbf{H} = \begin{bmatrix} \mathbf{I}_{3\times3} & \mathbf{0}_{3\times3} \end{bmatrix}$$

### Numerical Stabilization (Joseph Form)
To maintain positive semi-definiteness of the error covariance matrix $\mathbf{P}$ in floating-point computation, covariance update uses the Joseph form:

$$\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \mathbf{H})\mathbf{P}_{k|k-1}(\mathbf{I} - \mathbf{K}_k \mathbf{H})^T + \mathbf{K}_k \mathbf{R} \mathbf{K}_k^T$$

### Trajectory Projection & Closing Velocity
The tracker projects the obstacle's future position across a $1.0\text{-second}$ horizon:
$$\hat{\mathbf{p}}(t + \tau) = \hat{\mathbf{p}}(t) + \hat{\mathbf{v}}(t) \cdot \tau$$

The relative closing velocity toward the drone is:
$$v_{\text{closing}} = - (\hat{\mathbf{v}}_{\text{obs}} - \mathbf{v}_{\text{drone}}) \cdot \hat{\mathbf{d}}_{\text{sector}}$$

If $v_{\text{closing}} > 0.05\text{ m/s}$, the sector is flagged as an active closing danger, inflating its virtual size and causing the avoidance state machine to evade **away from the obstacle's projected trajectory** before it arrives.

### Stochastic Obstacle Noise (Ornstein-Uhlenbeck Process)
In `simulation/drone_sim.py`, dynamic obstacles do not follow pure sinusoidal motion. Instead, they incorporate Ornstein-Uhlenbeck mean-reverting drift:

$$dX_t = -\theta X_t dt + \sigma dW_t$$

Where $\theta = 1.5$ (reversion rate), $\sigma = 0.08\text{ m}$ (process noise), and $dW_t \sim \mathcal{N}(0, \sqrt{dt})$. This generates natural, non-deterministic movement that tests the Kalman Filter's tracking fidelity.

---

## 6. Avoidance Algorithms & Decision Logic

### 1. Reactive Sector State Machine (`avoidance/reactive_avoidance.py`)
A fast, robust state machine inspired by high-speed biological evasion:
* **Modes**: `FORWARD`, `AVOID`, `EMERGENCY_REVERSE`, `RETURN`, `VERTICAL_ESCAPE`, `CROSS_OVER`, `DESCEND`, `ESCAPING_THROUGH_GAP`, `GOAL_REACHED`.
* **Side-Switching Hysteresis**: When encountering an obstacle dead-center, hysteresis locks the chosen evasion direction (Left vs Right) for at least $0.35\text{ s}$ to prevent chatter or high-frequency oscillation.
* **Proactive Evasion**: Checks dynamic sector closing velocities. If an obstacle is sliding from Left to Right, the drone proactively steers Left (behind the obstacle's wake).

### 2. Dead-End & Ring Trap Recovery (360° LiDAR Inspection)
If a drone gets trapped in a concave structure (such as Scenario 5's U-Trap or Scenario 10's Circular Ring):
1. **Detection**: Tracks reversals. If **3 consecutive emergency reversals occur within 12 seconds**, the drone classifies the area as a dead-end trap.
2. **Inspection**: The drone halts and executes a full **72-ray ($5^\circ$ resolution) 360-degree perimeter sweep** plus an upward vertical ray test.
3. **Branching Decision**:
   - **Case A (Solid Enclosure)**: If horizontal sectors are blocked and upward vertical LiDAR is clear, it enters `VERTICAL_ESCAPE`: climbs over the wall ($Z = 1.60\text{ m}$), flies across the obstacle footprint (`CROSS_OVER`), and descends back to cruise altitude (`DESCEND`).
   - **Case B (Enclosure with Gap)**: If an opening arc $\ge 15^\circ$ is found (e.g. at bearing $142.5^\circ$), it calculates the gap centroid, sets heading toward the opening, and slips through (`ESCAPING_THROUGH_GAP`).

### 3. Artificial Potential Field (`avoidance/apf_avoidance.py`)
Computes an attractive potential gradient toward the goal combined with repulsive fields from all obstacle points:
$$U_{\text{att}} = \frac{1}{2} k_{\text{att}} \|\mathbf{p} - \mathbf{p}_{\text{goal}}\|^2$$
$$U_{\text{rep}} = \begin{cases} \frac{1}{2} k_{\text{rep}} \left(\frac{1}{d} - \frac{1}{d_0}\right)^2 & d \le d_0 \\ 0 & d > d_0 \end{cases}$$

### 4. Dynamic Window Approach (`avoidance/dwa_avoidance.py`)
Samples a grid of linear and angular velocities within the drone's reachable acceleration window, projecting forward trajectories and picking the path that maximizes heading alignment, clearance margin, and forward speed.

### 5. SINDy-Enhanced DWA (`avoidance/sindy_dwa_avoidance.py`)
Combines DWA with Sparse Identification of Nonlinear Dynamics (SINDy). Discovers the drone's true nonlinear equations of motion from flight telemetry:
$$\dot{\mathbf{x}} = \mathbf{\Theta}(\mathbf{x}, \mathbf{u}) \mathbf{\Xi}$$
Predicts rollout paths with aerodynamic drag and rotor-induced cross-coupling effects modeled directly from data.

---

## 7. Benchmark Flight Scenarios (1 to 10)

The simulation suite includes 10 rigorous test scenarios:

```
Scenario 1: Direct Frontal Obstacle       Scenario 4: Narrow Passage Corridor
     [GOAL] (6,0,1)                            [GOAL] (6,0,1)
        ^                                         ^
        |                                       [#] [#]  Gate 2 (x=3.2m, gap=0.84m)
       [#]  (x=2.5m, y=0.0m)                    [#] [#]  Gate 1 (x=2.5m, gap=0.76m)
        |                                         |
     [START] (0,0,1)                           [START] (0,0,1)

Scenario 5: Dead-End U-Trap               Scenario 7/9: Dynamic Moving Obstacle
     [GOAL] (6,0,1)                            [GOAL] (6,0,1)
        ^                                         ^
     [# # #] Front Wall (x=2.5m)                [<== [#] ==>] Sliding Obstacle (x=2.8m)
     [#   #] Wings (y=±0.36m, ±0.52m)             |
        |                                      [START] (0,0,1)
     [START] (0,0,1)
```

### Scenario Reference Table:
| ID | Scenario Name | Difficulty | Key Obstacle Coordinates | Core Test Objective |
| :---: | :--- | :---: | :--- | :--- |
| **1** | Direct Frontal Obstacle | Easy | `(2.50, 0.00, 1.00)m` | Head-on collision avoidance and smooth re-centering. |
| **2** | Offset Obstacle Left | Easy | `(2.50, +0.20, 1.00)m` | Asymmetric detection and rightward bias decision. |
| **3** | Offset Obstacle Right | Easy | `(2.50, -0.20, 1.00)m` | Asymmetric detection and leftward bias decision. |
| **4** | Narrow Passage (Slit) | Medium | Gates at $X=2.50\text{m}$ & $3.20\text{m}$ (gap $0.76\text{m}$) | Simultaneous bilateral clearance and corridor threading. |
| **5** | Dead-End U-Trap | Hard | Pocket: $X=2.50\text{m}$, Wings $Y=\pm0.36\text{m}, \pm0.52\text{m}$ | Emergency reversal oscillations & 3D vertical overflight. |
| **6** | Staggered Forest Maze | Hard | 4 Cylinders from $X=1.80\text{m}$ to $4.80\text{m}$ | Multi-stage continuous slalom navigation. |
| **7** | Dynamic Moving Obstacle | Dynamic | Block at $X=2.80\text{m}$ sliding laterally ($Y=\pm1.15\text{m}$) | Kalman tracking under Ornstein-Uhlenbeck stochastic drift. |
| **8** | 3D Cluttered Space | 3D Expert | Low wall ($Z=0.6\text{m}$), Overhang ($Z=1.6\text{m}$), Pillar | 3D spatial evasion: lateral bypass and vertical underpass. |
| **9** | Dynamic Lateral Intruder | Expert | Fast cube at $X=2.80\text{m}$ sliding laterally | High closing-velocity prediction and proactive evasion. |
| **10** | Circular Dead-End Ring | Extreme | 54 Cubes in a $R=2.60\text{m}$ ring around start point | Full 360° perimeter scan: vertical climb vs gap search. |

---

## 8. Interactive Web Dashboard & Customizer

The project includes an in-browser telemetry interface running on Python's standard library (`web_dashboard/app.py`).

### Accessing the Dashboard:
Run:
```bash
python run_sim.py --dashboard
```
Open **`http://127.0.0.1:8080`** in any modern web browser.

### Features:
1. **Dual-Viewport Architecture (2D Top-Down + 3D WebGL Orbit View)**:
   * **View Switcher**:
     * **`2D Top-Down`**: Classic tactical canvas view for drag-and-drop obstacle editing, mouse hovering coordinate readouts, and top-down LiDAR raycasting.
     * **`3D Orbit View`**: Full Three.js WebGL 3D simulation with Z-UP coordinate mapping identical to MuJoCo physics.
     * **`Split (2D + 3D)`**: Side-by-side synchronized view allowing live mouse obstacle rearrangement in 2D while watching the 3D quadcopter navigate in real time.
   * **3D Interactive Controls & Camera Presets**:
     * **Left-Click + Drag**: Orbit / Rotate 3D camera.
     * **Right-Click + Drag**: Pan camera laterally and vertically.
     * **Scroll Wheel**: Smooth zoom in / out.
     * **Quick Camera Presets**:
       * `Isometric`: Standard 45° perspective view of arena.
       * `Top-Down`: High-altitude birds-eye orthographic look-down.
       * `Side`: Ground-level elevation view showing climb/dive altitude profiles.
       * `Follow Drone`: Locks 3D orbit target to the drone's position as it maneuvers.
   * **3D Visual Elements**:
     * Realistic **Crazyflie Quadcopter 3D Model** with carbon arms, brushless motors, and spinning neon rotor discs ($45^\circ$ roll/pitch attitude tilt corresponding to flight dynamics).
     * **3D Obstacles**: Box and Cylinder meshes dynamically synchronized with MuJoCo dimensions and dynamic sliding states.
     * **3D LiDAR Lasers**: Omnidirectional 3D beam array with color-coded warning range (Green/Orange/Red) and surface impact dots.
     * **3D Flight Ribbon**: Continuous neon path strip recording the exact 3D climb/dive trajectory.
     * **3D Kalman Trajectory**: Dashed cyan future-prediction line projecting dynamic obstacle paths into 3D space.

2. **Interactive Arena Customizer**:
   * Switch between **Preset Scenarios** and **Arena Customizer**.
   * **Click & Drag**: Position obstacles anywhere on the 2D grid (instantly mirrors in 3D).
   * **Keyboard Controls**:
     * `A`: Add a Box obstacle.
     * `C`: Add a Cylinder obstacle.
     * `D`: Toggle dynamic sliding movement (orange color).
     * `Del` / `Backspace`: Remove selected obstacle.
     * `Space`: Execute / Pause flight simulation.
     * `S`: Save arena layout to `custom_arena.json`.
     * `L`: Load layout from `custom_arena.json`.

3. **Live Coordinates HUD**:
   * **Drone Floating Badge**: Shows live `(X, Y, Z)m` floating above the drone as it flies.
   * **Obstacle Coordinates**: Every obstacle displays its world coordinate `(X, Y)` directly underneath it.
   * **Bottom Telemetry Bar**: Shows real-time drone coordinates and live mouse cursor position `(X, Y)m`.

4. **Flight Kinematics & Moving Obstacle Analytics Suite**:
   * **`📊 Analysis & Graphs` Mode**:
     Accessible directly from the viewport mode switcher in the arena header or from the left sidebar's **Analysis** tab.
   * **Sub-Tab 1: 🛸 Drone Movement Analysis**:
     * **KPI Cards**: Peak Velocity ($V_{max}$), Average Velocity ($V_{avg}$), Path Efficiency ($6.0\text{m} / L_{path} \times 100\%$), Max Altitude Deviation ($\Delta Z_{max}$), and Closest Obstacle Margin.
     * **Flight Phase Distribution Timeline**: Multi-segmented color-coded bar displaying exact seconds and percentages spent across avoidance states (`FORWARD`, `AVOID_LEFT`, `AVOID_RIGHT`, `CLIMB`, `REVERSE`, `GOAL_APPROACH`, `HOVER`).
     * **Interactive Velocity & Speed Chart**: Time-series plot of Total Speed, Forward Velocity ($V_x$), Lateral Velocity ($V_y$), and Vertical Velocity ($V_z$).
     * **Interactive Altitude & Goal Convergence Chart**: Cruise altitude profile ($Z$) with dual-axis Distance-to-Goal curve.
   * **Sub-Tab 2: 🎯 Moving Objects & Kalman Filter Graphs**:
     * **Dynamic Obstacle KPI Cards**: Active moving obstacle target name, Closest Drone Proximity ($d_{min}$), Kalman Filter Position RMSE, Noise Variance Reduction %, and Lateral Oscillation Amplitude.
     * **Moving Object Trajectory Graph (True vs Noisy vs Kalman)**: Compares **Ground Truth Position ($Y_{true}$)** (Green), **Noisy Sensor Reading ($Y_{noisy}$)** with stochastic Ornstein-Uhlenbeck jitter (Orange dashed), and **Kalman Filter Estimated Position ($\hat{Y}_{kalman}$)** (Cyan glowing line).
     * **Separation Distance & Safety Margin Chart**: Euclidean distance between quadcopter and moving obstacle over time, annotated with the red **$0.35\text{m}$ Collision Safety Bubble**.
     * **Velocity & Noise Residual Analysis Chart**: Residual error curves comparing raw sensor noise error $|Y_{noisy} - Y_{true}|$ against Kalman residual error $|Y_{kalman} - Y_{true}|$, visually proving noise variance damping.
   * **Sub-Tab 3: 📋 Kinematics Data Table & CSV Export**:
     * Frame-by-frame numerical data log: Timestamp, 3D Position $(X, Y, Z)$, Speed, Flight State, Distance to Goal, Minimum Clearance, and Dynamic Obstacle Coordinates.
     * **Interactive Row Seeking**: Clicking any table row immediately jumps the playback slider and 3D quadcopter model to that exact moment in flight.
     * **`⬇ Export CSV`**: One-click download of the complete flight trace (`.csv`) for MATLAB, Python, or thesis plotting.

5. **Sensor & Trajectory Visualizers**:
   * **LiDAR Sector Radar**: 8-wedge spinning radar updating at $48\text{ Hz}$ (Green/Orange/Red).
   * **Telemetry Cards**: Flight result, path length, minimum clearance, collision steps, **Kalman RMSE Error**, and **Noise Variance Reduction %**.

---

## 9. Command-Line Guide & Execution Options

All features can be launched directly from the root directory:

### 1. Web Dashboard
```bash
python run_sim.py --dashboard
```

### 2. Standalone Desktop Tkinter Arena Editor
```bash
python interactive_arena.py
```

### 3. Native MuJoCo 3D Interactive Viewer
```bash
# Run Scenario 1 with 3D GUI
python run_sim.py --scenario 1 --algo reactive --gui

# Run Scenario 5 (Dead-End U-Trap) with 3D GUI
python run_sim.py --scenario 5 --algo reactive --gui

# Run Scenario 7 (Dynamic Moving Obstacle with Kalman Filter) with 3D GUI
python run_sim.py --scenario 7 --algo reactive --gui

# Run Scenario 9 (High-Speed Dynamic Intruder) with 3D GUI
python run_sim.py --scenario 9 --algo reactive --gui

# Run Scenario 10 (Closed Circular Ring Overflight)
python run_sim.py --scenario 10 --sub-scenario closed_ring --gui

# Run Scenario 10 (Ring with Random Gap Search)
python run_sim.py --scenario 10 --sub-scenario ring_with_gap --seed 42 --gui
```

### 4. Run Custom Arena in MuJoCo 3D Viewer
```bash
python run_sim.py --custom-file custom_arena.json --gui
```

### 5. Automated Dynamic Obstacle Analysis Suite
Generates high-resolution statistical plots (`dynamic_obstacle_analysis.png` and `dynamic_intruder_analysis.png`):
```bash
python analyze_dynamic_obstacle.py
```

### 6. Full Algorithm Benchmark Suite
Runs all scenarios across all algorithms and generates `BENCHMARK_RESULTS.json`:
```bash
python run_sim.py --benchmark
```

### 7. SINDy Nonlinear Dynamics Training
Discovers nonlinear quadcopter equations from simulation telemetry:
```bash
python run_sim.py --sindy-train
```

---

## 10. Mathematical & Algorithmic Formulations

### 1. SE(3) Geometric Controller
Given desired position $\mathbf{p}_{\text{des}}$, velocity $\mathbf{v}_{\text{des}}$, and acceleration $\mathbf{a}_{\text{des}}$, the position error dynamics are:
$$\mathbf{e}_p = \mathbf{p} - \mathbf{p}_{\text{des}}, \quad \mathbf{e}_v = \mathbf{v} - \mathbf{v}_{\text{des}}$$
$$\mathbf{F}_{\text{des}} = -k_p \mathbf{e}_p - k_v \mathbf{e}_v + m g \hat{\mathbf{z}}_W + m \mathbf{a}_{\text{des}}$$

Total scalar thrust command:
$$T = \mathbf{F}_{\text{des}} \cdot (\mathbf{R} \hat{\mathbf{z}}_B)$$

Desired body Z-axis:
$$\mathbf{z}_{B,\text{des}} = \frac{\mathbf{F}_{\text{des}}}{\|\mathbf{F}_{\text{des}}\|}$$

### 2. Motor Allocation (Mixer Matrix)
The 4 rotor thrusts $f_1, f_2, f_3, f_4$ are computed from total thrust $T$ and body moments $\tau_x, \tau_y, \tau_z$:
$$\begin{bmatrix} T \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix} = \begin{bmatrix} 1 & 1 & 1 & 1 \\ -d/\sqrt{2} & d/\sqrt{2} & d/\sqrt{2} & -d/\sqrt{2} \\ d/\sqrt{2} & -d/\sqrt{2} & d/\sqrt{2} & -d/\sqrt{2} \\ -k_m & -k_m & k_m & k_m \end{bmatrix} \begin{bmatrix} f_1 \\ f_2 \\ f_3 \\ f_4 \end{bmatrix}$$

### 3. Kalman Filter Gain & Innovation Covariance
$$\mathbf{y}_k = \mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_{k|k-1}$$
$$\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}$$
$$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T \mathbf{S}_k^{-1}$$
$$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k$$

---

## 11. Project Codebase Structure

```
mujoco_drone_sim/
├── run_sim.py                     # Main CLI simulation runner & entry point
├── benchmark.py                   # Automated evaluation suite across all algorithms
├── analyze_dynamic_obstacle.py    # Generates 5-panel Kalman tracking & error plots
├── interactive_arena.py           # Standalone Tkinter desktop visual arena editor
├── custom_arena.json              # Saved user-defined obstacle layout configuration
│
├── models/
│   ├── drone.xml                  # MuJoCo MJCF model of 6-DOF quadcopter with sensors
│   └── scene_generator.py         # Dynamic procedural MJCF generator for all 10 scenarios
│
├── perception/
│   ├── lidar_sensor.py            # 8-beam horizontal + 2-beam vertical LiDAR raycaster
│   └── kalman_filter.py           # 6-DOF Constant-Velocity Kalman Filter & multi-tracker
│
├── avoidance/
│   ├── base_avoidance.py          # Abstract base class for avoidance algorithms
│   ├── reactive_avoidance.py      # State machine with dynamic evasion & 360° trap escape
│   ├── apf_avoidance.py           # Artificial Potential Field (APF) implementation
│   ├── dwa_avoidance.py           # Dynamic Window Approach (DWA) implementation
│   ├── avoidance_3d.py            # 3D Volumetric altitude-adaptive avoidance
│   ├── sindy_dwa_avoidance.py     # SINDy-enhanced data-driven DWA
│   └── neural_brain_avoidance.py  # Adaptive neural network avoidance
│
├── controllers/
│   ├── dsl_pid_controller.py      # Calibrated DSL-PID SE(3) geometric attitude controller
│   └── pid_controller.py          # Standard cascaded position/velocity PID controller
│
├── simulation/
│   ├── drone_sim.py               # Main simulation loop, contact checks, and OU noise
│   └── scenarios.py               # Metadata and definitions for Scenarios 1 to 10
│
├── sindy/
│   ├── sindy_core.py              # Sequential Thresholded Ridge Regression (STLSQ)
│   ├── drone_identifier.py        # Discovers quadcopter dynamics from flight logs
│   └── obstacle_predictor.py      # SINDy trajectory predictor for dynamic obstacles
│
└── web_dashboard/
    ├── app.py                     # Multi-threaded HTTP server & simulation API backend
    └── static/
        └── index.html             # Interactive HTML5/Canvas UI with live coordinate HUD
```

---

*Documentation compiled for the MuJoCo Autonomous Quadcopter Simulation Project.*
