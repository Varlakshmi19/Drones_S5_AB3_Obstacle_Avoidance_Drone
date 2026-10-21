<p align="center">
  <img src="assets/amrita_logo.jpg" alt="Amrita Vishwa Vidyapeetham Logo" width="300">
</p>

<h1 align="center">Autonomous Obstacle Avoidance for Quadrotor Drones</h1>

<p align="center">
  <b>Data-Driven Control & Bio-Inspired Reactive Navigation in PyBullet Simulation</b><br>
  Team S5 AB3 &bull; Amrita Vishwa Vidyapeetham &bull; School of Artificial Intelligence
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.10%2B-blue.svg" alt="Python">
  <img src="https://img.shields.io/badge/Simulator-PyBullet-orange.svg" alt="PyBullet">
  <img src="https://img.shields.io/badge/Framework-gym--pybullet--drones-green.svg" alt="gym-pybullet-drones">
  <img src="https://img.shields.io/badge/Physics_Frequency-240_Hz-lightgrey.svg" alt="240Hz">
  <img src="https://img.shields.io/badge/Control_Frequency-48_Hz-blueviolet.svg" alt="48Hz">
  <img src="https://img.shields.io/badge/Success_Rate-100%25-brightgreen.svg" alt="100% Success">
</p>

---

## Team Members

| S. No. | Name | Roll Number | Email |
|:---:|---|---|---|
| 1 | Gade Varshitha | CB.SC.U4AIE24114 | varshithareddygade54@gmail.com |
| 2 | Jampana Lakshmi Tejaswi | CB.SC.U4AIE24116 | jlakshmitejaswi@gmail.com |
| 3 | Konkimalla Laxmi Vignesh | CB.SC.U4AIE24124 | laxmivigneshkonkimalla@gmail.com |
| 4 | Koruprolu Sri Durga Varalakshmi | CB.SC.U4AIE24125 | varshakoruprolu19@gmail.com |
| 5 | Ch. Mounica | CB.SC.U4AIE24169 | mounikachevvakula@gmail.com |

**Amrita Vishwa Vidyapeetham** | School of Artificial Intelligence | Coimbatore Campus | 2026

---

<a id="table-of-contents"></a>
## Table of Contents

- [1. Project Title & Executive Summary](#1-project-title--executive-summary)
- [2. Project Overview](#2-project-overview)
- [3. Key Features](#3-key-features)
- [4. How the Project Works (Architecture & Workflow)](#4-how-the-project-works-architecture--workflow)
- [5. Technologies Used](#5-technologies-used)
- [6. Project Folder Structure](#6-project-folder-structure)
- [7. Complete Mathematical Foundations & Equations](#7-complete-mathematical-foundations--equations)
  - [7.1 Coordinate Systems & Vector Kinematics](#71-coordinate-systems--vector-kinematics)
  - [7.2 Virtual Ray-Casting & Obstacle Perception](#72-virtual-ray-casting--obstacle-perception)
  - [7.3 Reactive State Machine (FSM) Decision Mathematics](#73-reactive-state-machine-fsm-decision-mathematics)
  - [7.4 Bio-Inspired Adaptive Neural Brain Dynamics](#74-bio-inspired-adaptive-neural-brain-dynamics)
  - [7.5 Synaptic Plasticity & Memory Hysteresis](#75-synaptic-plasticity--memory-hysteresis)
  - [7.6 Frustration Dynamics & Deadlock Detour Vector](#76-frustration-dynamics--deadlock-detour-vector)
  - [7.7 Speed Neuron & Safe Braking Distance](#77-speed-neuron--safe-braking-distance)
  - [7.8 DSLPID Flight Control Mathematics](#78-dslpid-flight-control-mathematics)
  - [7.9 Data-Driven Physics Discovery via SINDy & SINDYc](#79-data-driven-physics-discovery-via-sindy--sindyc)
- [8. Test Scenarios (1 to 10)](#8-test-scenarios-1-to-10)
- [9. Prerequisites](#9-prerequisites)
- [10. Installation and Setup](#10-installation-and-setup)
- [11. How to Run the Project](#11-how-to-run-the-project)
  - [11.1 Running the Reactive FSM Controller](#111-running-the-reactive-fsm-controller)
  - [11.2 Running the Adaptive Neural Brain](#112-running-the-adaptive-neural-brain)
  - [11.3 Running SINDy Dynamic Identification](#113-running-sindy-dynamic-identification)
- [12. Example Usage & Step-by-Step Walkthrough](#12-example-usage--step-by-step-walkthrough)
- [13. Code and Module Explanation](#13-code-and-module-explanation)
- [14. Datasets, Logs & Input Specifications](#14-datasets-logs--input-specifications)
- [15. Experimental Results and Outputs](#15-experimental-results-and-outputs)
  - [15.1 Reactive FSM Scenario Benchmark](#151-reactive-fsm-scenario-benchmark)
  - [15.2 Scenario Visualizations & Trajectories](#152-scenario-visualizations--trajectories)
  - [15.3 SINDy Discovered Mathematical Models & Accuracy](#153-sindy-discovered-mathematical-models--accuracy)
- [16. Testing and Validation](#16-testing-and-validation)
- [17. Troubleshooting Guide](#17-troubleshooting-guide)
- [18. Limitations](#18-limitations)
- [19. Future Improvements](#19-future-improvements)
- [20. Contributing](#20-contributing)
- [21. License and Acknowledgements](#21-license-and-acknowledgements)
- [22. References](#22-references)

---

<a id="1-project-title--executive-summary"></a>
## 1. Project Title & Executive Summary

**Autonomous Obstacle Avoidance for Quadrotor Drones: Data-Driven Control & Bio-Inspired Navigation**

This repository implements, evaluates, and mathematically identifies an end-to-end autonomous navigation and obstacle-avoidance system for a micro-quadrotor drone (Bitcraze Crazyflie 2.X) in the **PyBullet** physics engine using the **gym-pybullet-drones** environment.

The system addresses the challenge of real-time collision avoidance in cluttered, unknown, dynamic, and dead-end environments without requiring heavy SLAM or expensive GPUs. It provides two distinct, fully implemented navigation paradigms:
1. **Hierarchical Finite State Machine (FSM):** A deterministic, rule-based controller that evaluates directional range sensors, balances lateral opening scores, executes emergency reversals, and checks progress with a sliding-window stuck detector.
2. **Bio-Inspired Adaptive Neural Brain:** An implementation inspired by the research of *Arne Devos, Emad Ebeid, and Poramate Manoonpong (2018)*, employing a recurrent non-spiking neural network with plastic Hebbian self-synapses, mutual inhibition, and an internal frustration leaky integrator to escape complex local minima, traps, and U-shaped dead ends.
3. **Data-Driven Equation Discovery (SINDy / SINDYc):** A machine learning pipeline utilizing *Sparse Identification of Nonlinear Dynamics* (Brunton et al., PNAS 2016) that mines the raw flight simulation telemetry to discover the true underlying translational physics (recovering gravitational acceleration $g = 9.80\,\text{m/s}^2$ with $R^2 > 0.999$), quadrotor rotor mixing torques, and closed-loop control dynamics.

Across standardized test benchmarks (from simple frontal obstacles to moving intruders and circular ring traps), the system achieves a **100% mission success rate** with **zero recorded collisions**.

---

<a id="2-project-overview"></a>
## 2. Project Overview

### What the Project Does
The project equips an autonomous quadrotor with omnidirectional virtual range perception and fast local planning algorithms that safely guide it from a launch coordinate $p_0 = (0.0, 0.0, 1.0)\\text{ m}$ to a target waypoint $p}_{\text{goal}} = (6.0, 0.0, 1.0)\\text{m}$. Along this route, the drone autonomously detects obstacles, dynamically computes collision-free trajectories, escapes traps, and stabilizes itself at $48\,\text{Hz}$ via position-attitude PID control.

### What Problem It Solves
Traditional obstacle-avoidance techniques for micro-air vehicles (MAVs) present difficult trade-offs:
- **Global Path Planners (A\*, RRT\*, PRM):** Require a full prior 3D map of the environment and cannot handle dynamic moving obstacles in real time.
- **Deep Reinforcement Learning (DRL):** Demands millions of trial-and-error training steps, functions as an uninterpretable "black box", and frequently fails when encountering out-of-distribution obstacle shapes.
- **Vision/LiDAR SLAM Pipelines:** Demand massive onboard computing power and heavy payload sensors that drain miniature drone batteries within minutes.

This project solves these limitations by implementing computationally lightweight reactive and bio-inspired controllers that operate using only low-dimensional distance measurements.

### Why the Project Is Useful & Who Benefits
- **Robotics Researchers & Students:** Provides a transparent, fully open-source PyBullet codebase with reproducible benchmarks, exact mathematical derivations, and clean modular code.
- **Micro-UAV Developers:** Demonstrates real-time obstacle avoidance that can be executed on ultra-low-power microcontrollers (such as STM32 / ARM Cortex-M4 running at 168 MHz) onboard drones like the Crazyflie 2.1.
- **System Identification Engineers:** Demonstrates how SINDy can be applied to real drone telemetry to extract interpretable, sparse governing differential equations directly from flight logs.

---

<a id="3-key-features"></a>
## 3. Key Features

- **Dual Navigation Architectures:**
  - **Deterministic FSM Engine (`obstacle_avoidance.py`):** 5-state navigation machine (`FORWARD`, `AVOID`, `RETURN`, `EMERGENCY_REVERSE`, `GOAL_REACHED`).
  - **Bio-Inspired Neural Brain (`brain_avoidance.py` & `brain/neural_brain.py`):** 2-neuron recurrent steering network with Hebbian plasticity, continuous heading steering, speed regulation neuron, and frustration-driven goal detours.
- **Comprehensive 10-Scenario Benchmark Suite:**
  - Stationary cube barriers, offset obstacles, narrow slits, U-traps, cylindrical forest slaloms, oscillating lateral obstacles, 3D vertical climbing tiers, dynamic high-speed intruders, and 54-cube circular ring traps.
- **Dynamic Obstacle Physics Support:**
  - Real-time kinematics update for time-varying moving obstacles ($y(t) = y_0 + A \sin(\omega t)$) with active collision checking.
- **Virtual Range Sensor Array:**
  - 6-beam horizontal array in FSM ($0^\circ, \pm 35^\circ, \pm 70^\circ, 180^\circ$) and 12-beam body-fixed ray fan in Neural Brain ($-90^\circ$ to $+180^\circ$) simulated via PyBullet ray-testing.
- **Full Closed-Loop Physics Simulation:**
  - 240 Hz PyBullet multi-body rigid-body dynamics coupled with 48 Hz `DSLPIDControl` computing individual rotor RPMs ($0$ to $21{,}702\,\text{RPM}$).
- **SINDy Telemetry Discovery Engine:**
  - STLSQ (Sequentially Thresholded Least Squares) sparse regressor discovering translational forces, motor torque mixing matrices, and closed-loop velocity transfer functions directly from `.npz` flight logs.
- **Automatic Trajectory & Metric Logging:**
  - Tracks total flight time, distance travelled, control iterations, collision contact counts, avoidance transitions, and emergency reversals.

---

<a id="4-how-the-project-works-architecture--workflow"></a>
## 4. How the Project Works (Architecture & Workflow)

The system operates as a closed-loop perception-planning-control pipeline executed at $48\,\text{Hz}$ on top of the $240\,\text{Hz}$ PyBullet physical simulator.

### System Architecture Diagram

```text
 ┌────────────────────────────────────────────────────────────────────────┐
 │                      PyBullet Physics Engine (240 Hz)                  │
 │   - CF2X Quadrotor Rigid Body Dynamics                                │
 │   - Obstacle Geometries (Boxes, Cylinders, Dynamic Moving Bodies)      │
 │   - Contact Point Collision Detection                                  │
 └──────────────────────────────────┬─────────────────────────────────────┘
                                    │ Drone State: [p, q, v, w]
                                    ▼
 ┌────────────────────────────────────────────────────────────────────────┐
 │                 Virtual Range Perception (48 Hz)                       │
 │   - PyBullet rayTest / rayTestBatch                                    │
 │   - Ray starts with radial offset outside rotor diameter (d = 0.18 m)  │
 │   - Returns distance vector d_i in [0, R_sensor]                       │
 └──────────────────────────────────┬─────────────────────────────────────┘
                                    │ Distances & Bearing Angles
                                    ▼
         ┌──────────────────────────┴──────────────────────────┐
         │                                                     │
         ▼ (Paradigm 1: FSM)                                   ▼ (Paradigm 2: Neural Brain)
 ┌───────────────────────────────┐                     ┌───────────────────────────────┐
 │ Reactive Navigation FSM       │                     │ Bio-Inspired Neural Brain     │
 │ - Weighted Spatial Scores     │                     │ - Tanh Recurrent Steering     │
 │ - Threshold Conditionals      │                     │ - Plastic Hebbian Synapses    │
 │ - Stuck Sliding Window        │                     │ - Leaky Frustration Monitor   │
 │ - Emergency Reversals         │                     │ - Speed & Detour Modulation   │
 └───────────────┬───────────────┘                     └───────────────┬───────────────┘
                 │                                                     │
                 └──────────────────────────┬──────────────────────────┘
                                            │ Desired Velocity Vector v_des
                                            ▼
 ┌────────────────────────────────────────────────────────────────────────┐
 │                  Target Position Generator (Lookahead)                 │
 │            p_target = p_current + v_des * lookahead_time               │
 └──────────────────────────────────┬─────────────────────────────────────┘
                                    │ Target State [p_target, v_des, yaw]
                                    ▼
 ┌────────────────────────────────────────────────────────────────────────┐
 │                   DSLPIDControl Position & Attitude Controller         │
 │   - Position PD -> Desired Virtual Acceleration vector                 │
 │   - Geometric Attitude Extraction -> Desired Orientation Quaternion    │
 │   - Angular Velocity PD -> Desired Torques [tau_x, tau_y, tau_z, T]    │
 │   - Motor Mixing Matrix -> Individual Rotor RPM Commands (Motors 0..3) │
 └──────────────────────────────────┬─────────────────────────────────────┘
                                    │ Rotor RPM Commands (action[0, :])
                                    ▼
 ┌────────────────────────────────────────────────────────────────────────┐
 │                  PyBullet Step (5 physics ticks per control tick)      │
 └──────────────────────────────────┬─────────────────────────────────────┘
                                    │ Telemetry Streaming
                                    ▼
 ┌────────────────────────────────────────────────────────────────────────┐
 │         Logging & SINDy Dynamical Identification Pipeline              │
 │   - Saves logs/*.npz (Position, Orientation, Rates, RPM, Rays)         │
 │   - Identifies Governing Differential Equations via STLSQ              │
 └────────────────────────────────────────────────────────────────────────┘
```

---

<a id="5-technologies-used"></a>
## 5. Technologies Used

| Technology / Library | Version / Scope | Purpose in Project |
|---|---|---|
| **Python** | 3.10+ | Core programming language for all logic, physics interfaces, and math routines. |
| **PyBullet** | $\ge 3.2.5$ | Real-time multi-body physics engine providing rigid-body collision detection and ray-casting. |
| **gym-pybullet-drones** | Master (utiasDSL) | Quadrotor simulation framework providing the Crazyflie 2.X model, environments, and control classes. |
| **NumPy** | $\ge 1.22$ | Fast vectorized mathematical operations, matrix inversions, quaternions, and ray fans. |
| **SciPy** | $\ge 1.9$ | Signal processing tools including the Savitzky-Golay filter (`savgol_filter`) for numerical differentiation. |
| **Matplotlib** | $\ge 3.5$ | Plotting trajectory comparisons, neural activations, phase portraits, and SINDy validation graphs. |
| **DSLPIDControl** | Built-in | Non-linear geometric cascaded PID tracking controller tuned specifically for the CF2X quadrotor. |
| **SINDy / SINDYc** | Custom Implementation | Sparse Identification of Nonlinear Dynamics discovering equations of motion from simulation data. |

---

<a id="6-project-folder-structure"></a>
## 6. Project Folder Structure

The directory is structured modularly to separate rule-based navigation, neural control, mathematical modeling, and experimental results:

```text
Drones_S5_AB3_Obstacle_Avoidance_Drone/
│
├── README.md                      # Complete project documentation and mathematical derivations
├── RESULTS.md                     # Detailed experimental log data for Scenarios 1 to 6
├── FRP.md                         # Final Review Proposal detailing objectives, problem statement, and scope
├── Base_Paper.pdf                 # Reference research paper (Devos, Ebeid, Manoonpong, DSD 2018)
├── requirements.txt               # Top-level Python dependency specification
├── obstacle_avoidance.py          # Root copy of the reactive FSM navigation controller
│
├── pybullet/                      # Core PyBullet simulation and scientific research module
│   ├── obstacle_avoidance.py      # Primary FSM navigation engine (10 scenarios, dynamic obstacles, metrics)
│   ├── brain_avoidance.py         # Testbed running the Adaptive Neural Brain across scenarios 1 to 10
│   │
│   ├── brain/                     # Bio-inspired neural navigation package
│   │   ├── __init__.py            # Module exports (AdaptiveNeuralBrain, BrainConfig, RAY_ANGLES_DEG)
│   │   └── neural_brain.py        # Complete RNN architecture with Hebbian plasticity and frustration
│   │
│   ├── sindy/                     # Sparse Identification of Nonlinear Dynamics package
│   │   ├── __init__.py            # Module exports (SINDy, PolynomialLibrary, stlsq, finite_difference)
│   │   ├── sindy.py               # Lightweight STLSQ sparse regression and polynomial library implementation
│   │   └── identify_dynamics.py   # Script identifying translational, rotational, and closed-loop models
│   │
│   ├── models/                    # Identified mathematical models stored as serialized JSON
│   │   ├── sindy_translational.json  # Discovered translational physics (recovers gravity and thrust)
│   │   ├── sindy_rotational.json     # Discovered quadrotor torque mixing matrix for X-configuration
│   │   └── sindy_closed_loop.json    # Discovered velocity response transfer function (SINDYc)
│   │
│   ├── results/                   # Experimental plots, graphs, and performance reports
│   │   └── sindy/
│   │       ├── A_translational.png       # Parity plots: predicted vs actual accelerations (ax, ay, az)
│   │       ├── B_rotational.png          # Parity plots: predicted vs actual angular accelerations (dw)
│   │       ├── C_rollout_brain_s4.png    # Free RK4 integration rollout on held-out Scenario 4
│   │       └── report.json               # Quantitative R^2 scores, residual errors, and formulas
│   │
│   └── logs/                      # Telemetry datasets recorded from 48 Hz simulation runs
│       ├── brain_s1.npz           # Scenario 1 flight telemetry log
│       ├── brain_s2.npz           # Scenario 2 flight telemetry log
│       ├── brain_s3.npz           # Scenario 3 flight telemetry log
│       ├── brain_s4.npz           # Scenario 4 flight telemetry log (held-out test set for SINDy)
│       ├── brain_s5.npz           # Scenario 5 flight telemetry log
│       ├── brain_s6.npz           # Scenario 6 flight telemetry log
│       ├── brain_s7.npz           # Scenario 7 flight telemetry log (Dynamic moving obstacle)
│       ├── brain_s8.npz           # Scenario 8 flight telemetry log (3D clutter)
│       ├── brain_s9.npz           # Scenario 9 flight telemetry log (High-speed intruder)
│       └── brain_s10.npz          # Scenario 10 flight telemetry log (Circular dead-end ring)
│
├── terminalResults/               # Raw execution logs captured from terminal runs
│   ├── scenario1.txt              # Terminal output log for Scenario 1
│   ├── scenario2.txt              # Terminal output log for Scenario 2
│   ├── scenario3.txt              # Terminal output log for Scenario 3
│   ├── scenario4.txt              # Terminal output log for Scenario 4
│   ├── scenario5.txt              # Terminal output log for Scenario 5
│   └── scenario6.txt              # Terminal output log for Scenario 6
│
└── assets/                        # Documentation images and university branding
    └── amrita_logo.jpg            # Amrita Vishwa Vidyapeetham official emblem
```

---

<a id="7-complete-mathematical-foundations--equations"></a>
## 7. Complete Mathematical Foundations & Equations

This section provides the mathematical models, formulas, derivations, and step-by-step numerical examples corresponding to the actual source code.

```text
 ┌────────────────────────────────────────────────────────────────────────┐
 │                      Mathematical Architecture Overview                │
 ├────────────────────────────────────────────────────────────────────────┤
 │ 1. Geometry & Perception:                                              │
 │    - Goal Direction:         d_hat_g = (p_goal - p) / ||p_goal - p||   │
 │    - Ray-Casting Fan:        r_i(s)  = p + offset*u_theta + s*R*u_theta│
 │    - Elliptical Bubble:      R_react(theta) = ab / sqrt((b cos)^2 + ..)│
 │                                                                        │
 │ 2. Decision & Reactive FSM:                                            │
 │    - Spatial Free Space:     S_L = 0.65 d_L + 0.35 d_FL                │
 │    - Decision Hysteresis:    |S_L - S_R| >= delta_hysteresis           │
 │    - Stuck Detection:        ||p[k] - p[k-W]|| < delta_progress        │
 │                                                                        │
 │ 3. Bio-Inspired Neural Brain:                                          │
 │    - Leaky Membrane ODE:     tau * dx/dt = -x + drive                  │
 │    - Hebbian Plasticity:     dw_ii/dt = mu*(1+2F)*pre*post - gamma*w_ii│
 │    - Frustration Integrator: dF/dt = alpha_rise (if stalled)           │
 │    - Adaptive Heading Rate:  omega = k_g*err_g*(1-act) - k_turn*steer  │
 │                                                                        │
 │ 4. Flight Control & Dynamics:                                          │
 │    - Cascaded DSLPID:        a_des = K_p*e_p + K_d*e_v + g*z_W         │
 │    - Motor RPM Allocation:   Thrust = k_f * sum(Omega_i^2)             │
 │                                                                        │
 │ 5. Data-Driven Discovery:                                              │
 │    - SINDy Formulation:      dX/dt = Theta(X, U) * Xi                  │
 │    - STLSQ Optimization:     min ||dX - Theta*Xi||_2^2 + alpha*||Xi||_F│
 └────────────────────────────────────────────────────────────────────────┘
```

---

### 7.1 Coordinate Systems & Vector Kinematics

The simulation operates in a right-handed inertial Cartesian frame $\mathcal{F}_W = \{\mathbf{x}_W, \mathbf{y}_W, \mathbf{z}_W\}$ where $\mathbf{z}_W$ points upward against gravity.

#### Equation: Horizontal Unit Direction toward Goal
$$
\mathbf{d}_{\text{goal}} = \mathbf{p}_{\text{goal}} - \mathbf{p}(t) = \begin{bmatrix} x_{\text{goal}} - x(t) \\ y_{\text{goal}} - y(t) \\ 0 \end{bmatrix}
$$
$$
\hat{\mathbf{d}}_g = \frac{\mathbf{d}_{\text{goal}}}{\|\mathbf{d}_{\text{goal}}\|_2} = \begin{bmatrix} \frac{x_{\text{goal}} - x}{\sqrt{(x_{\text{goal}} - x)^2 + (y_{\text{goal}} - y)^2}} \\ \frac{y_{\text{goal}} - y}{\sqrt{(x_{\text{goal}} - x)^2 + (y_{\text{goal}} - y)^2}} \\ 0 \end{bmatrix}
$$

- **Meaning:** Computes the unit direction vector in the horizontal $xy$-plane pointing directly from the drone to the target waypoint.
- **Variables:**
  - $\mathbf{p}(t) = [x, y, z]^T$: Instantaneous 3D position of the drone in world coordinates ($\text{m}$).
  - $\mathbf{p}_{\text{goal}} = [6.0, 0.0, 1.0]^T$: Fixed 3D destination ($\text{m}$).
  - $\hat{\mathbf{d}}_g$: Normalized 2D horizontal unit vector.
- **Implementation:** `horizontal_unit_vector()` and `goal_direction()` in [pybullet/obstacle_avoidance.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/obstacle_avoidance.py#L341-L381).
- **Step-by-Step Numerical Example:**
  Suppose the drone has moved to $\mathbf{p} = [2.00, 0.50, 1.00]^T$:
  1. Difference vector: $\mathbf{d}_{\text{goal}} = [6.0 - 2.0, 0.0 - 0.5, 0.0]^T = [4.0, -0.5, 0.0]^T$.
  2. Euclidean norm: $\|\mathbf{d}_{\text{goal}}\| = \sqrt{4.0^2 + (-0.5)^2} = \sqrt{16.0 + 0.25} = \sqrt{16.25} \approx 4.0311\,\text{m}$.
  3. Normalized vector: $\hat{\mathbf{d}}_g = \left[\frac{4.0}{4.0311}, \frac{-0.5}{4.0311}, 0\right]^T \approx [0.9923, -0.1240, 0.0]^T$.

#### Equation: 2D Planar Rotation Matrix
To project sensor rays or lateral avoidance vectors at an angle $\theta$ relative to $\hat{\mathbf{d}}_g$:
$$
\mathbf{R}_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}, \quad \hat{\mathbf{u}}_\theta = \mathbf{R}_z(\theta) \hat{\mathbf{d}}_g
$$
- **Implementation:** `rotate_horizontal(vector, angle_degrees)` in [pybullet/obstacle_avoidance.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/obstacle_avoidance.py#L357-L371).

---

### 7.2 Virtual Ray-Casting & Obstacle Perception

The drone perceives its local surroundings by querying the PyBullet collision world via geometric rays.

#### Equation: Ray Parametrization & Distance Extraction
Each ray $i$ originating from the drone body is parametrized by $s \in [0, 1]$:
$$
\mathbf{r}_i(s) = \mathbf{r}_{\text{start}, i} + s \cdot (\mathbf{r}_{\text{end}, i} - \mathbf{r}_{\text{start}, i})
$$
where:
$$
\mathbf{r}_{\text{start}, i} = \mathbf{p} + d_{\text{offset}} \hat{\mathbf{u}}_{\theta_i}, \quad \mathbf{r}_{\text{end}, i} = \mathbf{r}_{\text{start}, i} + R_{\text{sensor}} \hat{\mathbf{u}}_{\theta_i}
$$
PyBullet's `p.rayTest` returns the normalized intersection fraction $f_i \in [0, 1]$ and the collided object identifier $\text{id}_{\text{hit}}$:
$$
d_i = \begin{cases} R_{\text{sensor}}, & \text{if } \text{id}_{\text{hit}} = -1 \text{ (no hit)} \\ f_i \cdot R_{\text{sensor}}, & \text{if } \text{id}_{\text{hit}} \ge 0 \text{ (obstacle detected)} \end{cases}
$$

- **Variables:**
  - $d_{\text{offset}} = 0.18\,\text{m}$ (FSM) or $0.10\,\text{m}$ (Brain): Radial clearance ensuring rays originate outside the quadrotor's propeller diameter ($0.065\,\text{m}$ radius) to avoid self-detection.
  - $R_{\text{sensor}} = 1.60\,\text{m}$: Maximum sensor detection range.
  - $d_i$: Measured Euclidean distance to obstacle along bearing angle $\theta_i$.
- **Implementation:** `cast_sensor_ray()` in [pybullet/obstacle_avoidance.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/obstacle_avoidance.py#L387-L431) and `cast_rays()` in [pybullet/brain_avoidance.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/brain_avoidance.py#L71-L89).

#### Equation: Elliptical Reactive Bubble (Neural Brain)
To prevent the drone from false-triggering avoidance on corridor side-walls when flying down narrow passages, the reaction threshold varies continuously with heading angle $\theta$ following an ellipse:
$$
R_{\text{react}}(\theta) = \frac{a \cdot b}{\sqrt{(b \cos\theta)^2 + (a \sin\theta)^2}}
$$
- **Variables:**
  - $a = 1.10\,\text{m}$: Major semi-axis straight ahead ($\theta = 0^\circ$).
  - $b = 0.32\,\text{m}$: Minor semi-axis directly sideways ($\theta = \pm 90^\circ$).
- **Numerical Example:**
  - Ahead ($\theta = 0^\circ$): $R_{\text{react}}(0^\circ) = \frac{1.10 \times 0.32}{\sqrt{(0.32 \times 1)^2 + 0}} = 1.10\,\text{m}$.
  - Oblique ($\theta = 45^\circ$): $\cos(45^\circ) = \sin(45^\circ) \approx 0.7071$.
    $\sqrt{(0.32 \times 0.7071)^2 + (1.10 \times 0.7071)^2} = \sqrt{0.0512 + 0.6050} = \sqrt{0.6562} \approx 0.8101$.
    $R_{\text{react}}(45^\circ) = \frac{0.352}{0.8101} \approx 0.4345\,\text{m}$.
  - Sideways ($\theta = 90^\circ$): $R_{\text{react}}(90^\circ) = 0.32\,\text{m}$.
- **Implementation:** [pybullet/brain/neural_brain.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/brain/neural_brain.py#L112-L113).

---

### 7.3 Reactive State Machine (FSM) Decision Mathematics

The FSM divides 3D space into six directional sectors relative to $\hat{\mathbf{d}}_g$:
- $\text{Far Left } (+70^\circ)$, $\text{Left } (+35^\circ)$, $\text{Front } (0^\circ)$, $\text{Right } (-35^\circ)$, $\text{Far Right } (-70^\circ)$, $\text{Rear } (180^\circ)$.

```text
                       Front (0°)
                           ▲
             Left (+35°)   │   Right (-35°)
                  \        │        /
                   \       │       /
         Far-Left   \      │      /   Far-Right
          (+70°)     \     │     /     (-70°)
               ◄─────── Drone ───────►
                           │
                           │
                           ▼
                      Rear (180°)
```

#### Equation: Spatial Free-Space Scores
$$
S_{\text{left}} = 0.65 \cdot d_{\text{left}} + 0.35 \cdot d_{\text{far\_left}}
$$
$$
S_{\text{right}} = 0.65 \cdot d_{\text{right}} + 0.35 \cdot d_{\text{far\_right}}
$$
- **Weights Meaning:** $0.65$ weights the primary turning zone ($35^\circ$), while $0.35$ provides contextual clearance in the wider periphery ($70^\circ$).
- **Decision Rule with Hysteresis:**
$$
\text{Side}^* = \begin{cases} \text{Previous Side}, & \text{if } |S_{\text{left}} - S_{\text{right}}| < 0.08\,\text{m} \text{ and Previous Side is set} \\ \text{LEFT}, & \text{if } S_{\text{left}} \ge S_{\text{right}} \\ \text{RIGHT}, & \text{if } S_{\text{right}} > S_{\text{left}} \end{cases}
$$
- **Purpose:** The $0.08\,\text{m}$ dead-band hysteresis prevents high-frequency limit-cycle chatter (rapid left-right oscillations) when facing symmetric obstacles.
- **Implementation:** `calculate_side_scores()` & `choose_avoidance_side()` in [pybullet/obstacle_avoidance.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/obstacle_avoidance.py#L475-L515).

#### Equation: Stuck & Trap Detection
Let the position history buffer of length $W = 96$ ticks ($2.0\,\text{s}$ at $48\,\text{Hz}$) be $\mathcal{H}_p = \{\mathbf{p}_{k-W+1}, \dots, \mathbf{p}_k\}$.
$$
\Delta p_{\text{progress}} = \|\mathbf{p}_k[0:2] - \mathbf{p}_{k-W+1}[0:2]\|_2
$$
The stuck flag is asserted when:
$$
\text{Stuck} = \left(\Delta p_{\text{progress}} < 0.10\,\text{m}\right) \;\wedge\; \left(\|\mathbf{p}_k - \mathbf{p}_{\text{goal}}\| > 0.25\,\text{m}\right)
$$
- **Interpretation:** If the drone covers less than $10\,\text{cm}$ in $2$ seconds while still outside the goal circle, it is trapped against a corner or dead-end, triggering `EMERGENCY_REVERSE`.
- **Implementation:** `progress_is_too_small()` in [pybullet/obstacle_avoidance.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/obstacle_avoidance.py#L517-L533).

---

### 7.4 Bio-Inspired Adaptive Neural Brain Dynamics

The Adaptive Neural Brain models the biological neural circuits observed in insects and flying organisms for obstacle avoidance (*Devos et al., 2018*).

```text
   Left Sensors                Front Sensor               Right Sensors
  [+8° to +90°]                    [0°]                   [-8° to -90°]
        │                           │                           │
        ▼                           ▼                           ▼
   Proximity I_L ──────────► [Frontal Cone] ◄────────── Proximity I_R
        │                           │                           │
        ▼ w_in                      ▼                           ▼ w_in
 ┌──────────────┐            Speed Neuron S             ┌──────────────┐
 │ Steering     │                   │                   │ Steering     │
 │ Neuron L     │◄──── w_cross ─────┼───── w_cross ────►│ Neuron R     │
 │ (self w_LL)  │                   │                   │ (self w_RR)  │
 └──────┬───────┘                   ▼                   └──────┬───────┘
        │                 v = V_MAX * s(S)                     │
        ▼                                                      ▼
  Activity a_L                                           Activity a_R
        │                                                      │
        └───────────────────────┬──────────────────────────────┘
                                ▼
               Steering: steer = a_L - a_R
               Heading:  d(psi)/dt = omega(steer, F, goal)
```

The recurrent steering network contains two coupled, continuous-time, leaky-integrator neurons:
- Left Steering Neuron $L$
- Right Steering Neuron $R$

#### Equation: Membrane Potential Leaky ODE
$$
\tau \frac{dx_L}{dt} = -x_L(t) + \text{drive}_L(t)
$$
$$
\tau \frac{dx_R}{dt} = -x_R(t) + \text{drive}_R(t)
$$
where $\tau = 0.18\,\text{s}$ is the membrane relaxation time constant. Discretized via forward Euler at step $\Delta t = 1/48\,\text{s}$:
$$
x_L[k+1] = x_L[k] + \frac{\Delta t}{\tau} \left(-x_L[k] + \text{drive}_L[k]\right)
$$
$$
x_R[k+1] = x_R[k] + \frac{\Delta t}{\tau} \left(-x_R[k] + \text{drive}_R[k]\right)
$$

#### Equation: Synaptic Drive Equations
$$
\text{drive}_L = w_{LL} a_L[k] + w_{\text{cross}} a_R[k] + w_{\text{in}} I_L + \text{bias} - \Delta_{\text{free}} - h_{\text{side}}
$$
$$
\text{drive}_R = w_{RR} a_R[k] + w_{\text{cross}} a_L[k] + w_{\text{in}} I_R + \text{bias} + \Delta_{\text{free}} + h_{\text{side}}
$$
- **Neuron Output & Normalized Firing Rate:**
$$
o_i = \tanh(x_i) \in (-1, 1), \quad a_i = \frac{o_i + 1}{2} \in [0, 1]
$$
- **Network Parameters:**
  - $w_{\text{self}, 0} = 1.40$: Baseline excitatory self-weight providing short-term turning memory.
  - $w_{\text{cross}} = -1.60$: Mutual inhibitory weight enforcing winner-take-all competitive selection.
  - $w_{\text{in}} = 4.00$: Sensory input gain amplifying obstacle detection.
  - $\text{bias} = -2.20$: Constant hyperpolarizing bias keeping neurons inactive ($a_i \approx 0$) in obstacle-free space.
  - $\Delta_{\text{free}} = w_{\text{free}} \left(\frac{\bar{d}_{\text{left}}}{R_s} - \frac{\bar{d}_{\text{right}}}{R_s}\right)$ with $w_{\text{free}} = 0.8$: Free-space peripheral bias.
  - $h_{\text{side}} = 0.08 \cdot \sigma_{\text{committed}}$ ($\pm 1$): Memory tie-breaker breaking dead-center balance.
- **Implementation:** [pybullet/brain/neural_brain.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/brain/neural_brain.py#L159-L167).

---

### 7.5 Synaptic Plasticity & Memory Hysteresis

In complex traps (such as U-shaped dead ends), standard reactive systems turn away from a wall, only to turn straight back toward the goal as soon as the obstacle exits the sensor cone, causing a fatal oscillation. The Adaptive Neural Brain solves this by dynamically adapting its self-excitatory weights $w_{LL}$ and $w_{RR}$ via Hebbian correlation learning.

#### Equation: Activity-Dependent Hebbian Plasticity
$$
\frac{dw_{ii}}{dt} = \mu \left(1 + 2F\right) \cdot I_i \cdot a_i - \gamma \left(w_{ii} - w_{\text{self}, 0}\right)
$$
In discrete simulation:
$$
w_{ii}[k+1] = \text{clip}\left(w_{ii}[k] + \Delta t \left[\mu (1 + 2F) I_i a_i - \gamma (w_{ii}[k] - w_{\text{self}, 0})\right], \; w_{\text{self}, 0}, \; w_{\text{self}, \text{max}}\right)
$$
- **Variables & Gains:**
  - $\mu = 0.90\,\text{s}^{-1}$: Hebbian growth rate (potentiates synapses during concurrent obstacle presence $I_i$ and turning activity $a_i$).
  - $\gamma = 0.15\,\text{s}^{-1}$: Exponential homeostatic relaxation rate returning weights to baseline $w_{\text{self}, 0} = 1.4$.
  - $w_{\text{self}, \text{max}} = 3.20$: Upper synaptic ceiling preventing self-latching runaway dynamics.
  - $F \in [0, 1]$: Frustration state scaling learning during stall events.
- **Physical Derivation of Hysteresis:**
  When $w_{ii}$ increases from $1.4$ toward $3.2$, the neuron's effective time-decay constant $\tau_{\text{eff}} = \frac{\tau}{1 - w_{ii} \frac{\partial a}{\partial x}}$ increases dramatically. The neuron remains active *even after the obstacle is no longer detected* ($I_i = 0$), forcing the drone to complete an extended detour until the obstacle is completely cleared.
- **Implementation:** [pybullet/brain/neural_brain.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/brain/neural_brain.py#L177-L180).

---

### 7.6 Frustration Dynamics & Deadlock Detour Vector

To guarantee escape from U-shaped traps and deep pockets, the brain tracks goal progress using a leaky integrator called **Frustration** ($F \in [0, 1]$).

#### Equation: Frustration Leaky Integrator
Let $d_{\text{goal}}(t) = \|\mathbf{p}(t) - \mathbf{p}_{\text{goal}}\|$ and $d_{\text{best}} = \min_{\tau \le t} d_{\text{goal}}(\tau)$.
$$
\frac{dF}{dt} = \begin{cases} +\alpha_{\text{rise}} = 0.35\,\text{s}^{-1}, & \text{if } t_{\text{no\_progress}} > 1.0\,\text{s} \;\wedge\; t_{\text{open}} < 1.0\,\text{s} \\ -\alpha_{\text{decay}} = 0.50\,\text{s}^{-1}, & \text{if } t_{\text{open}} \ge 1.0\,\text{s} \\ 0, & \text{otherwise} \end{cases}
$$

#### Equation: Adaptive Goal Detour & Heading Control
When $F > 0$, the drone rotates its goal attraction vector away from the obstacle by up to $\pm 80^\circ$:
$$
\theta_{\text{detour}} = \sigma_{\text{committed}} \cdot \theta_{\text{detour, max}} \cdot F(t) = \pm 80^\circ \cdot F(t)
$$
$$
e_{\text{goal}} = \text{wrap}\left(\text{atan2}(y_g - y, x_g - x) + \theta_{\text{detour}} - \psi\right)
$$
The heading angular velocity $\omega = \dot{\psi}$ command is synthesized as:
$$
\omega = \text{clip}\left(K_{\text{goal}} \cdot e_{\text{attract}} \cdot G_{\text{attract}} - K_{\text{turn}} \cdot (a_L - a_R) \cdot (1 - 0.5F), \; -\Omega_{\text{max}}, \; +\Omega_{\text{max}}\right)
$$
- **Variables & Gains:**
  - $K_{\text{goal}} = 2.20$, $K_{\text{turn}} = 2.60$, $\Omega_{\text{max}} = 2.60\,\text{rad/s}$.
  - $G_{\text{attract}} = \max(1 - a_{\text{avoid}}, 0.6F)$: Attenuates goal attraction when avoiding obstacles, but preserves turning drive when frustrated.
- **Implementation:** [pybullet/brain/neural_brain.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/brain/neural_brain.py#L182-L220).

---

### 7.7 Speed Neuron & Safe Braking Distance

Forward flight speed is regulated by the frontal range clearance $d_{\text{front}}$ to prevent high-speed collisions.

#### Equation: Normalized Frontal Clearance & Speed Law
$$
s = \text{clip}\left(\frac{d_{\text{front}} - d_{\text{stop}}}{d_{\text{slow}} - d_{\text{stop}}}, \; -1.0, \; 1.0\right)
$$
where $d_{\text{stop}} = 0.28\,\text{m}$ and $d_{\text{slow}} = 0.85\,\text{m}$.
$$
v = \begin{cases} V_{\text{max}} \cdot s \cdot \left(1 - 0.5 \frac{|\omega|}{\Omega_{\text{max}}}\right), & \text{if } s \ge 0 \text{ (forward flight)} \\ V_{\text{reverse}} \cdot s, & \text{if } s < 0 \;\wedge\; d_{\text{rear}} > 0.35\,\text{m} \text{ (backing out)} \\ 0, & \text{if } s < 0 \;\wedge\; d_{\text{rear}} \le 0.35\,\text{m} \text{ (trapped rear)} \end{cases}
$$
- **Variables:**
  - $V_{\text{max}} = 0.40\,\text{m/s}$: Maximum cruise speed.
  - $V_{\text{reverse}} = 0.18\,\text{m/s}$: Safe backing speed.
  - $\left(1 - 0.5 \frac{|\omega|}{\Omega_{\text{max}}}\right)$: Yaw-rate slowdown penalty (slows down during sharp turns).
- **Implementation:** [pybullet/brain/neural_brain.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/brain/neural_brain.py#L222-L235).

---

### 7.8 DSLPID Flight Control Mathematics

The high-level planner outputs a 2D velocity vector $\mathbf{v}_{\text{cmd}} = [v_x, v_y, 0]^T$. The low-level `DSLPIDControl` converts this into four individual motor RPMs ($n_0, n_1, n_2, n_3$) running at $48\,\text{Hz}$.

```text
 Desired State:               Current State:
 [p_target, v_des, yaw=0]     [p, q, v, w]
       │                            │
       └──────────────┬─────────────┘
                      ▼
         Position Error e_p & Velocity Error e_v
                      ▼
        Virtual Acceleration Command:
        a_des = K_p*e_p + K_d*e_v + K_i*int(e_p) + g*z_W
                      ▼
        Total Desired Thrust: T = m * ||a_des||
                      ▼
        Body z-Axis Extraction: z_B = a_des / ||a_des||
        Desired Rotation Matrix R_des = [x_B, y_B, z_B]
                      ▼
        Attitude & Angular Velocity PID -> Torques tau
                      ▼
        Quadrotor X-Frame Motor Mixing:
        [Omega_0^2, Omega_1^2, Omega_2^2, Omega_3^2]^T = M_mix^-1 * [T, tau_x, tau_y, tau_z]^T
```

#### Equation: Cascaded Position Control
Let $\mathbf{e}_p = \mathbf{p}_{\text{target}} - \mathbf{p}$ and $\mathbf{e}_v = \mathbf{v}_{\text{target}} - \mathbf{v}$.
$$
\mathbf{a}_{\text{des}} = \mathbf{K}_p \mathbf{e}_p + \mathbf{K}_d \mathbf{e}_v + \mathbf{K}_i \int_0^t \mathbf{e}_p(\tau) d\tau + g \mathbf{z}_W
$$
Total thrust along the drone's body z-axis $\mathbf{z}_B$ is:
$$
T_{\text{des}} = m \|\mathbf{a}_{\text{des}}\|_2
$$
The drone tilts its thrust vector along $\mathbf{z}_{B, \text{des}} = \frac{\mathbf{a}_{\text{des}}}{\|\mathbf{a}_{\text{des}}\|}$, yielding desired roll ($\phi_{\text{des}}$) and pitch ($\theta_{\text{des}}$) angles:
$$
\phi_{\text{des}} = \frac{1}{g} (a_{\text{des}, x} \sin\psi_{\text{des}} - a_{\text{des}, y} \cos\psi_{\text{des}})
$$
$$
\theta_{\text{des}} = \frac{1}{g} (a_{\text{des}, x} \cos\psi_{\text{des}} + a_{\text{des}, y} \sin\psi_{\text{des}})
$$

#### Equation: Motor Mixing for CF2X (X-Configuration)
The Crazyflie 2.X operates four fixed-pitch propellers mounted at $45^\circ$ from the body axes with arm length $L = 0.0397\,\text{m}$:
$$
\begin{bmatrix} T \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix} = \begin{bmatrix} k_f & k_f & k_f & k_f \\ -\frac{\sqrt{2}}{2} L k_f & -\frac{\sqrt{2}}{2} L k_f & \frac{\sqrt{2}}{2} L k_f & \frac{\sqrt{2}}{2} L k_f \\ -\frac{\sqrt{2}}{2} L k_f & \frac{\sqrt{2}}{2} L k_f & \frac{\sqrt{2}}{2} L k_f & -\frac{\sqrt{2}}{2} L k_f \\ -k_m & k_m & -k_m & k_m \end{bmatrix} \begin{bmatrix} \Omega_0^2 \\ \Omega_1^2 \\ \Omega_2^2 \\ \Omega_3^2 \end{bmatrix}
$$
- **Variables:**
  - $k_f = 3.16 \times 10^{-10}\,\text{N}\cdot\text{s}^2/\text{rad}^2$: Thrust coefficient.
  - $k_m = 7.94 \times 10^{-12}\,\text{N}\cdot\text{m}\cdot\text{s}^2/\text{rad}^2$: Drag moment coefficient.
  - $\Omega_i$: Angular velocity of rotor $i$ in $\text{rad/s}$ ($\text{RPM}_i = \frac{60}{2\pi} \Omega_i$).

---

### 7.9 Data-Driven Physics Discovery via SINDy & SINDYc

Rather than assuming ideal equations of motion, the project uses **Sparse Identification of Nonlinear Dynamics** (*Brunton, Proctor & Kutz, PNAS 2016*) to identify the governing differential equations directly from `.npz` flight logs.

#### Equation: SINDy Formulation & Candidate Library
Given $M$ snapshot measurements of states $\mathbf{X} \in \mathbb{R}^{M \times n}$ and control inputs $\mathbf{U} \in \mathbb{R}^{M \times m}$:
$$
\dot{\mathbf{X}} = \mathbf{\Theta}(\mathbf{X}, \mathbf{U}) \mathbf{\Xi}
$$
where $\mathbf{\Theta}(\mathbf{X}, \mathbf{U}) \in \mathbb{R}^{M \times p}$ is a library of candidate functions (polynomials, trigonometric terms, cross-products):
$$
\mathbf{\Theta}(\mathbf{X}, \mathbf{U}) = \begin{bmatrix} \mathbf{1} & \mathbf{X} & \mathbf{U} & \mathbf{X}^2 & \mathbf{X} \otimes \mathbf{U} & T \cos\phi\cos\theta & T\sin\theta & T\sin\phi & \dots \end{bmatrix}
$$

#### Equation: Sequentially Thresholded Least Squares (STLSQ)
The sparse coefficient matrix $\mathbf{\Xi} = [\boldsymbol{\xi}_1, \dots, \boldsymbol{\xi}_n]$ is solved per target column $\dot{\mathbf{x}}_j$:
1. Solve ridge regression on active column subset $\mathcal{A} \subseteq \{1, \dots, p\}$:
   $$
   \boldsymbol{\xi}_j = \left(\mathbf{\Theta}_{\mathcal{A}}^T \mathbf{\Theta}_{\mathcal{A}} + \alpha \mathbf{I}\right)^{-1} \mathbf{\Theta}_{\mathcal{A}}^T \dot{\mathbf{x}}_j
   $$
2. Sparsify by zeroing out coefficients below the threshold $\lambda$:
   $$
   \mathcal{A}_{k+1} = \left\{i \in \mathcal{A}_k : |\xi_{i, j}| \ge \lambda \max_l |\xi_{l, j}|\right\}
   $$
3. Repeat until the active set $\mathcal{A}$ converges.

- **Implementation:** `stlsq()` & `SINDy` class in [pybullet/sindy/sindy.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/sindy/sindy.py#L93-L188).

#### Equation: Discovered Physical Models (From Actual JSON Artifacts)
Running `python -m sindy.identify_dynamics` across the flight logs extracted the following equations:

1. **Model A: Translational Dynamics:**
   $$
   \frac{dv_z}{dt} = -9.796 + 1.170 \cdot T \cos(\text{roll}) \cos(\text{pitch}) \quad (R^2 = 0.9998)
   $$
   *Physical Discovery:* SINDy independently discovered Earth's gravity $g \approx 9.796\,\text{m/s}^2$ from raw simulation state logs without human intervention!
2. **Model B: Rotational Torque Mixing:**
   $$
   \begin{bmatrix} \dot{\omega}_x \\ \dot{\omega}_y \\ \dot{\omega}_z \end{bmatrix} = \begin{bmatrix} -63.87 & -63.70 & +63.34 & +64.23 \\ -63.14 & +63.59 & +63.13 & -63.58 \\ -35.70 & +36.82 & -36.55 & +35.44 \end{bmatrix} \begin{bmatrix} r_0 \\ r_1 \\ r_2 \\ r_3 \end{bmatrix} \quad (R^2 > 0.998)
   $$
   *Physical Discovery:* SINDy reconstructed the exact quadrotor X-frame geometry and rotor rotation directions (motors 0 & 2 CCW, motors 1 & 3 CW).
3. **Model C: Closed-Loop Command Response (SINDYc):**
   $$
   \frac{dv_x}{dt} = +0.2088 - 3.459 v_x + 8.762 v_{cx} - 1.641 v_x^2 - 3.507 v_x v_y + \dots
   $$
   $$
   \frac{dv_y}{dt} = -0.0804 + 0.2534 v_x - 3.300 v_y + 9.426 v_{cy} + \dots
   $$
   *Physical Discovery:* Models the entire PID + closed-loop airframe as a compact low-order polynomial filter mapping command velocity $[v_{cx}, v_{cy}]$ to drone acceleration $[\dot{v}_x, \dot{v}_y]$.

---

<a id="8-test-scenarios-1-to-10"></a>
## 8. Test Scenarios (1 to 10)

The system is evaluated against 10 distinct, reproducible obstacle scenarios configured in `create_test_scenario()` in [pybullet/obstacle_avoidance.py](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/obstacle_avoidance.py#L221-L334):

| ID | Scenario Name | Geometry & Placement | Difficulty | Expected Flight Behavior |
|:---:|---|---|:---:|---|
| **1** | **Direct Frontal Obstacle** | One $0.30\,\text{m}$ cube at $[2.5, 0.0, 1.0]$ | Easy | Detects frontal block, selects left or right bypass, smoothly returns to centerline. |
| **2** | **Offset Obstacle Left** | One $0.30\,\text{m}$ cube at $[2.5, +0.18, 1.0]$ | Easy | Detects open right flank, cleanly bypasses without emergency reversals. |
| **3** | **Offset Obstacle Right** | One $0.30\,\text{m}$ cube at $[2.5, -0.18, 1.0]$ | Easy | Detects open left flank, cleanly bypasses on the left side. |
| **4** | **Narrow Passage / Slit** | 4 cubes forming a narrow corridor slit at $x=2.5$ and $x=3.0$ with width $0.76\,\text{m}$ | Medium | Traverses the narrow slit; triggers coordinated lateral micro-adjustments. |
| **5** | **Dead-End / U-Trap** | 5 cubes forming a concave enclosure at $x=2.0$ to $2.5$ | Hard | Detects trapped enclosure, initiates `EMERGENCY_REVERSE`, turns around perimeter. |
| **6** | **Staggered Forest** | 5 vertical cylinders ($r=0.15\,\text{m}, h=1.6\,\text{m}$) staggered along $x=1.8$ to $4.8$ | Hard | Executes continuous slalom weaving between obstacles. |
| **7** | **Dynamic Moving Obstacle** | Cube at $x=2.8$ sliding along $y(t) = 0.75 \sin(1.5 t)$ | Dynamic | Predicts time-varying clearance, decelerates, and overtakes the moving obstacle. |
| **8** | **3D Cluttered Environment** | Barriers at varying heights: low ($z=0.55\,\text{m}$), high overhead ($z=1.45\,\text{m}$) | 3D Expert | Adjusts target altitude from $1.0\,\text{m}$ to $1.35\,\text{m}$ to climb over low barriers. |
| **9** | **Dynamic Lateral Intruder** | Cube at $x=3.0$ rapidly oscillating along $y(t) = 1.20 \sin(2.5 t)$ | Dynamic Expert | Detects closing lateral intrusion, brakes, and clears the path. |
| **10** | **Circular Dead-End Ring** | 54 cubes forming a continuous circular enclosure ($r=2.0\,\text{m}$) | Extreme Trap | Ascends to $1.50\,\text{m}$ to fly over the ring wall and reach the goal. |

---

<a id="9-prerequisites"></a>
## 9. Prerequisites

Before installing the project, verify that your environment has:
- **Operating System:** Windows 10/11, Ubuntu 20.04/22.04 LTS, or macOS 12+ (Apple Silicon or Intel).
- **Python Version:** Python 3.10 or 3.11 (64-bit).
- **C++ Build Tools:**
  - *Windows:* Microsoft C++ Build Tools (Visual Studio 2019/2022 with Desktop C++ workload) required for compiling PyBullet C extensions.
  - *Linux:* `sudo apt update && sudo apt install build-essential python3-dev`
- **Git:** Git installed and accessible from terminal.

---

<a id="10-installation-and-setup"></a>
## 10. Installation and Setup

Follow these ordered steps to set up the environment:

### Step 1: Clone the Repository
```bash
git clone https://github.com/Varlakshmi19/Drones_S5_AB3_Obstacle_Avoidance_Drone.git
cd Drones_S5_AB3_Obstacle_Avoidance_Drone
```

### Step 2: Create and Activate a Dedicated Virtual Environment

- **On Windows (PowerShell):**
  ```powershell
  python -m venv venv
  .\venv\Scripts\Activate.ps1
  ```

- **On Windows (Command Prompt `cmd`):**
  ```cmd
  python -m venv venv
  venv\Scripts\activate.bat
  ```

- **On Linux / macOS:**
  ```bash
  python3 -m venv venv
  source venv/bin/activate
  ```

### Step 3: Upgrade Package Managers
```bash
python -m pip install --upgrade pip setuptools wheel
```

### Step 4: Install Dependencies
Install `gym-pybullet-drones` from the official repository along with supporting numerical tools:
```bash
pip install -r requirements.txt
pip install pybullet numpy scipy matplotlib
```

*(Optional)* Verify that the PyBullet installation succeeds:
```bash
python -c "import pybullet as p; client = p.connect(p.DIRECT); print('PyBullet initialized successfully, client ID:', client); p.disconnect(client)"
```

---

<a id="11-how-to-run-the-project"></a>
## 11. How to Run the Project

All commands should be executed from the `pybullet/` directory (or workspace root by referencing `pybullet/`).

```bash
cd pybullet
```

### 11.1 Running the Reactive FSM Controller

The reactive FSM controller is contained in `obstacle_avoidance.py`. You can select any scenario (1 through 10) directly via command-line arguments:

```bash
# Run Scenario 1 (Direct Frontal Obstacle) with 3D GUI
python obstacle_avoidance.py 1

# Run Scenario 4 (Narrow Passage / Slit)
python obstacle_avoidance.py 4

# Run Scenario 5 (Dead-End U-Trap)
python obstacle_avoidance.py 5

# Run Scenario 7 (Dynamic Moving Obstacle)
python obstacle_avoidance.py 7

# Run Scenario 10 (Circular Dead-End Ring)
python obstacle_avoidance.py 10
```

Alternatively, open `obstacle_avoidance.py` in an editor and change the top variable:
```python
TEST_SCENARIO = 5   # Change to any value between 1 and 10
```
Then run:
```bash
python obstacle_avoidance.py
```

### 11.2 Running the Adaptive Neural Brain

The bio-inspired neural network controller is executed via `brain_avoidance.py`:

```bash
# 1. Run Scenario 1 with full PyBullet visualization GUI
python brain_avoidance.py --scenario 1

# 2. Run Scenario 5 (U-Trap) with GUI and observe Hebbian learning in terminal
python brain_avoidance.py --scenario 5

# 3. Run Scenario 4 and record a telemetry log (saved to logs/brain_s4.npz)
python brain_avoidance.py --scenario 4 --log

# 4. Benchmark all Scenarios (1 through 6) in headless mode and print comparative table
python brain_avoidance.py --all --headless
```

### 11.3 Running SINDy Dynamic Identification

To run sparse regression on the simulation logs and regenerate the identified governing equations:

```bash
# Run SINDy across all logs, using Scenario 4 as the held-out validation test
python -m sindy.identify_dynamics

# Hold out Scenario 6 as the validation test
python -m sindy.identify_dynamics --test 6

# View the generated identification models
type models\sindy_translational.json
type models\sindy_rotational.json
type models\sindy_closed_loop.json
```

All generated parity plots and RK4 integration rollouts will be saved to `results/sindy/`.

---

<a id="12-example-usage--step-by-step-walkthrough"></a>
## 12. Example Usage & Step-by-Step Walkthrough

Below is an annotated terminal run of **Scenario 5 (U-Trap Dead-End)** using `obstacle_avoidance.py`:

```text
============================================================
RUNNING SCENARIO 5: Dead-End / U-Trap
Difficulty: Hard
Description: Enclosed U-shaped obstacle arrangement blocking forward, left, and right directions.
Expected: Drone detects trapped condition, initiates emergency reversal, turns toward open space, and navigates around perimeter.
============================================================

Scenario=5 | Mode=FORWARD            | Side=None  | Position=(0.00, 0.00) | Front=1.60 | Left=1.60 | Right=1.60 | Goal distance=6.00
Scenario=5 | Mode=FORWARD            | Side=None  | Position=(0.73, 0.00) | Front=1.45 | Left=1.60 | Right=1.60 | Goal distance=5.27
Scenario=5 | Mode=AVOID              | Side=LEFT  | Position=(1.75, 0.01) | Front=0.57 | Left=0.45 | Right=0.46 | Goal distance=4.25
Scenario=5 | Mode=EMERGENCY_REVERSE  | Side=LEFT  | Position=(1.92, 0.08) | Front=0.28 | Left=0.29 | Right=0.31 | Goal distance=4.08
Scenario=5 | Mode=EMERGENCY_REVERSE  | Side=LEFT  | Position=(1.71, 0.06) | Front=0.49 | Left=0.48 | Right=0.49 | Goal distance=4.29
Scenario=5 | Mode=AVOID              | Side=RIGHT | Position=(1.82, -0.42)| Front=0.98 | Left=0.62 | Right=1.40 | Goal distance=4.20
Scenario=5 | Mode=RETURN             | Side=RIGHT | Position=(3.45, -0.85)| Front=1.60 | Left=0.72 | Right=1.60 | Goal distance=2.69
Scenario=5 | Mode=FORWARD            | Side=RIGHT | Position=(5.20, -0.06)| Front=1.60 | Left=1.60 | Right=1.60 | Goal distance=0.80

Drone successfully reached the goal.

========================================
TEST SCENARIO: 5
RESULT: SUCCESS
CONTROL STEPS: 693
COMPLETION TIME: 18.45 s
FINAL POSITION: (5.833, -0.171, 1.001)
PATH LENGTH: 11.034 m
COLLISION STEPS: 0
AVOIDANCE EVENTS: 4
EMERGENCY REVERSALS: 3
========================================
```

**Interpretation:**
1. **Entry:** Drone advances forward at $0.35\,\text{m/s}$ until reaching $x = 1.75\,\text{m}$.
2. **Trap Detection:** At $x = 1.92\,\text{m}$, the front obstacle drops to $0.28\,\text{m} < D_{\text{emergency}} = 0.30\,\text{m}$, while both lateral sensors register walls ($< 0.48\,\text{m}$).
3. **Emergency Reverse:** The drone reverses backward for $1.2\,\text{s}$ at $0.22\,\text{m/s}$, opening up clearance.
4. **Side Inversion:** The controller enforces `force_opposite_side = True`, switching from LEFT to RIGHT.
5. **Perimeter Bypass:** The drone navigates around the outer perimeter of the trap and safely navigates to the goal $(6.0, 0.0, 1.0)$ with zero collisions.

---

<a id="13-code-and-module-explanation"></a>
## 13. Code and Module Explanation

### 1. `obstacle_avoidance.py`
The standalone reactive FSM controller.
- **`create_test_scenario(scenario)`:** Spawns PyBullet bodies (cubes, cylinders, dynamic obstacles) for scenarios 1–10.
- **`update_dynamic_obstacles(sim_time)`:** Evaluates periodic kinematics for moving obstacles.
- **`read_sensors(current_position)`:** Executes 6 directional ray tests and returns distance maps.
- **`calculate_side_scores()` & `choose_avoidance_side()`:** Computes weighted clearance scores and applies dead-band hysteresis.
- **`make_velocity_command()`:** Computes horizontal 2D velocity vectors based on current FSM state.
- **`run()`:** Instantiates `CtrlAviary`, executes the 48 Hz control loop, queries `DSLPIDControl`, tracks metrics, and checks contact collisions.

### 2. `brain/neural_brain.py`
The bio-inspired RNN navigation engine.
- **`BrainConfig`:** Dataclass containing parameters for reaction bubble, cruise speeds, membrane time constants, Hebbian learning rate, and detour limits.
- **`AdaptiveNeuralBrain`:**
  - `ray_world_angles()`: Transforms body-relative ray angles to global world frame.
  - `step(position_xy, distances)`: Main algorithmic pipeline: sensory preprocessing $\to$ recurrent membrane integration $\to$ Hebbian weight update $\to$ frustration monitoring $\to$ goal detour calculation $\to$ speed modulation.

### 3. `brain_avoidance.py`
Integrator connecting the neural brain to PyBullet.
- Replaces the FSM logic with `brain.step()`.
- Supports CLI flags (`--scenario`, `--all`, `--headless`, `--log`).
- Saves structured telemetry arrays to `logs/brain_s*.npz`.

### 4. `sindy/sindy.py`
Lightweight, dependency-free mathematical equation discovery engine.
- **`PolynomialLibrary`:** Constructs polynomial candidate feature matrix up to degree $d$ with optional physics-informed nonlinear terms.
- **`stlsq()`:** Implements Sequentially Thresholded Least Squares with internal column normalization and ridge regularization.
- **`SINDy`:** High-level estimator exposing `.fit()`, `.predict()`, `.score()` ($R^2$), and `.simulate()` (RK4 numerical integrator).

### 5. `sindy/identify_dynamics.py`
Data processing and discovery script.
- Loads flight logs from `logs/*.npz`.
- Constructs feature matrices for translational motion, rotational torques, and closed-loop control.
- Exports discovered symbolic models to `models/*.json`.
- Evaluates $R^2$ scores on held-out test runs and renders parity comparison plots to `results/sindy/`.

---

<a id="14-datasets-logs--input-specifications"></a>
## 14. Datasets, Logs & Input Specifications

The repository contains pre-recorded 48 Hz simulation logs in `pybullet/logs/` stored as compressed NumPy archives (`.npz`).

### Array Structure of `.npz` Telemetry Files
Each file contains:
- `data`: 2D float array of shape $(N_{\text{steps}}, 50)$ containing 50 synchronized telemetry variables:

| Column Index | Field Name | Units | Description |
|---|---|---|---|
| `0` | `t` | $\text{s}$ | Simulation timestamp |
| `1:4` | `x, y, z` | $\text{m}$ | Drone 3D position in world frame |
| `4:8` | `qx, qy, qz, qw` | &mdash; | Orientation unit quaternion |
| `8:11` | `roll, pitch, yaw` | $\text{rad}$ | Euler attitude angles |
| `11:14` | `vx, vy, vz` | $\text{m/s}$ | Linear velocities in world frame |
| `14:17` | `wx, wy, wz` | $\text{rad/s}$ | Angular velocities in body frame |
| `17:21` | `rpm0..rpm3` | $\text{RPM}$ | Individual rotor rotational speeds |
| `21:24` | `vcx, vcy, vcz` | $\text{m/s}$ | Desired velocity commands |
| `24:27` | `tx, ty, tz` | $\text{m}$ | Target position lookahead |
| `27:37` | `I_L, I_R, o_L, o_R, w_LL, w_RR, F, psi, omega, speed` | Various | Internal neural states |
| `37:49` | `ray_-90 .. ray_180` | $\text{m}$ | 12 body-frame range sensor readings |

- `columns`: String list of column header names.
- `dt`: Control timestep ($1/48 \approx 0.02083\,\text{s}$).
- `scenario`: Integer scenario identifier (1 to 10).

---

<a id="15-experimental-results-and-outputs"></a>
## 15. Experimental Results and Outputs

### 15.1 Reactive FSM Scenario Benchmark

The quantitative performance across the six primary standardized scenarios is summarized below (all verified against actual simulation logs):

| Scenario | Scenario Name | Result | Control Steps | Time (s) | Final Position $(x, y, z)$ | Path Length (m) | Collisions | Avoid Events | Emergency Reversals |
|:---:|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **1** | Direct Frontal Obstacle | **SUCCESS** | 356 | 10.39 | $(5.821, +0.158, 1.001)$ | 6.363 | **0** | 1 | 0 |
| **2** | Offset Obstacle Left | **SUCCESS** | 307 | 9.38 | $(5.772, -0.026, 1.001)$ | 5.846 | **0** | 0 | 0 |
| **3** | Offset Obstacle Right | **SUCCESS** | 306 | 9.36 | $(5.760, -0.000, 1.001)$ | 5.809 | **0** | 0 | 0 |
| **4** | Narrow Passage / Slit | **SUCCESS** | 746 | 18.95 | $(5.848, -0.180, 1.001)$ | 11.155 | **0** | 4 | 4 |
| **5** | Dead-End U-Trap | **SUCCESS** | 693 | 18.45 | $(5.833, -0.171, 1.001)$ | 11.034 | **0** | 4 | 3 |
| **6** | Staggered Forest | **SUCCESS** | 396 | 11.28 | $(5.953, +0.222, 1.001)$ | 7.081 | **0** | 2 | 0 |

#### Overall Benchmark Summary:
- **Success Rate:** $\mathbf{100\%}$ ($6/6$ missions completed within tolerance).
- **Collision Rate:** $\mathbf{0}$ recorded collision steps across all runs.
- **Average Completion Time:** $12.97\,\text{seconds}$.
- **Average Path Length:** $7.881\,\text{meters}$.

---

### 15.2 Scenario Visualizations & Trajectories

Below are simulation visual captures recorded during test executions:

#### Scenario 1: Direct Obstacle
The drone detects the frontal block, turns around the obstacle, and returns to the path.
<p align="center">
  <img src="https://github.com/user-attachments/assets/b8d7cb6e-5c22-4f45-809e-bd42f5700f91" alt="Scenario 1 Simulation" width="700">
</p>

#### Scenario 2: Left-Side Obstacle
The drone detects asymmetric clearance on the right side and navigates toward the goal.
<p align="center">
  <img src="https://github.com/user-attachments/assets/da31bd8b-9ef1-4158-b463-632f7f2482fb" alt="Scenario 2 Simulation" width="700">
</p>

#### Scenario 3: Right-Side Obstacle
The drone bypasses on the left side with minimal deviation.
<p align="center">
  <img src="https://github.com/user-attachments/assets/258e9c0d-d51f-46f2-a288-a71aa64d2ef8" alt="Scenario 3 Simulation" width="700">
</p>

#### Scenario 4: Narrow Passage
The drone executes sequential avoidance maneuvers and reversals to negotiate the slit.
<p align="center">
  <img src="https://github.com/user-attachments/assets/cbad036e-8fd1-48aa-9229-10cfe3defa23" alt="Scenario 4 Simulation" width="700">
</p>

#### Scenario 5: Dead-End U-Trap
The drone detects a pocket, executes an emergency reverse, and navigates around the perimeter.
<p align="center">
  <img src="https://github.com/user-attachments/assets/1d616558-eade-4a0b-94c2-10706f907fbb" alt="Scenario 5 Simulation" width="700">
</p>

#### Scenario 6: Staggered Multi-Obstacle Forest
The drone weaves through multiple cylindrical obstacles in a continuous slalom path.
<p align="center">
  <img src="https://github.com/user-attachments/assets/28c8d82a-d4e0-4d4d-b766-6632a0193b26" alt="Scenario 6 Simulation" width="700">
</p>

---

### 15.3 SINDy Discovered Mathematical Models & Accuracy

The SINDy discovery pipeline evaluated on held-out flight logs produced high predictive accuracy:

#### SINDy Discovered Model Verification

```text
 ┌────────────────────────────────────────────────────────────────────────┐
 │ Model A: Translational Physics                                         │
 │ - Target: dv_z/dt                                                      │
 │ - Discovered: -9.796 + 1.170 * T cos(roll)cos(pitch)                   │
 │ - Held-Out R^2: 0.99978 (Accurately identifies gravity: g = 9.80 m/s^2)│
 ├────────────────────────────────────────────────────────────────────────┤
 │ Model B: Rotational Mixing Matrix                                      │
 │ - Discovered: dw/dt = M_rot * [r0, r1, r2, r3]^T                       │
 │ - Held-Out R^2:                                                        │
 │     dw_x/dt:  0.99813                                                  │
 │     dw_y/dt:  0.99974                                                  │
 │     dw_z/dt:  0.99885                                                  │
 ├────────────────────────────────────────────────────────────────────────┤
 │ Model C: Closed-Loop SINDYc Validation on Held-Out Scenario 4          │
 │ - End-Point Position Drift Error: 0.021 m (over 5.74 m path)           │
 │ - Velocity RMSE:                  0.075 m/s                            │
 └────────────────────────────────────────────────────────────────────────┘
```

The identified models and figures are saved under [pybullet/results/sindy/](file:///c:/Users/varsh/Downloads/gitDrones/Drones_S5_AB3_Obstacle_Avoidance_Drone/pybullet/results/sindy).

---

<a id="16-testing-and-validation"></a>
## 16. Testing and Validation

You can validate each component of the codebase with the following tests:

### Test 1: Validate PyBullet Engine and FSM
```bash
python obstacle_avoidance.py 1
```
- **Expected Outcome:** A PyBullet GUI window opens displaying the drone, the red target marker at `(6.0, 0.0, 1.0)`, and one red obstacle cube. The drone navigates around the obstacle and prints `RESULT: SUCCESS`.

### Test 2: Validate Neural Brain Headless Execution
```bash
python brain_avoidance.py --scenario 2 --headless
```
- **Expected Outcome:** Runs without launching a graphical window, completing within ~2 seconds and outputting summary metrics with `result: SUCCESS`.

### Test 3: Validate SINDy Identification Pipeline
```bash
python -m sindy.identify_dynamics
```
- **Expected Outcome:** Loads `logs/brain_s*.npz`, prints the discovered mathematical equations for models A, B, and C, and verifies that held-out $R^2$ exceeds $0.98$.

---

<a id="17-troubleshooting-guide"></a>
## 17. Troubleshooting Guide

| Issue / Symptom | Possible Root Cause | Diagnostic Step | Solution |
|---|---|---|---|
| `ModuleNotFoundError: No module named 'gym_pybullet_drones'` | Environment missing the gym drone package. | Run `python -c "import gym_pybullet_drones"` in terminal. | Re-run: `pip install git+https://github.com/utiasDSL/gym-pybullet-drones.git` |
| `AttributeError: module 'pybullet' has no attribute 'rayTestBatch'` | Outdated PyBullet build installed. | Check version: `python -c "import pybullet; print(pybullet.__file__)"` | Upgrade PyBullet: `pip install --upgrade pybullet` |
| PyBullet GUI window crashes or freezes upon opening | OpenGL graphics driver incompatibility on Windows/Linux. | Check terminal for display driver errors. | Update your GPU display drivers or run with `--headless` flag. |
| Drone hits obstacle in Scenario 5 (U-Trap) | Sensor offset too small causing self-ray collision. | Verify `RAY_START_OFFSET` in `brain_avoidance.py` is $\ge 0.10\,\text{m}$. | Ensure `RAY_START_OFFSET = 0.10` and `SENSOR_RANGE = 1.6`. |
| SINDy discovers zero terms (`dX/dt = 0`) | Threshold $\lambda$ set too high for unscaled data. | Inspect `identify_dynamics.py` threshold settings. | Use `THRUST_SCALE = 1e-8` and ensure threshold is between $0.02$ and $0.05$. |

---

<a id="18-limitations"></a>
## 18. Limitations

1. **Simulation vs. Real-World Physics:** Evaluated in PyBullet. Real-world physical flights will encounter aerodynamic ground effect, battery voltage sag, motor response latency, and turbulent wind gusts.
2. **Simplified Sensor Perception:** Uses geometric ray-casting rather than noisy physical LiDAR point clouds or stereo depth cameras.
3. **Planar-Biased Trajectories:** The FSM operates primarily in the 2D horizontal plane at constant altitude ($z = 1.0\,\text{m}$), except in Scenario 8 and 10 where vertical maneuvers are pre-programmed.
4. **Local Minimum Horizon:** While the bio-inspired brain's frustration module successfully escapes U-traps and circular rings, it does not guarantee globally optimal path lengths compared to full A\* or RRT\* search over a known map.

---

<a id="19-future-improvements"></a>
## 19. Future Improvements

- **Physical Hardware Deployment:** Transfer the bio-inspired RNN controller onto an actual Bitcraze Crazyflie 2.1 equipped with the Multi-ranger deck (VL53L1X time-of-flight sensors).
- **Depth Camera Perception:** Integrate depth images from a simulated RGB-D sensor with a lightweight convolutional feature extractor feeding the neural brain.
- **Model Predictive Path Integral (MPPI) Control:** Use the SINDy-discovered closed-loop dynamics model as the internal transition model for real-time model predictive control.
- **Full 3D Volumetric Avoidance:** Expand the ray fan into a 3D spherical Fibonacci lattice for autonomous multi-altitude urban canyon navigation.

---

<a id="20-contributing"></a>
## 20. Contributing

Contributions, issues, and feature suggestions are welcome:
1. Fork the repository on GitHub.
2. Create a feature branch (`git checkout -b feature/dynamic-3d-avoidance`).
3. Commit your changes (`git commit -m "Add 3D spherical perception fan"`).
4. Push to the branch (`git push origin feature/dynamic-3d-avoidance`).
5. Open a Pull Request with a clear description of the modifications and test results.

---

<a id="21-license-and-acknowledgements"></a>
## 21. License and Acknowledgements

This project was developed as part of the **Data-Driven Control of Drones** curriculum at **Amrita Vishwa Vidyapeetham**, School of Artificial Intelligence, Coimbatore Campus.

### Acknowledgements & External References
- **Base Research Paper:** Arne Devos, Emad Ebeid, and Poramate Manoonpong, *"Development of Autonomous Drones for Adaptive Obstacle Avoidance in Real World Environments"*, 21st Euromicro Conference on Digital System Design (DSD), 2018.
- **Gym Simulation Suite:** Jacopo Panerati, Hehui Zheng, SiQi Zhou, James Xu, Amanda Prorok, and Angela P. Schoellig, *"Learning to Fly—a Gym Environment with PyBullet Physics for Reinforcement Learning in Autonomous Robotics"*, IEEE/RSJ IROS, 2021. [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones).
- **SINDy Algorithm:** Steven L. Brunton, Joshua L. Proctor, and J. Nathan Kutz, *"Discovering governing equations from data by sparse identification of nonlinear dynamical systems"*, PNAS, 113(15), 2016.

---

<a id="22-references"></a>
## 22. References

1. Devos, A., Ebeid, E., & Manoonpong, P. (2018). *Development of Autonomous Drones for Adaptive Obstacle Avoidance in Real World Environments*. In 2018 21st Euromicro Conference on Digital System Design (DSD) (pp. 535-542). IEEE. [DOI: 10.1109/DSD.2018.00094](https://ieeexplore.ieee.org/document/8491889).
2. Panerati, J., et al. (2021). *Learning to Fly—a Gym Environment with PyBullet Physics for Reinforcement Learning in Autonomous Robotics*. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS).
3. Brunton, S. L., Proctor, J. L., & Kutz, J. N. (2016). *Discovering governing equations from data by sparse identification of nonlinear dynamical systems*. Proceedings of the National Academy of Sciences (PNAS), 113(15), 3932-3937.
4. Mellinger, D., & Kumar, V. (2011). *Minimum snap trajectory generation and control for quadrotors*. IEEE International Conference on Robotics and Automation (ICRA).
