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
9. [Simulation Execution Guide & Command-Line Reference](#9-simulation-execution-guide--command-line-reference)
10. [Mathematical & Algorithmic Formulations](#10-mathematical--algorithmic-formulations)
11. [Project Codebase Structure](#11-project-codebase-structure)

---

## 1. Project Overview & Objectives

This project implements an autonomous quadcopter obstacle avoidance system simulated inside the **MuJoCo physics engine**. It bridges the gap between theoretical path planning and realistic physical dynamics by simulating true rigid-body multi-rotor aerodynamics, motor delays, sensor measurement noise, and stochastic moving obstacles.

### Key Capabilities:
* **Physics Fidelity**: Native MuJoCo 6-DOF quadcopter model (Crazyflie 2.x scale, mass = 47.2 g) simulated with Runge-Kutta 4th-order (RK4) integration at 500 Hz physics and 48 Hz control frequency.
* **Kalman Filter Tracking**: 6-DOF Constant-Velocity Kalman Filter tracks moving obstacles with noisy LiDAR range measurements, achieving **~50% noise variance reduction** and projecting future obstacle trajectories 1.0 second into the future.
* **Stochastic Dynamic Obstacles & Noise Control**: Moving obstacles do not follow predictable deterministic paths; they are driven by an **Ornstein-Uhlenbeck stochastic drift process** with random phase, frequency, and wandering drift. Operators can toggle between ** No Noise**, **Low Noise** (default), and ** High Noise** profiles in real time.
* **6 Navigation Strategies**:
  1. **Reactive Sector State Machine** (with closing-velocity evasion and hysteresis side-switching)
  2. **Artificial Potential Fields (APF)** (continuous potential gradients)
  3. **Dynamic Window Approach (DWA)** (velocity-space rollout optimization)
  4. **3D Volumetric Avoidance** (vertical overflight of walls and ducking under overhangs)
  5. **SINDy-Enhanced DWA** (data-driven discovered non-linear quadcopter dynamics)
  6. **Adaptive Neural Brain** (neural weight updates)
* **Dead-End & Trap Solving**: Autonomous 360° LiDAR perimeter inspection that detects concave U-traps and circular dead-ends, initiating 3D vertical climb-overs or finding horizontal escape gaps.
* **Interactive Tooling & Dashboard**: Real-time Web Dashboard (`http://127.0.0.1:8080`) with mouse/keyboard obstacle placement, live 2D/3D dual-viewport, tri-state **Noise Level Control** buttons, live LiDAR radar, Kalman trajectory curves, and desktop Tkinter visual arena editor.

> [!TIP]
> **Looking for instructions on how to run?** Jump directly to [Section 9: Simulation Execution Guide & Command-Line Reference](#9-simulation-execution-guide--command-line-reference) for the complete step-by-step setup, Web Dashboard, 3D viewer, benchmark commands, and noise control guide.


---

## 2. End-to-End System Architecture

The software architecture follows a modular sense-estimate-plan-control feedback loop:

![flow](./assets/MujocoFlow.drawio.png)

---

## 3. Physical Platform & Simulation Engine

### Drone Physical Parameters (Crazyflie 2.x Scale)
The quadcopter is modeled in `models/drone.xml` with high physical precision:
* **Total Mass**: 0.0472 kg (47.2 g)
* **Arm Length**: 0.040 m (4.0 cm)
* **Inertia Matrix**:
  - `I_xx = 1.4 × 10⁻⁵ kg·m²`
  - `I_yy = 1.4 × 10⁻⁵ kg·m²`
  - `I_zz = 2.17 × 10⁻⁵ kg·m²`
* **Rotor Count**: 4 motors in an "X" configuration:
  * Motor 1: Front-Left (Counter-Clockwise)
  * Motor 2: Front-Right (Clockwise)
  * Motor 3: Rear-Left (Clockwise)
  * Motor 4: Rear-Right (Counter-Clockwise)
* **Maximum Thrust**: 0.155 N per rotor (0.62 N total thrust ≈ 1.34× thrust-to-weight ratio).
* **Torque-to-Thrust Ratio**: `k_m = 0.00596 m`.

### Physics Time-Stepping:
* **Physics Integration Step**: `Δt_physics = 0.002 s` (500 Hz).
* **Control Step**: `Δt_control = 1/48 s ≈ 0.02083 s` (48 Hz).
* Each control step runs **10 physics sub-steps** using Runge-Kutta 4th order (RK4) integration for numerical stability and realistic contact dynamics.

---

## 4. Perception & Sensor Array

The drone utilizes native MuJoCo raycasting (`mujoco.mj_ray`) configured to ignore the drone's own body geoms while reflecting off ground, curbs, and static/moving obstacles.

### Beam Layout:
| Sensor Name | Angle Relative to Forward | Purpose |
| :--- | :---: | :--- |
| **Front** | 0° | Forward collision detection in line of sight |
| **Left** | +35° | Front-left flank detection (turns & corridor walls) |
| **Far Left** | +70° | Oblique lateral clearance |
| **Side Left** | +90° | Full lateral clearance while overtaking |
| **Right** | -35° | Front-right flank detection |
| **Far Right** | -70° | Oblique lateral clearance |
| **Side Right** | -90° | Full lateral clearance while overtaking |
| **Rear** | 180° | Clearance monitoring during emergency reversals |
| **Up** | +Z (Vertical) | Overflight clearance for 3D climb-overs |
| **Down** | -Z (Vertical) | Ground proximity and altitude holding |

### Range & Safety Thresholds:
* **Maximum Range**: 2.00 m (readings beyond this register as clear space).
* **Caution Distance (`d_danger`)**: 0.75 m (beams turn **Orange**, triggers avoidance steering).
* **Emergency Distance (`d_emerg`)**: 0.30 m (beams turn **Red**, triggers immediate reverse thrust).

---

## 5. Kalman Filter Tracking & Trajectory Prediction

When obstacles move dynamically in real-world conditions, rangefinder hits exhibit measurement noise and jitter. Without filtering, raw velocity estimation amplifies noise, causing flight instability.

### State Space Formulation
The tracking system uses a 6-DOF Constant-Velocity (CV) Kalman Filter implemented in `perception/kalman_filter.py`:

```math
\mathbf{x} = \begin{bmatrix} x & y & z & v_x & v_y & v_z \end{bmatrix}^T \in \mathbb{R}^6
```

#### Mathematical Term Breakdown for State Vector:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>x</b> | Complete dynamic state vector | ℝ<sup>6</sup> | [m, m, m, m/s, m/s, m/s]<sup>T</sup> | The complete 6-dimensional kinematic state of the tracked obstacle. |
| x | Longitudinal Cartesian position | ℝ (Scalar) | m (meters) | Position along the inertial world X-axis (primary flight corridor direction). |
| y | Lateral Cartesian position | ℝ (Scalar) | m (meters) | Position along the inertial world Y-axis (lateral cross-track axis). |
| z | Vertical Cartesian altitude | ℝ (Scalar) | m (meters) | Altitude above ground along the inertial world Z-axis. |
| v<sub>x</sub> | Longitudinal linear velocity | ℝ (Scalar) | m/s (meters / second) | Speed of the obstacle along the world X-axis. |
| v<sub>y</sub> | Lateral linear velocity | ℝ (Scalar) | m/s (meters / second) | Speed of the obstacle along the world Y-axis (e.g. lateral sliding intruder speed). |
| v<sub>z</sub> | Vertical linear velocity | ℝ (Scalar) | m/s (meters / second) | Speed of the obstacle along the vertical Z-axis. |
| <b>p</b> | 3D Spatial position sub-vector | ℝ<sup>3</sup> | m | Block [x, y, z]<sup>T</sup> denoting physical 3D center-of-mass coordinates. |
| <b>v</b> | 3D Linear velocity sub-vector | ℝ<sup>3</sup> | m/s | Block [v<sub>x</sub>, v<sub>y</sub>, v<sub>z</sub>]<sup>T</sup> representing obstacle translational velocity. |

---

#### State Transition Model:
The continuous-time linear motion model is discretized at the filter update step $`\Delta t = \frac{1}{48} \approx 0.02083\text{ s}`$:

```math
\mathbf{x}_k = \mathbf{F} \mathbf{x}_{k-1} + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})
```

The $`6 \times 6`$ State Transition Matrix $`\mathbf{F}`$ is structured as:

```math
\mathbf{F} = \begin{bmatrix} \mathbf{I}_{3\times3} & \Delta t \mathbf{I}_{3\times3} \\ \mathbf{0}_{3\times3} & \mathbf{I}_{3\times3} \end{bmatrix} = \begin{bmatrix}
1 & 0 & 0 & \Delta t & 0 & 0 \\
0 & 1 & 0 & 0 & \Delta t & 0 \\
0 & 0 & 1 & 0 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
```

#### Mathematical Term Breakdown for State Transition:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>x</b><sub>k</sub> | State vector at time step k | ℝ<sup>6</sup> | [m, m/s]<sup>T</sup> | True obstacle state at current discrete timestamp t<sub>k</sub> = k Δ t. |
| <b>x</b><sub>k-1</sub> | State vector at previous step k-1 | ℝ<sup>6</sup> | [m, m/s]<sup>T</sup> | Obstacle state at previous discrete timestamp t<sub>k-1</sub> = (k-1)Δ t. |
| k | Discrete time step index | ℤ<sup>+</sup> | Dimensionless (count) | Control loop cycle index running at 48 Hz. |
| <b>F</b> | State transition matrix | ℝ<sup>6 × 6</sup> | Mixed (1, s) | Propagates constant-velocity kinematics forward by interval Δ t. |
| <b>I</b><sub>3×3</sub> | 3 × 3 Identity matrix | ℝ<sup>3 × 3</sup> | Dimensionless | Maps positions and velocities onto themselves across time steps. |
| <b>0</b><sub>3×3</sub> | 3 × 3 Zero null matrix | ℝ<sup>3 × 3</sup> | Dimensionless | Enforces zero direct coupling from position into velocity. |
| Δ t | Discrete filter sampling interval | ℝ<sup>+</sup> (Scalar) | s (seconds) | Δ t = 1/48 ≈ 0.020833 s (matches 48 Hz control loop). |
| <b>w</b><sub>k</sub> | Discrete process noise vector | ℝ<sup>6</sup> | [m, m/s]<sup>T</sup> | Stochastic disturbances, accelerations, and unmodeled trajectory deviations. |
| 𝒩(<b>0</b>, <b>Q</b>) | Multivariate Gaussian distribution | Probability measure | — | Zero-mean Gaussian distribution with covariance matrix <b>Q</b>. |
| <b>0</b> | Mean disturbance vector | ℝ<sup>6</sup> | [m, m/s]<sup>T</sup> | Unbiased disturbance hypothesis (E[<b>w</b><sub>k</sub>] = <b>0</b>). |
| <b>Q</b> | Process noise covariance matrix | ℝ<sup>6 × 6</sup> | Mixed (m<sup>2</sup>, m<sup>2</sup>/s, m<sup>2</sup>/s<sup>2</sup>) | Quantifies kinematic uncertainty due to obstacle maneuvering. |

**Detailed Transition Mechanics**:

**1. Position Integration**: The new position is predicted by integrating current velocity over time step $`\Delta t`$:

```math
\mathbf{p}_k = \mathbf{p}_{k-1} + \mathbf{v}_{k-1} \Delta t
```

* $`\mathbf{p}_k \in \mathbb{R}^3`$: Predicted position $`[\text{m}]`$ at step $`k`$.
* $`\mathbf{p}_{k-1} \in \mathbb{R}^3`$: Position $`[\text{m}]`$ at step $`k-1`$.
* $`\mathbf{v}_{k-1} \in \mathbb{R}^3`$: Linear velocity $`[\text{m/s}]`$ at step $`k-1`$.
* $`\Delta t`$: Integration time increment ($`0.02083\text{ s}`$).

**2. Velocity Persistence**: In the constant-velocity kinematic hypothesis, nominal obstacle velocity persists without deterministic acceleration:

```math
\mathbf{v}_k = \mathbf{v}_{k-1}
```

* $`\mathbf{v}_k \in \mathbb{R}^3`$: Predicted velocity $`[\text{m/s}]`$ at step $`k`$.

**3. Process Noise ($`\mathbf{w}_k`$)**: True obstacle accelerations, maneuvers, and stochastic deviations are modeled through the process noise vector $`\mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})`$.

The discrete process noise covariance matrix $`\mathbf{Q} \in \mathbb{R}^{6 \times 6}`$ is derived from a continuous-time white-noise acceleration model with spectral density $`q = \sigma_a^2`$:

```math
\mathbf{Q} = \begin{bmatrix}
\frac{\Delta t^3}{3} \mathbf{I}_{3\times3} & \frac{\Delta t^2}{2} \mathbf{I}_{3\times3} \\
\frac{\Delta t^2}{2} \mathbf{I}_{3\times3} & \Delta t \mathbf{I}_{3\times3}
\end{bmatrix} q
```

#### Mathematical Term Breakdown for Process Covariance $`\mathbf{Q}`$:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>Q</b> | Process noise covariance matrix | ℝ<sup>6 × 6</sup> | Mixed | Uncertainty injected into the state per time step Δ t. |
| (Δ t<sup>3</sup>)/3 <b>I</b><sub>3×3</sub> | Position-position covariance block | ℝ<sup>3 × 3</sup> | s<sup>3</sup> (scaled by q) → m<sup>2</sup> | Variance added to 3D position due to integrated white acceleration. |
| (Δ t<sup>2</sup>)/2 <b>I</b><sub>3×3</sub> | Position-velocity cross-covariance | ℝ<sup>3 × 3</sup> | s<sup>2</sup> (scaled by q) → m<sup>2</sup>/s | Correlation between position error and velocity error over step Δ t. |
| Δ t <b>I</b><sub>3×3</sub> | Velocity-velocity covariance block | ℝ<sup>3 × 3</sup> | s (scaled by q) → m<sup>2</sup>/s<sup>2</sup> | Variance added directly to obstacle linear velocity per step. |
| q | Continuous acceleration spectral density | ℝ<sup>+</sup> (Scalar) | m<sup>2</sup>/s<sup>3</sup> | q = σ<sub>a</sub><sup>2</sup> = 0.35<sup>2</sup> = 0.1225 m<sup>2</sup>/s<sup>3</sup>. Power spectral density of random accelerations. |
| σ<sub>a</sub> | Maneuver acceleration standard deviation | ℝ<sup>+</sup> (Scalar) | m/s<sup>2</sup> | Expected standard deviation of obstacle acceleration maneuvers (≈ 0.35 m/s<sup>2</sup>). |

---

#### Measurement Model:
The perception system acquires 3D LiDAR point hits on obstacle surfaces. The measurement vector $`\mathbf{z}_k \in \mathbb{R}^3`$ directly measures position, while linear velocity is unobservable directly from single-frame LiDAR:

```math
\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k, \quad \mathbf{v}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{R})
```

The $`3 \times 6`$ Observation Matrix $`\mathbf{H}`$ projects the 6-state vector onto the observable 3D position subspace:

```math
\mathbf{H} = \begin{bmatrix} \mathbf{I}_{3\times3} & \mathbf{0}_{3\times3} \end{bmatrix} = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0
\end{bmatrix}
```

The measurement noise covariance $`\mathbf{R} \in \mathbb{R}^{3 \times 3}`$ is governed by the LiDAR range sensor uncertainty:

```math
\mathbf{R} = \sigma_{\text{sensor}}^2 \mathbf{I}_{3\times3}
```

#### Mathematical Term Breakdown for Measurement Model:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>z</b><sub>k</sub> | Raw sensor measurement vector | ℝ<sup>3</sup> | m (meters) | 3D Cartesian coordinates of the LiDAR point reflection [z<sub>x</sub>, z<sub>y</sub>, z<sub>z</sub>]<sup>T</sup>. |
| <b>H</b> | Observation projection matrix | ℝ<sup>3 × 6</sup> | Dimensionless | Maps the 6-DOF state vector to the 3-DOF observable position subspace. |
| <b>v</b><sub>k</sub> | Measurement sensor noise vector | ℝ<sup>3</sup> | m (meters) | Additive Gaussian sensor noise perturbing the true surface hit position. |
| <b>R</b> | Measurement noise covariance matrix | ℝ<sup>3 × 3</sup> | m<sup>2</sup> (meters squared) | Diagonal covariance capturing sensor range variance across X, Y, Z. |
| σ<sub>sensor</sub> | LiDAR measurement noise std dev | ℝ<sup>+</sup> (Scalar) | m (meters) | Sensor standard deviation: 0.06 m (Low Noise default), 0.12 m (High Noise), 0.00 m (No Noise). |

---

### Numerical Stabilization: Joseph Form Covariance Update
In conventional textbook Kalman filtering, the a posteriori error covariance is updated using:

```math
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}
```

#### Numerical Failure of the Standard Update:
While mathematically equivalent in infinite precision, this simplified equation exhibits severe numerical flaws in floating-point software:
1. **Loss of Symmetry**: Numerical round-off errors in computing $`(\mathbf{I} - \mathbf{K}_k \mathbf{H})\mathbf{P}_{k|k-1}`$ inevitably cause $`\mathbf{P}_{k|k} \ne \mathbf{P}_{k|k}^T`$.
2. **Loss of Positive Definiteness**: Because the update subtracts two positive semi-definite matrices ($`\mathbf{P}_{k|k-1} - \mathbf{K}_k \mathbf{H} \mathbf{P}_{k|k-1}`$), machine precision truncation can yield tiny negative eigenvalues. This produces complex or non-physical standard deviations ($`\sigma = \sqrt{P_{ii}}`$) and eventually leads to filter divergence (covariance collapse).

#### The Stabilized Joseph Formulation:
To rigorously guarantee numerical stability, symmetry, and positive semi-definiteness across millions of simulation cycles, `perception/kalman_filter.py` implements the **Bucy-Joseph stabilized covariance equation**:

```math
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H})\mathbf{P}_{k|k-1}(\mathbf{I} - \mathbf{K}_k \mathbf{H})^T + \mathbf{K}_k \mathbf{R} \mathbf{K}_k^T
```

#### Mathematical Term Breakdown for Joseph Update:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>P</b><sub>k∣k</sub> | A posteriori error covariance matrix | ℝ<sup>6 × 6</sup> | Mixed (m<sup>2</sup>, m<sup>2</sup>/s, m<sup>2</sup>/s<sup>2</sup>) | Filtered uncertainty of state estimate at step k after incorporating measurement <b>z</b><sub>k</sub>. |
| <b>P</b><sub>k∣k-1</sub> | A priori error covariance matrix | ℝ<sup>6 × 6</sup> | Mixed (m<sup>2</sup>, m<sup>2</sup>/s, m<sup>2</sup>/s<sup>2</sup>) | Predicted uncertainty at step k before measurement incorporation (<b>P</b><sub>k∣k-1</sub> = <b>F</b> <b>P</b><sub>k-1∣k-1</sub> <b>F</b><sup>T</sup> + <b>Q</b>). |
| <b>I</b> | Identity matrix | ℝ<sup>6 × 6</sup> | Dimensionless | 6-dimensional identity matrix. |
| <b>K</b><sub>k</sub> | Optimal Kalman gain matrix | ℝ<sup>6 × 3</sup> | Mixed (dimensionless, 1/s) | Weights the measurement residual against model predictions: <b>K</b><sub>k</sub> = <b>P</b><sub>k∣k-1</sub> <b>H</b><sup>T</sup> <b>S</b><sub>k</sub><sup>-1</sup>. |
| <b>H</b> | Observation matrix | ℝ<sup>3 × 6</sup> | Dimensionless | Observation projection matrix [<b>I</b><sub>3×3</sub>   <b>0</b><sub>3×3</sub>]. |
| (<b>I</b> - <b>K</b><sub>k</sub> <b>H</b>) | Closed-loop error state matrix | ℝ<sup>6 × 6</sup> | Dimensionless | Governs propagation of estimation error under optimal feedback. |
| (·)<sup>T</sup> | Matrix transpose operator | Operator | — | Swaps rows and columns to formulate symmetric congruence transformations. |
| <b>R</b> | Measurement noise covariance | ℝ<sup>3 × 3</sup> | m<sup>2</sup> | Sensor variance matrix (σ<sub>sensor</sub><sup>2</sup> <b>I</b><sub>3×3</sub>). |

**Mathematical Proof of Guarantee**:
* **Term 1**: $`(\mathbf{I} - \mathbf{K}_k \mathbf{H})\mathbf{P}_{k|k-1}(\mathbf{I} - \mathbf{K}_k \mathbf{H})^T`$ is a congruence transformation of the prior covariance $`\mathbf{P}_{k|k-1}`$. Since $`\mathbf{P}_{k|k-1} \succ 0`$, this term is mathematically guaranteed to remain symmetric and positive semi-definite ($`\mathbf{A} \mathbf{P} \mathbf{A}^T \succeq 0`$).
* **Term 2**: $`\mathbf{K}_k \mathbf{R} \mathbf{K}_k^T`$ is a quadratic form of the positive-definite measurement noise covariance $`\mathbf{R} \succ 0`$.
* **Sum**: The sum of two strictly symmetric, positive semi-definite matrices is unconditionally symmetric and strictly positive semi-definite for **any arbitrary gain matrix** $`\mathbf{K}_k`$, regardless of floating-point truncation errors.

---

### Trajectory Projection & Closing Velocity Evasion

#### 1.0-Second Forward Trajectory Projection:
Once the Kalman Filter estimates filtered obstacle position $`\hat{\mathbf{p}}(t)`$ and velocity $`\hat{\mathbf{v}}(t)`$, the tracking subsystem projects the obstacle's future spatial coordinates across a forward lookahead horizon $`\tau \in [0, 1.0\text{ s}]`$:

```math
\hat{\mathbf{p}}(t + \tau) = \hat{\mathbf{p}}(t) + \hat{\mathbf{v}}(t) \cdot \tau
```

#### Mathematical Term Breakdown for Trajectory Projection:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>p̂</b>(t + τ) | Projected future 3D position vector | ℝ<sup>3</sup> | m (meters) | Future spatial coordinate of obstacle at future time t + τ. |
| <b>p̂</b>(t) | Filtered current 3D position estimate | ℝ<sup>3</sup> | m (meters) | Position extracted from Kalman state <b>x̂</b><sub>k∣k</sub>[0:3] at current time t. |
| <b>v̂</b>(t) | Filtered current 3D velocity estimate | ℝ<sup>3</sup> | m/s (meters / second) | Velocity extracted from Kalman state <b>x̂</b><sub>k∣k</sub>[3:6] at current time t. |
| t | Current simulation timestamp | ℝ<sup>+</sup> (Scalar) | s (seconds) | Elapsed physical simulation time. |
| τ | Forward projection lookahead horizon | ℝ<sup>+</sup> (Scalar) | s (seconds) | Lookahead parameter evaluated from τ = 0.0 s to τ = 1.0 s (48 control cycles). |

This forward trajectory creates a dynamic swept safety cylinder in space, allowing the avoidance system to perceive not just where an obstacle is now, but where it will intersect the flight corridor over the next 48 control cycles.

#### Relative Closing Velocity Calculation:
The closing velocity measures how rapidly an obstacle is collapsing the distance toward the quadcopter along a specific LiDAR sensor sector:

```math
\mathbf{v}_{\text{rel}} = \hat{\mathbf{v}}_{\text{obs}} - \mathbf{v}_{\text{drone}}
```

```math
v_{\text{closing}} = - (\mathbf{v}_{\text{rel}} \cdot \hat{\mathbf{d}}_{\text{sector}})
```

#### Mathematical Term Breakdown for Closing Velocity:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>v</b><sub>rel</sub> | Relative 3D velocity vector | ℝ<sup>3</sup> | m/s | Velocity vector of the obstacle viewed from the drone's moving reference frame. |
| <b>v̂</b><sub>obs</sub> | Filtered obstacle 3D velocity vector | ℝ<sup>3</sup> | m/s | Velocity sub-vector from the obstacle's Kalman state [v̂<sub>x</sub>, v̂<sub>y</sub>, v̂<sub>z</sub>]<sup>T</sup>. |
| <b>v</b><sub>drone</sub> | Quadcopter translational velocity vector | ℝ<sup>3</sup> | m/s | True quadcopter inertial velocity [v<sub>dx</sub>, v<sub>dy</sub>, v<sub>dz</sub>]<sup>T</sup> from MuJoCo physics. |
| v<sub>closing</sub> | Scalar sector closing velocity | ℝ (Scalar) | m/s | Rate of distance reduction along sector ray: positive if converging, negative if separating. |
| <b>d̂</b><sub>sector</sub> | Outward sector ray unit vector | ℝ<sup>3</sup> | Dimensionless | Unit vector along centerline of evaluated LiDAR sector (e.g. Front 0<sup>°</sup>, Left 35<sup>°</sup>). |
| · | Euclidean inner product (dot product) | Operator | — | Computes spatial projection of relative velocity onto sensor beam direction. |
| - | Inversion negative sign | Operator | — | Reverses sign so that converging vectors (<b>v</b><sub>rel</sub> opposite <b>d̂</b><sub>sector</sub>) yield v<sub>closing</sub> &gt; 0. |

**Decision & Sector Inflation Logic**:
* If $`v_{\text{closing}} > 0.05\text{ m/s}`$, the obstacle is rapidly converging toward the drone on that sector.
* The system applies **Dynamic Sector Inflation**:

```math
d_{\text{danger,eff}} = d_{\text{danger}} + \gamma \cdot \max(0, v_{\text{closing}}) \cdot \tau_{\text{proj}}
```

#### Mathematical Term Breakdown for Dynamic Sector Inflation:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| d<sub>danger,eff</sub> | Effective dynamic danger threshold | ℝ<sup>+</sup> (Scalar) | m (meters) | Dynamically expanded avoidance trigger distance for the evaluated sector. |
| d<sub>danger</sub> | Static nominal danger threshold | ℝ<sup>+</sup> (Scalar) | m (meters) | Base caution clearance distance (0.75 m). |
| γ | Dynamic inflation scaling factor | ℝ<sup>+</sup> (Scalar) | Dimensionless | Tuning gain set to γ = 0.40 in simulation. |
| max(0, v<sub>closing</sub>) | Non-negative closing velocity rectifier | ℝ<sup>+</sup> (Scalar) | m/s | Ensures receding obstacles (v<sub>closing</sub> ≤ 0) do not unsafely shrink safety margins below d<sub>danger</sub>. |
| τ<sub>proj</sub> | Anticipation projection horizon | ℝ<sup>+</sup> (Scalar) | s (seconds) | Predictive time horizon constant (τ<sub>proj</sub> = 1.0 s). |

* **Wake Evasion Rule**: Rather than attempting to cross in front of a fast-moving obstacle, the state machine biases its avoidance vector behind the obstacle's dynamic wake ($`\mathbf{v}_{\text{evade}} \propto -\hat{\mathbf{v}}_{\text{obs},\perp}`$), guaranteeing collision-free clearance.

---

### Stochastic Obstacle Noise & Multi-Tier Noise Control

In `simulation/drone_sim.py`, dynamic obstacles do not adhere to predictable, deterministic sinusoidal motions. Real-world obstacles wander, experience wind buffeting, and exhibit sensor measurement fluctuations. This behavior is modeled using a continuous **Ornstein-Uhlenbeck (OU) mean-reverting stochastic process**:

```math
dX_t = -\theta X_t dt + \sigma_{\text{obs}} dW_t
```

#### Mathematical Term Breakdown for Ornstein-Uhlenbeck Process:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| dX<sub>t</sub> | Stochastic displacement differential | ℝ (Scalar) | m (meters) | Infinitesimal stochastic perturbation added to obstacle coordinate over dt. |
| X<sub>t</sub> | Current perturbation coordinate offset | ℝ (Scalar) | m (meters) | Instantaneous deviation of obstacle from its nominal sliding path. |
| -θ X<sub>t</sub> dt | Mean-reverting deterministic drift | ℝ (Scalar) | m (meters) | Negative feedback pulling perturbation back toward zero (X=0). |
| θ | Mean reversion rate coefficient | ℝ<sup>+</sup> (Scalar) | s<sup>-1</sup> (1/s) | Governs return speed to nominal path (θ = 1.5 s<sup>-1</sup>, corresponding to mean relaxation time τ<sub>relax</sub> = 1/θ ≈ 0.67 s). |
| dt | Differential physics time step | ℝ<sup>+</sup> (Scalar) | s (seconds) | Physics integration interval Δ t<sub>physics</sub> = 0.002 s (500 Hz). |
| σ<sub>obs</sub> | Diffusion coefficient (process noise std) | ℝ<sup>+</sup> (Scalar) | m/√s | Governs intensity of random wandering perturbations (0.00 m, 0.08 m, or 0.16 m). |
| dW<sub>t</sub> | Standard Wiener process increment | Random variable | √s | Gaussian white noise increment: dW<sub>t</sub> ~ 𝒩(0, dt). |

#### Operational Noise Level Profiles:
The dashboard and simulation engine feature **Noise Level Control**, allowing users to benchmark navigation and filtering under three distinct environmental regimes:

| Noise Level | Process Noise (σ<sub>obs</sub>) | Sensor Noise (σ<sub>sensor</sub>) | Physical Characteristic & Benchmark Purpose |
| :---: | :---: | :---: | :--- |
| 🟢 **No Noise** | 0.00 m | 0.00 m | Pure deterministic motion (X<sub>t</sub> ≡ 0). Used for baseline algorithm benchmarking and establishing theoretical optimal path lengths. |
| 🟡 **Low Noise** *(Default)* | 0.08 m | 0.06 m | Moderate stochastic wander and realistic LiDAR jitter. The Kalman Filter achieves **45.8% noise variance reduction**, maintaining smooth flight without chattering. |
| 🔴 **High Noise** | 0.16 m | 0.12 m | Severe stochastic perturbation and heavy sensor degradation. Stress-tests the Kalman tracking filter, which achieves **58.8% variance reduction** to maintain safety margins. |

---

## 6. Avoidance Algorithms & Decision Logic

### 1. Reactive Sector State Machine (`avoidance/reactive_avoidance.py`)
A fast, robust state machine inspired by high-speed biological evasion:
* **Modes**: `FORWARD`, `AVOID`, `EMERGENCY_REVERSE`, `RETURN`, `VERTICAL_ESCAPE`, `CROSS_OVER`, `DESCEND`, `ESCAPING_THROUGH_GAP`, `GOAL_REACHED`.
* **Side-Switching Hysteresis**: When encountering an obstacle dead-center, hysteresis locks the chosen evasion direction (Left vs Right) for at least 0.35 s to prevent chatter or high-frequency oscillation.
* **Proactive Evasion**: Checks dynamic sector closing velocities. If an obstacle is sliding from Left to Right, the drone proactively steers Left (behind the obstacle's wake).

### 2. Dead-End & Ring Trap Recovery (360° LiDAR Inspection)
If a drone gets trapped in a concave structure (such as Scenario 5's U-Trap or Scenario 10's Circular Ring):
1. **Detection**: Tracks reversals. If **3 consecutive emergency reversals occur within 12 seconds**, the drone classifies the area as a dead-end trap.
2. **Inspection**: The drone halts and executes a full **72-ray (5° resolution) 360-degree perimeter sweep** plus an upward vertical ray test.
3. **Branching Decision**:
   - **Case A (Solid Enclosure)**: If horizontal sectors are blocked and upward vertical LiDAR is clear, it enters `VERTICAL_ESCAPE`: climbs over the wall (Z = 1.60 m), flies across the obstacle footprint (`CROSS_OVER`), and descends back to cruise altitude (`DESCEND`).
   - **Case B (Enclosure with Gap)**: If an opening arc ≥ 15° is found (e.g. at bearing 142.5°), it calculates the gap centroid, sets heading toward the opening, and slips through (`ESCAPING_THROUGH_GAP`).

### 3. Artificial Potential Field (`avoidance/apf_avoidance.py`)
The Artificial Potential Field (APF) treats the quadcopter as a point mass navigating a smooth continuous scalar potential field $`U(\mathbf{p}) = U_{\text{att}}(\mathbf{p}) + U_{\text{rep}}(\mathbf{p})`$ generated by attractive goal forces and repulsive obstacle boundaries.

#### Attractive Potential & Gradient Steering Force:
The goal position $`\mathbf{p}_{\text{goal}} \in \mathbb{R}^3`$ acts as a global potential well with quadratic growth:

```math
U_{\text{att}}(\mathbf{p}) = \frac{1}{2} k_{\text{att}} \|\mathbf{p} - \mathbf{p}_{\text{goal}}\|^2
```

The resulting attractive force $`\mathbf{F}_{\text{att}} \in \mathbb{R}^3`$ points in the direction of steepest potential descent ($`-\nabla U_{\text{att}}`$):

```math
\mathbf{F}_{\text{att}}(\mathbf{p}) = -\nabla U_{\text{att}}(\mathbf{p}) = -k_{\text{att}} (\mathbf{p} - \mathbf{p}_{\text{goal}}) = k_{\text{att}} (\mathbf{p}_{\text{goal}} - \mathbf{p})
```

#### Mathematical Term Breakdown for Attractive Potential & Force:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| U<sub>att</sub>(<b>p</b>) | Scalar attractive potential field value | ℝ<sup>+</sup> (Scalar) | J (virtual energy) | Global quadratic potential well centered at the destination waypoint. |
| 1/2 | Quadratic scaling factor | ℝ (Scalar) | Dimensionless | Standard harmonic potential coefficient ensuring clean derivative cancellation. |
| k<sub>att</sub> | Attractive proportional spring gain | ℝ<sup>+</sup> (Scalar) | N/m (stiffness) | Proportional attraction gain (k<sub>att</sub> = 1.25 N/m) establishing nominal pull toward goal. |
| <b>p</b> | Quadcopter 3D spatial position vector | ℝ<sup>3</sup> | m (meters) | Current center-of-mass coordinates [x, y, z]<sup>T</sup> in inertial world frame. |
| <b>p</b><sub>goal</sub> | Target waypoint position vector | ℝ<sup>3</sup> | m (meters) | Destination coordinates [x<sub>goal</sub>, y<sub>goal</sub>, z<sub>goal</sub>]<sup>T</sup> (e.g. [6.0, 0.0, 1.0]<sup>T</sup> m). |
| ‖ <b>p</b> - <b>p</b><sub>goal</sub>‖ | Euclidean distance to destination | ℝ<sup>+</sup> (Scalar) | m (meters) | Euclidean L<sub>2</sub> norm: √((x - x<sub>g</sub>)<sup>2</sup> + (y - y<sub>g</sub>)<sup>2</sup> + (z - z<sub>g</sub>)<sup>2</sup>). |
| <b>F</b><sub>att</sub>(<b>p</b>) | 3D Attractive virtual steering force | ℝ<sup>3</sup> | N (Newtons) | Virtual force pulling the drone toward the waypoint along line of sight. |
| -∇ | Negative spatial gradient operator | Operator | m<sup>-1</sup> (1/m) | Vector operator [-∂/(∂ x), -∂/(∂ y), -∂/(∂ z)]<sup>T</sup> directing force toward lowest potential. |
| (<b>p</b><sub>goal</sub> - <b>p</b>) | 3D Displacement error vector | ℝ<sup>3</sup> | m (meters) | Directional vector pointing from drone's current position toward the goal. |

---

#### Repulsive Potential & Hyperbolic Barrier Force:
Obstacle surfaces detected by the LiDAR array exert repulsive potential fields active only within an influence horizon $`d_0 = 1.20\text{ m}`$:

```math
U_{\text{rep},i}(\mathbf{p}) = \begin{cases}
\frac{1}{2} k_{\text{rep}} \left(\frac{1}{d_i(\mathbf{p})} - \frac{1}{d_0}\right)^2 & \text{if } d_i(\mathbf{p}) \le d_0 \\
0 & \text{if } d_i(\mathbf{p}) > d_0
\end{cases}
```

where $`d_i(\mathbf{p}) = \|\mathbf{p} - \mathbf{p}_{\text{obs},i}\|`$ is the Euclidean distance from the drone to obstacle point $`i`$, and $`k_{\text{rep}} = 0.85`$ is the repulsive gain.

Applying the chain rule, the gradient of Euclidean distance with respect to drone position is the outward unit normal vector:

```math
\nabla d_i(\mathbf{p}) = \frac{\mathbf{p} - \mathbf{p}_{\text{obs},i}}{\|\mathbf{p} - \mathbf{p}_{\text{obs},i}\|} = \hat{\mathbf{n}}_i
```

Taking the negative gradient of $`U_{\text{rep},i}`$ yields the repulsive force vector:

```math
\mathbf{F}_{\text{rep},i}(\mathbf{p}) = -\nabla U_{\text{rep},i}(\mathbf{p}) = k_{\text{rep}} \left(\frac{1}{d_i(\mathbf{p})} - \frac{1}{d_0}\right) \frac{1}{d_i(\mathbf{p})^2} \frac{\mathbf{p} - \mathbf{p}_{\text{obs},i}}{\|\mathbf{p} - \mathbf{p}_{\text{obs},i}\|}
```

#### Mathematical Term Breakdown for Repulsive Potential & Force:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| U<sub>rep,i</sub>(<b>p</b>) | Repulsive potential from obstacle contact i | ℝ<sup>+</sup> (Scalar) | J (virtual energy) | Compact-support repulsive potential field generated by obstacle surface contact point i. |
| k<sub>rep</sub> | Repulsive barrier gain constant | ℝ<sup>+</sup> (Scalar) | N·m<sup>2</sup> | Repulsive field strength coefficient (k<sub>rep</sub> = 0.85 N·m<sup>2</sup>). |
| d<sub>i</sub>(<b>p</b>) | Euclidean distance to obstacle hit i | ℝ<sup>+</sup> (Scalar) | m (meters) | Radial distance measured by the i-th LiDAR beam hit. |
| <b>p</b><sub>obs,i</sub> | 3D Spatial coordinate of obstacle hit i | ℝ<sup>3</sup> | m (meters) | Reflected laser point coordinate [x<sub>obs,i</sub>, y<sub>obs,i</sub>, z<sub>obs,i</sub>]<sup>T</sup>. |
| d<sub>0</sub> | Influence boundary cutoff distance | ℝ<sup>+</sup> (Scalar) | m (meters) | Influence distance threshold (d<sub>0</sub> = 1.20 m); beyond d<sub>0</sub>, repulsive field is identically zero. |
| (1/d<sub>i</sub> - 1/d<sub>0</sub>) | Boundary regularization term | ℝ<sup>+</sup> (Scalar) | m<sup>-1</sup> (1/m) | Ensures potential smoothly vanishes to zero at d<sub>i</sub> = d<sub>0</sub>, eliminating force step discontinuities. |
| 1/d<sub>i</sub><sup>2</sup> | Inverse-square hyperbolic barrier | ℝ<sup>+</sup> (Scalar) | m<sup>-2</sup> (1/m<sup>2</sup>) | Accelerates repulsive force toward infinity as d<sub>i</sub> → 0, preventing boundary penetration. |
| ∇ d<sub>i</sub>(<b>p</b>) | Spatial gradient of distance function | ℝ<sup>3</sup> | Dimensionless | Spatial derivative pointing along steepest increase of distance away from obstacle. |
| <b>n̂</b><sub>i</sub> | Outward unit normal vector | ℝ<sup>3</sup> | Dimensionless | Radial unit vector (<b>p</b> - <b>p</b><sub>obs,i</sub>)/(‖ <b>p</b> - <b>p</b><sub>obs,i</sub>‖ ) pushing the drone directly away from obstacle hit i. |
| <b>F</b><sub>rep,i</sub>(<b>p</b>) | 3D Repulsive force vector from contact i | ℝ<sup>3</sup> | N (Newtons) | Directed repulsive force pushing drone radially away from obstacle surface i. |

---

#### Resultant Net Force & Velocity Setpoint:
The total steering force is the vector superposition of goal attraction and all active repulsive obstacle contacts:

```math
\mathbf{F}_{\text{total}}(\mathbf{p}) = \mathbf{F}_{\text{att}}(\mathbf{p}) + \sum_{i=1}^{N_{\text{obs}}} \mathbf{F}_{\text{rep},i}(\mathbf{p})
```

The controller converts this net force into a normalized velocity setpoint bounded by maximum cruise speed $`v_{\text{max}} = 1.20\text{ m/s}`$:

```math
\mathbf{v}_{\text{des}} = v_{\text{max}} \cdot \frac{\mathbf{F}_{\text{total}}(\mathbf{p})}{\|\mathbf{F}_{\text{total}}(\mathbf{p})\|}
```

#### Mathematical Term Breakdown for Net Force & Desired Velocity:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>F</b><sub>total</sub>(<b>p</b>) | Net composite virtual steering force | ℝ<sup>3</sup> | N (Newtons) | Vector sum of goal attraction and all active obstacle repulsion vectors. |
| N<sub>obs</sub> | Number of active obstacle contacts | ℤ<sup>+</sup> (Integer) | Dimensionless (count) | Count of LiDAR beam hits detecting obstacle surfaces within boundary d<sub>0</sub> = 1.20 m. |
| <b>v</b><sub>des</sub> | Commanded 3D linear velocity setpoint | ℝ<sup>3</sup> | m/s (meters / second) | Desired velocity vector fed directly into the SE(3) translational tracking controller. |
| v<sub>max</sub> | Maximum nominal cruising speed | ℝ<sup>+</sup> (Scalar) | m/s (meters / second) | Cruise speed ceiling (v<sub>max</sub> = 1.20 m/s) ensuring stable aerodynamic flight. |
| <b>F</b><sub>total</sub>/(‖ <b>F</b><sub>total</sub>‖ ) | Unit direction vector of net force | ℝ<sup>3</sup> | Dimensionless | Normalized directional unit vector defining desired flight heading. |

#### Local Minima Handling:
In symmetric concave geometries (such as dead-end U-traps), the attractive force pointing toward the goal directly opposes the repulsive forces from symmetric side walls, causing $`\mathbf{F}_{\text{total}} \to \mathbf{0}`$. To escape stagnation, the APF module interfaces with the **Dead-End Monitor**: when velocity remains near zero ($`\|\mathbf{v}\| < 0.08\text{ m/s}`$) for $`> 1.5\text{ s}`$, it injects a transverse orthogonal force perturbation $`\mathbf{F}_{\text{escape}} \perp (\mathbf{p}_{\text{goal}} - \mathbf{p})`$ or triggers a 360° perimeter scan to climb over the obstacle.

---

### 4. Dynamic Window Approach (`avoidance/dwa_avoidance.py`)
Samples a grid of linear and angular velocities within the drone's reachable acceleration window, projecting forward trajectories and picking the path that maximizes heading alignment, clearance margin, and forward speed.

---

### 5. SINDy-Enhanced DWA (`avoidance/sindy_dwa_avoidance.py`)
Standard Dynamic Window Approach implementations assume simplified linear kinematic equations (e.g., Dubins car or uncoupled point mass), failing to account for rotor-induced drag, aerodynamic ground effect, and roll-pitch-yaw gyroscopic cross-coupling. SINDy-Enhanced DWA integrates **Sparse Identification of Nonlinear Dynamics (SINDy)** to discover true flight dynamics directly from simulation telemetry:

```math
\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u}) \approx \mathbf{\Theta}(\mathbf{x}, \mathbf{u}) \mathbf{\Xi}
```

#### Mathematical Term Breakdown for SINDy Model:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>ẋ</b> | Time derivative of dynamic state | ℝ<sup>6</sup> | [m/s, m/s<sup>2</sup>]<sup>T</sup> | Instantaneous rate of change of 6-DOF state [ẋ, ẏ, ż, v̇<sub>x</sub>, v̇<sub>y</sub>, v̇<sub>z</sub>]<sup>T</sup>. |
| <b>x</b> | Quadcopter dynamic state vector | ℝ<sup>6</sup> | [m, m/s]<sup>T</sup> | Quadcopter position and linear velocity vector [x, y, z, v<sub>x</sub>, v<sub>y</sub>, v<sub>z</sub>]<sup>T</sup>. |
| <b>u</b> | Control input vector | ℝ<sup>4</sup> | N (or PWM) | Quadcopter actuator input vector [f<sub>1</sub>, f<sub>2</sub>, f<sub>3</sub>, f<sub>4</sub>]<sup>T</sup> representing individual rotor forces. |
| <b>f</b>(<b>x</b>, <b>u</b>) | True unknown nonlinear governing physics | ℝ<sup>6</sup> × ℝ<sup>4</sup> → ℝ<sup>6</sup> | [m/s, m/s<sup>2</sup>]<sup>T</sup> | True non-linear aerodynamic and rigid-body equations evaluated by MuJoCo RK4. |
| <b>Θ</b>(<b>x</b>, <b>u</b>) | Candidate basis function row vector | ℝ<sup>1 × K</sup> | Mixed | Vector of nonlinear candidate functions evaluated at state-control point (<b>x</b>, <b>u</b>). |
| <b>Ξ</b> | Sparse dynamic coefficient matrix | ℝ<sup>K × 6</sup> | Mixed | Identified sparse weight matrix capturing active physical terms while setting spurious terms to zero. |

#### Formulation & Library Construction:
**1. Telemetry Data**: Flight state logs $`\mathbf{X} \in \mathbb{R}^{M \times 6}`$ and control commands $`\mathbf{U} \in \mathbb{R}^{M \times 4}`$ are recorded at 48 Hz across dynamic maneuvers ($`M`$ time samples, typically $`M \approx 2400`$ for a 50-second run).

**2. Candidate Function Library Matrix $`\mathbf{\Theta}(\mathbf{X}, \mathbf{U}) \in \mathbb{R}^{M \times K}`$**: constructed with non-linear candidate functions:

```math
\mathbf{\Theta}(\mathbf{X}, \mathbf{U}) = \begin{bmatrix} \mathbf{1} & \mathbf{X} & \mathbf{U} & \mathbf{X}^2 & \mathbf{X} \otimes \mathbf{U} & \sin(\mathbf{X}) & \cos(\mathbf{X}) & \mathbf{X} |\mathbf{X}| \end{bmatrix}
```

#### Mathematical Term Breakdown for Candidate Library Functions:
| Library Block | Mathematical Expression | Dimensions in Matrix | Physical SI Units | Dynamic Phenomenon Represented |
| :--- | :--- | :---: | :---: | :--- |
| <b>1</b> | [1, 1, …, 1]<sup>T</sup> | ℝ<sup>M × 1</sup> | Dimensionless | Constant acceleration biases, gravity vector projection g, and steady trim offsets. |
| <b>X</b> | [x, y, z, v<sub>x</sub>, v<sub>y</sub>, v<sub>z</sub>] | ℝ<sup>M × 6</sup> | m, m/s | Linear kinematic drift, velocity-to-position integration, and linear damping. |
| <b>U</b> | [u<sub>1</sub>, u<sub>2</sub>, u<sub>3</sub>, u<sub>4</sub>] | ℝ<sup>M × 4</sup> | N | Linear motor thrust and control torque contributions. |
| <b>X</b><sup>2</sup> | [x<sup>2</sup>, y<sup>2</sup>, …, v<sub>z</sub><sup>2</sup>] | ℝ<sup>M × 6</sup> | m<sup>2</sup>, m<sup>2</sup>/s<sup>2</sup> | Centripetal acceleration components and quadratic non-linear stiffness. |
| <b>X</b> ⊗ <b>U</b> | Kronecker product x<sub>i</sub> u<sub>j</sub> | ℝ<sup>M × 24</sup> | m·N, m/s·N | Control-state bilinear cross-coupling (e.g. forward velocity roll-coupling under thrust). |
| sin(<b>X</b>), cos(<b>X</b>) | [sin(x), …, cos(v<sub>z</sub>)] | ℝ<sup>M × 12</sup> | Dimensionless | Trigonometric attitude projection terms and Euler angle geometric couplings. |
| <b>X</b> ∣ <b>X</b>∣ | [v<sub>x</sub>∣ v<sub>x</sub>∣ , v<sub>y</sub>∣ v<sub>y</sub>∣ , v<sub>z</sub>∣ v<sub>z</sub>∣ ] | ℝ<sup>M × 3</sup> | m<sup>2</sup>/s<sup>2</sup> | **Nonlinear Aerodynamic Drag**: captures quadratic air resistance -1/2ρ C<sub>d</sub> A v ∣ v∣ with proper physical sign preservation. |

**3. Sparse Regression via STLSQ**: the sparse coefficient matrix $`\mathbf{\Xi} \in \mathbb{R}^{K \times 6}`$ is solved using **Sequential Thresholded Least Squares (STLSQ)**:

```math
\min_{\mathbf{\Xi}} \frac{1}{2} \|\dot{\mathbf{X}} - \mathbf{\Theta}(\mathbf{X}, \mathbf{U})\mathbf{\Xi}\|_F^2 + \lambda \|\mathbf{\Xi}\|_1
```

#### Mathematical Term Breakdown for STLSQ Optimization:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>Ẋ</b> | State derivative time-history matrix | ℝ<sup>M × 6</sup> | [m/s, m/s<sup>2</sup>] | Numerically differentiated state history from MuJoCo telemetry logs over M samples. |
| <b>Θ</b>(<b>X</b>, <b>U</b>) | Candidate function evaluation matrix | ℝ<sup>M × K</sup> | Mixed | Evaluated candidate library matrix across all M flight data rows (K ≈ 56 columns). |
| <b>Ξ</b> | Dynamic coefficient matrix | ℝ<sup>K × 6</sup> | Mixed | Matrix of polynomial/aerodynamic coefficients multiplying the candidate basis functions. |
| ‖ ·‖ <sub>F</sub> | Frobenius matrix norm | Norm operator | Units of <b>Ẋ</b> | Measures the total root-mean-squared dynamic reconstruction error: √(Σ<sub>i,j</sub> (Ẋ<sub>ij</sub> - (<b>Θ</b><b>Ξ</b>)<sub>ij</sub>)<sup>2</sup>). |
| λ | L<sub>1</sub> Sparsity penalty parameter | ℝ<sup>+</sup> (Scalar) | Regularization scale | Sparsity-inducing hyperparameter (λ = 0.05); drives non-essential coefficients to zero. |
| ‖ <b>Ξ</b>‖ <sub>1</sub> | Element-wise L<sub>1</sub> matrix norm | Norm operator | Mixed | Sum of absolute values Σ<sub>i,j</sub> ∣ Ξ<sub>ij</sub>∣ enforcing parsimonious model selection. |
| ε | Hard thresholding cutoff value | ℝ<sup>+</sup> (Scalar) | Mixed | Threshold parameter (ε = 0.08); coefficients below ε are clamped to exact zero at each STLSQ step. |

#### Forward Trajectory Rollout Optimization:
During avoidance planning, candidate velocities $`(v, \omega)`$ in the dynamic window $`V_d`$ are numerically integrated forward using the discovered physical model:

```math
\mathbf{x}_{\text{pred}}(t + \tau) = \mathbf{x}(t) + \int_{0}^{\tau} \mathbf{\Theta}(\mathbf{x}_{\text{pred}}, \mathbf{u}) \mathbf{\Xi} \, dt, \quad \tau \in [0, 1.2\text{ s}]
```

The optimal trajectory maximizes the composite objective function:

```math
J(v, \omega) = \alpha \cdot \text{heading}(v, \omega) + \beta \cdot \text{clearance}(v, \omega) + \gamma \cdot \text{velocity}(v, \omega)
```

#### Mathematical Term Breakdown for DWA Objective Function:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| J(v, ω) | Scalar trajectory quality score | ℝ (Scalar) | Dimensionless | Overall evaluation metric; candidate (v, ω) maximizing J is executed by the drone. |
| v | Candidate linear cruise speed | ℝ (Scalar) | m/s (meters / second) | Candidate translational speed sampled within reachable dynamic window [v<sub>min</sub>, v<sub>max</sub>]. |
| ω | Candidate yaw angular velocity | ℝ (Scalar) | rad/s (radians / second) | Candidate turn rate sampled within reachable window [ω<sub>min</sub>, ω<sub>max</sub>]. |
| α | Heading alignment weight factor | ℝ<sup>+</sup> (Scalar) | Dimensionless | Weight for target waypoint alignment (α = 0.35). |
| heading(v, ω) | Normalized heading alignment metric | ℝ ∈ [0, 1] | Dimensionless | Metric 1 - (∣ ψ<sub>pred</sub> - ψ<sub>goal</sub>∣ )/π, scoring 1.0 when pointing directly toward goal. |
| β | Obstacle clearance weight factor | ℝ<sup>+</sup> (Scalar) | Dimensionless | Weight for maximizing clearance from detected obstacle boundaries (β = 0.45). |
| clearance(v, ω) | Normalized minimum clearance metric | ℝ ∈ [0, 1] | Dimensionless | Metric min<sub>i</sub> (d<sub>i</sub>(v, ω))/d<sub>safe</sub>, rewarding trajectories that keep distance from obstacles. |
| γ | Forward speed weight factor | ℝ<sup>+</sup> (Scalar) | Dimensionless | Weight for flight efficiency and swift progress (γ = 0.20). |
| velocity(v, ω) | Normalized forward speed metric | ℝ ∈ [0, 1] | Dimensionless | Metric v/v<sub>max</sub>, rewarding higher cruising velocities when open path is clear. |

Because SINDy captures quadcopter inertia and rotor response lag, DWA rollouts accurately reflect physical drift, eliminating the overshoot and under-steering common in idealized planners.

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
| **1** | Direct Frontal Obstacle | Easy | `(2.50, 0.00, 1.00) m` | Head-on collision avoidance and smooth re-centering. |
| **2** | Offset Obstacle Left | Easy | `(2.50, +0.20, 1.00) m` | Asymmetric detection and rightward bias decision. |
| **3** | Offset Obstacle Right | Easy | `(2.50, -0.20, 1.00) m` | Asymmetric detection and leftward bias decision. |
| **4** | Narrow Passage (Slit) | Medium | Gates at X = 2.50m & 3.20m (gap 0.76m) | Simultaneous bilateral clearance and corridor threading. |
| **5** | Dead-End U-Trap | Hard | Pocket: X = 2.50m, Wings Y = ±0.36m, ±0.52m | Emergency reversal oscillations & 3D vertical overflight. |
| **6** | Staggered Forest Maze | Hard | 4 Cylinders from X = 1.80m to 4.80m | Multi-stage continuous slalom navigation. |
| **7** | Dynamic Moving Obstacle | Dynamic | Block at X = 2.80m sliding laterally (Y = ±1.15m) | Kalman tracking under Ornstein-Uhlenbeck stochastic drift. |
| **8** | 3D Cluttered Space | 3D Expert | Low wall (Z = 0.6m), Overhang (Z = 1.6m), Pillar | 3D spatial evasion: lateral bypass and vertical underpass. |
| **9** | Dynamic Lateral Intruder | Expert | Fast cube at X = 2.80m sliding laterally | High closing-velocity prediction and proactive evasion. |
| **10** | Circular Dead-End Ring | Extreme | 54 Cubes in a R = 2.60m ring around start point | Full 360° perimeter scan: vertical climb vs gap search. |

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
     * Realistic **Crazyflie Quadcopter 3D Model** with carbon arms, brushless motors, and spinning neon rotor discs (45° roll/pitch attitude tilt corresponding to flight dynamics).
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
   * **Drone Floating Badge**: Shows live `(X, Y, Z) m` floating above the drone as it flies.
   * **Obstacle Coordinates**: Every obstacle displays its world coordinate `(X, Y)` directly underneath it.
   * **Bottom Telemetry Bar**: Shows real-time drone coordinates and live mouse cursor position `(X, Y) m`.

4. **Flight Kinematics & Moving Obstacle Analytics Suite**:
   * **` Analysis & Graphs` Mode**:
     Accessible directly from the viewport mode switcher in the arena header or from the left sidebar's **Analysis** tab.
   * **Sub-Tab 1:  Drone Movement Analysis**:
      * **KPI Cards**: Peak Velocity (V_max), Average Velocity (V_avg), Path Efficiency (6.0m / L_path × 100%), Max Altitude Deviation (ΔZ_max), and Closest Obstacle Margin.
     * **Flight Phase Distribution Timeline**: Multi-segmented color-coded bar displaying exact seconds and percentages spent across avoidance states (`FORWARD`, `AVOID_LEFT`, `AVOID_RIGHT`, `CLIMB`, `REVERSE`, `GOAL_APPROACH`, `HOVER`).
     * **Interactive Velocity & Speed Chart**: Time-series plot of Total Speed, Forward Velocity (V_x), Lateral Velocity (V_y), and Vertical Velocity (V_z).
     * **Interactive Altitude & Goal Convergence Chart**: Cruise altitude profile (Z) with dual-axis Distance-to-Goal curve.
   * **Sub-Tab 2:  Moving Objects & Kalman Filter Graphs**:
     * **Dynamic Obstacle KPI Cards**: Active moving obstacle target name, Closest Drone Proximity (d_min), Kalman Filter Position RMSE, Noise Variance Reduction %, and Lateral Oscillation Amplitude.
     * **Moving Object Trajectory Graph (True vs Noisy vs Kalman)**: Compares **Ground Truth Position (Y_true)** (Green), **Noisy Sensor Reading (Y_noisy)** with stochastic Ornstein-Uhlenbeck jitter (Orange dashed), and **Kalman Filter Estimated Position (Y_kalman)** (Cyan glowing line).
     * **Separation Distance & Safety Margin Chart**: Euclidean distance between quadcopter and moving obstacle over time, annotated with the red **0.35m Collision Safety Bubble**.
     * **Velocity & Noise Residual Analysis Chart**: Residual error curves comparing raw sensor noise error |Y_noisy - Y_true| against Kalman residual error |Y_kalman - Y_true|, visually proving noise variance damping.
   * **Sub-Tab 3:  Kinematics Data Table & CSV Export**:
     * Frame-by-frame numerical data log: Timestamp, 3D Position (X, Y, Z), Speed, Flight State, Distance to Goal, Minimum Clearance, and Dynamic Obstacle Coordinates.
     * **Interactive Row Seeking**: Clicking any table row immediately jumps the playback slider and 3D quadcopter model to that exact moment in flight.
     * **`⬇ Export CSV`**: One-click download of the complete flight trace (`.csv`) for MATLAB, Python, or thesis plotting.

5. **Sensor & Trajectory Visualizers**:
   * **LiDAR Sector Radar**: 8-wedge spinning radar updating at 48 Hz (Green/Orange/Red).
   * **Telemetry Cards**: Flight result, path length, minimum clearance, collision steps, **Kalman RMSE Error**, and **Noise Variance Reduction %**.

6. **Noise Level Control (Interactive Tri-State Selector)**:
   * **Location**: Prominently integrated into the left sidebar under *Flight Configuration*.
   * **Three Selectable Options**:
     *  **No Noise** — Noise disabled ($`\sigma_{\text{obs}} = 0.00\text{ m}`$, $`\sigma_{\text{sensor}} = 0.00\text{ m}`$). Establishes baseline deterministic ground-truth flight performance.
     *  **Low Noise** *(Active Default)* — Low-intensity stochastic noise applied ($`\sigma_{\text{obs}} = 0.08\text{ m}`$, $`\sigma_{\text{sensor}} = 0.06\text{ m}`$). Standard operational mode representing realistic turbulence and LiDAR measurement jitter.
     *  **High Noise** — High-intensity stochastic noise applied ($`\sigma_{\text{obs}} = 0.16\text{ m}`$, $`\sigma_{\text{sensor}} = 0.12\text{ m}`$). Extreme operational mode stress-testing the Kalman Filter's tracking resilience and closing-velocity evasion responsiveness.
   * **Visual Active/Inactive Highlighting**:
     * Single-selection radio behavior with mutual exclusivity (only one active noise level at any given time).
     * The selected button is illuminated with glowing borders and ambient gradient fills color-coded to the operational state (Emerald Green for No Noise, Amber Yellow for Low Noise, Coral Red for High Noise).
     * Inactive buttons are displayed with subtle translucency, muted borders, and interactive hover sliding animations.
   * **Telemetry & Analytics Integration**:
     * Live status badge pill (` None`, ` Low`, ` High`) in the section header.
     * Dynamic text callout dynamically updating the active configuration description.
     * Seamless API connection: `/api/run` and `/api/run_custom` pass `noise_level` to the MuJoCo simulation worker, updating real-time Kalman RMSE and noise variance reduction metrics on the graphs.
     * Full bidirectional synchronization with the legacy `Obstacle Noise` checkbox.

---

## 9. Simulation Execution Guide & Command-Line Reference

This section provides a complete, step-by-step guide to installing dependencies, launching the simulation in all operational modes, interacting with the graphical interfaces, configuring noise levels, running automated benchmarks, and troubleshooting common environment issues.

---

### 1. Prerequisites & Environment Setup

The simulation is built on standard Python 3.8+ and runs natively on **Windows**, **Linux**, and **macOS**.

#### A. System Requirements:
* **Python**: Version `3.8` to `3.12` installed and available in system PATH.
* **GPU / Graphics**: OpenGL 3.3+ capable GPU (Intel HD Graphics 4000+, NVIDIA GTX/RTX, AMD Radeon, or Apple Silicon Metal).
* **RAM**: 4 GB minimum (8 GB recommended for batch benchmarking).

#### B. Clone & Virtual Environment Setup:
Open PowerShell (Windows) or Terminal (Linux/macOS) in the project directory:

```bash
# Navigate to the repository root directory
cd Drones_S5_AB3_Obstacle_Avoidance_Drone

# Create a virtual environment named 'venv'
python -m venv venv

# Activate virtual environment:
# On Windows PowerShell:
.\venv\Scripts\Activate.ps1
# (If PowerShell throws an execution policy error, run once:
#  Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process)
# On Windows Command Prompt:
.\venv\Scripts\activate.bat
# On Linux / macOS:
source venv/bin/activate
```

#### C. Install Dependencies:
Install all required libraries via `pip`:

```bash
# Option A: Install from dedicated MuJoCo requirements file
pip install -r mujoco_drone_sim/requirements.txt

# Option B: Direct one-line pip install
pip install mujoco>=3.1.0 numpy scipy matplotlib flask pysindy scikit-learn
```

Verify that MuJoCo 3.x is properly loaded:
```bash
python -c "import mujoco; print('MuJoCo successfully loaded! Version:', mujoco.__version__)"
```

*Note for Linux users*: If running the desktop Tkinter arena customizer, install Python Tkinter:
```bash
sudo apt-get update && sudo apt-get install python3-tk
```

---

### 2. Quick Start (Instant Run)

Launch the interactive Web Dashboard with a single command:
```bash
# From repository root:
python run_mujoco.py --dashboard

# Or from inside mujoco_drone_sim directory:
python run_sim.py --dashboard
```
Then open your web browser and navigate to:
```
http://127.0.0.1:8080
```

To immediately launch the native MuJoCo 3D Physics Viewer:
```bash
python run_sim.py --scenario 1 --algo reactive --gui
```

---

### 3. Execution Mode 1: Interactive Web Telemetry Dashboard

The Web Dashboard (`mujoco_drone_sim/web_dashboard/app.py`) provides an interactive interface featuring real-time 2D/3D dual-viewports, dynamic obstacle placement, live LiDAR radar, Kalman tracking graphs, and the **Noise Level Control** selector.

#### How to Launch:
```bash
# From repository root:
python run_sim.py --dashboard

# Or launch directly from the web_dashboard directory:
python mujoco_drone_sim/web_dashboard/app.py
```
By default, the server listens on port `8080`. To specify an alternate port:
```bash
python run_sim.py --dashboard --port 8085
```

#### Step-by-Step Dashboard Usage Instructions:
1. **Open the Dashboard**: Open your browser and navigate to `http://127.0.0.1:8080`.
2. **Select Flight Scenario**:
   * Open the **Flight Scenario** dropdown in the left control panel.
   * Choose from any of the 10 benchmark scenarios (e.g. *Scenario 1: Direct Frontal Obstacle*, *Scenario 5: Dead-End U-Trap*, *Scenario 7: Dynamic Moving Obstacle*, or *Scenario 10: Circular Ring Trap*).
3. **Select Avoidance Strategy**:
   * Choose from the 6 available navigation algorithms: `Reactive (Default)`, `APF (Potential Fields)`, `DWA (Dynamic Window)`, `SINDy-DWA (Learned Physics)`, `3D Avoidance (Volumetric)`, or `Neural Brain`.
4. **Select Noise Level (Tri-State Selector)**:
   * In the **Noise Level Control** section in the left sidebar, click one of the three mutually exclusive buttons:
     *  **No Noise**: Disables all stochastic process noise and sensor jitter ($`\sigma_{\text{obs}} = 0.00\text{ m}, \sigma_{\text{sensor}} = 0.00\text{ m}`$). Use this mode for baseline algorithm comparison and establishing theoretical optimal paths.
     *  **Low Noise** *(Default)*: Applies standard operational noise ($`\sigma_{\text{obs}} = 0.08\text{ m}, \sigma_{\text{sensor}} = 0.06\text{ m}`$). Simulates realistic aerodynamic wind buffeting and LiDAR sensor jitter. The Kalman Filter achieves **$`45.8\%`$ variance reduction**.
     *  **High Noise**: Injects severe turbulence and heavy sensor degradation ($`\sigma_{\text{obs}} = 0.16\text{ m}, \sigma_{\text{sensor}} = 0.12\text{ m}`$). Stress-tests the Kalman tracking filter and dynamic sector inflation logic, achieving **$`58.8\%`$ variance reduction**.
   * Notice the active button illuminates with its color-coded glowing border and the header badge updates (` None`, `Low`, or ` High`).
5. **Run the Simulation**:
   * Click the **`▶ Start Simulation`** button. The server launches the MuJoCo simulation worker in a background thread.
6. **Interact with the Live Viewports**:
   * **Interactive Canvas**: Click or drag obstacles on the top-down 2D canvas to reposition them in real time while the quadcopter navigates.
   * **LiDAR Radar**: Observe the 8-beam spinning radar displaying real-time proximity (Green = Clear, Orange = Caution, Red = Emergency Reverse).
   * **Telemetry Cards**: Monitor real-time Flight Status, Current Speed, Altitude, Distance to Goal, Minimum Obstacle Clearance, **Kalman Filter RMSE**, and **Noise Variance Reduction %**.
7. **Inspect Analytics & Trajectory Graphs**:
   * Switch between the three lower tabs:
     * **Telemetry Over Time**: Altitude hold, velocity curves, and goal convergence.
     * **Moving Objects & Kalman Filter Graphs**: Ground Truth ($`Y_{\text{true}}`$) vs Noisy Sensor ($`Y_{\text{noisy}}`$) vs Kalman Estimated ($`Y_{\text{kalman}}`$) curves, safety bubble margins, and residual error damping.
     * **Kinematics Data Table**: Frame-by-frame data log with instant row seeking and **`⬇ Export CSV`** download.

---

### 4. Execution Mode 2: Native MuJoCo 3D Interactive Viewer

The native MuJoCo 3D graphical viewer (`--gui` / `-g`) provides interactive visualization with full physics rendering at 500 Hz.

#### Launch Syntax:
```bash
python run_sim.py --scenario <ID> --algo <ALGO> --gui
```

#### Viewer Mouse & Keyboard Controls:
| Action / Key | Function in 3D Viewer |
| :--- | :--- |
| **Left Click + Drag** | **Orbit / Rotate Camera**: Rotates the 3D perspective around the focal point. |
| **Right Click + Drag** | **Pan / Translate Camera**: Moves the camera horizontally and vertically across the arena. |
| **Scroll Wheel / Double Right Click** | **Zoom In / Out**: Adjusts camera distance from the quadcopter. |
| **`Spacebar`** | **Pause / Resume**: Freezes simulation physics; press again to resume. |
| **`Backspace`** | **Reset Simulation**: Instantly reloads initial conditions and repositions the drone at (0, 0, 1) m. |
| **`Tab`** | **Cycle Camera Viewpoint**: Cycles between Free Camera, Fixed Observer, Drone Tracking, and On-board Camera. |
| **`[` and `]`** | **Slow / Fast Motion**: Halves or doubles the physical simulation speed. |
| **`0` to `9`** | **Toggle Debug Overlays**: Toggles rendering of contact points, sensor rays, body frames, and bounding geoms. |

---

### 5. Execution Mode 3: Testing & Comparing Avoidance Algorithms

Test any of the 6 navigation algorithms by passing the `--algo` (or `-a`) flag:

```bash
# 1. Reactive Sector State Machine (Dynamic closing-velocity evasion + 360° trap escape)
python run_sim.py --scenario 1 --algo reactive --gui

# 2. Artificial Potential Field (APF continuous potential gradient navigation)
python run_sim.py --scenario 1 --algo apf --gui

# 3. Dynamic Window Approach (DWA forward trajectory velocity sampling)
python run_sim.py --scenario 1 --algo dwa --gui

# 4. 3D Volumetric Avoidance (Vertical climb-overs and altitude adaptation)
python run_sim.py --scenario 4 --algo 3d --gui

# 5. SINDy-Enhanced DWA (Data-driven nonlinear aerodynamic drag prediction)
python run_sim.py --scenario 7 --algo sindy_dwa --gui

# 6. Adaptive Neural Brain (Adaptive weight updates for complex steering)
python run_sim.py --scenario 1 --algo neural --gui
```

---

### 6. Execution Mode 4: Testing All 10 Benchmark Flight Scenarios

Run individual test scenarios by specifying `--scenario` (or `-s`) from `1` to `10`:

```bash
# Scenario 1: Direct Frontal Obstacle (Baseline single obstacle avoidance)
python run_sim.py --scenario 1 --algo reactive --gui

# Scenario 2: High-Speed Frontal Obstacle (Rapid reflex evasion)
python run_sim.py --scenario 2 --algo reactive --gui

# Scenario 3: Static Cluttered Forest (Multi-obstacle random pillar navigation)
python run_sim.py --scenario 3 --algo reactive --gui

# Scenario 4: Narrow Passage Corridor (Precision gate alignment through 0.76m gaps)
python run_sim.py --scenario 4 --algo reactive --gui

# Scenario 5: Dead-End U-Trap (Concave enclosure: 360° LiDAR inspection & vertical climb-over)
python run_sim.py --scenario 5 --algo reactive --gui

# Scenario 6: Zig-Zag Wall Slalom (Staggered wall slalom maneuvering)
python run_sim.py --scenario 6 --algo reactive --gui

# Scenario 7: Dynamic Moving Obstacle (Lateral sinusoidal slider with Kalman Filter tracking)
python run_sim.py --scenario 7 --algo reactive --gui

# Scenario 8: Multi-Obstacle Dynamic Clutter (Multiple asynchronous moving obstacles)
python run_sim.py --scenario 8 --algo reactive --gui

# Scenario 9: High-Speed Dynamic Intruder (Fast intercepting obstacle testing closing-velocity inflation)
python run_sim.py --scenario 9 --algo reactive --gui

# Scenario 10: Closed Circular Ring (Complete cylindrical trap with climb-over)
python run_sim.py --scenario 10 --sub-scenario closed_ring --gui

# Scenario 10: Ring with Random Gap Search (Slotted cylindrical wall: finds opening arc >= 15°)
python run_sim.py --scenario 10 --sub-scenario ring_with_gap --seed 42 --gui
```

---

### 7. Execution Mode 5: Configuring Noise Levels via CLI

You can customize stochastic process noise and sensor measurement error directly from the command line:

```bash
#  No Noise: Disable obstacle stochastic perturbations completely
python run_sim.py --scenario 7 --algo reactive --no-obs-noise --gui

#  Low Noise: Default realistic operating regime
python run_sim.py --scenario 7 --algo reactive --obs-noise-std 0.08 --sensor-noise 0.06 --gui

#  High Noise: Severe turbulence and heavy sensor degradation
python run_sim.py --scenario 7 --algo reactive --obs-noise-std 0.16 --sensor-noise 0.12 --gui

# Filter Ablation Test: Disable Kalman filter to test raw noisy sensor evasion
python run_sim.py --scenario 7 --algo reactive --no-kalman --gui
```

---

### 8. Execution Mode 6: Desktop Arena Customizer & Custom Layouts

Create custom obstacle arenas using the standalone visual Tkinter GUI editor:

```bash
python interactive_arena.py
```

#### How to Use the Arena Customizer:
1. **Add Obstacles**: Click **`Add Box`** or **`Add Cylinder`** to place shapes in the arena.
2. **Move & Resize**: Drag shapes across the canvas to reposition; adjust length, width, and height in the property sidebar.
3. **Set Start & Goal**: Reposition the Green Start waypoint and Red Goal destination.
4. **Save Configuration**: Click **`Save Arena`** to export the layout to `custom_arena.json`.
5. **Run in MuJoCo**:
   ```bash
   python run_sim.py --custom-file custom_arena.json --gui
   ```

---

### 9. Execution Mode 7: Automated Multi-Scenario Benchmark Suite

Run the headless automated evaluation suite to benchmark all 6 algorithms across all 10 scenarios:

```bash
python run_sim.py --benchmark
```

#### Benchmark Output:
* Executes all scenario-algorithm permutations under standardized conditions.
* Computes comprehensive performance metrics: **Success Rate (%)**, **Flight Duration (s)**, **Total Path Length (m)**, **Minimum Obstacle Clearance (m)**, **Collision Count**, and **Flight Smoothness Score**.
* Exports full structured analytics to `BENCHMARK_RESULTS.json`.

---

### 10. Execution Mode 8: Dynamic Obstacle Statistical Analysis Suite

Generate high-resolution statistical analysis plots comparing Kalman tracking against noisy sensor readings:

```bash
python analyze_dynamic_obstacle.py
```

#### Generated Plots:
* **`dynamic_obstacle_analysis.png`**: 5-panel comprehensive figure showing Ground Truth vs Noisy Sensor vs Kalman Estimated trajectory, spatial tracking error over time, Kalman innovation residual distribution, error histogram, and covariance trace convergence.
* **`dynamic_intruder_analysis.png`**: High-speed closing-velocity evaluation showing dynamic sector inflation performance under Scenario 9.

---

### 11. Execution Mode 9: SINDy Non-linear Model Training

Discover data-driven aerodynamic and rigid-body governing equations from recorded simulation telemetry:

```bash
python run_sim.py --sindy-train
```

* Flies dynamic excitation maneuvers to collect telemetry logs ($`\mathbf{X}, \mathbf{U}`$).
* Constructs candidate nonlinear library $`\mathbf{\Theta}(\mathbf{X}, \mathbf{U})`$ including quadratic aerodynamic drag terms $`\mathbf{X}|\mathbf{X}|`$.
* Applies Sequential Thresholded Least Squares (STLSQ) to identify sparse physical equations.
* Saves discovered model parameters to `sindy_drone_model.json` for SINDy-DWA planning.

---

### 12. Complete CLI Arguments Reference Table

| Flag | Short Flag | Type | Default Value | Description |
| :--- | :---: | :---: | :---: | :--- |
| `--scenario` | `-s` | `int` | `1` | Scenario ID to execute (`1` through `10`). |
| `--algo` | `-a` | `str` | `'reactive'` | Avoidance algorithm (`reactive`, `apf`, `dwa`, `sindy_dwa`, `3d`, `neural`). |
| `--gui` | `-g` | `flag` | `False` | Launches the interactive native MuJoCo 3D graphics viewer. |
| `--dashboard` | — | `flag` | `False` | Launches the interactive Web Telemetry Dashboard server on port 8080. |
| `--port` | `-p` | `int` | `8080` | HTTP port number for the Web Dashboard server. |
| `--no-obs-noise` | — | `flag` | `False` | Disables stochastic Ornstein-Uhlenbeck obstacle drift (matches  No Noise). |
| `--obs-noise-std` | — | `float` | `0.08` | Standard deviation of obstacle stochastic process noise (0.00 to 0.20 m). |
| `--sensor-noise` | — | `float` | `0.06` | Standard deviation of Gaussian LiDAR measurement noise (0.00 to 0.15 m). |
| `--no-kalman` | — | `flag` | `False` | Disables Kalman filtering, passing raw noisy sensor hits directly to avoidance. |
| `--custom-file` | `-c` | `str` | `None` | Path to custom obstacle configuration JSON file exported from the arena editor. |
| `--benchmark` | `-b` | `flag` | `False` | Executes automated batch benchmark across all algorithms and scenarios. |
| `--sindy-train` | — | `flag` | `False` | Trains SINDy nonlinear dynamics model on flight telemetry and exports weights. |
| `--max-steps` | `-m` | `int` | `5000` | Maximum simulation physics steps per run (5000 steps ≈ 10 s flight). |
| `--seed` | — | `int` | `None` | Random seed for stochastic noise reproducibility. |
| `--sub-scenario` | — | `str` | `'closed_ring'` | Sub-scenario variant for Scenario 10 (`closed_ring` or `ring_with_gap`). |

---

### 13. Troubleshooting & FAQs

#### Q1: "Could not create MuJoCo GL context" on Linux / WSL2
* **Cause**: Headless server or WSL2 environment without an active X11 display.
* **Fix**: For headless benchmark runs, omit the `--gui` flag (runs purely in headless software mode). For headless GPU rendering or off-screen rendering, set:
  ```bash
  export MUJOCO_GL=egl
  ```
  If using WSL2 with VcXsrv or WSLg:
  ```bash
  export DISPLAY=:0
  ```

#### Q2: "Port 8080 already in use" when starting the Web Dashboard
* **Cause**: Another service or a previous instance of `app.py` is holding port 8080.
* **Fix**: Run the dashboard on a different port:
  ```bash
  python run_sim.py --dashboard --port 8085
  ```

#### Q3: "ModuleNotFoundError: No module named 'tkinter'"
* **Cause**: Linux systems often package Tkinter separately from Python.
* **Fix**: Install Tkinter via system package manager:
  ```bash
  sudo apt-get install python3-tk
  ```

#### Q4: Web Dashboard graphs show empty or flat curves
* **Cause**: Simulation has not been started yet.
* **Fix**: Click the green **`▶ Start Simulation`** button in the left sidebar. The graphs dynamically populate with telemetry as the quadcopter navigates the arena.

---

## 10. Mathematical & Algorithmic Formulations

### 1. SE(3) Geometric Controller
The quadcopter is governed by non-linear rigid-body dynamics on the Special Euclidean Group $`\mathrm{SE}(3) = \mathbb{R}^3 \rtimes \mathrm{SO}(3)`$. The controller implemented in `controllers/dsl_pid_controller.py` decouples translational trajectory tracking from full $`\mathrm{SO}(3)`$ rotational attitude tracking without Euler angle gimbal lock singularities.

#### A. Translational Position Error Dynamics & Desired Force Vector:
Given current position $`\mathbf{p} \in \mathbb{R}^3`$, velocity $`\mathbf{v} \in \mathbb{R}^3`$, and reference trajectory setpoints $`(\mathbf{p}_{\text{des}}, \mathbf{v}_{\text{des}}, \mathbf{a}_{\text{des}})`$, the translational error states are defined as:

```math
\mathbf{e}_p = \mathbf{p} - \mathbf{p}_{\text{des}}, \quad \mathbf{e}_v = \mathbf{v} - \mathbf{v}_{\text{des}}
```

The desired net control force vector $`\mathbf{F}_{\text{des}} \in \mathbb{R}^3`$ in the inertial world frame is computed as:

```math
\mathbf{F}_{\text{des}} = -k_p \mathbf{e}_p - k_v \mathbf{e}_v + m g \hat{\mathbf{z}}_W + m \mathbf{a}_{\text{des}}
```

#### Mathematical Term Breakdown for Translational Dynamics & Desired Force:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>e</b><sub>p</sub> | Position tracking error vector | ℝ<sup>3</sup> | m | Spatial deviation between actual drone position <b>p</b> and desired waypoint <b>p</b><sub>des</sub>. |
| <b>p</b> | Quadcopter center-of-mass position | ℝ<sup>3</sup> | m | Current 3D coordinates [x, y, z]<sup>T</sup> in inertial world coordinate frame. |
| <b>p</b><sub>des</sub> | Commanded reference position | ℝ<sup>3</sup> | m | Target reference waypoint [x<sub>des</sub>, y<sub>des</sub>, z<sub>des</sub>]<sup>T</sup> from planner. |
| <b>e</b><sub>v</sub> | Velocity tracking error vector | ℝ<sup>3</sup> | m/s | Velocity discrepancy between current velocity <b>v</b> and target velocity <b>v</b><sub>des</sub>. |
| <b>v</b> | Quadcopter translational velocity | ℝ<sup>3</sup> | m/s | Current linear velocity [ẋ, ẏ, ż]<sup>T</sup> in inertial world coordinate frame. |
| <b>v</b><sub>des</sub> | Commanded reference velocity | ℝ<sup>3</sup> | m/s | Target velocity vector from path planner (e.g. [1.20, 0, 0]<sup>T</sup> m/s). |
| <b>F</b><sub>des</sub> | Commanded desired net force vector | ℝ<sup>3</sup> | N | Total 3D force vector required in world frame to achieve trajectory tracking. |
| k<sub>p</sub> | Proportional position gain matrix | ℝ<sup>3×3</sup> (Diagonal) | N/m | k<sub>p</sub> = diag(8.5, 8.5, 12.0) N/m; vertical gain is stiffer to guarantee tight altitude hold. |
| -k<sub>p</sub> <b>e</b><sub>p</sub> | Proportional restoring force | ℝ<sup>3</sup> | N | Elastic spring-like force pulling the vehicle back toward the desired reference position. |
| k<sub>v</sub> | Derivative velocity damping gain | ℝ<sup>3×3</sup> (Diagonal) | N·s/m | k<sub>v</sub> = diag(4.5, 4.5, 6.0) N·s/m; suppresses position overshoot and oscillations. |
| -k<sub>v</sub> <b>e</b><sub>v</sub> | Velocity damping force | ℝ<sup>3</sup> | N | Dissipates kinetic energy and provides critical motion damping. |
| m | Total quadcopter vehicle mass | ℝ<sup>+</sup> (Scalar) | kg | m = 0.0472 kg (Crazyflie 2.x scale mass: 47.2 grams). |
| g | Standard acceleration of gravity | ℝ<sup>+</sup> (Scalar) | m/s<sup>2</sup> | Standard Earth gravitational acceleration constant (g = 9.81 m/s<sup>2</sup>). |
| <b>ẑ</b><sub>W</sub> | World vertical upward unit vector | ℝ<sup>3</sup> | Dimensionless | Cartesian unit vector [0, 0, 1]<sup>T</sup> defining upward direction in world frame. |
| m g <b>ẑ</b><sub>W</sub> | Feedforward gravity counter-force | ℝ<sup>3</sup> | N | Upward force [0, 0, 0.463]<sup>T</sup> N precisely offsetting vehicle weight to achieve perfect hover. |
| <b>a</b><sub>des</sub> | Commanded feedforward acceleration | ℝ<sup>3</sup> | m/s<sup>2</sup> | Reference linear acceleration [ẍ<sub>des</sub>, ÿ<sub>des</sub>, z̈<sub>des</sub>]<sup>T</sup>. |
| m <b>a</b><sub>des</sub> | Feedforward d'Alembert inertial force | ℝ<sup>3</sup> | N | Dynamic feedforward force eliminating trajectory tracking phase lag during accelerated maneuvers. |

---

#### B. Total Scalar Collective Thrust Command:
The quadcopter's 4 fixed rotors can only produce thrust aligned with the vehicle body vertical axis $`\hat{\mathbf{z}}_B = [0, 0, 1]^T`$. In the world coordinate system, this direction is given by the rotation matrix $`\mathbf{R} \in \mathrm{SO}(3)`$ as $`\mathbf{z}_B = \mathbf{R} \hat{\mathbf{z}}_B`$.

The required scalar collective thrust $`T`$ is obtained by projecting the desired force vector $`\mathbf{F}_{\text{des}}`$ onto the drone's actual current body $`Z`$-axis:

```math
T = \mathbf{F}_{\text{des}} \cdot (\mathbf{R} \hat{\mathbf{z}}_B) = \mathbf{F}_{\text{des}} \cdot \mathbf{z}_B
```

#### Mathematical Term Breakdown for Collective Thrust:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| T | Commanded total scalar thrust | ℝ<sup>+</sup> (Scalar) | N | Total aerodynamic lift force demanded from all 4 rotors combined. |
| <b>R</b> | Current quadcopter rotation matrix | SO(3) ⊂ ℝ<sup>3×3</sup> | Dimensionless | Orthonormal matrix transforming vectors from body frame ℱ<sub>B</sub> to world frame ℱ<sub>W</sub>. |
| <b>ẑ</b><sub>B</sub> | Body vertical unit axis | ℝ<sup>3</sup> | Dimensionless | Unit vector [0, 0, 1]<sup>T</sup> fixed in the quadcopter airframe body frame. |
| <b>z</b><sub>B</sub> | Actual world-frame body Z-axis | ℝ<sup>3</sup> | Dimensionless | Direction <b>R</b><b>ẑ</b><sub>B</sub> along which physical rotor thrust is currently exerted in space. |
| · | Euclidean dot product | Operator | — | Projects desired force along actual rotor thrust axis, avoiding vertical overshoot during aggressive bank angles. |

---

#### C. Desired Attitude Orthonormal Triad Construction:
To orient thrust toward $`\mathbf{F}_{\text{des}}`$, the desired body $`Z`$-axis $`\mathbf{z}_{B,\text{des}}`$ must align with the normalized force direction:

```math
\mathbf{z}_{B,\text{des}} = \frac{\mathbf{F}_{\text{des}}}{\|\mathbf{F}_{\text{des}}\|}
```

Given a desired heading yaw angle $`\psi_{\text{des}}`$, an auxiliary reference vector $`\mathbf{x}_C`$ is defined in the horizontal plane:

```math
\mathbf{x}_C = \begin{bmatrix} \cos\psi_{\text{des}} & \sin\psi_{\text{des}} & 0 \end{bmatrix}^T
```

The remaining orthonormal body axes $`\mathbf{y}_{B,\text{des}}`$ and $`\mathbf{x}_{B,\text{des}}`$ are constructed via sequential Gram-Schmidt vector cross products:

```math
\mathbf{y}_{B,\text{des}} = \frac{\mathbf{z}_{B,\text{des}} \times \mathbf{x}_C}{\|\mathbf{z}_{B,\text{des}} \times \mathbf{x}_C\|}
```

```math
\mathbf{x}_{B,\text{des}} = \mathbf{y}_{B,\text{des}} \times \mathbf{z}_{B,\text{des}}
```

Assembling these column vectors forms the complete singularity-free desired rotation matrix:

```math
\mathbf{R}_{\text{des}} = \begin{bmatrix} \mathbf{x}_{B,\text{des}} & \mathbf{y}_{B,\text{des}} & \mathbf{z}_{B,\text{des}} \end{bmatrix} \in \mathrm{SO}(3)
```

#### Mathematical Term Breakdown for Attitude Triad Construction:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>z</b><sub>B,des</sub> | Desired body Z-axis unit vector | ℝ<sup>3</sup> | Dimensionless | Unit vector pointing in the direction of required net accelerating force in world frame. |
| ‖ <b>F</b><sub>des</sub> ‖ | Norm of desired force vector | ℝ<sup>+</sup> (Scalar) | N | Total magnitude of required force vector √(F<sub>x</sub><sup>2</sup> + F<sub>y</sub><sup>2</sup> + F<sub>z</sub><sup>2</sup>). |
| ψ<sub>des</sub> | Commanded target heading (yaw) | ℝ (Scalar) | rad | Desired azimuthal flight direction in the horizontal X-Y plane. |
| <b>x</b><sub>C</sub> | Auxiliary planar heading reference | ℝ<sup>3</sup> | Dimensionless | Unit vector pointing in direction of desired yaw angle in horizontal plane. |
| × | Vector cross product operator | Operator | — | Computes orthogonal normal vector to span perpendicular body coordinate axes. |
| <b>y</b><sub>B,des</sub> | Desired lateral body Y-axis | ℝ<sup>3</sup> | Dimensionless | Port-side unit vector perpendicular to both desired thrust direction and horizontal heading. |
| <b>x</b><sub>B,des</sub> | Desired forward body X-axis | ℝ<sup>3</sup> | Dimensionless | Forward-pointing unit vector completing the orthonormal right-handed triad. |
| <b>R</b><sub>des</sub> | Desired attitude rotation matrix | SO(3) ⊂ ℝ<sup>3×3</sup> | Dimensionless | Complete singularity-free target orientation matrix: <b>R</b><sub>des</sub><sup>T</sup> <b>R</b><sub>des</sub> = <b>I</b>, det(<b>R</b><sub>des</sub>) = +1. |

---

#### D. $`\mathrm{SO}(3)`$ Rotational Tracking & Moment Control:
The attitude orientation error matrix on the Lie group $`\mathrm{SO}(3)`$ is converted to a vector error $`\mathbf{e}_R \in \mathbb{R}^3`$ using the vee operator $`(\cdot)^\vee`$:

```math
\mathbf{e}_R = \frac{1}{2} \left( \mathbf{R}_{\text{des}}^T \mathbf{R} - \mathbf{R}^T \mathbf{R}_{\text{des}} \right)^\vee
```

where the vee operator maps skew-symmetric Lie algebra elements $`\mathfrak{so}(3)`$ to Euclidean angular vectors:

```math
\begin{bmatrix} 0 & -e_z & e_y \\ e_z & 0 & -e_x \\ -e_y & e_x & 0 \end{bmatrix}^\vee = \begin{bmatrix} e_x \\ e_y \\ e_z \end{bmatrix}
```

The angular rate tracking error is:

```math
\mathbf{e}_\Omega = \boldsymbol{\Omega} - \mathbf{R}^T \mathbf{R}_{\text{des}} \boldsymbol{\Omega}_{\text{des}}
```

The commanded three-axis body torque vector $`\boldsymbol{\tau} = [\tau_x, \tau_y, \tau_z]^T`$ is computed as:

```math
\boldsymbol{\tau} = -k_R \mathbf{e}_R - k_\Omega \mathbf{e}_\Omega + \boldsymbol{\Omega} \times (\mathbf{J} \boldsymbol{\Omega})
```

#### Mathematical Term Breakdown for Attitude Error & Moments:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>e</b><sub>R</sub> | SO(3) attitude orientation error | ℝ<sup>3</sup> | rad | Vector representing angular discrepancy between current and target 3D orientation. |
| <b>R</b><sub>des</sub><sup>T</sup> <b>R</b> | Relative rotation matrix | SO(3) ⊂ ℝ<sup>3×3</sup> | Dimensionless | Attitude error rotation transforming from actual to desired frame. |
| (·)<sup>∨</sup> | Skew-symmetric vee isomorphism | 𝔰𝔬(3) → ℝ<sup>3</sup> | Dimensionless | Maps 3 × 3 skew-symmetric Lie algebra matrices to Euclidean 3-vectors. |
| <b>e</b><sub>Ω</sub> | Angular rate tracking error vector | ℝ<sup>3</sup> | rad/s | Difference between actual body angular rate and reference angular rate pulled back to body frame. |
| <b>Ω</b> | Actual body angular velocity vector | ℝ<sup>3</sup> | rad/s | Current tri-axial body rate [p, q, r]<sup>T</sup> measured directly by quadcopter gyroscopes. |
| <b>Ω</b><sub>des</sub> | Commanded reference angular velocity | ℝ<sup>3</sup> | rad/s | Target angular rate setpoint from trajectory generator. |
| <b>R</b><sup>T</sup> <b>R</b><sub>des</sub> <b>Ω</b><sub>des</sub> | Coordinate-transformed target rate | ℝ<sup>3</sup> | rad/s | Pulls reference angular rate from desired frame into current vehicle body frame. |
| <b>τ</b> | Commanded 3-axis body control torque | ℝ<sup>3</sup> | N·m | Vector of control moments [τ<sub>x</sub>, τ<sub>y</sub>, τ<sub>z</sub>]<sup>T</sup> exerted about vehicle principal axes. |
| τ<sub>x</sub> | Commanded roll control moment | ℝ (Scalar) | N·m | Moment about body X-axis governing lateral roll tilt. |
| τ<sub>y</sub> | Commanded pitch control moment | ℝ (Scalar) | N·m | Moment about body Y-axis governing longitudinal pitch tilt. |
| τ<sub>z</sub> | Commanded yaw control moment | ℝ (Scalar) | N·m | Moment about body Z-axis governing azimuthal heading rotation. |
| k<sub>R</sub> | Attitude stiffness gain matrix | ℝ<sup>3×3</sup> (Diagonal) | N·m/rad | Gain k<sub>R</sub> = diag(0.08, 0.08, 0.05) N·m/rad restoring orientation alignment. |
| k<sub>Ω</sub> | Angular rate damping gain matrix | ℝ<sup>3×3</sup> (Diagonal) | N·m·s/rad | Gain k<sub>Ω</sub> = diag(0.015, 0.015, 0.010) N·m·s/rad providing gyroscopic rate damping. |
| <b>J</b> | Vehicle mass moment of inertia tensor | ℝ<sup>3×3</sup> (Diagonal) | kg·m<sup>2</sup> | Diagonal inertia tensor <b>J</b> = diag(1.40, 1.40, 2.17) × 10<sup>-5</sup> kg·m<sup>2</sup>. |
| <b>Ω</b> × (<b>J</b> <b>Ω</b>) | Gyroscopic Coriolis torque compensation | ℝ<sup>3</sup> | N·m | Feedforward decoupling torque that cancels Euler rigid-body cross-coupling during aggressive turns. |

---

### 2. Motor Allocation & Mixer Matrix
The four individual rotor thrusts $`\mathbf{f} = [f_1, f_2, f_3, f_4]^T`$ required to achieve collective thrust $`T`$ and control moments $`\boldsymbol{\tau}`$ are resolved via the linear allocation mixer:

```math
\begin{bmatrix} T \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix} = \begin{bmatrix}
1 & 1 & 1 & 1 \\
\frac{d}{\sqrt{2}} & -\frac{d}{\sqrt{2}} & \frac{d}{\sqrt{2}} & -\frac{d}{\sqrt{2}} \\
-\frac{d}{\sqrt{2}} & -\frac{d}{\sqrt{2}} & \frac{d}{\sqrt{2}} & \frac{d}{\sqrt{2}} \\
k_m & -k_m & -k_m & k_m
\end{bmatrix} \begin{bmatrix} f_1 \\ f_2 \\ f_3 \\ f_4 \end{bmatrix}
```

#### Mathematical Term Breakdown for Motor Mixer Matrix:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>f</b> | Rotor individual thrust vector | ℝ<sup>4</sup> | N | Aerodynamic thrust forces [f<sub>1</sub>, f<sub>2</sub>, f<sub>3</sub>, f<sub>4</sub>]<sup>T</sup> produced by each of the 4 propellers. |
| f<sub>1</sub> | Front-Left rotor thrust (Motor 1) | ℝ<sup>+</sup> (Scalar) | N | Thrust of Front-Left motor (rotates Counter-Clockwise, CCW). |
| f<sub>2</sub> | Front-Right rotor thrust (Motor 2) | ℝ<sup>+</sup> (Scalar) | N | Thrust of Front-Right motor (rotates Clockwise, CW). |
| f<sub>3</sub> | Rear-Left rotor thrust (Motor 3) | ℝ<sup>+</sup> (Scalar) | N | Thrust of Rear-Left motor (rotates Clockwise, CW). |
| f<sub>4</sub> | Rear-Right rotor thrust (Motor 4) | ℝ<sup>+</sup> (Scalar) | N | Thrust of Rear-Right motor (rotates Counter-Clockwise, CCW). |
| d | Vehicle arm radius | ℝ<sup>+</sup> (Scalar) | m | Radial distance from center-of-mass to motor axis (d = 0.040 m = 4.0 cm). |
| d/√2 | Perpendicular geometric moment arm | ℝ<sup>+</sup> (Scalar) | m | Perpendicular distance from roll/pitch axes to motor in 45<sup>°</sup> "X" geometry (0.040/√2 ≈ 0.02828 m). |
| k<sub>m</sub> | Rotor torque-to-thrust proportionality ratio | ℝ<sup>+</sup> (Scalar) | m | Aerodynamic drag coefficient ratio (k<sub>m</sub> = 0.00596 m); relates reactive rotor drag torque Q<sub>i</sub> = k<sub>m</sub> f<sub>i</sub> to thrust. |

**Detailed Mechanical Row Interpretation**:
* **Row 1 (Total Thrust $`T`$)**: Sum of all 4 individual rotor forces: $`T = f_1 + f_2 + f_3 + f_4`$.
* **Row 2 (Roll Moment $`\tau_x`$)**: Generated by differential thrust between the left motors (1, 3) and right motors (2, 4). In an "X" frame with arm length $`d = 0.040\text{ m}`$, the perpendicular moment arm is $`d/\sqrt{2} \approx 0.02828\text{ m}`$.
* **Row 3 (Pitch Moment $`\tau_y`$)**: Generated by differential thrust between the front motors (1, 2) and rear motors (3, 4) with perpendicular moment arm $`d/\sqrt{2}`$.
* **Row 4 (Yaw Moment $`\tau_z`$)**: Generated by rotor aerodynamic reactive drag torque $`Q_i = k_m f_i`$, where $`k_m = 0.00596\text{ m}`$ is the torque-to-thrust ratio. Under the sign convention used here, counter-clockwise motors (Motors 1 and 4) contribute positive yaw torque ($`+k_m`$) and clockwise motors (Motors 2 and 3) contribute negative yaw torque ($`-k_m`$).

The individual rotor thrust commands are bounded within physical limits $`f_i \in [0.0, 0.155]\text{ N}`$ and converted to motor PWM commands.

---

### 3. Kalman Filter Gain & Innovation Covariance Formulation
During each sensor observation cycle, the Kalman Filter updates its state through four sequential mathematical steps:

#### Step 1: Innovation Residual ($`\mathbf{y}_k \in \mathbb{R}^3`$)
Computes the spatial discrepancy between the noisy LiDAR measurement and the a priori position prediction:

```math
\mathbf{y}_k = \mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_{k|k-1}
```

#### Step 2: Innovation Covariance ($`\mathbf{S}_k \in \mathbb{R}^{3\times3}`$)
Measures the composite uncertainty of the residual by projecting a priori state covariance into measurement space and adding sensor noise:

```math
\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}
```

#### Step 3: Optimal Kalman Gain Matrix ($`\mathbf{K}_k \in \mathbb{R}^{6\times3}`$)
Calculates the optimal weighting between kinematic model prediction and new LiDAR observation:

```math
\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T \mathbf{S}_k^{-1}
```

* When sensor noise is high ($`\mathbf{R}`$ large), $`\mathbf{K}_k \to 0`$, favoring smooth model integration.
* When prediction uncertainty is high ($`\mathbf{P}_{k|k-1}`$ large), $`\mathbf{K}_k \to \mathbf{H}^+`$ (pseudo-inverse of $`\mathbf{H}`$), favoring new sensor measurements.

#### Step 4: A Posteriori State Estimate Update ($`\hat{\mathbf{x}}_{k|k} \in \mathbb{R}^6`$)
Updates both the position and unmeasured velocity states using the weighted innovation:

```math
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k
```

#### Mathematical Term Breakdown for Kalman Filter Estimation Steps:
| Term / Symbol | Mathematical Description | Vector / Matrix Space | Physical SI Units | Simulation Interpretation & Role |
| :--- | :--- | :---: | :---: | :--- |
| <b>y</b><sub>k</sub> | Innovation residual vector | ℝ<sup>3</sup> | m | Difference between observed LiDAR hit <b>z</b><sub>k</sub> and predicted obstacle position <b>H</b><b>x̂</b><sub>k∣k-1</sub>. |
| <b>z</b><sub>k</sub> | Raw sensor measurement vector | ℝ<sup>3</sup> | m | 3D position vector [z<sub>x</sub>, z<sub>y</sub>, z<sub>z</sub>]<sup>T</sup> from the current LiDAR frame. |
| <b>x̂</b><sub>k∣k-1</sub> | A priori state estimate vector | ℝ<sup>6</sup> | [m, m/s]<sup>T</sup> | State predicted from previous step prior to measurement incorporation (<b>x̂</b><sub>k∣k-1</sub> = <b>F</b><b>x̂</b><sub>k-1∣k-1</sub>). |
| <b>H</b> | Observation projection matrix | ℝ<sup>3×6</sup> | Dimensionless | Projects 6-DOF state to 3D position subspace [<b>I</b><sub>3×3</sub>    <b>0</b><sub>3×3</sub>]. |
| <b>S</b><sub>k</sub> | Innovation covariance matrix | ℝ<sup>3×3</sup> | m<sup>2</sup> | Total expected variance of innovation residual <b>y</b><sub>k</sub>, combining prediction uncertainty and sensor noise. |
| <b>P</b><sub>k∣k-1</sub> | A priori state error covariance | ℝ<sup>6×6</sup> | Mixed (m<sup>2</sup>, m<sup>2</sup>/s, m<sup>2</sup>/s<sup>2</sup>) | Covariance matrix of state prediction error before incorporating current measurement. |
| <b>R</b> | Measurement noise covariance | ℝ<sup>3×3</sup> | m<sup>2</sup> | Sensor noise covariance σ<sub>sensor</sub><sup>2</sup> <b>I</b><sub>3×3</sub> (0.06<sup>2</sup> <b>I</b> ≈ 0.0036 <b>I</b> m<sup>2</sup>). |
| <b>K</b><sub>k</sub> | Optimal Kalman gain matrix | ℝ<sup>6×3</sup> | Mixed (dimensionless, 1/s) | Blends predicted model trajectory with fresh LiDAR point measurements to minimize trace of <b>P</b><sub>k∣k</sub>. |
| <b>S</b><sub>k</sub><sup>-1</sup> | Inverse innovation covariance matrix | ℝ<sup>3×3</sup> | m<sup>-2</sup> | Precision matrix weighting the spatial innovation vector. |
| <b>x̂</b><sub>k∣k</sub> | A posteriori filtered state estimate | ℝ<sup>6</sup> | [m, m/s]<sup>T</sup> | Optimal filtered obstacle state incorporating measurement <b>z</b><sub>k</sub> at step k. |

The covariance matrix $`\mathbf{P}_{k|k}`$ is then updated using the numerically stabilized Joseph equation detailed in Section 5.

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
