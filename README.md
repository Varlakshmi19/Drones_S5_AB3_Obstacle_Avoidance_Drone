<p align="center">
  <img src="assets/amrita_logo.jpg"
       alt="Amrita Vishwa Vidyapeetham Logo"
       width="300">
</p>

<h1 align="center">Autonomous Obstacle Avoidance for Quadrotor Drones</h1>

<p align="center">
  <b>Data-Driven Control of Drones</b><br>
  Team S5 AB3<br>
  Amrita Vishwa Vidyapeetham
</p>

---

# Team Members

| S. No. | Name | Roll Number | Email |
|---:|---|---|---|
| 1 | Gade Varshitha | CB.SC.U4AIE24114 | varshithareddygade54@gmail.com |
| 2 | Jampana Lakshmi Tejaswi | CB.SC.U4AIE24116 | jlakshmitejaswi@gmail.com |
| 3 | Konkimalla Laxmi Vignesh | CB.SC.U4AIE24124 | laxmivigneshkonkimalla@gmail.com |
| 4 | Koruprolu Sri Durga Varalakshmi | CB.SC.U4AIE24125 | varshakoruprolu19@gmail.com|
| 5 | Ch. Mounica | CB.SC.U4AIE24169 | mounikachevvakula@gmail.com |

**Amrita Vishwa Vidyapeetham** | School of Artificial Intelligence | Coimbatore Campus | 2026

---
<a id="table-of-contents"></a>
# Table of Contents
 [Abstract](#abstract)
1. [Introduction](#introduction)
2. [Problem Statement](#problem-statement)
3. [Objectives](#objectives)
4. [Base Paper](#base-paper)
5. [Project Scope](#project-scope)

6. [Methodology](#methodology)
   - [6.1 Simulation Environment](#61-simulation-environment)
   - [6.2 Drone Model](#62-drone-model)
   - [6.3 Obstacle Configuration](#63-obstacle-configuration)
   - [6.4 Virtual Obstacle Sensing](#64-virtual-obstacle-sensing)
   - [6.5 Goal Direction Calculation](#65-goal-direction-calculation)
   - [6.6 Obstacle-Avoidance Decision](#66-obstacle-avoidance-decision)
   - [6.7 Navigation State Machine](#67-navigation-state-machine)
   - [6.8 Velocity Command Generation](#68-velocity-command-generation)
   - [6.9 PID-Based Drone Control](#69-pid-based-drone-control)
   - [6.10 Emergency Recovery](#610-emergency-recovery)
   - [6.11 Stuck Detection](#611-stuck-detection)
   - [6.12 Complete Control Loop](#612-complete-control-loop)

7. [Experimental Setup](#experimental-setup)

8. [Experimental Results](#experimental-results)
   - [8.1 Scenario Results](#81-scenario-results)
   - [8.2 Direct Obstacle](#82-scenario-1---direct-obstacle)
   - [8.3 Left-Side Obstacle](#83-scenario-2---left-side-obstacle)
   - [8.4 Right-Side Obstacle](#84-scenario-3---right-side-obstacle)
   - [8.5 Narrow Passage](#85-scenario-4---narrow-passage)
   - [8.6 Dead-End-Like Configuration](#86-scenario-5---dead-end-like-configuration)
   - [8.7 Multiple Staggered Obstacles](#87-scenario-6---multiple-staggered-obstacles)

9. [Performance Analysis](#performance-analysis)
10. [Key Observations](#key-observations)
11. [Project Structure](#project-structure)
12. [Technologies Used](#technologies-used)
13. [How to Run](#how-to-run)
14. [Experimental Metrics](#experimental-metrics)
15. [Limitations](#limitations)
16. [Future Work](#future-work)
17. [Conclusion](#conclusion)
18. [References](#references)

---
# Abstract                                                                                 <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>


Autonomous navigation in cluttered environments is an important capability for quadrotor drones used in applications such as inspection, surveillance, search and rescue, and indoor navigation. A drone operating autonomously must detect nearby obstacles, select a safe direction of motion, recover from difficult configurations, and reach a predefined target without collision.

This project implements and evaluates a reactive obstacle-avoidance framework for a quadrotor drone in a PyBullet-based simulation environment. The system uses virtual range sensing to obtain obstacle-distance information around the drone. These measurements are processed to determine whether the drone can continue toward the goal or should initiate an avoidance maneuver. A state-based navigation mechanism handles normal movement, obstacle avoidance, return-to-goal behavior, and emergency recovery, while PID-based position control is used for drone motion.

The implementation was evaluated using six predefined obstacle configurations. The performance was measured using control steps, completion time, travelled path length, collision steps, avoidance events, and emergency reversals. All six test scenarios successfully reached the target, giving a 100% success rate with zero recorded collision steps. The average completion time was 12.97 seconds and the average path length was 7.881 m.

---
<a id="introduction"></a>
# 1. Introduction                                                                               <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

Quadrotor drones are increasingly being used in environments where autonomous navigation is required. Although a drone can follow a predefined trajectory in an obstacle-free environment, navigation becomes more challenging when obstacles are present.

An autonomous obstacle-avoidance system must continuously determine whether the current direction is safe and, when necessary, select an alternative direction that allows the drone to continue toward its target.

Traditional approaches can use cameras, LiDAR, SLAM, or other sensing systems to construct detailed representations of the environment. Such approaches can provide rich information but may require greater computational resources and more complex sensing systems.

The base paper considered in this project investigates computationally inexpensive adaptive obstacle avoidance for autonomous drones. The work focuses on enabling a drone to adapt its motion to obstacles and avoid difficult situations such as deadlocks and corners.

This project studies the obstacle-avoidance problem through simulation and implements a reactive navigation framework in the `gym-pybullet-drones` environment. The system is evaluated under multiple predefined obstacle configurations.

---
<a id="problem-statement"></a>
# 2. Problem Statement                                                                           <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

The objective of this project is to develop and evaluate an autonomous quadrotor navigation system capable of reaching a predefined target while avoiding obstacles in a simulated environment.

The system should:

- Detect obstacles surrounding the drone.
- Estimate obstacle proximity using virtual sensing.
- Determine whether the current direction is safe.
- Select an appropriate avoidance direction when an obstacle is detected.
- Continue navigation toward the target after obstacle avoidance.
- Recover from difficult navigation situations.
- Reach the target without collision.
- Provide measurable performance results for different environments.

---
<a id="objectives"></a>
# 3. Objectives                                                                                  <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

The main objectives of the project are:

### 3.1 Obstacle Detection

Detect nearby obstacles using virtual range-sensing directions.

### 3.2 Reactive Obstacle Avoidance

Select an appropriate direction when the drone encounters an obstacle.

### 3.3 Goal-Oriented Navigation

Ensure that obstacle avoidance remains oriented toward the predefined target.

### 3.4 Recovery Mechanism

Provide an emergency recovery mechanism when normal obstacle avoidance is insufficient.

### 3.5 Stable Drone Control

Use position control to generate stable motion toward the desired position.

### 3.6 Experimental Evaluation

Evaluate the system under multiple obstacle configurations using quantitative performance metrics.

---
<a id="base-paper"></a>
# 4. Base Paper                                                                                  <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>
## Development of Autonomous Drones for Adaptive Obstacle Avoidance in Real World Environments

The project is based on the study:

**Arne Devos, Emad Ebeid, and Poramate Manoonpong, "Development of Autonomous Drones for Adaptive Obstacle Avoidance in Real World Environments."**

The paper investigates an adaptive obstacle-avoidance approach designed to be computationally inexpensive. The motivation is to allow autonomous drones to operate in complex environments while avoiding difficult configurations such as deadlocks and corners.

The paper discusses a closed-loop obstacle-avoidance system and validates the approach through simulation and implementation on a physical drone platform.

### Relation to the Present Project

The present project studies the obstacle-avoidance problem described in the base paper and implements a simulation-based reactive obstacle-avoidance framework using the `gym-pybullet-drones` environment.

The project focuses on:

- Virtual obstacle sensing
- Reactive obstacle avoidance
- Goal-oriented navigation
- State-based recovery
- PID-based drone control
- Quantitative simulation evaluation

The implementation should therefore be considered an implementation and experimental study of the obstacle-avoidance problem rather than a claim of reproducing every component of the base paper exactly.

---
<a id="project-scope"></a>
# 5. Project Scope                                                                               <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

The project is currently focused on simulation-based autonomous obstacle avoidance.

The overall feedback loop can be represented as:

```text
             ┌──────────────────────┐
             │      Drone State     │
             └──────────┬───────────┘
                        │
                        ▼
             ┌──────────────────────┐
             │  Virtual Sensing     │
             │  / Ray Measurements  │
             └──────────┬───────────┘
                        │
                        ▼
             ┌──────────────────────┐
             │ Obstacle Evaluation  │
             └──────────┬───────────┘
                        │
                        ▼
             ┌──────────────────────┐
             │ Navigation State     │
             │      Machine         │
             └──────────┬───────────┘
                        │
                        ▼
             ┌──────────────────────┐
             │ Desired Position /   │
             │ Navigation Command   │
             └──────────┬───────────┘
                        │
                        ▼
             ┌──────────────────────┐
             │    PID Controller    │
             └──────────┬───────────┘
                        │
                        ▼
             ┌──────────────────────┐
             │   Drone Simulation   │
             │      in PyBullet     │
             └──────────┬───────────┘
                        │
                        └───────────────► Feedback
```
<a id="methodology"></a>
# 6. Methodology                                                                                 <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

## 6.1 Simulation Environment

The project uses the `gym-pybullet-drones` simulation framework together with the PyBullet physics simulator.

PyBullet provides the physics-based simulation environment in which the quadrotor interacts with the surrounding environment and obstacles. The `gym-pybullet-drones` framework provides the drone model, simulation environment, control interfaces, and supporting utilities required for quadrotor simulation.

The main obstacle-avoidance implementation is located at:

```text  
gym-pybullet-drones-main/
└── gym_pybullet_drones/
    └── examples/
        └── obstacle_avoidance.py

```
## 6.2 Drone Model

The simulation uses the Crazyflie 2.X quadrotor model (`CF2X`) provided by the `gym-pybullet-drones` framework.

The drone is initialized at:

- Start position: `(0.0, 0.0, 1.0)` m
- Goal position: `(6.0, 0.0, 1.0)` m
- Flight altitude: `1.0` m

The drone is simulated using PyBullet physics with:

- Physics frequency: 240 Hz
- Control frequency: 48 Hz

The controller used for the drone is `DSLPIDControl`.

The drone maintains approximately the same altitude while the obstacle-avoidance system primarily modifies its horizontal motion.


## 6.3 Obstacle Configuration

Six predefined obstacle scenarios are implemented to evaluate different navigation situations.

| Scenario | Configuration |
|---:|---|
| 1 | One obstacle directly ahead |
| 2 | One obstacle slightly toward the left |
| 3 | One obstacle slightly toward the right |
| 4 | Narrow passage formed by multiple obstacles |
| 5 | Dead-end-like arrangement with both sides partially blocked |
| 6 | Multiple staggered obstacles |

Each obstacle is represented as a stationary cube in the PyBullet environment.

The obstacle size used in the experiments is:

$$
0.30m \times 0.30m \times 0.30m
$$

The obstacle configurations are created using the `create_test_scenario()` function.

---

## 6.4 Virtual Obstacle Sensing

The system uses six virtual distance sensors implemented using PyBullet ray casting.

The six sensing directions are:

| Sensor | Angle relative to goal direction |
|---|---:|
| Far Left | +70° |
| Left | +35° |
| Front | 0° |
| Right | -35° |
| Far Right | -70° |
| Rear | 180° |

The goal direction is first calculated from the current drone position toward the target.

Each sensor then casts a ray from the drone in the corresponding direction.

The maximum sensing range is:

$$
R_s = 1.6m
$$

For every ray, the system determines whether an obstacle is encountered and calculates the distance to the detected obstacle.

The front obstacle detection threshold is:

$$
D_{detect}=0.75m
$$

An obstacle is considered to be within the emergency range when:

$$
D_{emergency}=0.30m
$$

The sensor visualization is also displayed in the PyBullet GUI using debug lines.

---

## 6.5 Goal Direction Calculation

The drone continuously calculates the horizontal direction from its current position toward the goal.

Let the current position be:

$$
\mathbf{p}=
[x,y,z]^T
$$

and the goal position be:

$$
\mathbf{p}_g=
[x_g,y_g,z_g]^T
$$

The goal displacement is:

$$
\mathbf{d}_g=\mathbf{p}_g-\mathbf{p}
$$

The vertical component is ignored for horizontal navigation.

The normalized goal direction is:

$$
\hat{\mathbf{d}}_g=
\frac{\mathbf{d}_g}
{\|\mathbf{d}_g\|}
$$

This direction is used as the reference direction for both normal movement and the virtual sensor orientations.

---

## 6.6 Obstacle-Avoidance Decision

When an obstacle is detected in front of the drone, the system selects either the left or right side for avoidance.

The available space on each side is represented using a weighted score.

The left score is calculated as:

$$
S_L =
0.65d_L+
0.35d_{FL}
$$

where:

- $d_L$ is the distance measured by the left sensor.
- $d_{FL}$ is the distance measured by the far-left sensor.

Similarly, the right score is:

$$
S_R =
0.65d_R+
0.35d_{FR}
$$

where:

- $d_R$ is the distance measured by the right sensor.
- $d_{FR}$ is the distance measured by the far-right sensor.

The side with the larger score is considered to have more available space.

Therefore:

$$
Side =
\begin{cases}
LEFT, & S_L \geq S_R\\
RIGHT, & S_R > S_L
\end{cases}
$$

A small score difference is handled using the previously selected side to reduce rapid left-right switching.

---

## 6.7 Navigation State Machine

The obstacle-avoidance system uses a state-based navigation mechanism.

The main navigation states are:

The navigation states are described below.

### FORWARD

In the `FORWARD` state, the drone moves toward the goal using the goal direction.

The drone remains in this state while the path ahead is sufficiently clear.

If the front sensor detects an obstacle within the detection threshold, the system switches to the `AVOID` state.

### AVOID

In the `AVOID` state, the drone moves laterally around the detected obstacle while maintaining a small forward component.

The avoidance direction is selected using the left and right free-space scores.

The drone remains in avoidance mode for a minimum number of control steps to prevent unstable switching.

Once the front direction remains clear for a specified number of consecutive sensor readings, the system transitions to the `RETURN` state.

### RETURN

The `RETURN` state is used to gradually move the drone back toward the direct path to the goal after passing an obstacle.

If another obstacle is detected, the system returns to the `AVOID` state.

When the drone is sufficiently close to the original path, the system switches back to `FORWARD`.

### EMERGENCY_REVERSE

The `EMERGENCY_REVERSE` state is activated when the drone is in a dangerous or trapped configuration.

This state can be triggered when:

- The front obstacle is within the emergency distance.
- Both left and right sides are blocked.
- The stuck detector indicates insufficient progress.

The drone temporarily moves backward to create additional space.

After the reverse maneuver, the system selects an avoidance direction and returns to the `AVOID` state.

The system can also select the opposite side after an emergency reversal to reduce the possibility of repeatedly entering the same blocked configuration.

### GOAL_REACHED

The `GOAL_REACHED` state is entered when the drone comes within the predefined goal tolerance.

The desired velocity is then set to zero and the simulation is terminated successfully.

---

## 6.8 Velocity Command Generation

After the navigation state is selected, the system generates a desired velocity command for the drone.

The velocity command depends on the current navigation mode.

### Forward Motion

In the `FORWARD` state, the drone moves directly toward the goal:

`v = v_forward × d_goal`

where:

- `v_forward` is the forward speed.
- `d_goal` is the normalized horizontal direction toward the goal.

The forward speed used in the implementation is `0.35 m/s`.

### Avoidance Motion

During the `AVOID` state, the drone combines forward motion with lateral motion.

For left-side avoidance:

`v = v_avoid × d_goal + v_side × d_left`

For right-side avoidance:

`v = v_avoid × d_goal + v_side × d_right`

The parameters used are:

- Avoidance forward speed: `0.18 m/s`
- Side speed: `0.38 m/s`

This allows the drone to move around an obstacle instead of stopping completely.

### Return Motion

After the obstacle is cleared, the `RETURN` state uses a reduced forward velocity of `0.28 m/s` to move the drone back toward the goal path.

### Emergency Reverse

When the drone is trapped, stuck, or the front obstacle is dangerously close, the system commands reverse motion:

`v = -v_reverse × d_goal`

The reverse speed is `0.22 m/s`.

The vertical velocity component is kept at zero so that the obstacle-avoidance system primarily operates in the horizontal plane.

---
## 6.9 PID-Based Drone Control

The generated velocity commands are converted into drone control inputs using the `DSLPIDControl` controller provided by the `gym-pybullet-drones` framework.

At every control step, the system provides the controller with:

- Current drone position
- Current drone orientation
- Current linear velocity
- Current angular velocity
- Desired target position
- Desired target velocity
- Desired roll, pitch, and yaw orientation

The target position is generated slightly ahead of the drone in the desired direction of motion. This helps the position and velocity commands remain consistent.

The controller then computes the required motor RPM values for the four motors of the quadrotor.

The resulting motor commands are passed to the PyBullet simulation.

The control loop therefore follows:

```text
Sensor Measurements
        ↓
Obstacle Evaluation
        ↓
Navigation State
        ↓
Desired Velocity
        ↓
Target Position
        ↓
DSLPIDControl
        ↓
Motor RPM Commands
        ↓
Quadrotor Motion
        ↓
Updated Drone State
        ↓
Sensor Measurements
```
The control frequency is 48 Hz, meaning the navigation and control system is updated 48 times per second.

---
## 6.10 Emergency Recovery




The system includes an emergency recovery mechanism for situations where normal obstacle avoidance may not be sufficient.

Emergency recovery is triggered when one of the following conditions occurs:

- The front obstacle is within the emergency distance.
- Both the left and right sides are blocked.
- The drone is detected to be stuck.

When an emergency condition occurs, the navigation state changes to `EMERGENCY_REVERSE`.

The drone then moves backward for a predefined number of control steps.

The reverse duration is:`1.2 seconds`

After the reverse operation is completed, the system returns to the `AVOID` state and selects an avoidance direction.

The implementation also attempts to select the opposite side after an emergency reversal when possible. This helps prevent the drone from repeatedly choosing the same blocked direction.

The purpose of this mechanism is to provide recovery from difficult configurations such as dead ends and narrow obstacle arrangements.

---
## 6.11 Stuck Detection

The system continuously monitors the drone's recent positions to determine whether it has become stuck.

A position history is maintained using a fixed-size sliding window.

The stuck detector uses:

- Stuck detection window: `2.0 seconds`
- Minimum required progress: `0.10 m`

If the drone travels less than `0.10 m` during the observation window while still being away from the goal, it is considered to have made insufficient progress.

The system then triggers the `EMERGENCY_REVERSE` state.

This mechanism helps the drone recover from situations where obstacle avoidance repeatedly produces little or no forward progress.

---
## 6.12 Complete Control Loop

The complete obstacle-avoidance process is executed repeatedly at the control frequency.

The overall procedure is:

1. Obtain the current drone state from the PyBullet simulation.
2. Read the six virtual distance sensors.
3. Calculate the distance from the drone to the goal.
4. Determine whether the front path is blocked.
5. Check for emergency conditions.
6. Check whether the drone is stuck.
7. Select the appropriate navigation state.
8. Select the left or right avoidance direction when required.
9. Generate the desired velocity command.
10. Generate the target position for the controller.
11. Compute motor RPM commands using `DSLPIDControl`.
12. Apply the motor commands to the drone.
13. Record performance metrics.
14. Repeat until the goal is reached or the simulation time limit is reached.

The control loop can be summarized as:

```text
             Current Drone State
                     ↓
              Virtual Sensors
                     ↓
             Obstacle Detection
                     ↓
          ┌──────────┴──────────┐
          │                     │
      Path Clear            Obstacle
          │                     │
          ↓                     ↓
       FORWARD              AVOIDANCE
          │                     │
          │              ┌──────┴──────┐
          │              │             │
          │           Left Side     Right Side
          │              │             │
          │              └──────┬──────┘
          │                     ↓
          │                  RETURN
          │                     │
          └──────────────┬──────┘
                         ↓
                  Goal Evaluation
                         ↓
                ┌────────┴────────┐
                │                 │
             Reached          Not Reached
                │                 │
                ↓                 ↓
          GOAL_REACHED      Repeat Control Loop
```

The loop continues until the drone reaches the predefined goal within the specified goal tolerance.

---
<a id="experimental-setup"></a>
# 7. Experimental Setup                                                                          <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>
The system was evaluated using six predefined obstacle scenarios.

For every scenario, the following metrics were recorded:
* Control steps
* Completion time
* Final position
* Path length
* Collision steps
* Avoidance events
* Emergency reversals

The goal position was:
`(6.0, 0.0, 1.0) m`

A scenario was considered successful when the drone entered the predefined goal tolerance.
---
<a id="experimental-results"></a>
# 8. Experimental Results                                                                        <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

## 8.1 Scenario Results
The six scenarios were executed independently.

| Scenario | Result | Control Steps | Time (s) | Path Length (m) | Collision Steps | Avoidance Events | Emergency Reversals |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| 1 | SUCCESS | 356 | 10.39 | 6.363 | 0 | 1 | 0 |
| 2 | SUCCESS | 307 | 9.38 | 5.846 | 0 | 0 | 0 |
| 3 | SUCCESS | 306 | 9.36 | 5.809 | 0 | 0 | 0 |
| 4 | SUCCESS | 746 | 18.95 | 11.155 | 0 | 4 | 4 |
| 5 | SUCCESS | 693 | 18.45 | 11.034 | 0 | 4 | 3 |
| 6 | SUCCESS | 396 | 11.28 | 7.081 | 0 | 2 | 0 |

## 8.2 Scenario 1 - Direct Obstacle

Scenario 1 contains a single obstacle directly in front of the drone.

The drone detected the obstacle and initiated one avoidance event. It successfully moved around the obstacle and continued toward the goal.

* **Result:** SUCCESS
* **Control Steps:** 356
* **Completion Time:** 10.39 s
* **Path Length:** 6.363 m
* **Collision Steps:** 0
* **Avoidance Events:** 1
* **Emergency Reversals:** 0
* <img width="1277" height="991" alt="image" src="https://github.com/user-attachments/assets/b8d7cb6e-5c22-4f45-809e-bd42f5700f91" />


## 8.3 Scenario 2 - Left-Side Obstacle

Scenario 2 places the obstacle slightly toward the left of the goal direction.

The drone successfully reached the target without requiring a recorded avoidance transition.

* **Result:** SUCCESS
* **Control Steps:** 307
* **Completion Time:** 9.38 s
* **Path Length:** 5.846 m
* **Collision Steps:** 0
* **Avoidance Events:** 0
* **Emergency Reversals:** 0
* <img width="1293" height="1029" alt="image" src="https://github.com/user-attachments/assets/da31bd8b-9ef1-4158-b463-632f7f2482fb" />


## 8.4 Scenario 3 - Right-Side Obstacle

Scenario 3 places the obstacle slightly toward the right.

The drone successfully reached the goal without collision.

* **Result:** SUCCESS
* **Control Steps:** 306
* **Completion Time:** 9.36 s
* **Path Length:** 5.809 m
* **Collision Steps:** 0
* **Avoidance Events:** 0
* **Emergency Reversals:** 0
* <img width="1278" height="990" alt="image" src="https://github.com/user-attachments/assets/258e9c0d-d51f-46f2-a288-a71aa64d2ef8" />


## 8.5 Scenario 4 - Narrow Passage

Scenario 4 contains multiple obstacles forming a narrow passage.

This scenario required significantly more navigation effort than the simpler cases. The system generated multiple avoidance events and emergency reversals before successfully reaching the goal.

* **Result:** SUCCESS
* **Control Steps:** 746
* **Completion Time:** 18.95 s
* **Path Length:** 11.155 m
* **Collision Steps:** 0
* **Avoidance Events:** 4
* **Emergency Reversals:** 4
* <img width="1275" height="997" alt="image" src="https://github.com/user-attachments/assets/cbad036e-8fd1-48aa-9229-10cfe3defa23" />


### 8.6 Scenario 5 - Dead-End-Like Configuration

Scenario 5 was designed to test recovery from a difficult obstacle configuration where both sides are partially blocked.

The drone successfully recovered using emergency reverse behavior and subsequently continued obstacle avoidance.

* **Result:** SUCCESS
* **Control Steps:** 693
* **Completion Time:** 18.45 s
* **Path Length:** 11.034 m
* **Collision Steps:** 0
* **Avoidance Events:** 4
* **Emergency Reversals:** 3
* <img width="1278" height="995" alt="image" src="https://github.com/user-attachments/assets/1d616558-eade-4a0b-94c2-10706f907fbb" />


## 8.7 Scenario 6 - Multiple Staggered Obstacles

Scenario 6 contains multiple obstacles arranged at different lateral positions.

The drone successfully navigated through the obstacle field using multiple avoidance decisions.

* **Result:** SUCCESS
* **Control Steps:** 396
* **Completion Time:** 11.28 s
* **Path Length:** 7.081 m
* **Collision Steps:** 0
* **Avoidance Events:** 2
* **Emergency Reversals:** 0
* <img width="1277" height="991" alt="image" src="https://github.com/user-attachments/assets/28c8d82a-d4e0-4d4d-b766-6632a0193b26" />

<a id="performance-analysis"></a>
# 9. Performance Analysis                                                                       <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

The experimental results demonstrate that the proposed reactive navigation framework successfully completed all six predefined scenarios.

### Success Rate
All six scenarios reached the goal:
* Successful scenarios = 6
* Total scenarios = 6
* Success Rate = 100%

### Collision Performance
No collision steps were recorded in any of the six scenarios.
* Total collision steps = 0

This indicates that the obstacle-detection and avoidance logic successfully prevented recorded contacts with the test obstacles under the evaluated simulation conditions.

### Completion Time
The fastest scenario was Scenario 3 with a completion time of:
* 9.36 s

The slowest scenario was Scenario 4 with:
* 18.95 s

The more complex scenarios required additional avoidance and recovery actions, resulting in longer completion times.

### Path Length
The shortest recorded path was Scenario 3:
* 5.809 m

The longest recorded path was Scenario 4:
* 11.155 m

The increase in path length for complex scenarios is expected because the drone must deviate from the direct route to avoid obstacles.

### Emergency Recovery
Emergency reversals were mainly required in the more constrained scenarios:
* Scenario 1: 0
* Scenario 2: 0
* Scenario 3: 0
* Scenario 4: 4
* Scenario 5: 3
* Scenario 6: 0

This shows that the emergency recovery mechanism was particularly important for the narrow-passage and dead-end-like environments.

<a id="key-observations"></a>
# 10. Key Observations                                                                           <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>
The experiments produced the following observations:

1. The drone successfully reached the goal in all six tested scenarios.
2. No collision steps were recorded across the experiments.
3. Simple obstacle configurations required fewer avoidance actions.
4. Narrow and constrained environments increased both path length and completion time.
5. The emergency reverse mechanism helped the drone recover from difficult configurations.
6. The state-based navigation approach reduced unnecessary switching between normal movement and avoidance.
7. The virtual ray sensors provided sufficient local obstacle information for the tested simulation environments.
8. The PID controller maintained stable drone motion while the navigation layer modified the desired horizontal movement.


<a id="project-structure"></a>
# 11. Project Structure                                                                         <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

The repository is intentionally kept minimal. The `gym-pybullet-drones` library is installed as an external dependency instead of copying the complete library into this repository.

```text
Drones_S5_AB3_Obstacle_Avoidance/
│
├── README.md
├── RESULTS.md
├── requirements.txt
├── obstacle_avoidance.py
├── .gitignore
└── assets/
    └── amrita_logo.jpg
```

### Main Files

- **`obstacle_avoidance.py`** – Main simulation, obstacle sensing, navigation state machine, avoidance logic, recovery mechanism, and experiment execution.
- **`RESULTS.md`** – Recorded experimental results and scenario-wise performance analysis.
- **`requirements.txt`** – Python dependency required to run the simulation.
- **`assets/amrita_logo.jpg`** – Project/README logo.


# 12. Technologies Used                                                                          <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>
| Technology | Purpose |
| :--- | :--- |
| Python | Main programming language |
| PyBullet | Physics simulation |
| gym-pybullet-drones | Quadrotor simulation framework |
| NumPy | Numerical computations |
| DSLPIDControl | Drone position and velocity control |
| PyBullet Ray Casting | Virtual obstacle sensing |
| Git | Version control |
| GitHub | Project repository |

<a id="how-to-run"></a>
# 13. How to Run                                                                                 <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

### 13.1 Create a Virtual Environment

```bash
python -m venv venv
```

Activate it on Windows:

```bash
venv\Scripts\activate
```

### 13.2 Install Dependencies

```bash
pip install -r requirements.txt
```

### 13.3 Select a Test Scenario

Open `obstacle_avoidance.py` and change:

```python
TEST_SCENARIO = 6
```

The available scenarios are:

| Value | Scenario |
|---:|---|
| 1 | Direct frontal obstacle |
| 2 | Left-side obstacle |
| 3 | Right-side obstacle |
| 4 | Narrow passage |
| 5 | Dead-end-like configuration |
| 6 | Multiple staggered obstacles |

### 13.4 Run the Simulation

```bash
python obstacle_avoidance.py
```

A PyBullet GUI window will open and the selected scenario will be simulated.

<a id="experimental-metrics"></a>
# 14. Experimental Metrics                                                                       <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

The simulation reports the following metrics after every run.

### Control Steps
Number of control-loop iterations required to reach the goal.

### Completion Time
Total simulation execution time until the goal is reached.

### Final Position
The final `(x, y, z)` position of the drone.

### Path Length
Total distance travelled by the drone during the simulation.

### Collision Steps
Number of control steps during which contact between the drone and an obstacle was detected.

### Avoidance Events
Number of transitions into the obstacle-avoidance state.

### Emergency Reversals
Number of times the emergency reverse mechanism was activated.

<a id="limitations"></a>
## 15. Limitations                                                                               <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

The current implementation has several limitations:
* The experiments are performed in simulation rather than on a physical drone.
* The obstacle sensors are simulated using PyBullet ray casting.
* The system uses local obstacle information rather than a complete map of the environment.
* The tested environments are predefined scenarios.
* The approach has not been evaluated under real-world sensor noise.
* The system primarily performs horizontal obstacle avoidance while maintaining approximately constant altitude.
* The current evaluation does not compare the method against multiple alternative obstacle-avoidance algorithms.

<a id="future-work"></a>
# 16. Future Work                                                                                <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

Future improvements can include:
* Testing the algorithm on physical quadrotor hardware.
* Replacing virtual ray sensors with real LiDAR or depth sensors.
* Adding camera-based obstacle detection.
* Extending obstacle avoidance to three-dimensional motion.
* Testing randomized obstacle environments.
* Introducing sensor noise and disturbances into the simulation.
* Comparing performance with other navigation algorithms.
* Optimizing the controller for real-time embedded deployment.
* Integrating SLAM or mapping for larger environments.
* Evaluating energy consumption and computational efficiency.
* Adding dynamic moving obstacles.
* Performing larger-scale statistical evaluation across multiple randomized trials.
---
<a id="conclusion"></a>
#  17. Conclusion                                                                                <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>

This project implemented a reactive autonomous obstacle-avoidance framework for a quadrotor drone using PyBullet and the gym-pybullet-drones simulation environment.

The system combines virtual ray-based obstacle sensing, goal-oriented navigation, side-selection logic, state-based obstacle avoidance, emergency recovery, and PID-based drone control.

Six predefined scenarios were used to evaluate the system, ranging from simple frontal obstacles to narrow passages, dead-end-like configurations, and multiple staggered obstacles.

All six scenarios successfully reached the predefined goal, resulting in a 100% observed success rate and zero recorded collision steps. The more constrained scenarios required additional avoidance and emergency recovery actions, demonstrating the importance of the recovery mechanism.

Overall, the simulation demonstrates that a computationally lightweight reactive navigation strategy can provide effective obstacle avoidance in the tested environments.

<a id="references"></a>
# 18. References                                                                                 <sub><sub><sub><a href="#table-of-contents">⬆Table of Contents</a></sub></sub></sub>
1. Arne Devos, Emad Ebeid, and Poramate Manoonpong. "Development of Autonomous Drones for Adaptive Obstacle Avoidance in Real World Environments."
[Base Paper](https://ieeexplore.ieee.org/iel7/8490807/8491778/08491889.pdf)
2. [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) simulation framework
3. PyBullet physics simulation engine.

