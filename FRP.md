# Final Review Proposal (FRP)

# Project Title

**Obstacle Avoidance System for Autonomous Drones**

---

# 1. Introduction

Autonomous drones are increasingly used in applications such as surveillance, inspection, search and rescue, agriculture, and delivery. For these applications, a drone must be capable of navigating through an environment while detecting and avoiding obstacles in real time.

This project proposes the development of a **rule-based obstacle avoidance system for a quadcopter drone**. The system will enable the drone to navigate toward a predefined goal while detecting obstacles using simulated onboard sensors and dynamically selecting an appropriate avoidance direction.

The project will initially use a **PyBullet-based simulation environment** for developing and testing the obstacle avoidance algorithm. The developed navigation logic will then be integrated with **ArduPilot SITL and Gazebo Harmonic** to provide a more realistic flight simulation environment.

The proposed system will use sensor measurements and predefined navigation rules rather than machine learning. This makes the system interpretable, computationally lightweight, and suitable for real-time simulation.

---

# 2. Problem Statement

A drone navigating autonomously toward a target may encounter obstacles that block its direct path. A simple fixed-direction movement strategy is insufficient because the drone must continuously:

- Detect obstacles before collision.
- Determine the relative location of obstacles.
- Select a safe direction for avoidance.
- Move around the obstacle.
- Determine when the obstacle has been cleared.
- Return toward the original goal direction.
- Handle situations where an obstacle is extremely close.
- Stop after reaching the goal.

Therefore, the project aims to develop a rule-based navigation system capable of performing these tasks autonomously in simulation.

---

# 3. Objectives

The main objectives of the proposed project are:

1. To develop a simulated autonomous drone navigation system.
2. To implement real-time obstacle detection using range-based sensors.
3. To divide sensor information into multiple directional regions.
4. To develop a rule-based obstacle avoidance algorithm.
5. To dynamically select the safer side for obstacle avoidance.
6. To implement emergency behavior when an obstacle is detected at a very short distance.
7. To allow the drone to return toward the goal after avoiding an obstacle.
8. To implement a goal detection mechanism and stop the drone after reaching the goal.
9. To test the system under multiple obstacle configurations.
10. To port and validate the developed navigation logic using **ArduPilot SITL and Gazebo Harmonic**.
11. To evaluate the system using navigation and performance metrics such as path length, completion time, control steps, and avoidance events.

---

# 4. Proposed Methodology

The proposed system will consist of the following major components:

```text
Simulation Environment
        |
        v
Drone + Sensors
        |
        v
Sensor Data Acquisition
        |
        v
Obstacle Detection
        |
        v
Rule-Based Decision Making
        |
        ___________________
        |                  |
        v                  v
    Safe Direction      Emergency
        |               Handling
        v
Obstacle Avoidance
        |
        v
Return Toward Goal
        |
        v
Goal Detection
        |
        v
    Stop
```
## 4.1 Simulation Environment

The project will be developed and tested using two simulation environments:

```
PyBullet / gym-pybullet-drones
ArduPilot SITL + Gazebo Harmonic
mujoco
```

PyBullet will be used for the initial development and validation of the navigation algorithm. ArduPilot SITL and Gazebo Harmonic will then be used to validate the algorithm in an autopilot-based simulation environment.

# 5. Sensor-Based Obstacle Detection

The drone will use range measurements to detect obstacles in its surrounding environment.

In the PyBullet implementation, virtual sensing directions will be defined around the drone. The sensing directions include:

```
Far Left
Left
Front
Right
Far Right
Rear
```

The sensor measurements will be continuously evaluated to determine whether an obstacle is present in the drone's path.

For the ArduPilot + Gazebo implementation, a **360-degree GPU LiDAR** will be used. The LiDAR data will be received from Gazebo and processed by the Python navigation controller.

The LiDAR scan will be divided into logical directional regions corresponding to the navigation directions used by the obstacle avoidance algorithm.

# 6. Rule-Based Obstacle Avoidance

The proposed system will use a finite-state, rule-based navigation strategy.

The major navigation states are:
```
FORWARD
    |
    | Obstacle detected
    v
AVOID
    |
    | Obstacle cleared
    v
RETURN
    |
    v
FORWARD
    |
    | Goal reached
    v
GOAL_REACHED
```
An additional emergency state will handle obstacles detected at very short distances:

```
EMERGENCY_REVERSE
```
## 6.1 Forward State

During normal navigation, the drone moves toward the goal.

The sensor measurements are continuously monitored.

If the front region becomes blocked beyond the specified detection threshold, the obstacle avoidance process is triggered.

____________________________________________________________________________________________________________________________________

## 6.2 Side Selection

When an obstacle is detected, the system evaluates the free space on the left and right sides.

The side scores are calculated using both the immediate side measurement and the corresponding far-side measurement.

The side with greater available free space is selected as the avoidance direction.

This allows the drone to make a dynamic decision instead of always avoiding obstacles in a fixed direction.

__________________________________________________________________________________________________________________________________________
## 6.3 Avoidance State

After selecting an avoidance direction, the drone moves laterally around the obstacle while maintaining forward progress where appropriate.

The sensor measurements continue to be monitored during this state.

The drone does not return to normal goal-directed navigation until sufficient clearance from the obstacle has been confirmed.

___________________________________________________________________________________________________________________________________________
## 6.4 Return State

Once the obstacle has been cleared, the drone transitions back toward the goal direction.

The return state allows the drone to recover from the lateral deviation introduced during obstacle avoidance.

The drone then returns to the normal forward navigation state.

_______________________________________________________________________________________________________________________________________________
## 6.5 Emergency Reverse

If the detected obstacle is extremely close to the drone, continuing forward or immediately moving laterally may not be safe.

Therefore, an emergency reverse behavior will be used.

The drone temporarily moves away from the obstacle to increase the available clearance before continuing with obstacle avoidance.
_______________________________________________________________________________________________________________________________________________
## 6.6 Goal Reached

A predefined goal position will be used for navigation.

The current drone position will be continuously compared with the goal position.

When the drone enters the specified goal tolerance region:
```
GOAL_REACHED

```
will be triggered.

The drone will then stop its movement.

A visual goal marker will also be displayed in the Gazebo environment to make the target location clearly visible during simulation.

______________________________________________________________________________________________________________________________

# 7. ArduPilot + Gazebo Integration

After validating the navigation algorithm in PyBullet, the system will be integrated with:

ArduPilot SITL
Gazebo Harmonic
ardupilot_gazebo
Python MAVLink communication

The architecture will be:
```
Gazebo Harmonic
      |
      | LiDAR Data
      v
Python Navigation Controller
      |
      | MAVLink Velocity Commands
      v
ArduPilot SITL
      |
      v
Simulated Iris Drone
```
Gazebo will provide the simulated environment and sensor data.

ArduPilot will handle the drone's low-level flight control.

The Python controller will perform high-level navigation and obstacle avoidance decisions.

# 8. Communication

The Python controller will communicate with ArduPilot SITL using MAVLink through pymavlink.

The controller will send velocity commands to the simulated drone.

ArduPilot will convert these high-level commands into the required low-level flight control actions.

The Gazebo LiDAR data will be received through Gazebo Transport.

Therefore, the system will combine:
```
Gazebo Transport
        |
        v
Sensor Data
        |
        v
Python Controller
        |
        v
MAVLink
        |
        v
ArduPilot
```
# 9. Testing

The system will be evaluated using multiple obstacle configurations.

Different scenarios will be created to test:

```
Direct obstacle blockage.
Obstacles requiring left-side avoidance.
Obstacles requiring right-side avoidance.
Obstacles at different distances.
Multiple obstacle configurations.
Close-range emergency situations.
Successful goal reaching.
```

The same general navigation logic will be evaluated across different simulation environments.

# 10. Performance Evaluation

The following metrics will be used to evaluate the system:

### Path Length

The total distance travelled by the drone from the start position to the goal.

### Completion Time

The time required for the drone to complete the navigation task.

### Control Steps

The number of control iterations required to complete the scenario.

### Avoidance Events

The number of times the obstacle avoidance mechanism is activated.

### Emergency Reversals

The number of emergency reverse actions required during navigation.

### Goal Completion

Whether the drone successfully reaches the predefined goal.

The results from different scenarios will be compared to evaluate the effectiveness of the navigation strategy.

# 11. Expected Outcomes

The expected outcomes of the project are:

* 1) A functional autonomous drone navigation system in simulation.
2) Successful obstacle detection using range-based sensing.
3) Dynamic selection of obstacle avoidance direction.
4) Successful navigation around obstacles.
5) Emergency handling for close-range obstacles.
6) uccessful return toward the goal after obstacle avoidance.
7) Automatic stopping after reaching the goal.
8) Successful integration with ArduPilot SITL.
9) Successful simulation using Gazebo Harmonic.
10) Performance evaluation across multiple obstacle scenarios.*

# 12. Technologies Used
| Component              | Technology                      |
| ---------------------- | ------------------------------- |
| Programming Language   | Python                          |
| Initial Simulation     | PyBullet                        |
| Drone Simulation       | gym-pybullet-drones             |
| Autopilot              | ArduPilot SITL                  |
| Simulation Environment | Gazebo Harmonic                 |
| Gazebo Integration     | ardupilot_gazebo                |
| Communication          | MAVLink                         |
| Python MAVLink Library | pymavlink                       |
| Obstacle Sensor        | GPU LiDAR                       |
| Navigation Method      | Rule-Based Finite-State Control |

# 13. Scope

The project focuses on autonomous obstacle avoidance in a simulated environment.

The current scope includes:

```
Single-drone navigation.
Static obstacle avoidance.
Range-based obstacle detection.
Goal-directed navigation.
Rule-based decision making.
ArduPilot SITL integration.
Gazebo Harmonic simulation.
```

The project does not currently focus on:

```
Multi-drone coordination.
Dynamic moving obstacles.
Real-world flight testing.
Machine learning-based navigation.
Vision-based obstacle detection.
```

These areas can be considered as possible future extensions.

# 14. Future Enhancements

Possible future enhancements include:

**Support for dynamic and moving obstacles.**
**Integration of camera-based obstacle detection.**
**Integration of additional sensors.**
**More complex environments.**
**Multi-drone coordination.**
**Real-world hardware testing.**
**Optimization of navigation parameters.**
**Comparison with other classical navigation algorithms.**

# 15. Deliverables

The proposed project will deliver:

```
PyBullet-based obstacle avoidance implementation.
ArduPilot SITL-based drone controller.
Gazebo Harmonic simulation environment.
GPU LiDAR-based obstacle detection.
Rule-based navigation state machine.
Goal detection and stopping mechanism.
Multiple obstacle test scenarios.
Performance results and evaluation.
Documentation and setup instructions.
```

# 16. Conclusion

The proposed project aims to develop and evaluate a rule-based autonomous drone obstacle avoidance system. The system will combine range-based sensing, finite-state decision making, goal-directed navigation, and emergency obstacle handling.

The navigation algorithm will first be developed and evaluated in PyBullet and subsequently integrated with ArduPilot SITL and Gazebo Harmonic. This provides a progression from algorithm development to autopilot-based simulation while maintaining an interpretable and computationally lightweight rule-based approach.

The final system is expected to demonstrate reliable obstacle avoidance and goal-directed navigation across multiple simulated environments and obstacle configurations.

# Structure
```
Drones_S5_AB3_Obstacle_Avoidance_Drone/
│
├── FRP.md                         ← Professor's proposed-work document
│
├── README.md                      ← Overall project README
│
├── obstacle_avoidance.py          ← PyBullet implementation
├── RESULTS.md
├── requirements.txt
│
├── terminalResults/
│
└── ArduPilot_Gazebo/
    │
    ├── README.md                  ← ArduPilot/Gazebo instructions
    ├── requirements.txt
    ├── obstacle_controller.py
    └── iris_runway.sdf
```

