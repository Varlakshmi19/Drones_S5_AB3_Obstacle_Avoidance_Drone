# Experimental Results

## 1. Overview
The implemented autonomous obstacle-avoidance system was evaluated using six predefined test scenarios in the PyBullet simulation environment.

The experiments were designed to test the drone under different obstacle configurations, ranging from a single obstacle directly in the flight path to narrow passages, dead-end-like configurations, and multiple staggered obstacles.

The following performance metrics were recorded for every scenario:
* Control Steps
* Completion Time
* Final Position
* Path Length
* Collision Steps
* Avoidance Events
* Emergency Reversals

---

## 2. Experimental Setup
The drone starts from:
* **Start Position:** (0.0, 0.0, 1.0) m
* **Goal Position:** (6.0, 0.0, 1.0) m
* **Flight Altitude:** 1.0 m

The simulation uses the Crazyflie 2.X (CF2X) quadrotor model with PyBullet physics.

The simulation and controller frequencies are:
* **Physics Frequency:** 240 Hz
* **Control Frequency:** 48 Hz

Six obstacle configurations were tested independently by changing the `TEST_SCENARIO` value in `obstacle_avoidance.py`.

---
## 3. Test Scenarios

| Scenario | Description |
| :--- | :--- |
| 1 | One obstacle directly ahead |
| 2 | One obstacle slightly toward the left |
| 3 | One obstacle slightly toward the right |
| 4 | Narrow passage formed by multiple obstacles |
| 5 | Dead-end-like arrangement with both sides partially blocked |
| 6 | Multiple staggered obstacles |

---

## 4. Experimental Results

The results obtained from the six simulation runs are shown below.

| Scenario | Result | Control Steps | Completion Time (s) | Final Position (x, y, z) | Path Length (m) | Collision Steps | Avoidance Events |
| :---: | :---: | :---: | :---: | :--- | :---: | :---: | :---: |
| 1 | SUCCESS | 356 | 10.39 | (5.821, 0.158, 1.001) | 6.363 | 0 | 1 |
| 2 | SUCCESS | 307 | 9.38 | (5.772, -0.026, 1.001) | 5.846 | 0 | 0 |
| 3 | SUCCESS | 306 | 9.36 | (5.760, -0.000, 1.001) | 5.809 | 0 | 0 |
| 4 | SUCCESS | 746 | 18.95 | (5.848, -0.180, 1.001) | 11.155 | 0 | 4 |
| 5 | SUCCESS | 693 | 18.45 | (5.833, -0.171, 1.001) | 11.034 | 0 | 4 |
| 6 | SUCCESS | 396 | 11.28 | (5.953, 0.222, 1.001) | 7.081 | 0 | 2 |

---
## 5. Overall Performance

All six test scenarios successfully reached the predefined goal.

### Success Rate
The observed success rate was:

**100%**

All 6 out of 6 scenarios were completed successfully.

### Collision Performance
The system recorded:

**0 collision steps**

across all six scenarios.

This indicates that no control step was recorded in which the drone was in contact with one of the predefined obstacles.

### Average Performance
Across the six scenarios:

* **Average completion time:** approximately 12.97 s
* **Average path length:** approximately 7.88 m
* **Average control steps:** approximately 467.3
* **Total avoidance events:** 11
* **Total emergency reversals:** 7
---
## 6. Scenario-wise Analysis

### Scenario 1 – Direct Frontal Obstacle
The drone encountered one obstacle directly in its forward path.

The system detected the obstacle and initiated one avoidance event. No emergency reversal was required.

The drone successfully reached the goal in 10.39 seconds with a path length of 6.363 m.

---

### Scenario 2 – Left-Side Obstacle
The obstacle was positioned slightly toward the left of the nominal flight direction.

The drone reached the goal without triggering an explicit avoidance event or emergency reversal.

The completion time was 9.38 seconds, with a path length of 5.846 m.

---

### Scenario 3 – Right-Side Obstacle
The obstacle was positioned slightly toward the right of the nominal flight direction.

The drone successfully reached the target without recorded avoidance events or emergency reversals.

The completion time was 9.36 seconds, and the travelled path length was 5.809 m.

---

### Scenario 4 – Narrow Passage
Scenario 4 represented a more constrained environment using multiple obstacles to form a narrow passage.

The drone required 4 avoidance events and 4 emergency reversals before successfully reaching the goal.

The scenario required the highest number of control steps:
**746 control steps**

and produced the longest recorded path:
**11.155 m**

The completion time was 18.95 seconds.

This scenario demonstrates the importance of the emergency recovery mechanism when the available free space is highly constrained.

---

### Scenario 5 – Dead-End-Like Configuration (Continued)
The drone successfully recovered from the constrained configuration using:
* 4 avoidance events
* 3 emergency reversals

The drone reached the target in 18.45 seconds after travelling 11.034 m.

The result demonstrates that the recovery mechanism can help the drone escape a difficult local configuration and continue toward the goal.

---

### Scenario 6 – Multiple Staggered Obstacles
Scenario 6 contained multiple obstacles positioned at different locations along the drone's path.

The drone successfully navigated through the environment with:
* 2 avoidance events
* 0 emergency reversals

The completion time was 11.28 seconds, and the total path length was 7.081 m.

---

## 7. Key Observations

The experimental results provide the following observations:
1. The proposed reactive navigation system successfully completed all six predefined scenarios.
2. No collision steps were recorded in any of the six experiments.
3. Simple obstacle configurations required fewer avoidance actions.
4. More constrained environments, particularly Scenarios 4 and 5, required emergency recovery.
5. Scenario 4 produced the highest completion time and longest travelled path.
6. The emergency-reverse mechanism was activated only in the more difficult scenarios.
7. The drone maintained approximately the desired flight altitude of 1.0 m throughout the experiments.
8. The final x-coordinate was close to the target x-coordinate of 6.0 m in every scenario.
9. The results indicate that the state-based navigation mechanism can handle both ordinary obstacle avoidance and difficult local configurations in the tested simulation environments.

---




## 8. Performance Summary (Continued)

| Metric | Overall Observation |
| :--- | :--- |
| Collision steps | 0 |
| Total avoidance events | 11 |
| Total emergency reversals | 7 |
| Average completion time | 12.97 s |
| Average path length | 7.88 m |

---

## 9. Conclusion

The experimental evaluation demonstrates that the implemented reactive obstacle-avoidance framework can successfully navigate the six predefined obstacle configurations in the PyBullet simulation environment.

The system achieved a 100% observed success rate with zero recorded collision steps. Simple scenarios were completed with minimal intervention, while the more constrained scenarios triggered additional avoidance and emergency-recovery actions.

The results support the effectiveness of combining virtual ray-based sensing, goal-oriented navigation, state-based obstacle avoidance, emergency recovery, and PID-based drone control for the tested simulation environments.

These results are limited to the predefined simulation scenarios and should not be interpreted as evidence of equivalent performance in real-world environments.
