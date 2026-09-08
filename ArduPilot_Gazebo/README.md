# ArduPilot + Gazebo Harmonic Obstacle Avoidance

This folder contains the ArduPilot + Gazebo Harmonic implementation of the drone obstacle avoidance system.

The system uses a simulated Iris quadcopter, GPU LiDAR-based obstacle detection, and a rule-based navigation algorithm.

No machine learning is used in this implementation.

## Features

- ArduPilot SITL-based flight control
- Gazebo Harmonic simulation
- Iris quadcopter
- 360° GPU LiDAR
- Rule-based obstacle detection
- Rule-based obstacle avoidance
- Goal-based navigation
- Emergency reverse behavior
- Visual goal marker in Gazebo
- MAVLink communication using 'pymavlink'

## System Architecture

```text
      Gazebo Harmonic
             |
             v
       360° GPU LiDAR
             |
             v
   obstacle_controller.py
             |
          MAVLink
             |
             v
      ArduPilot SITL
             |
             v
      Iris Quadcopter
             |
             v
     Obstacle Avoidance
             |
             v
            Goal
```
## 1. Repository Structure

```text
ArduPilot_Gazebo/
│
├── obstacle_controller.py
├── iris_runway.sdf
├── README.md
└── requirements.txt
```

### obstacle_controller.py

Main Python navigation controller.

The controller:

1. Connects to ArduPilot SITL using MAVLink.
2. Arms the drone.
3. Takes off to the specified altitude.
4. Receives 360° GPU LiDAR measurements from Gazebo.
5. Processes the LiDAR measurements into directional obstacle information.
6. Detects obstacles in the flight path.
7. Selects an appropriate avoidance direction.
8. Moves around the obstacle.
9. Returns toward the goal direction.
10. Stops when the goal is reached.

### iris_runway.sdf

Gazebo Harmonic simulation world.

It contains:

* Iris drone
* Runway
* GPU LiDAR sensor
* Obstacle
* Goal marker
* Gazebo simulation plugins

### requirements.txt

Contains the Python dependencies required by the controller.

## 2. Prerequisites

The following software is required:

* Ubuntu 24.04
* Python 3.10 or later
* ArduPilot
* Gazebo Harmonic
* ardupilot_gazebo
* Gazebo Transport Python bindings
* Gazebo Messages Python bindings
* pymavlink

The project can also be run inside WSL2 on Windows.

## 3. Install ArduPilot
clone ArduPilot

```bash
git clone https://github.com
```

Enter the ArduPilot directory:

```bash
cd ardupilot
```

Run the ArduPilot setup script:

```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Reload the shell environment:

```bash
. ~/.profile
```

Build ArduCopter:

```bash
./waf configure --board sitl
./waf copter
```

## 4. Install Gazebo Harmonic

Install Gazebo Harmonic according to the official Gazebo installation instructions.

Verify the installation:

```bash
gz sim --version
```

The output should indicate Gazebo Harmonic / Gazebo Sim 8.x.

## 5. Install ArduPilot-Gazebo Plugin

Clone the ArduPilot-Gazebo plugin:

```bash
mkdir -p ~/gz_ws/src
cd ~/gz_ws/src

git clone https://github.com
```

Build the plugin:

```bash
cd ~/gz_ws
mkdir -p build
cd build

cmake ../src/ardupilot_gazebo
make -j\$(nproc)
```

## 6. Configure Gazebo Environment

Add the following environment variables to `~/.bashrc`:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=\(HOME/gz_ws/src/ardupilot_gazebo/build:\){GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=\$HOME/gz_ws/src/ardupilot_gazebo/models:\(HOME/gz_ws/src/ardupilot_gazebo/worlds:\){GZ_SIM_RESOURCE_PATH}
```

Reload the environment:

```bash
source ~/.bashrc
```

## 7. Install Python Dependencies

Create a virtual environment:

```bash
python3 -m venv venv-ardupilot
```

Activate it:

```bash
source venv-ardupilot/bin/activate
```

Install the Python dependencies:

```bash
pip install -r requirements.txt
```

## 8. Gazebo Python Bindings

The controller uses Gazebo Transport and Gazebo Messages to receive LiDAR data.

Verify that the following imports work:

```bash
python3 -c "from gz.transport13 import Node; print('Gazebo Transport OK')"
```

and:

```bash
python3 -c "from gz.msgs10.laserscan_pb2 import LaserScan; print('Gazebo Messages OK')"
```

The exact Gazebo Python package names may depend on the installed Gazebo Harmonic/Ubuntu package versions.

## 9. Run the Simulation

Three terminals are recommended.

### Terminal 1 — Start Gazebo

Source the Gazebo environment:

```bash
source ~/.bashrc
```

Start the project world:

```bash
gz sim -r path/to/ArduPilot_Gazebo/iris_runway.sdf
```

Replace:

```text
path/to/ArduPilot_Gazebo/
```

with the location where this repository was cloned.

For example:

```bash
cd path/to/Drones_SS_A03_Obstacle_Avoidance_Drone/ArduPilot_Gazebo
```

then:

```bash
gz sim -r iris_runway.sdf
```

Gazebo should open and display the simulated drone, obstacle, runway, and goal marker.

### Terminal 2 — Start ArduPilot SITL

Open a second terminal.

Go to the ArduCopter directory:

```bash
cd ~/ardupilot/ArduCopter
```

Start SITL:

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model json --speedup 1 --console --map --out=127.0.0.1:14550
```

ArduPilot SITL will communicate with the Python controller through:

```text
127.0.0.1:14550
```

### Terminal 3 — Run the Controller

Open a third terminal.

Navigate to the project directory:

```bash
cd path/to/Drones_SS_A03_Obstacle_Avoidance_Drone/ArduPilot_Gazebo
```

Activate the Python virtual environment:

```bash
source venv-ardupilot/bin/activate
```

Run the controller:

```bash
python3 obstacle_controller.py
```
## 10. Execution Flow

After starting the controller, the following sequence occurs:

```text
Connect to ArduPilot
        |
        v
       ARM
        |
        v
     TAKEOFF
        |
        v
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
        |
        v
      STOP
```

If an obstacle is detected at a dangerously short distance, the controller can enter:

```text
EMERGENCY_REVERSE
```

to move away from the obstacle before continuing navigation.

## 11. LiDAR Processing

The Iris drone is equipped with a 360° GPU LiDAR sensor.

The LiDAR publishes data through:

```text
/iris/lidar
```

The controller receives the scan and evaluates several logical directions:

```text
Far Left
Left
Front
Right
Far Right
Rear
```

These measurements are used to determine:

* Whether an obstacle is in the current path
* Which side has more free space
* Whether the obstacle has been cleared
* Whether emergency reverse movement is required

# 12. Navigation States

The controller uses a finite-state navigation system.

### FORWARD

The drone moves toward the goal.

### AVOID

When an obstacle is detected, the controller selects the clearer side and moves laterally around the obstacle.

### RETURN

After clearing the obstacle, the drone returns toward the goal direction.

### EMERGENCY_REVERSE

If an obstacle is detected at a very short distance, the drone moves backward to increase separation.

### GOAL_REACHED

When the drone reaches the goal within the specified tolerance, forward movement is stopped.

## 13. Goal

The Gazebo world contains a visual goal marker.

The goal marker is included in:

```text
iris_runway.sdf
```

The marker is visual only and does not contain a collision element.

Therefore, it does not interfere with LiDAR obstacle detection.

The controller uses the predefined goal position to determine when the drone has completed navigation.

## 14. Configuration

Important navigation parameters can be modified in:


```text
obstacle_controller.py
```

Examples include:

```python
FORWARD_SPEED
AVOID_FORWARD_SPEED
SIDE_SPEED
RETURN_SPEED
REVERSE_SPEED
SENSOR_RANGE
DETECTION_DISTANCE
EMERGENCY_DISTANCE
CLEAR_DISTANCE
GOAL_TOLERANCE
```

Changing these values affects the drone's navigation and obstacle avoidance behavior.

## 15. Troubleshooting

### Gazebo cannot find the world

Make sure the Gazebo resource path is configured:

```bash
source ~/.bashrc
```

Then run:

```bash
gz sim -r iris_runway.sdf
```

from the `ArduPilot_Gazebo` directory.

### LiDAR topic is not available

Check the available Gazebo topics:

```bash
gz topic -l
```

The expected LiDAR topic is:

```text
/iris/lidar
```

To inspect the LiDAR data:

```bash
gz topic -e -t /iris/lidar
```

### Controller cannot connect to SITL

Make sure ArduPilot SITL is running with:

```text
--out=127.0.0.1:14550
```

The Python controller connects to:

```text
udp:127.0.0.1:14550
```

### pymavlink is missing

Activate the virtual environment:

## 16. Stopping the Simulation

Stop the Python controller:

```text
Ctrl + C
```

Stop Gazebo:

```text
Ctrl + C
```

Stop ArduPilot SITL:

```text
Ctrl + C
```

## 17. Notes

* This implementation is completely rule-based.
* No machine learning model is used.
* ArduPilot is responsible for low-level flight control.
* Python performs high-level navigation and obstacle avoidance.
* Gazebo provides the simulated environment and sensor data.
* GPU LiDAR is used for obstacle detection.
* The goal marker is provided for visualization only.

