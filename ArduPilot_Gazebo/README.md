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