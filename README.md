<div align="center">

# CrazyFlie Control

[![ROS2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&labelColor=2D2D2D&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/index.html)
&nbsp;&nbsp;
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-FF7043?style=for-the-badge&labelColor=2D2D2D&logo=gazebo&logoColor=white)](https://gazebosim.org/docs/harmonic/getstarted/)
&nbsp;&nbsp;
[![Python](https://img.shields.io/badge/Python-3.10-4CAF50?style=for-the-badge&labelColor=2D2D2D&logo=python&logoColor=white)](https://python.org)
&nbsp;&nbsp;
[![Docker](https://img.shields.io/badge/Docker-Ubuntu-2496ED?style=for-the-badge&labelColor=2D2D2D&logo=docker&logoColor=white)](https://docs.docker.com/)

A Crazyflie drone simulation running inside Gazebo, controlled via ROS 2. The drone takes off, hovers, then follows a figure-8 (well, kind of) trajectory using a PD position and attitude controller.

![Drone Simulation](assets/drone.gif)

</div>

## Stack

- ROS 2 Humble
- Gazebo Harmonic
- Docker (Ubuntu, WSL2 on Windows)

## Setup

Open the project in VS Code and reopen in container. Then:

```bash
cd /home/developer/ros2_ws
source ./setup.sh <your_unique_id>
./build.sh
source install/setup.bash
```

## Running

Before running anything, when terminal is first created we need to run:

```bash
source install/setup.bash
```

```bash
# Terminal 1 - simulation
ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py

# Terminal 2 - mixer
ros2 run cf_control mixer

# Terminal 3 - your controller
cd /home/developer/ros2_ws/src/my_controller
python3 drone_controller.py
```

The control interface is `/cf_control/control_command` - publish collective thrust and torque there.
