# quadruped_skate_mpc

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Overview

This repository contains a ROS2 control package for Unitree A1 in Mujoco simulation.

## Installation Guide

Follow these steps to install and run the project locally.

### Prerequisites

Make sure you have the following software installed if you are running on the local machine:

- [ROS](https://docs.ros.org/)
- [Mujoco](https://mujoco.org/)

### Package Install

You can now install the package using the following commands.

```bash
git clone https://github.com/LizaP9/quadruped_skate_mpc.git
cd quadruped_skate_mpc
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
source install/setup.bash
```

## Usage

To launch Mujoco simulation with Unitree A1.

```bash
ros2 launch quadruped_skate_mpc a1_mujoco.launch.py
```

To publish state:

```
ros2 topic pub /a1/fsm std_msgs/msg/String '{data: "PREPARE_PUSH"}'
```

