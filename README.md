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

## Inverse Kinematics Module

The package includes an inverse kinematics (IK) module for the A1 quadruped robot. This module provides analytical inverse kinematics solutions for controlling the robot's joint angles based on desired foot positions and body pose.

### Using the Inverse Kinematics Solver

```python
from inverse_kinematics import InverseKinematics

# Initialize IK solver with default A1 dimensions
ik_solver = InverseKinematics()

# Define foot positions (FR, FL, RR, RL) - each [x, y, z] in world frame
feet_positions = np.array([
    [0.1838, -0.147, -0.3],    # FR
    [0.1838,  0.147, -0.3],    # FL
    [-0.1838, -0.147, -0.3],   # RR
    [-0.1838,  0.147, -0.3]    # RL
])

# Define body position and orientation
position = [0.0, 0.0, 0.3]  # [x, y, z]
orientation = [0.0, 0.0, 0.0]  # [roll, pitch, yaw] in radians

# Solve inverse kinematics
joint_angles = ik_solver.solve(feet_positions, position, orientation)

# Result: 12 joint angles in order [FR_hip, FR_thigh, FR_calf,
#                                   FL_hip, FL_thigh, FL_calf,
#                                   RR_hip, RR_thigh, RR_calf,
#                                   RL_hip, RL_thigh, RL_calf]
```

### Running IK Tests

The package includes a test script for the inverse kinematics solver that tests various body positions and orientations:

```bash
cd /path/to/quadruped_skate_mpc
python3 scripts/test_inverse_kinematics.py
```

This will generate a JSON file (`ik_test_results.json`) with test results.

### Visualizing IK Test Results

To visualize the test results in MuJoCo:

```bash
cd /path/to/quadruped_skate_mpc
python3 scripts/visualize_ik_tests.py
```

Controls in the visualization:
- **Left/Right arrows**: Cycle through different test poses
- **Space**: Pause/resume simulation
- **Backspace**: Reset simulation

## Future Development

The inverse kinematics module is planned to be implemented in C++ in the future for performance optimization while maintaining the same API.