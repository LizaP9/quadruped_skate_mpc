# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This repository contains a ROS2 control package for a Unitree A1 quadruped robot with skates in MuJoCo simulation. The project focuses on foot trajectory planning and control for a quadruped robot on skates.

## Build and Run Commands

### Building the Project

```bash
# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Build the package
colcon build

# Source the workspace setup
source install/setup.bash
```

### Running the Simulation

```bash
# Launch the MuJoCo simulation with Unitree A1
ros2 launch quadruped_skate_mpc a1_mujoco.launch.py
```

### Publishing Test Commands

```bash
# Change the state of the robot (e.g., to prepare for pushing)
ros2 topic pub /a1/fsm std_msgs/msg/String '{data: "PREPARE_PUSH"}'
```

## Code Architecture

### Key Components

1. **FootTrajectoryPlanner Node**: The main control node that plans foot trajectories for the robot based on the current FSM state.
   - Subscribes to `/a1/fsm` topic for state changes
   - Publishes to `/a1/foot_des_positions` with desired foot positions
   - Implements Bezier curves for smooth trajectory planning

2. **MuJoCo Simulation**: Runs a physics simulation of the A1 robot with skates
   - Visualizes the robot in MuJoCo environment
   - Publishes robot state information on ROS2 topics
   - Handles control inputs for robot movement

### State Machine

The robot operates in several states:
- `STAND_ON_BOARD`: Default standing position
- `PREPARE_PUSH`: Prepares and executes a pushing motion
- `RETURN_TO_BOARD`: Returns to a position on the skateboard

### Custom Messages

- `States.msg`: Contains joint positions, velocities, torques, IMU data, and force sensor readings
- `FootPosition.msg`: Contains desired positions for all four feet (fr, fl, rr, rl)

## Development Notes

1. The project uses a Bezier curve implementation for smooth foot trajectories during state transitions.

2. The MuJoCo simulation handles both the A1 robot and the skate as a single combined model with:
   - Position state (32 elements): A1 CoM (3), A1 quaternion (4), A1 joints (12), skate CoM (3), skate quaternion (4), skate joints (6)
   - Velocity state (30 elements): A1 CoM linear vel (3), A1 angular vel (3), A1 joint vel (12), skate CoM linear vel (3), skate angular vel (3), skate joint vel (6)

3. The current implementation focuses on trajectory planning for the feet to execute skating motions.