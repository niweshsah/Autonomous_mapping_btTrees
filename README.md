# Autonomous Mapping using Frontier Exploration with Behavior Trees

## Overview

This project implements an advanced autonomous mapping system for robotic exploration using cutting-edge technologies in ROS 2. By combining Frontier Exploration techniques with Behavior Trees, the system enables intelligent and efficient environment mapping with a TurtleBot3 robot.

## Features

- **Frontier-Based Exploration**: Autonomous exploration of unknown environments using intelligent frontier detection
- **Behavior Trees**: Modular and flexible decision-making framework for navigation and exploration
- **Real-Time SLAM**: Accurate simultaneous localization and mapping using SLAM Toolbox
- **TurtleBot3 Simulation**: Comprehensive virtual testing environment for robotic navigation

## System Architecture

The autonomous mapping system integrates several key components:
- ROS 2 Humble
- py_trees for Behavior Tree implementation
- SLAM Toolbox for mapping
- Nav2 for navigation
- TurtleBot3 robot simulation

## Prerequisites

Ensure the following software is installed:

- Ubuntu 22.04 LTS
- ROS 2 Humble
- TurtleBot3 packages
- Gazebo Simulation
- SLAM Toolbox
- Nav2

## Installation

### 1. Setup ROS 2 Workspace

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

### 2. Clone the Repository

```bash
git clone https://github.com/niweshsah/Autonomous_mapping_btTrees.git
```

### 3. Build the Workspace

```bash
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the Autonomous Mapping System

Open multiple terminals and execute the following commands:

### Terminal 1: Behavior Tree Frontier Exploration
```bash
ros2 run my_behavior_tree frontier_btTree
```

### Terminal 2: SLAM Toolbox
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

### Terminal 3: TurtleBot3 Gazebo Simulation
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 4: Navigation with Nav2
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    params_file:=/home/niweshsah/ros_ws/config/nav2_params.yaml \
    use_sim_time:=True
```

## Important Notes

- Verify ROS 2 Humble is properly installed and sourced
- Ensure the workspace is built and sourced after cloning
- The behavior tree node autonomously controls frontier-based exploration
- Default world is TurtleBot3 World, but custom worlds can be used
- Includes an unstuck mechanism for handling navigation challenges

## Troubleshooting

If the robot becomes persistently stuck:
- Use the **Reset** button in RViz for Nav2
- Check navigation parameters in the configuration file
- Verify sensor data and mapping accuracy

## Demo

A video demonstration of the autonomous mapping system is available. Please refer to the project repository for the latest demo.


## Contact

email : sahniwesh@gmail.com
