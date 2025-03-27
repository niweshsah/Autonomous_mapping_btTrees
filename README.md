# Autonomous Mapping using Frontier Exploration with Behavior Trees

This project implements an autonomous mapping system using **Frontier Exploration** along with **Behavior Trees** in ROS 2. The system utilizes **SLAM Toolbox**, **TurtleBot3 Simulation**, and **py\_trees** for efficient decision-making and mapping.

## Features

- **Frontier-Based Exploration**: The robot autonomously explores unknown environments.
- **Behavior Trees**: Modular decision-making for exploration and navigation.
- **SLAM Toolbox**: Real-time SLAM for mapping the environment.
- **TurtleBot3 Simulation**: Virtual robot for testing the mapping pipeline.

## Prerequisites

Before running the system, ensure the following are installed:

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **TurtleBot3 packages**
- **Gazebo Simulation environment**
- **SLAM Toolbox**

## Installation

### 1. Clone the Repository

```bash
cd ~/ros_ws/src
git clone https://github.com/niweshsah/Autonomous_mapping_btTrees.git
```

### 2. Build the Workspace

```bash
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the Autonomous Mapping

After building the workspace, execute the following commands in separate terminals:

### 1. Run the Behavior Tree for Frontier Exploration

```bash
ros2 run my_behavior_tree frontier_btTree
```

### 2. Start the SLAM Toolbox

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

### 3. Launch the TurtleBot3 Gazebo Simulation

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 4. Start Navigation with Nav2

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py params_file:=/home/niweshsah/py_trees1_ws/config/nav2_params.yaml use_sim_time:=True
```

## Notes

- Ensure **ROS 2 Humble** is properly installed and sourced before running the commands.
- The workspace should be properly built and sourced after cloning.
- The behavior tree node controls the frontier-based exploration autonomously.
- The default world used is the **TurtleBot3 World**, but users can choose any custom world for mapping.
- **Prerequisites**: Ensure that **TurtleBot3 packages** and **Ubuntu 22.04** are installed before running the system.
- The code includes an **unstuck mechanism**, but if the robot gets stuck for a long time, simply pressing **Reset** for **Nav2** in **RViz** usually resolves the issue.
- Ensure **ROS 2 Humble** is properly installed and sourced before running the commands.
- The workspace should be properly built and sourced after cloning.
- The behavior tree node controls the frontier-based exploration autonomously.

