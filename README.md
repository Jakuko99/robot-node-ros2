# Optimization of Mapping Tasks in a Robotic Swarm
This repository contains source code for the implementation of a reinforcement learning-based approach to optimize mapping tasks in a robotic swarm. The code is organized into two main ROS packages: `robot_node` and `robot_sim`. Map merging is provided by the `map_merger` package, which is a custom implementation of a map merging algorithm, that uses ACO to highlight overlapping regions of a map. Package `sim_srvs` contains definitions of custom ROS services used in the simulation, mainly for storing data, so it can be paired to particular simulation runs. We used ROS2 Humble in this project.

## How to run the code
Create directory (or use existing one) for ROS2 workspace, then clone this repository into its `src` folder, if necessary use `git checkout <branch>` to switch to given branch of this repository:
```bash
mkdir ros_ws && cd ros_ws
git clone https://github.com/Jakuko99/robot-node-ros2 src
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch robot_sim sim_launch.py # launch robot simulation for example
```

## Robot node package

## Robot sim package

## Map merger package

## Supporting packages

## Scripts
In the root directory of this repository are couple of scripts, that are either used for training or serve as a helpers for various usecases.

### Setup script

### Training container