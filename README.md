# Optimization of Mapping Tasks in a Robotic Swarm
This repository contains source code for the implementation of a reinforcement learning-based approach to optimize mapping tasks in a robotic swarm. The code is organized into two main ROS packages: `robot_node` and `robot_sim`. Map merging is provided by the `map_merger` package, which is a custom implementation of a map merging algorithm, that uses ACO to highlight overlapping regions of a map. Package `sim_srvs` contains definitions of custom ROS services used in the simulation, mainly for storing data, so it can be paired to particular simulation runs. We used ROS2 Humble in this project.

## Repository branches
This repository contains multiple branches, most of them are either past implementations of the optimization or are used for testing, below is short description for each branch:
- **master** - stable implementation of optimization algorithm using RL networks
- **demo_sim** - contains simple simulation used as a showcase of exploration, algorithms is based on frontier detection
- **new_gen_swarm** - branch that contains most of the commits for the RL based optimization of the mapping
- **quad_robots** - testing branch with 4 robots, other implementations use only two for faster testing
- **robot_node** - contains source code in C++ for a node that can be used on real robot and retains all interfaces that simulation provides to be interchangeable
- **single_robot** - testing branch with single robot

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

### Launch robot simulation
In order to run the roboti simulation you have few options: simple world containing multiple rooms, randomly generated rooms or Gibson environments, that were adjusted by removing the roof and floor of the models, so they only provide walls for the robot to move around. The Gibson models are not part of this repository as some of them are larger, but you can find archive containing these models in the [Release section](https://github.com/Jakuko99/robot-node-ros2/releases/tag/GIBSON_ENV), then place the desired models inside `robot_sim/gazebo/allensville/meshes` in order for the simulation to work properly. Each simulation launch file also executes SLAMtoolbox for mapping, Nav2 for robot movement and custom map merging node to create global map of the environment from mapping data from multiple robots.

For starting robot simulation use the following commands:
```bash
ros2 launch robot_sim sim_launch.py # static environment
ros2 launch robot_sim random_sim_launch.py # randomly generated environment
ros2 launch robot_sim gibson_launch.py sdf_file:="gibson_lindenwood.sdf" # Gibson environment

ros2 launch robot_node swarm_launch.py # run node that uses reinforcement network to explore
```

When launching Gibson environment you can select a SDF file that contains different models from `robot_sim/gazebo` directory, if no argument is given then file `gibson_lindenwood.sdf` is used, which contains, as the name suggests it, the Lindenwood model inside it. Every launch file lauches Rviz2 instance as well for vizualization purposes except the Gibson one, as this file is used mainly for training purposes and does not need GUI. If you need to run Rviz2 in this case you can start it using this command, while being in `ros_ws` directory:
```bash
rviz2 -d src/robot_sim/rviz/gazebo_rviz.rviz --ros-args -r /goal_pose:=/kris_robot1/goal_pose
```
It also remaps `goal_pose` topic in order to be able to manually send a target position to first robot, it also can be changed to control second robot (`kris_robot2`) instead.


### Launch robot control 
This node uses simple frontier detection algorithm to explore the environment around the robot based on the local map obtained from SLAM toolbox. Unlike the other nodes, which are written in Python, this one is written in C++ for its simplicity and can be used alongside static environment simulation to showcase the exploration. Run this node using:
```bash
ros2 run robot_control robot_control
```

This is a simple node, that takes the local occupancy map of the robot as input and publishes a goal pose to the robot's navigation stack based on frontier locations to explore environment around it.

## ROS2 package overview
This repository contains all of the necessary packages to run the simulation and train a reinforcement learning model. Below is a short descriptions of each package and its main features.

### Robot node package
This package is responsible for exploring the environment using a reinforcement learning network with PPO. Also the same node is used to train the model by changing the parameters in `swarm_launch.py`.

### Robot sim package

### Map merger package

### Supporting packages

## Scripts
In the root directory of this repository are couple of scripts, that are either used for training or serve as a helpers for various usecases.

### Setup script
File `setup.sh` is used to setup entire ROS2 workspace, it installs required packages such as Rviz2, Nav2, Gazebo and others. It can also be used as a base for setting up custom containers, but in that case package `ros-humble-desktop` needs to be replaced with `ros-humble-ros-base` if you don't need graphical interface for your ROS2 install.

### Training container
Current version of training contrainer is created using Apptainer (previously Singularity) platform. The definition file `train_env.def` contains commands to install necessary ROS2 packages and setup the workspace in the container. Also it is made in the way that on every start-up it pulls code changes from GitHub, so there is no need to rebuild the image after each change in the code. For building and runnning the image you can use these commands:
```bash
export SIM_FOLDER="/tmp/sim" # or other place where to store temporary files needed for simulation
export WRKDIR="$PWD" # here the export directory will be copied
export MESH_PATH="/path/to/mesh_files" # where are .obj files for Gibson simulation stored

apptainer build train_env.sif train_env.def
mkdir -p $SIM_FOLDER # create directory if it does not exist
apptainer run train_env.sif
mv $SIM_FOLDER/ros_ws/export $WRKDIR # move exported files from simulation into the work folder
rm -r $SIM_FOLDER # cleanup
```
After starting the container the script `run_model_training.py` handles the simulation according to the created launch files and after defined time the simulation is stopped and necessary data are saved to the drive. In this file you can also configure the number of simulations or their type (random or predefined Gibson models). For example how to run training for example on a cluster that uses Slurm for job quierying see file `train_script.sh` in the root of this repository.

## Acknowledgements
For more information about Gibson environment visit its [website](https://gibsonenv.github.io/) or check out the source code on [GitHub](https://github.com/StanfordVL/iGibson) to learn about usage and implementations.