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
Contains files for setting up simulation in static environment, random environment or using modified Gibson dataset files for exploration. can be combined with either robot_control or robot_node package for autonomous exploration. Also includes files for setting up RViZ2 visualisation, that enables to control one of the robots manually through properly remapping its goal_pose topic, as the robots run in a namespace.

### Map merger package
Dynamicaly scans available topics for the ones containing local occupancy maps from the robots and merges them into one global map. Another feature of this node is the ability to highlight the overlapped regions of the local maps and also add current robot positions to the final map. In normal operation each robot in the swarm has its own map merger node to generate global map to base the decisions of the the RL network upon.

### Supporting packages
Below is a overview of other packages contained in this repository:

#### Dynamic TF Publisher package
This package provides a service for dynamically publishing transforms in the ROS2 ecosystem. It allows robots to broadcast their position and orientation information in the global coordinate frame, which is essential for multi-robot coordination and map merging.

#### Sim Services package
Defines custom ROS services (`sim_srvs`) used for data management during simulations. These services enable:
- Storing and retrieving simulation metadata
- Associating training data with specific simulation runs
- Exporting training results and metrics
- Managing simulation-specific configurations

These services are crucial for organizing and tracking reinforcement learning training data during distributed multi-robot simulations.

## Installation and Dependencies

Before running the code, ensure you have the following dependencies installed:
- ROS2 Humble (`ros-humble-desktop` or `ros-humble-ros-base`)
- Gazebo (simulator)
- Nav2 (navigation framework)
- SLAM Toolbox (for SLAM-based mapping)
- Python 3.8+ with packages: `numpy`, `tensorflow`, `opencv`

Run the setup script to install all dependencies:
```bash
cd /path/to/ros_ws/src
bash setup.sh
```

## Configuration

### Navigation and Mapping Configuration
- Map merger configuration can be modified in `map_merger/config/`
- Robot simulation parameters (e.g., sensor noise, dynamics) are configurable in launch files

### Reinforcement Learning Configuration
- Training parameters (learning rate, episode length, etc.) can be modified in `robot_node/launch/swarm_launch.py`
- Reward function configurations are tunable in the training scripts

## Training and Optimization

### Training Pipeline
1. **Data Generation**: Launch simulation using the `train_sim_launch.py` or `static_train_launch.py` files
2. **Model Training**: Run the Python training script which uses Stable-Baselines3 PPO implementation
3. **Evaluation**: Validate trained models on test environments (Gibson dataset or random environments)
4. **Deployment**: Export trained models for use in real-world robot applications

### Distributed Training
For large-scale training on clusters, use the provided Apptainer container setup with the Slurm job scheduler. See `train_script.sh` for example cluster submission configurations.

## Visualization and Debugging

### RViZ2 Configuration
- Main visualization config: `robot_sim/rviz/gazebo_rviz.rviz`
- Displays include: robot positions, occupancy maps, local SLAM maps, global merged map, navigation goals, and sensor data
- Manual robot control: Remap `/goal_pose` topic to specific robot namespace

### Gazebo Environment Visualization
- Gazebo is launched automatically with `sim_launch.py`, `random_sim_launch.py`, and `gibson_launch.py`
- Sensor visualizations (laser scans) are available in real-time
- Environment statistics (exploration coverage, time taken) can be logged

## Output and Results

### Training Outputs
- Trained models are saved in `export/` directory after training completes
- Training metrics (episode rewards, exploration coverage) are logged as CSV files
- Model checkpoints are saved at regular intervals for recovery

### Map Data
- Global merged maps from simulations are exported in PNG format
- Occupancy grids are stored for post-training analysis
- Trajectory data for each robot is available in CSV and PNG format as plots

## Performance Notes

- **Training Speed**: Single simulation run typically takes 5-15 minutes depending on environment complexity
- **Computational Requirements**: Minimum 4GB RAM, recommended 8GB+ for distributed training

## Future Enhancements

Potential areas for expansion:
- Multi-agent communication and cooperation strategies
- Transfer learning between different environments
- Real-world robot deployment and fine-tuning
- Integration with more advanced SLAM algorithms

## Contributing

To contribute to this project:
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/your-feature`)
3. Commit your changes with clear messages
4. Push to your branch and open a pull request

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