"""Launch file to start the robot simulation environment."""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join("src/robot_sim", "launch")
    return LaunchDescription(
        [
            ExecuteProcess(  # gazebo simulation
                cmd=[
                    "ign",
                    "gazebo",
                    "-r",
                    "src/robot_sim/gazebo/world.sdf",
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", "src/robot_sim/rviz/gazebo_rviz.rviz"],
                parameters=[{"use_sim_time": True}],
                remappings=[("/goal_pose", "/kris_robot1/goal_pose")],
            ),
            ExecuteProcess(  # ros gz topic bridge
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    "--ros-args",
                    "-p",
                    "config_file:=src/robot_sim/gazebo/bridge_config.yaml",
                ]
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "tf2_ros",
                    "static_transform_publisher",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "merge_map",
                    "map1",
                ]
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "tf2_ros",
                    "static_transform_publisher",
                    "1.2",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "merge_map",
                    "map2",
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "mapping_launch.py")
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "nav_launch.py")
                ),
            ),
        ]
    )
