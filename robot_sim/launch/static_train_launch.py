"""Launch file to start the robot simulation environment."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join("src/robot_sim", "launch")
    sdf_file = LaunchConfiguration("sdf_file")
    launch_description = LaunchDescription()

    launch_description.add_action(
        DeclareLaunchArgument(
            "sdf_file",
            default_value="gibson_lindenwood.sdf",
            description="Name of the SDF file for the simulation environment",
        )
    )

    launch_description.add_action(
        ExecuteProcess(  # gazebo simulation
            cmd=[
                "ign",
                "gazebo",
                # "-s",
                "-r",
                PathJoinSubstitution(["src", "robot_sim", "gazebo", sdf_file]),
                # "--headless-rendering",
            ],
        )
    )

    launch_description.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", "src/robot_sim/rviz/gazebo_rviz.rviz"],
            parameters=[{"use_sim_time": True}],
            remappings=[("/goal_pose", "/kris_robot1/goal_pose")],
        )
    )

    launch_description.add_action(
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
        )
    )

    launch_description.add_action(
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
                "global_map",
                "kris_robot1_map",
            ]
        )
    )

    launch_description.add_action(
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "tf2_ros",
                "static_transform_publisher",
                "1.0",
                "0",
                "0",
                "0",
                "0",
                "0",
                "global_map",
                "kris_robot2_map",
            ]
        )
    )
    launch_description.add_action(
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "tf2_ros",
                "static_transform_publisher",
                "1.0",
                "1.0",
                "0",
                "0",
                "0",
                "0",
                "global_map",
                "kris_robot3_map",
            ]
        )
    )
    launch_description.add_action(
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "tf2_ros",
                "static_transform_publisher",
                "0",
                "1.0",
                "0",
                "0",
                "0",
                "0",
                "global_map",
                "kris_robot4_map",
            ]
        )
    )

    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "mapping_launch.py")),
        )
    )

    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav_launch.py")),
        )
    )

    return launch_description
