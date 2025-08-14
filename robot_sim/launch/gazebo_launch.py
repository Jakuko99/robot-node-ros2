import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
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
            # ExecuteProcess(
            #     cmd=[
            #         "ros2",
            #         "run",
            #         "tf2_ros",
            #         "static_transform_publisher",
            #         "0",
            #         "0",
            #         "0",
            #         "0",
            #         "0",
            #         "0",
            #         "base_link",
            #         "merge_map",
            #     ]
            # ),
        ]
    )
