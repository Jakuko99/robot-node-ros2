import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # return LaunchDescription(
    #     [
    #         ExecuteProcess(
    #         #     cmd=[
    #         #         "ros2",
    #         #         "launch",
    #         #         "nav2_bringup",
    #         #         "navigation_launch.py",
    #         #         f"use_sim_time:=true",
    #         #     ],
    #         #     output="screen",
    #         # ),
    #         ExecuteProcess(
    #             cmd=[
    #                 "ros2",
    #                 "launch",
    #                 "slam_toolbox",
    #                 "online_async_launch.py",
    #                 "use_sim_time:=true",
    #             ]
    #         ),
    #     ]
    # )
    return LaunchDescription(
        [
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=["src/robot_sim/config/slam_toolbox_params.yaml"],
                remappings=[("/scan", "/scan")],
            ),
        ]
    )
