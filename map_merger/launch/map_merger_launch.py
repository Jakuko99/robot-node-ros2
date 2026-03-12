import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                name="kris_robot1_merger",
                package="map_merger",
                executable="map_merger",
                output="screen",
                parameters=[
                    {
                        "robot_name": "kris_robot1",
                        "merge_topic_name": "pheromones",
                    }
                ],
            ),
        ]
    )
