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
                package="dynamic_tf_publisher",
                executable="dynamic_tf_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "publish_rate": 10.0,
                        "frame_id": "global_map",
                        "child_frame_id": "robot1_map",
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                    }
                ],
            ),
        ]
    )
