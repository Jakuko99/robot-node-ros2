import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    resource_dir = os.path.join("src/robot_network", "resource")
    return LaunchDescription(
        [
            Node(
                package="robot_network",
                executable="robot_network",
                output="screen",
                parameters=[
                    {
                        "train_network": False,
                        "network_model_path": f"{resource_dir}/robot_swarm_optimizer.pth",
                        "global_map_topic": "global_map",
                        "goal_marker_topic": "mapping_goals",
                    }
                ],
            ),
        ]
    )
