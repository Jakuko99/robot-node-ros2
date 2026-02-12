import os
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
                        "train_network": True,
                        "network_model_path": f"{resource_dir}",
                        "global_map_topic": "kris_robot1/map",
                        "goal_marker_topic": "mapping_goals",
                        "goal_frame_id": "kris_robot1_map",
                    }
                ],
            ),
        ]
    )
