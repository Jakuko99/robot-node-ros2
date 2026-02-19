import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    resource_dir = os.path.join("src/coordinator_node", "resource")
    return LaunchDescription(
        [
            Node(
                package="coordinator_node",
                executable="coordinator_node",
                output="screen",
                parameters=[
                    {
                        "train_network": True,
                        "network_model_path": f"{resource_dir}",
                        "global_map_topic": "/global_map",
                        "goal_marker_topic": "/mapping_goals",
                        "global_frame_id": "global_map",
                        "goal_process_interval": 5.0,
                    }
                ],
            ),
        ]
    )
