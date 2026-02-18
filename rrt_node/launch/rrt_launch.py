import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    resource_dir = os.path.join("src/rrt_node", "resource")
    return LaunchDescription(
        [
            Node(
                package="rrt_node",
                executable="rrt_node",
                output="screen",
                parameters=[
                    {
                        "robot_name": "kris_robot1",
                        "tree_update_interval": 5.0,
                    }
                ],
            ),
        ]
    )
