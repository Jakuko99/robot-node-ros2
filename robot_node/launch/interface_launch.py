import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    resource_dir = os.path.join("src/robot_node", "resource")
    return LaunchDescription(
        [
            Node(
                package="robot_node",
                executable="robot_node",
                name="kris_robot1_node",
                output="screen",
                parameters=[
                    {
                        "robot_name": "kris_robot1",
                        "goal_process_interval": 5.0,
                    }
                ],
            ),
        ]
    )
