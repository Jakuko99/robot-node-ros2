from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=["src/robot_sim/config/slam_toolbox_params.yaml"],
                remappings=[
                    ("/map", "/map1"),
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=["src/robot_sim/config/slam_toolbox_params1.yaml"],
                remappings=[
                    ("/map", "/map2"),
                ],
            ),
            Node(
                package="merge_map",
                executable="merge_map",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
