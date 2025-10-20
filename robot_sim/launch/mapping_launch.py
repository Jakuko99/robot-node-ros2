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
                    ("/map", "/kris_robot1/map"),
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=["src/robot_sim/config/slam_toolbox_params1.yaml"],
                remappings=[
                    ("/map", "/kris_robot2/map"),
                ],
            ),
            # Node(
            #     package="merge_map",
            #     executable="merge_map",
            #     output="screen",
            #     parameters=[{"use_sim_time": True}],
            #     remappings=[
            #         ("/map1", "/kris_robot1/map"),
            #         ("/map2", "/kris_robot2/map"),
            #     ],
            # ),

            Node(
                package="multirobot_map_merge",
                executable="map_merge",
                name="map_merge_node",
            )
        ]
    )
