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
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=["src/robot_sim/config/slam_toolbox_params2.yaml"],
                remappings=[
                    ("/map", "/kris_robot3/map"),
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=["src/robot_sim/config/slam_toolbox_params3.yaml"],
                remappings=[
                    ("/map", "/kris_robot4/map"),
                ],
            ),
            # Node(
            #     package="merge_map",
            #     executable="merge_map",
            #     output="screen",
            #     parameters=[{"use_sim_time": True}],
            #     remappings=[("/merge_map", "/merge_map1")],
            # ),
            # Node(
            #     package="merge_map",
            #     executable="merge_map",
            #     output="screen",
            #     parameters=[{"use_sim_time": True}],
            #     remappings=[
            #         ("/merge_map", "/merge_map2"),
            #         ("/map1", "/map3"),
            #         ("/map2", "/map4"),
            #     ],
            # ),
            # Node(
            #     package="merge_map",
            #     executable="merge_map",
            #     output="screen",
            #     parameters=[{"use_sim_time": True}],
            #     remappings=[
            #         ("/merge_map1", "/map1"),
            #         ("/merge_map2", "/map2"),
            #     ],
            # ),
        ]
    )
