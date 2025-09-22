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
                    # ("/tf", "/kris_robot1/tf"),
                    # ("/tf_static", "/kris_robot1/tf_static"),
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
                    # ("/tf", "/kris_robot2/tf"),
                    # ("/tf_static", "/kris_robot2/tf_static"),
                ],
            ),
            Node(
                package="merge_map",
                executable="merge_map",
                output="screen",
                parameters=[{"use_sim_time": True}],
                remappings=[
                    # ("/tf", "/kris_robot1/tf"),
                    # ("/tf_static", "/kris_robot1/tf_static"),
                    # ("/tf", "/kris_robot2/tf"),
                    # ("/tf_static", "/kris_robot2/tf_static"),
                ],
            ),
        ]
    )
