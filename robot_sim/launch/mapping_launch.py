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
                package="dynamic_tf_publisher",
                executable="dynamic_tf_publisher",
                name="dynamic_tf_publisher_robot1",
                output="screen",
                parameters=[
                    {
                        "publish_rate": 10.0,
                        "frame_id": "global_map",
                        "child_frame_id": "kris_robot1_map",
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                    }
                ],
            ),
            Node(
                package="dynamic_tf_publisher",
                executable="dynamic_tf_publisher",
                name="dynamic_tf_publisher_robot2",
                output="screen",
                parameters=[
                    {
                        "publish_rate": 10.0,
                        "frame_id": "global_map",
                        "child_frame_id": "kris_robot2_map",
                        "x": 1.0,
                        "y": 0.0,
                        "z": 0.0,
                    }
                ],
            ),
            Node(
                package="map_merger",
                executable="map_merger",
                output="screen"
            ),
        ]
    )
