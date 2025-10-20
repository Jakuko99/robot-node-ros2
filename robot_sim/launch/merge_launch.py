from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [            
            Node(
                package="multirobot_map_merge",
                executable="map_merge",
                name="map_merge_node",
                parameters=[
                    {
                        "merging_rate": 4.0,
                        "discovery_rate": 0.05,
                        "estimation_rate": 0.5,
                        "known_init_poses": False,
                        "estimation_confidence": 1.0,
                        "robot_map_topic": "map",
                        "robot_map_updates_topic": "map_updates",
                        "robot_namespace": "kris_robot",
                        "merged_map_topic": "global_map",
                        "world_frame": "global_map",
                    }
                ],
            ),
        ]
    )
