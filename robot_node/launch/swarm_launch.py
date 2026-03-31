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
                        "pheromone_map_topic": "pheromones",
                        "map_frame_id": "global_map",
                        "goal_topic": "goal_pose",
                        "static_transform_x": 0.0,
                        "static_transform_y": 0.0,
                    }
                ],
            ),
            # Node(
            #     package="robot_node",
            #     executable="robot_node",
            #     name="kris_robot2_node",
            #     output="screen",
            #     parameters=[
            #         {
            #             "robot_name": "kris_robot2",
            #             "goal_process_interval": 5.0,
            #             "pheromone_map_topic": "pheromones",
            #             "map_frame_id": "kris_robot2_pheromone_map",
            #             "goal_topic": "goal_pose",
            #             "static_transform_x": 1.0,
            #             "static_transform_y": 0.0,
            #         }
            #     ],
            # ),
        ]
    )
