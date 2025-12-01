from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    launch_dir = os.path.join("src/kris_robot", "launch")
    return LaunchDescription(
        [
            Node(
                package="kris_robot",
                executable="kris_robot",
                name="kris_robot1_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "robot_description": "/home/ubuntu/ros_ws/src/kris_robot/urdf/kris_robot.urdf",
                        "base_frame_id": "kris_robot1_base_link",
                        "odom_frame_id": "kris_robot1_odom",
                        "robot_namespace": "kris_robot1",
                    },
                ],
            ),
            # Node(
            #     package="sllidar_ros2",
            #     executable="sllidar_node",
            #     name="sllidar_node",
            #     parameters=[
            #         {
            #             "channel_type": "serial",
            #             "serial_port": "/dev/serial0",
            #             "serial_baudrate": 115200,
            #             "frame_id": "kris_robot1_laser_frame",
            #             "inverted": False,
            #             "angle_compensate": True,
            #             "topic_name": "kris_robot1/scan",
            #         }
            #     ],
            #     output="screen",
            # ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(launch_dir, "nav_launch.py")
            #     ),
            # )
        ]
    )
