from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kris_robot",
                executable="kris_robot",
                name="kris_robot",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "robot_description": "/home/ubuntu/ros_ws/src/kris_robot/urdf/kris_robot.urdf"
                    },
                ],
            ),
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": "/dev/ttyUSB0",
                        "serial_baudrate": "115200",
                        "frame_id": "laser_frame",
                        "inverted": "false",
                        "angle_compensate": "true",
                        "topic_name": "kris_robot1/scan",
                    }
                ],
                output="screen",
            ),
        ]
    )
