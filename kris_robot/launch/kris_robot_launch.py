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
                        "robot_description": "/home/ubuntu/ros_ws/src/kris_robot/urdf/kris_robot.urdf",
                        "base_frame_id": "kris_robot1_base_link",
                        "odom_frame_id": "kris_robot1_odom",
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
                        "serial_port": "/dev/serial0",
                        "serial_baudrate": 115200,
                        "frame_id": "laser_frame",
                        "inverted": False,
                        "angle_compensate": True,
                        "topic_name": "kris_robot1/scan",
                    }
                ],
                output="screen",
            ),
            Node(
                package="dynamic_tf_publisher",
                executable="dynamic_tf_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "publish_rate": 10.0,
                        "frame_id": "global_map",
                        "child_frame_id": "kris_robot1_map",
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                    }
                ],
            ),
        ]
    )
