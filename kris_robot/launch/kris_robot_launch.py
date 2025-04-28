from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kris_robot",
            executable="kris_robot",
            name="kris_robot",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"robot_description": "~/ros_ws/src/kris_robot/urdf/kris_robot.urdf"},
            ]
        )
    ])