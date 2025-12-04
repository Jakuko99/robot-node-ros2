import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join("src/kris_robot", "launch")
    config_dir = os.path.join("src/kris_robot", "config")
    return LaunchDescription(
        [
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": "/dev/serial0",
                        "serial_baudrate": 115200,
                        "frame_id": "kris_robot1_laser_frame",
                        "inverted": False,
                        "angle_compensate": True,
                        "topic_name": "kris_robot1/scan",
                    }
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": "kris_robot1",
                    "autostart": "true",
                    "params_file": f"{config_dir}/nav2_params1.yaml",
                    "use_lifecycle_mgr": "true",
                }.items(),
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[f"{config_dir}/slam_toolbox_params1.yaml"],
                remappings=[
                    ("/map", "/kris_robot1/map"),
                ],
            ),
            Node(
                package="dynamic_tf_publisher",
                executable="dynamic_tf_publisher",
                name="dynamic_tf_publisher_robot1",
                output="screen",
                parameters=[
                    {
                        "publish_rate": 5.0,
                        "frame_id": "global_map",
                        "child_frame_id": "kris_robot1_map",
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                    }
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "gpio",
                    "-g",
                    "mode",
                    "18",
                    "pwm",
                    "&&",
                    "gpio",
                    "-g",
                    "pwm",
                    "18",
                    "675",
                ]
            ),
            RegisterEventHandler(
                OnShutdown=ExecuteProcess(
                    cmd=[
                        "gpio",
                        "-g",
                        "pwm",
                        "18",
                        "0",
                    ]
                )
            ),
        ]
    )
