import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = "/home/ubuntu/ros_ws/src/robot_sim"
    launch_dir = os.path.join(bringup_dir, "launch")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": "kris_robot1",
                    "use_sim_time": "true",
                    "autostart": "true",
                    "params_file": "/home/ubuntu/ros_ws/src/robot_sim/config/nav2_params1.yaml",
                    "use_lifecycle_mgr": "true",
                    # "map_subscribe_transient_local": "true",                    
                }.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": "kris_robot2",
                    "use_sim_time": "true",
                    "autostart": "true",
                    "params_file": "/home/ubuntu/ros_ws/src/robot_sim/config/nav2_params2.yaml",
                    "use_lifecycle_mgr": "true",
                    # "map_subscribe_transient_local": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": "kris_robot3",
                    "use_sim_time": "true",
                    "autostart": "true",
                    "params_file": "/home/ubuntu/ros_ws/src/robot_sim/config/nav2_params3.yaml",
                    "use_lifecycle_mgr": "true",
                    # "map_subscribe_transient_local": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": "kris_robot4",
                    "use_sim_time": "true",
                    "autostart": "true",
                    "params_file": "/home/ubuntu/ros_ws/src/robot_sim/config/nav2_params4.yaml",
                    "use_lifecycle_mgr": "true",
                    # "map_subscribe_transient_local": "true",
                }.items(),
            ),
        ]
    )
