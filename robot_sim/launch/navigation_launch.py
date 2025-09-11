import os
import logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace, SetRemap

# logging.root.setLevel(logging.WARN)

def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    return LaunchDescription(
        [
            SetRemap("cmd_vel", "kris_robot1/cmd_vel"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "autostart": "true",
                    "params_file": "src/robot_sim/config/nav2_params1.yaml",
                    "use_lifecycle_mgr": "true",
                    "map_subscribe_transient_local": "true",
                }.items(),
            ),
        ]
    )
