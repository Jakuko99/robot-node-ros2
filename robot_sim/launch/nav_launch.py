import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join("src/robot_sim", "launch")
    config_dir = os.path.join("src/robot_sim", "config")
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
                    "params_file": f"{config_dir}/nav2_params1.yaml",
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
                    "params_file": f"{config_dir}/nav2_params2.yaml",
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
                    "params_file": f"{config_dir}/nav2_params3.yaml",
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
                    "params_file": f"{config_dir}/nav2_params4.yaml",
                    "use_lifecycle_mgr": "true",
                    # "map_subscribe_transient_local": "true",
                }.items(),
            ),
        ]
    )
