from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory("mpu9250driver")
    parameter_file = LaunchConfiguration("params_file")
    ekf_config_file = LaunchConfiguration("ekf_config_file")

    params_declare = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(share_dir, "params", "mpu9250.yaml"),
        description="Path to the ROS2 parameters file to use.",
    )

    declare_ekf_config_file_cmd = DeclareLaunchArgument(
        name="ekf_config_file",
        default_value=os.path.join(share_dir, "params", "ekf_config.yaml"),
        description="Full path to the EKF configuration YAML file",
    )

    mpu9250driver_node = Node(
        package="mpu9250driver",
        executable="mpu9250driver",
        name="mpu9250driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[parameter_file],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_file,
        ],
    )

    ld.add_action(params_declare)
    ld.add_action(declare_ekf_config_file_cmd)
    ld.add_action(mpu9250driver_node)
    ld.add_action(ekf_node)

    return ld
