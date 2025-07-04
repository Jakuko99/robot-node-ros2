import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            ExecuteProcess(  # gazebo simulation
                cmd=[
                    "ign",
                    "gazebo",
                    "-r",
                    "src/robot_sim/gazebo/world.sdf",
                ],
            ),
            ExecuteProcess(  # rviz visualization
                cmd=[
                    "rviz2",
                    "-d",
                    "src/robot_sim/rviz/gazebo_rviz.rviz",
                ],
            ),
            ExecuteProcess(  # ros gz topic bridge
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    "--ros-args",
                    "-p",
                    "config_file:=src/robot_sim/gazebo/bridge_config.yaml",
                ]
            ),
        ]
    )


# ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/cmd_vel
# ign gazebo world.sdf
# ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
