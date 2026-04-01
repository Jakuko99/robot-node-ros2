#!/usr/bin/env python3
import os
import sys
import asyncio

from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


async def main():
    # Name of the ROS 2 package containing your launch file
    package_name = "robot_sim"
    launch_file = "sim_launch.py"

    # Locate the launch file
    launch_file_path = os.path.join(
        get_package_share_directory(package_name), "launch", launch_file
    )

    optimizer_file_path = os.path.join(
        get_package_share_directory("robot_node"), "launch", "swarm_launch.py"
    )

    if not os.path.exists(launch_file_path):
        print(f"Error: Launch file not found at {launch_file_path}")
        sys.exit(1)

    # Create a LaunchService and include the launch file
    launch_service = LaunchService(noninteractive=True)
    launch_description = PythonLaunchDescriptionSource(launch_file_path)
    optimizer_launch_description = PythonLaunchDescriptionSource(optimizer_file_path)
    launch_service.include_launch_description(
        launch_description
    )  # [ERROR] [launch]: Caught exception in launch (see debug for traceback): 'PythonLaunchDescriptionSource' object has no attribute 'visit' ???
    launch_service.include_launch_description(optimizer_launch_description)

    # Run the launch service
    print(f"Launching: {launch_file_path}")
    return await launch_service.run_async()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nLaunch interrupted by user.")
