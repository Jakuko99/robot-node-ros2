#!/usr/bin/env python3
import os
import sys
import asyncio

from launch import LaunchService, LaunchDescription
from launch.launch_description_sources import (
    get_launch_description_from_python_launch_file as load_launch_description,
)
from ament_index_python.packages import get_package_share_directory


async def main(random_env: bool = False):
    sim_launch_file = "sim_launch.py" if not random_env else "random_sim_launch.py"

    sim_launch_file_path = os.path.join(
        get_package_share_directory("robot_sim"), "launch", sim_launch_file
    )
    optimizer_file_path = os.path.join(
        get_package_share_directory("robot_node"), "launch", "swarm_launch.py"
    )

    if not os.path.exists(sim_launch_file_path) or not os.path.exists(optimizer_file_path):
        print(f"Error: One or more launch files not found at the specified paths")
        sys.exit(1)

    launch_service = LaunchService(noninteractive=True)

    launch_description: LaunchDescription = load_launch_description(sim_launch_file_path)
    optimizer_launch_description: LaunchDescription = load_launch_description(optimizer_file_path)

    launch_service.include_launch_description(launch_description)
    launch_service.include_launch_description(optimizer_launch_description)

    launch_service.run()
    return await launch_service.run_async()


# TODO: figure out how to interrupt this after fixed time and start again

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nLaunch interrupted by user.")
