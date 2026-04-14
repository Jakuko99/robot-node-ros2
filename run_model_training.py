#!/usr/bin/env python3
import os
import sys
import asyncio
import threading
from time import sleep

from launch import LaunchService, LaunchDescription
from launch.launch_description_sources import (
    get_launch_description_from_python_launch_file as load_launch_description,
)
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.client import Client
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy import Future

from robot_sim.generate_environment import EnvironmentGenerator
from sim_srvs.srv import SimulationOutput

# ----- CONFIGURATION -----
RANDOM_ENV: bool = True
NUM_SIMULATIONS: int = 4
SIM_PERIOD: int = 600  # duration of each simulation run in seconds
# -------------------------


def prepare_launch(launch_serv: LaunchService, random_env: bool = False):
    sim_launch_file = "sim_launch.py" if not random_env else "train_sim_launch.py"

    sim_launch_file_path = os.path.join(
        get_package_share_directory("robot_sim"), "launch", sim_launch_file
    )
    optimizer_file_path = os.path.join(
        get_package_share_directory("robot_node"), "launch", "swarm_launch.py"
    )

    if not os.path.exists(sim_launch_file_path) or not os.path.exists(optimizer_file_path):
        print(f"Error: One or more launch files not found at the specified paths")
        sys.exit(1)

    launch_description: LaunchDescription = load_launch_description(sim_launch_file_path)
    optimizer_launch_description: LaunchDescription = load_launch_description(optimizer_file_path)

    launch_serv.include_launch_description(launch_description)
    launch_serv.include_launch_description(optimizer_launch_description)


async def run_sim(launch_serv: LaunchService):
    await launch_serv.run_async()


def sim_shutdown(
    ls: LaunchService,
    node: Node,
    clients: list[Client],
    sim_nr: int,
    wait_period: int = 5,
    save_model: bool = True,
):
    sleep(wait_period)
    if save_model:
        for client in clients:
            while not client.wait_for_service(timeout_sec=1.0):
                print(f"Waiting for service {client.srv_name} to become available...")

            try:
                req = (
                    Trigger.Request()
                    if client.srv_type == Trigger
                    else SimulationOutput.Request(id=sim_nr)
                )
                future: Future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
                if future.result() is not None:
                    print(f"Service call to {client.srv_name} succeeded: {future.result().message}")

                else:
                    print(f"Service call to {client.srv_name} failed")

            except Exception as e:
                print(f"Error calling service {client.srv_name}: {e}")

    sleep(5)  # give some time for services to complete before shutting down
    ls.shutdown()


if __name__ == "__main__":
    # ROS 2 environment setup
    rclpy.init()
    node: Node = rclpy.create_node("training_launcher")
    cli1: Client = node.create_client(Trigger, "/kris_robot1/save_model")
    cli2: Client = node.create_client(Trigger, "/kris_robot2/save_model")
    cli3: Client = node.create_client(SimulationOutput, "/kris_robot1/export_map")
    cli4: Client = node.create_client(SimulationOutput, "/kris_robot2/export_map")  # backup export

    try:
        for i in range(NUM_SIMULATIONS):
            launch_service = LaunchService(noninteractive=True)
            prepare_launch(launch_service, random_env=RANDOM_ENV)

            if RANDOM_ENV:  # generate new random environment for each simulation run
                print(
                    f"Generating random environment for simulation run {i + 1}/{NUM_SIMULATIONS} ..."
                )
                generator = EnvironmentGenerator(width=20, height=20, num_rooms=20)
                generator.generate()
                generator.export_to_world(
                    f"{os.path.dirname(__file__)}/robot_sim/gazebo/random_environment.sdf",
                    include_robots=True,
                )
                generator.export_ground_truth(f"export/ground_truth-{i + 1}.png")
                print("Done generating environment.")

            sim_thread = threading.Thread(
                target=lambda: sim_shutdown(
                    ls=launch_service,
                    node=node,
                    clients=[cli1, cli2, cli3, cli4],
                    wait_period=SIM_PERIOD,
                    sim_nr=i + 1,
                )
            )
            print(f"Starting simulation run {i + 1}/{NUM_SIMULATIONS} ...")
            sim_thread.start()  # start shutdown thread
            asyncio.run(run_sim(launch_service))  # run simulation

            # Clean up and prepare for next run
            sim_thread.join()
            del launch_service
            del sim_thread

            if i < NUM_SIMULATIONS - 1:
                print(
                    f"Simulation run {i + 1}/{NUM_SIMULATIONS} completed. Restarting in 5 seconds..."
                )
                sleep(5)

    except Exception as e:
        print(f"\nAn error occurred: {e}")

    print("\nAll simulation runs completed. Shutting down ROS 2 environment.")
    rclpy.shutdown()
