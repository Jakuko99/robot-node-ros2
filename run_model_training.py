#!/usr/bin/env python3
import os
import sys
import glob
import asyncio
import threading
from time import sleep
from random import choice

from launch import LaunchService, LaunchDescription
from launch.launch_description_sources import (
    get_launch_description_from_python_launch_file as load_launch_description,
)
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.client import Client
from rclpy.node import Node
from rclpy import Future

from robot_sim.generate_environment import EnvironmentGenerator
from robot_node.data_logger import DataLogger, Batch
from sim_srvs.srv import SimulationOutput, ExplorationStatus
from launch.actions import SetLaunchConfiguration

# ----- CONFIGURATION -----
ONLINE_TRAINING: bool = True
MODEL_TESTING: bool = True
RANDOM_ENV: bool = False
PLOT_RESULTS: bool = True
NUM_SIMULATIONS: int = 4
SIM_PERIOD: int = 3600  # duration of each simulation run in seconds
CHECK_INTERVAL: int = 600  # interval in seconds for checking simulation progress
OVERLAP_THRESHOLD: float = 0.6  # threshold for ratio of overlapped vs total cells
EXPLORATION_THRESHOLD: float = 0.7  # threshold for ratio of explored vs total cells
LOG_FILE: str = "export/training_log.log"
METRICS_FILE: str = "export/exploration_metrics.csv"
# -------------------------


def log_message(message: str):
    try:
        with open(LOG_FILE, "a") as f:
            f.write(f"{message}\n")
    except Exception as e:
        print(f"ERROR: Error writing to log file: {e}")
    print(message)  # also print to console for real-time feedback


def prepare_launch(
    launch_serv: LaunchService,
    random_env: bool = False,
    sdf_file: str = "gibson_lindenwood.sdf",
):
    if os.getenv("MESH_PATH") is None and random_env is False:
        log_message(
            "ERROR: MESH_PATH environment variable not set. Forcing random environment generation for all simulation runs."
        )
        global RANDOM_ENV
        RANDOM_ENV = True
        random_env = True

    sim_launch_file = "gibson_launch.py" if not random_env else "train_sim_launch.py"

    sim_launch_file_path = os.path.join(
        get_package_share_directory("robot_sim"), "launch", sim_launch_file
    )
    optimizer_file_path = os.path.join(
        get_package_share_directory("robot_node"), "launch", "swarm_launch.py"
    )

    if not os.path.exists(sim_launch_file_path) or not os.path.exists(optimizer_file_path):
        log_message(f"Error: One or more launch files not found at the specified paths")
        sys.exit(1)

    launch_description: LaunchDescription = load_launch_description(sim_launch_file_path)
    if not random_env:
        log_message(f"Using SDF file for simulation: {sdf_file}")
        launch_description.entities.insert(
            0,
            SetLaunchConfiguration("sdf_file", sdf_file),
        )

    optimizer_launch_description: LaunchDescription = load_launch_description(optimizer_file_path)

    launch_serv.include_launch_description(launch_description)
    launch_serv.include_launch_description(optimizer_launch_description)


async def run_sim(launch_serv: LaunchService):
    await launch_serv.run_async()


def call_service(
    client: Client,
    node: Node,
    sim_nr: int = 0,
    retry_limit: int = 10,
    request_type=SimulationOutput.Request,
) -> Future:
    retry_count: int = 0
    future: Future = Future()

    while not client.wait_for_service(timeout_sec=1.0):
        log_message(f"Waiting for service {client.srv_name} to become available...")
        retry_count += 1

        if retry_count >= retry_limit:
            log_message(
                f"Service {client.srv_name} did not become available after {retry_limit} attempts. Skipping."
            )
            break

    try:
        req = request_type(id=sim_nr)
        future: Future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
        if future.result() is not None:
            log_message(f"Service call to {client.srv_name} succeeded: {future.result().message}")

        else:
            log_message(f"Service call to {client.srv_name} failed")

    except Exception as e:
        log_message(f"Error calling service {client.srv_name}: {e}")

    return future


def sim_shutdown(
    ls: LaunchService,
    node: Node,
    clients: list[Client],
    sim_nr: int,
    wait_period: int = 5,
    save_model: bool = True,
    exploration_client: Client = None,
):
    if exploration_client:
        for i in range(wait_period // CHECK_INTERVAL):
            sleep(CHECK_INTERVAL)
            log_message(
                f"UPDATE: Simulation run {sim_nr}: {(i+1) * CHECK_INTERVAL} seconds elapsed..."
            )

            # check the ratio of overlapped vs free cells to determine if there is still meaningful exploration happening
            call_result: Future = call_service(
                exploration_client, node, request_type=ExplorationStatus.Request
            )
            if call_result.result() is not None:
                ratio: float = call_result.result().overlap_ratio

                with open(METRICS_FILE, "a") as f:
                    f.write(
                        f"{sim_nr}-{i},{call_result.result().overlap_ratio:.4f},{call_result.result().explore_ratio:.4f}\n"
                    )

                if (
                    ratio >= OVERLAP_THRESHOLD and call_result.result().success and i > 1
                ):  # longer grace period
                    log_message(
                        f"UPDATE: Overlap ratio {ratio:.2f} exceeds threshold of {OVERLAP_THRESHOLD:.2f}. Ending simulation run {sim_nr} early."
                    )
                    break

                elif (
                    call_result.result().explore_ratio >= EXPLORATION_THRESHOLD
                    and call_result.result().success
                ):
                    log_message(
                        f"UPDATE: Exploration ratio {ratio:.2f} is above exploration threshold of {EXPLORATION_THRESHOLD:.2f}. Ending simulation run {sim_nr} early."
                    )
                    break

    else:
        sleep(wait_period)

    if save_model:
        for client in clients:
            call_service(client, node, sim_nr=sim_nr)

    sleep(5)  # give some time for services to complete before shutting down
    ls.shutdown()


def plot_metrics(metrics_file: str = METRICS_FILE):
    try:
        data: dict[str, list[float]] = {"overlap_ratio": [], "explore_ratio": [], "tick_labels": []}
        with open(metrics_file, "r") as f:
            for line in f:
                tick_str, overlap_str, explore_str = line.strip().split(",")
                data["tick_labels"].append(tick_str)
                data["overlap_ratio"].append(float(overlap_str))
                data["explore_ratio"].append(float(explore_str))

        # Generate plots using matplotlib
        import matplotlib.pyplot as plt

        plt.figure(figsize=(15, 5))
        plt.plot(data["overlap_ratio"], label="Overlap Ratio")
        plt.plot(data["explore_ratio"], label="Explore Ratio")
        plt.axhline(y=OVERLAP_THRESHOLD, color="r", linestyle="--", label="Overlap Threshold")
        plt.axhline(
            y=EXPLORATION_THRESHOLD,
            color="g",
            linestyle="--",
            label="Exploration Threshold",
        )
        plt.xlabel("Simulation - interval")
        plt.xticks(range(len(data["tick_labels"])), data["tick_labels"], rotation=45)
        plt.ylabel("Ratio")
        plt.title("Simulation Exploration Metrics Over Time")
        plt.legend()
        plt.grid()
        plt.tight_layout()
        plt.savefig("export/exploration_metrics.png")
        log_message("Metrics plot generated and saved to export/exploration_metrics.png")

    except Exception as e:
        log_message(f"Error generating metrics plot: {e}")


if __name__ == "__main__":
    if ONLINE_TRAINING:
        # ROS 2 environment setup
        rclpy.init()
        node: Node = rclpy.create_node("training_launcher")
        cli1: Client = node.create_client(SimulationOutput, "/kris_robot1/save_model")
        cli2: Client = node.create_client(SimulationOutput, "/kris_robot2/save_model")
        cli3: Client = node.create_client(SimulationOutput, "/kris_robot3/save_model")
        cli4: Client = node.create_client(SimulationOutput, "/kris_robot1/export_map")
        cli5: Client = node.create_client(SimulationOutput, "/kris_robot2/export_map")  # backup
        exploration_client: Client = node.create_client(
            ExplorationStatus, "/kris_robot1/exploration_progress"
        )

        gibson_files: list[str] = [
            "gibson_beechwood.sdf",
            "gibson_corozal.sdf",
            "gibson_cosmos.sdf",
            "gibson_hanson.sdf",
            "gibson_ihlen.sdf",
            "gibson_lindenwood.sdf",
            "gibson_marstons.sdf",
            "gibson_muleshoe.sdf",
            "gibson_shelbyville.sdf",
            "gibson_stockman.sdf",
            "gibson_tolstoy.sdf",
            "gibson_uvalda.sdf",
            "gibson_wiconisco.sdf",
            "gibson_woodbine.sdf",
        ]

        test_files: list[str] = [
            "gibson_benevolence.sdf",
            "gibson_markleeville.sdf",
            "gibson_mifflinburg.sdf",
            "gibson_ranchester.sdf",
        ]

        try:
            for i in range(NUM_SIMULATIONS):
                launch_service = LaunchService(noninteractive=True)
                prepare_launch(
                    launch_service,
                    random_env=RANDOM_ENV,
                    sdf_file=choice(gibson_files if not MODEL_TESTING else test_files),
                )

                if RANDOM_ENV:  # generate new random environment for each simulation run
                    log_message(
                        f"Generating random environment for simulation run {i + 1}/{NUM_SIMULATIONS} ..."
                    )
                    generator = EnvironmentGenerator(width=20, height=20, num_rooms=20)
                    generator.generate()
                    generator.export_to_world(
                        f"{os.path.dirname(__file__)}/robot_sim/gazebo/random_environment.sdf",
                        include_robots=True,
                    )
                    generator.export_ground_truth(f"export/ground_truth-{i + 1}.png")
                    log_message("Done generating environment.")

                sim_thread = threading.Thread(
                    target=lambda: sim_shutdown(
                        ls=launch_service,
                        node=node,
                        clients=[cli1, cli2, cli3, cli4, cli5],
                        wait_period=SIM_PERIOD,
                        sim_nr=i + 1,
                        exploration_client=exploration_client,
                    )
                )
                log_message(f"Starting simulation run {i + 1}/{NUM_SIMULATIONS} ...")
                sim_thread.start()  # start shutdown thread
                asyncio.run(run_sim(launch_service))  # run simulation

                # Clean up and prepare for next run
                sim_thread.join()
                del launch_service
                del sim_thread

                if i < NUM_SIMULATIONS - 1:
                    log_message(
                        f"Simulation run {i + 1}/{NUM_SIMULATIONS} completed. Restarting in 5 seconds..."
                    )
                    sleep(5)

        except Exception as e:
            log_message(f"\nAn error occurred: {e}")

        log_message("\nAll simulation runs completed. Shutting down ROS 2 environment.")
        rclpy.shutdown()

        if PLOT_RESULTS:
            try:
                for file in glob.glob("export/kris_robot*_training_*.csv"):
                    data: dict[int, Batch] = DataLogger.load_from_csv(file)
                    DataLogger.subplot_data(
                        data,
                        [
                            "loss",
                            "policy_loss",
                            "avg_reward",
                            "entropy",
                            "coverage_gain",
                            "frontier_gain",
                            "overlap_growth",
                            "crowding_penalty",
                            "redundancy_penalty",
                        ],
                        f"{os.path.splitext(file)[0]}.png",
                        data_label="Epoch",
                    )

                plot_metrics()
                log_message("Plots generated and saved in export directory.")

            except Exception as e:
                log_message(f"Error generating plots: {e}")

    else:
        from robot_node.offline_trainer import train_model

        train_model(
            dataset_dir="export/datasets",
            output_model="export/trained_model.pt",
            epochs=160,
            batch_size=32,
            gamma=0.995,
            learning_rate=3e-6,
            value_coef=0.5,
            entropy_coef=0.02,
            checkpoint_dir="export/checkpoints",
            checkpoint_every=20,
        )

        print("Model training completed.")
