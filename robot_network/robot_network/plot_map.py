import matplotlib.pyplot as plt
import numpy as np
import time
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
import os
import json


def plot_map_from_file(file_path: str):
    with open(file_path, "r") as f:
        start = time.time()
        lines = f.readlines()

        mat = np.zeros((len(lines), len(lines[0].strip().split(","))), dtype=float)

        l = 0
        for line in lines:
            i = 0
            for val in line.strip().split(","):
                mat[l, i] = int(125 if val == "0.5" else (0 if val == "0.0" else 255))
                i += 1
            l += 1

    plt.imshow(mat, cmap="gray", origin="lower")
    plt.tight_layout()
    plt.savefig(f"{os.path.dirname(__file__)}/../resource/map_plot.png", dpi=300)
    print(
        f"Map ({mat.shape[1]}x{mat.shape[0]}) plotted and saved to 'map_plot.png' in {time.time() - start:.2f} seconds."
    )


class DatasetNode:
    def __init__(self, namespace: str):
        self.namespace = namespace
        self.node = rclpy.create_node(f"dataset_node_{namespace}")
        self.map_subscription = self.node.create_subscription(
            OccupancyGrid,
            f"{namespace}/map",  # Adjust topic name as needed
            self.handle_map_message,
            10,
        )
        self.odom_subscription = self.node.create_subscription(
            Odometry,
            f"{namespace}/odom",
            self.handle_odom_message,
            10,
        )
        self.map_received: bool = False
        self.last_timestamp: int = None

    def handle_map_message(self, msg: OccupancyGrid):
        start = time.time()

        mat = np.zeros((msg.info.height, msg.info.width), dtype=float)

        l, i = 0, 0
        for val in msg.data:
            mat[l, i] = float(0.5 if val == -1 else (1 if val == 0 else 0))
            # 0.5: unknown, 1: free, 0: occupied
            i += 1
            if i >= msg.info.width:
                i = 0
                l += 1

        file_time = int(time.time())  # ensure same time for both files
        with open(
            f"{os.path.dirname(__file__)}/../resource/dataset/{self.namespace}/map_data_{file_time}.txt",
            "w",
        ) as f:
            for row in mat:
                f.write(",".join(str(val) for val in row) + "\n")

        with open(
            f"{os.path.dirname(__file__)}/../resource/dataset/{self.namespace}/map_metadata_{file_time}.json",
            "w",
        ) as f:
            metadata_dict = {
                "resolution": msg.info.resolution,
                "width": msg.info.width,
                "height": msg.info.height,
                "origin": {
                    "position": {
                        "x": msg.info.origin.position.x,
                        "y": msg.info.origin.position.y,
                        "z": msg.info.origin.position.z,
                    },
                    "orientation": {
                        "x": msg.info.origin.orientation.x,
                        "y": msg.info.origin.orientation.y,
                        "z": msg.info.origin.orientation.z,
                        "w": msg.info.origin.orientation.w,
                    },
                },
            }
            json.dump(metadata_dict, f, indent=4)

        # plt.imshow(mat, cmap="gray", origin="lower")
        # plt.axis("off")
        # plt.tight_layout()
        # plt.savefig(f"{os.path.dirname(__file__)}/../resource/dataset/{namespace}/map_plot.png", dpi=300)
        print(
            f"Saved map data from '{self.namespace}' in {time.time() - start:.2f} seconds."
        )
        self.map_received = True
        self.last_timestamp = file_time

    def handle_odom_message(self, msg: Odometry):
        if not self.map_received:
            return  # Wait until the first map is received

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        with open(
            f"{os.path.dirname(__file__)}/../resource/dataset/{self.namespace}/odom_data_{self.last_timestamp}.json",
            "w",
        ) as f:
            odom_dict = {
                "position": {"x": pos.x, "y": pos.y, "z": pos.z},
                "orientation": {"x": ori.x, "y": ori.y, "z": ori.z, "w": ori.w},
            }
            json.dump(odom_dict, f, indent=4)

        self.map_received = False  # Reset for the next map-odom pair


if __name__ == "__main__":
    plot_map_from_file(
        f"/home/ubuntu/ros_ws/src/robot_network/resource/dataset/kris_robot1/map_data_1759757888.txt"
    )

    # rclpy.init()
    # node = DatasetNode("kris_robot1")
    # node1 = DatasetNode("kris_robot2")
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(node.node)
    # executor.add_node(node1.node)
    # executor.spin()
    # node.node.destroy_node()
    # node1.node.destroy_node()
    # rclpy.shutdown()
