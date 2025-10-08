import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
import matplotlib.pyplot as plt
import pickle as pkl
import numpy as np
import time
import os


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
        self.odom_ready: bool = False
        self.last_timestamp: int = None

    def handle_map_message(self, msg: OccupancyGrid):
        self.map_mat = np.reshape(msg.data, (msg.info.height, msg.info.width)).astype(
            float
        )
        self.map_mat[self.map_mat == -1] = 0.5
        self.map_mat[self.map_mat == 0] = 0.0
        self.map_mat[self.map_mat == 100] = 1.0

        self.metadata_dict = {
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

        self.map_received = True

    def handle_odom_message(self, msg: Odometry):
        if not self.map_received:
            return  # Wait until the first map is received

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        self.odom_dict = {
            "position": {
                "x": pos.x,
                "y": pos.y,
                "z": pos.z,
            },
            "orientation": {
                "x": ori.x,
                "y": ori.y,
                "z": ori.z,
                "w": ori.w,
            },
        }

        self.odom_ready = True
        self.map_received = False  # Reset for the next map-odom pair
        self.save_data()

    def save_data(self):
        if self.odom_ready:
            timestamp = int(time.time())
            compressed_data = {
                "map": self.map_mat,
                "metadata": self.metadata_dict,
                "odom": self.odom_dict,
            }

            with open(
                os.path.join(
                    f"{os.path.dirname(__file__)}/../resource/dataset_compressed/{self.namespace}",
                    f"compressed_data_{timestamp}.pkl",
                ),
                "wb",
            ) as f:
                pkl.dump(compressed_data, f)

            self.odom_ready = False


if __name__ == "__main__":
    rclpy.init()
    node = DatasetNode("kris_robot1")
    node1 = DatasetNode("kris_robot2")
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node.node)
    executor.add_node(node1.node)
    executor.spin()
    node.node.destroy_node()
    node1.node.destroy_node()
    rclpy.shutdown()
