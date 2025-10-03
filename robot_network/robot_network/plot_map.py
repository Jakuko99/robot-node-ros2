import matplotlib.pyplot as plt
import numpy as np
import time
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
import os


def plot_map_from_file(file_path: str):
    with open(file_path, "r") as f:
        start = time.time()
        lines = f.readlines()

        mat = np.zeros((len(lines), len(lines[0].strip().split(","))), dtype=float)

        l = 0
        for line in lines:
            i = 0
            for val in line.strip().split(","):
                mat[l, i] = int(125 if val == '0.5' else (0 if val == '0.0' else 255))
                i += 1
            l += 1

    plt.imshow(mat, cmap="gray", origin="lower")
    plt.colorbar(ticks=[0, 125, 255], label="Cell Value")
    plt.savefig("map_plot.png", dpi=300)
    print(
        f"Map ({mat.shape[1]}x{mat.shape[0]}) plotted and saved to 'map_plot.png' in {time.time() - start:.2f} seconds."
    )


def init_node(namespace: str):
    node = rclpy.create_node(f"map_plotter_{namespace}")
    subscription = node.create_subscription(
        OccupancyGrid,
        f"{namespace}/map",  # Adjust topic name as needed
        lambda msg: handle_map_message(msg, namespace),
        10,
    )
    odom_subscription = node.create_subscription(
        Odometry,
        f"{namespace}/odom",
        lambda msg: handle_odom_message(msg, namespace),
        10
    )
    return node


def handle_map_message(msg: OccupancyGrid, namespace: str):
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

    with open(f"{os.path.dirname(__file__)}/../resource/dataset/{namespace}/map_data_{int(time.time())}.txt", "w") as f:
        for row in mat:
            f.write(",".join(str(val) for val in row) + "\n")

    # plt.imshow(mat, cmap="gray", origin="lower")    
    # plt.axis("off")
    # plt.tight_layout()
    # plt.savefig(f"{os.path.dirname(__file__)}/../resource/dataset/{namespace}/map_plot.png", dpi=300)
    print(f"Saved map data from '{namespace}' in {time.time() - start:.2f} seconds.")

def handle_odom_message(msg: Odometry, namespace: str):
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    
    with open(f"{os.path.dirname(__file__)}/../resource/dataset/{namespace}/odom_data_{int(time.time())}.txt", "w") as f:
        f.write(f"Position: x={pos.x}, y={pos.y}, z={pos.z}\n")
        f.write(f"Orientation: x={ori.x}, y={ori.y}, z={ori.z}, w={ori.w}\n")

if __name__ == "__main__":
    plot_map_from_file(f"/home/ubuntu/ros_ws/src/robot_network/resource/dataset/kris_robot2/map_data_1759481999.txt")
    # rclpy.init()
    # node = init_node("kris_robot1")
    # node1 = init_node("kris_robot2")
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(node)
    # executor.add_node(node1)
    # executor.spin()
    # node.destroy_node()
    # node1.destroy_node()
    # rclpy.shutdown()
