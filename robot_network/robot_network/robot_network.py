import rclpy
from rclpy.node import Node, Publisher
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
from tf2_ros import TransformBroadcaster
import math
import random


class RobotNetwork(Node):
    def __init__(self, namespace: str = ""):
        super().__init__("robot")

        # Publishers and subscribers
        self.velocity_publisher = self.create_publisher(Twist, f"{namespace}/cmd_vel", 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, f"{namespace}/odom", self.odom_callback, 10
        )
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, f"{namespace}/map", self.map_callback, 10
        )

        self.get_logger().info(f"Robot node initialized in namespace: {namespace}")

    def odom_callback(self, msg: Odometry):
        self.get_logger().info(f"Received odometry: {msg}")

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info(f"Received map data: {msg.info.width}x{msg.info.height}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotNetwork("kris_robot1")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
