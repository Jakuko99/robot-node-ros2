import rclpy
from rclpy.node import Node, Publisher
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
from tf2_ros import TransformBroadcaster

from neural_network import RobotSwarmOptimizerNetwork


class RobotNetwork(Node):
    def __init__(self):
        super().__init__("robot")
        self.declare_parameter("namespace", "kris_robot")

        # ----- Subscribers -----
        self.odom_subscriber = self.create_subscription(
            Odometry,
            f"{self.get_parameter('namespace').value}/odom",
            self.odom_callback,
            10,
        )
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            f"{self.get_parameter('namespace').value}/map",
            self.map_callback,
            10,
        )
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            f"{self.get_parameter('namespace').value}/scan",
            self.scan_callback,
            10,
        )
        self.poi_subscriber = self.create_subscription(
            PoseStamped,
            f"{self.get_parameter('namespace').value}/poi",
            self.poi_callback,
            10,
        )

        # ----- Publishers -----
        self.goal_publisher = self.create_publisher(
            PoseStamped, f"{self.get_parameter('namespace').value}/goal_pose", 10
        )
        self.explored_map_publisher = self.create_publisher(
            OccupancyGrid, f"{self.get_parameter('namespace').value}/explored_map", 10
        )

        self.get_logger().info(
            f"Robot node initialized in namespace: {self.get_parameter('namespace').value}"
        )

    def odom_callback(self, msg: Odometry):
        self.get_logger().info(f"Received odometry: {msg}")

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info(f"Received map data: {msg.info.width}x{msg.info.height}")

    def scan_callback(self, msg: LaserScan):
        self.get_logger().info(f"Received laser scan with {len(msg.ranges)} ranges")

    def poi_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received POI at position: {msg.pose.position}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotNetwork()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
