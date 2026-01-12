import rclpy
import numpy as np
from time import sleep
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from rclpy.node import Node, Publisher, Subscription

from robot_network.robot_watcher import RobotWatcher
from robot_network.neural_network import RobotSwarmOptimizerNetwork


class RobotNetwork(Node):
    def __init__(self):
        super().__init__("robot_network_node")
        # ----- Parameters -----
        self.declare_parameter("train_network", False)
        self.declare_parameter("network_model_path", "")
        self.declare_parameter("global_map_topic", "global_map")
        self.declare_parameter("goal_marker_topic", "mapping_goals")

        # ----- Subscribers -----
        self.global_map_subscriber: Subscription[OccupancyGrid] = (
            self.create_subscription(
                OccupancyGrid,
                self.get_parameter("global_map_topic")
                .get_parameter_value()
                .string_value,
                self.map_callback,
                10,
            )
        )

        # ----- Publishers -----
        self.point_publisher: Publisher[PointStamped] = self.create_publisher(
            PointStamped,
            self.get_parameter("goal_marker_topic").get_parameter_value().string_value,
            10,
        )

        # ----- Variables -----
        self.current_map: OccupancyGrid = None
        self.current_map_width: int = 0
        self.current_map_height: int = 0
        self.robots: dict[str, RobotWatcher] = {}
        self.train_network: bool = (
            self.get_parameter("train_network").get_parameter_value().bool_value
        )
        self.optimizer_network = RobotSwarmOptimizerNetwork(
            train=self.train_network,
            model_path=self.get_parameter("network_model_path")
            .get_parameter_value()
            .string_value,
        )

        self.get_logger().info(f"Robot network node initialized")

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info(f"Received map data: {msg.info.width}x{msg.info.height}")
        self.current_map = msg
        self.current_map_width = msg.info.width
        self.current_map_height = msg.info.height

        self.get_logger().info(
            f"Frontier count: {self.optimizer_network._count_frontiers(msg)}"
        )

        self.process_goals()

    def process_goals(self):
        if self.current_map:
            width, height = self.current_map_width, self.current_map_height

            if self.train_network:
                self.optimizer_network.train()

            else:
                self.optimizer_network.eval()

        self.discover_robots()
        for watcher_node in self.robots.values():
            rclpy.spin_once(watcher_node, timeout_sec=0.5)

    def discover_robots(self):
        topics: list[tuple[str, list[str]]] = self.get_topic_names_and_types()

        for topic, types in topics:
            if (topic.endswith("/odom")) and ("nav_msgs/msg/Odometry" in types):
                robot_name = topic.split("/")[1]
                if robot_name not in self.robots:
                    self.robots[robot_name] = RobotWatcher(robot_name)
                    self.get_logger().info(f"Discovered robot: {robot_name}")

    def publish_point(self, x: float, y: float):
        point_msg: PointStamped = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "global_map"
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0

        self.point_publisher.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNetwork()
    while rclpy.ok():
        node.discover_robots()
        rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
