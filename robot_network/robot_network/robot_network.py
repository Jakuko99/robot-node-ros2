import rclpy
import numpy as np
from time import sleep
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node, Publisher, Subscription

from robot_network.robot_watcher import RobotWatcher
from robot_network.neural_network import RobotSwarmOptimizerNetwork
from robot_network.reinforcement_network import ReinforcementSwarmNetwork
from std_srvs.srv import Trigger


class RobotNetwork(Node):
    def __init__(self):
        super().__init__("robot_network_node")

        # ----- Parameters -----
        self.declare_parameter("train_network", False)
        self.declare_parameter("network_model_path", "")
        self.declare_parameter("trained_model_path", "")
        self.declare_parameter("global_map_topic", "global_map")
        self.declare_parameter("goal_marker_topic", "mapping_goals")
        self.declare_parameter("goal_frame_id", "global_map")

        # ----- Variables -----
        self.current_map: OccupancyGrid = None
        self.current_map_width: int = 0
        self.current_map_height: int = 0
        self.robots: dict[str, RobotWatcher] = {}
        self.train_network: bool = (
            self.get_parameter("train_network").get_parameter_value().bool_value
        )
        self.goal_frame_id: str = (
            self.get_parameter("goal_frame_id").get_parameter_value().string_value
        )
        self.static_transforms: dict[str, TFMessage] = {}

        self.optimizer_network = ReinforcementSwarmNetwork(
            train=self.train_network,
            model_path=self.get_parameter("network_model_path")
            .get_parameter_value()
            .string_value,
            trained_model_path=self.get_parameter("trained_model_path")
            .get_parameter_value()
            .string_value,
            parent=self,
        )

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

        self.tf_subscriber: Subscription[TFMessage] = self.create_subscription(
            TFMessage,
            "/tf_static",
            self.store_transforms,
            10,
        )

        # ----- Publishers -----
        self.point_publisher: Publisher[PointStamped] = self.create_publisher(
            PointStamped,
            self.get_parameter("goal_marker_topic").get_parameter_value().string_value,
            10,
        )

        # ----- Services -----
        self.save_model_service = self.create_service(
            Trigger,
            "save_model",
            self.optimizer_network.save_model,
        )
        # ros2 service call /save_model std_srvs/srv/Trigger

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

    def store_transforms(self, msg: TFMessage):
        for transform in msg.transforms:
            if transform.child_frame_id not in self.static_transforms:
                self.static_transforms[transform.child_frame_id] = transform
                self.get_logger().info(
                    f"Received new static transform for {transform.child_frame_id}"
                )

    def process_goals(self):
        if self.current_map:
            if self.train_network:
                self.optimizer_network.train()
                reward = self.optimizer_network.train_network(self.current_map)
                self.get_logger().info(f"Training reward: {reward}")

            else:
                self.optimizer_network.eval()
                self.optimizer_network.train_network(self.current_map)

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
                    self.optimizer_network.add_robot(
                        robot_name, self.robots[robot_name]
                    )

                    self.get_logger().info(f"Discovered robot: {robot_name}")

    def publish_point(self, x: float, y: float):
        point_msg: PointStamped = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.goal_frame_id
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0

        self.point_publisher.publish(point_msg)

    def apply_transform(
        self, robot_name: str, point: list[float, float]
    ) -> list[float, float]:  # Apply static transform to point
        if robot_name + "_map" in self.static_transforms:
            transform: TransformStamped = self.static_transforms[robot_name + "_map"]
            point[0] += transform.transform.translation.x
            point[1] += transform.transform.translation.y
            self.get_logger().info(f"Transformed point for {robot_name}: {point}")
            return point
        else:
            self.get_logger().warn(f"No static transform found for {robot_name}_map")
            return point


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
