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
        self.optimizer_networks: dict[str, ReinforcementSwarmNetwork] = {}
        self.train_network: bool = (
            self.get_parameter("train_network").get_parameter_value().bool_value
        )
        self.goal_frame_id: str = (
            self.get_parameter("goal_frame_id").get_parameter_value().string_value
        )
        self.network_model_path = (
            self.get_parameter("network_model_path").get_parameter_value().string_value
        )
        self.trained_model_path = (
            self.get_parameter("trained_model_path").get_parameter_value().string_value
        )
        self.static_transforms: dict[str, TFMessage] = {}

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
            self.save_all_models,
        )
        # ros2 service call /save_model std_srvs/srv/Trigger

        self.get_logger().info(f"Robot network node initialized")

    def save_all_models(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Save all robot network models."""
        try:
            for robot_name, network in self.optimizer_networks.items():
                request_inner = Trigger.Request()
                response_inner = Trigger.Response()
                network.save_model(request_inner, response_inner)
                if not response_inner.success:
                    response.success = False
                    response.message = f"Failed to save model for {robot_name}: {response_inner.message}"
                    return response

            response.success = True
            response.message = f"Saved {len(self.optimizer_networks)} robot models"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error saving models: {str(e)}"
            return response

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info(f"Received map data: {msg.info.width}x{msg.info.height}")
        self.current_map = msg
        self.current_map_width = msg.info.width
        self.current_map_height = msg.info.height

        # Log frontier count (using first network if available, or create temporary one)
        if self.optimizer_networks:
            first_network = next(iter(self.optimizer_networks.values()))
            self.get_logger().info(
                f"Frontier count: {first_network._count_frontiers(msg)}"
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
            # Process each robot's network
            for robot_name, network in self.optimizer_networks.items():
                if self.train_network:
                    network.train()
                    reward = network.train_network(self.current_map)
                    self.get_logger().info(f"{robot_name} training reward: {reward}")
                else:
                    network.eval()
                    network.train_network(self.current_map)

        self.discover_robots()
        for watcher_node in self.robots.values():
            rclpy.spin_once(watcher_node, timeout_sec=0.5)

    def discover_robots(self):
        topics: list[tuple[str, list[str]]] = self.get_topic_names_and_types()

        for topic, types in topics:
            if (topic.endswith("/odom")) and ("nav_msgs/msg/Odometry" in types):
                robot_name = topic.split("/")[1]
                if robot_name not in self.robots:
                    # Create robot watcher
                    robot_watcher = RobotWatcher(robot_name)
                    self.robots[robot_name] = robot_watcher

                    # Create dedicated network for this robot
                    model_path = (
                        f"{self.network_model_path}/{robot_name}_optimizer.pt"
                        if self.network_model_path
                        else ""
                    )

                    network = ReinforcementSwarmNetwork(
                        robot_watcher=robot_watcher,
                        train=self.train_network,
                        model_path=model_path,
                        parent=self,
                    )
                    self.optimizer_networks[robot_name] = network

                    self.get_logger().info(
                        f"Discovered robot: {robot_name} with dedicated network"
                    )

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
