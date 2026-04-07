import rclpy
from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.node import Node, Publisher, Subscription
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from time import time
import math
import torch
import numpy as np
import os

from robot_node.decision_network import DecisionNetwork


class RobotNode(Node):
    def __init__(self):
        super().__init__(f"robot_node")

        # ----- Parameters ------
        self.declare_parameter("robot_name", "kris_robot1")
        self.declare_parameter("pheromone_map_topic", "pheromone_map")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("goal_process_interval", 5.0)
        self.declare_parameter("goal_topic", "goal_pose")
        self.declare_parameter("static_transform_x", 0.0)
        self.declare_parameter("static_transform_y", 0.0)
        self.declare_parameter("goal_timeout", 30.0)
        self.declare_parameter("training_interval", 10.0)
        self.declare_parameter("model_path", "model.pth")

        self.namespace: str = self.get_parameter("robot_name").get_parameter_value().string_value
        self.map_frame_id: str = (
            self.get_parameter("map_frame_id").get_parameter_value().string_value
        )
        self.pheromone_map_topic: str = (
            self.get_parameter("pheromone_map_topic").get_parameter_value().string_value
        )
        self.goal_process_interval: float = (
            self.get_parameter("goal_process_interval").get_parameter_value().double_value
        )
        self.goal_topic: str = self.get_parameter("goal_topic").get_parameter_value().string_value
        self.static_transform_x: float = (
            self.get_parameter("static_transform_x").get_parameter_value().double_value
        )
        self.static_transform_y: float = (
            self.get_parameter("static_transform_y").get_parameter_value().double_value
        )
        self.goal_timeout: float = (
            self.get_parameter("goal_timeout").get_parameter_value().double_value
        )
        self.training_interval: float = (
            self.get_parameter("training_interval").get_parameter_value().double_value
        )
        self.model_path: str = self.get_parameter("model_path").get_parameter_value().string_value

        # ----- Variables -----
        self.last_odom: Odometry = None
        self.last_goal: PoseStamped = None
        self.current_map: OccupancyGrid = None
        self.current_local_map: OccupancyGrid = None
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self.moving: bool = False
        self.goal_publish_time: float = time()
        map_qos: QoSProfile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )  # correct map QoS profile

        self.decision_network: DecisionNetwork = DecisionNetwork(
            self.namespace, pheromone_decay=0.1, parent=self
        )
        device: torch.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.decision_network.to(device)

        if os.path.exists(self.model_path):  # auto load model if it exists
            try:
                self.decision_network.load_state_dict(
                    torch.load(self.model_path, map_location=device)
                )
                self.get_logger().info(f"Loaded model from {self.model_path}")
            except RuntimeError as e:
                self.get_logger().error(f"Error loading model from {self.model_path}: {e}")

        # ----- Timers -----
        self.goal_timer: Timer = self.create_timer(
            self.goal_process_interval, callback=self.goal_timer_callback
        )

        self.training_timer: Timer = self.create_timer(
            self.training_interval, callback=self.training_timer_callback
        )

        self.discovery_timer: Timer = self.create_timer(
            10.0, callback=self.robot_discovery_callback
        )

        # ----- Clients -----

        # ----- Services -----
        self.create_service(
            Trigger,
            f"{self.namespace}/save_model",
            lambda req, res: self.decision_network.save_model(req, res, self.model_path),
        )
        self.create_service(
            Trigger,
            f"{self.namespace}/load_model",
            lambda req, res: self.decision_network.load_model(req, res, self.model_path),
        )

        # ----- Subscribers -----
        self.odom_sub: Subscription[Odometry] = self.create_subscription(
            Odometry,
            f"{self.namespace}/odom",
            self.odom_callback,
            10,
        )

        self.map_sub: Subscription[OccupancyGrid] = self.create_subscription(
            OccupancyGrid,
            f"{self.namespace}/{self.pheromone_map_topic}",
            self.map_callback,
            map_qos,
        )

        self.local_map_sub: Subscription[OccupancyGrid] = self.create_subscription(
            OccupancyGrid,
            f"{self.namespace}/map",
            self.local_map_callback,
            map_qos,
        )

        self.goal_sub: list[Subscription[PoseStamped]] = list()

        # ----- Publishers -----
        self.goal_pub: Publisher[PoseStamped] = self.create_publisher(
            PoseStamped, f"{self.namespace}/{self.goal_topic}", 10
        )

        # ----- Initialization -----
        self.get_logger().info(f"Robot node '{self.namespace}' initialized")

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.theta = 2.0 * math.atan2(qz, qw)

    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg
        self.decision_network.update_state(self.current_map, self.last_odom)

    def local_map_callback(self, msg: OccupancyGrid):
        self.current_local_map = msg

    def goal_timer_callback(self):
        if self.current_local_map is None or self.last_odom is None or self.current_map is None:
            return

        if self.moving:
            if self.last_goal and self.goal_reached(self.last_goal, self.last_odom, threshold=0.75):
                self.get_logger().info("Goal reached")
                self.moving = False
                return

            if time() - self.goal_publish_time > self.goal_timeout:
                self.get_logger().warn("Goal timeout exceeded, canceling goal")
                self.moving = False
            return

        if not self.moving:
            new_goal: tuple[float, float] = self.decision_network.generate_goal()
            if new_goal:
                self.publish_goal(
                    new_goal[0] + self.static_transform_x, new_goal[1] + self.static_transform_y
                )

    def training_timer_callback(self):
        metrics = self.decision_network.train_epoch()
        if metrics is None:
            return

        self.get_logger().info(
            "train_epoch: "
            f"loss={metrics['loss']:.4f}, "
            f"policy={metrics['policy_loss']:.4f}, "
            f"value={metrics['value_loss']:.4f}, "
            f"entropy={metrics['entropy']:.4f}, "
            f"avg_reward={metrics['avg_reward']:.4f}, "
            f"batch={metrics['batch_size']}"
        )

    def publish_goal(self, x: float, y: float, theta: float = 0.0):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = f"{self.namespace}_map"
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        if theta == 0.0:  # compute theta based on current position and goal
            dx: float = x - self.x
            dy: float = y - self.y
            theta: float = math.atan2(dy, dx)

        # Convert theta to quaternion for orientation
        qz: float = math.sin(theta / 2.0)
        qw: float = math.cos(theta / 2.0)
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        self.last_goal = goal_pose

        self.goal_pub.publish(goal_pose)
        self.goal_publish_time = time()
        self.moving = True

    def other_goal_callback(self, msg: PoseStamped, namespace: str):
        pass

    def robot_discovery_callback(self):
        # nodes: list[str] = self.get_node_names()
        topics: list[str] = [topic[0] for topic in self.get_topic_names_and_types()]

        for topic in topics:
            if topic.endswith("/navigate_to_pose") and topic.split("/")[1] != self.namespace:
                other_robot_namespace: str = topic.split("/")[1]
                if other_robot_namespace in [sub.topic.split("/")[1] for sub in self.goal_sub]:
                    continue  # already subscribed to this robot's goal topic

                self.get_logger().info(f"Discovered new robot: {other_robot_namespace}")
                self.goal_sub.append(
                    self.create_subscription(
                        PoseStamped,
                        f"/{other_robot_namespace}/{self.goal_topic}",
                        lambda msg, ns=other_robot_namespace: self.other_goal_callback(msg, ns),
                        10,
                    )
                )

    @property
    def is_moving(self) -> bool:
        return self.moving

    @staticmethod
    def goal_reached(
        goal_pose: PoseStamped,
        current_odom: Odometry,
        threshold: float = 0.5,
    ) -> bool:
        dx: float = goal_pose.pose.position.x - current_odom.pose.pose.position.x
        dy: float = goal_pose.pose.position.y - current_odom.pose.pose.position.y
        distance: float = math.sqrt(dx**2 + dy**2)
        return distance < threshold


def main():
    rclpy.init()
    robot_node = RobotNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(robot_node)
    except KeyboardInterrupt:
        robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
