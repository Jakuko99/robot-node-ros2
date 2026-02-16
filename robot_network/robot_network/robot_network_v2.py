import rclpy
from nav_mgs.msg import OccupancyGrid
from robot_network.robot_network import robot_watcher
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node, Publisher, Subscription
from rclpy.timer import Timer
from std_srvs.srv import Trigger
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)  # correct map QoS profile

from robot_network.robot_watcher_v2 import RobotWatcher
from robot_network.reinforcement_network import ReinforcementSwarmNetwork


class RobotNetwork(Node):
    def __init__(self):
        super().__init__("robot_network")

        # ----- Parameters -----
        self.declare_parameter("train_network", False)
        self.declare_parameter("network_model_path", "")
        self.declare_parameter("global_map_topic", "global_map")
        self.declare_parameter("goal_marker_topic", "mapping_goals")
        self.declare_parameter("global_frame_id", "global_map")
        self.declare_parameter("use_local_maps", False)
        self.declare_parameter("goal_process_interval", 2.0)
        self.declare_parameter("robot_spin_interval", 2.0)

        # ----- Variables -----
        self.current_global_map: OccupancyGrid = None
        self.robots: dict[str, RobotWatcher] = {}
        self.optimizer_networks: dict[str, ReinforcementSwarmNetwork] = {}
        self.train_network: bool = (
            self.get_parameter("train_network").get_parameter_value().bool
        )
        self.global_frame_id: str = (
            self.get_parameter("global_frame_id").get_parameter_value().string_value
        )
        self.network_model_path: str = (
            self.get_parameter("network_model_path").get_parameter_value().string_value
        )
        self.use_local_maps: bool = (
            self.get_parameter("use_local_maps").get_parameter_value().bool_value
        )
        self.static_transforms: dict[str, TFMessage] = {}
        map_qos: QoSProfile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ----- Services -----
        self.save_model_service = self.create_service(
            Trigger,
            "save_model",
            self.save_all_models,
        )
        # ros2 service call /save_model std_srvs/srv/Trigger

        # ----- Timers -----
        self.discovery_timer: Timer = self.create_timer(
            5.0, callback=self.discover_robots
        )
        self.robot_spin_timer: Timer = self.create_timer(
            self.get_parameter("robot_spin_interval")
            .get_parameter_value()
            .double_value,
            callback=self.spin_robots,
        )
        self.goal_timer: Timer = self.create_timer(
            self.get_parameter("goal_process_interval")
            .get_parameter_value()
            .double_value,
            callback=self.process_goals,
        )

        # ----- Subscribers -----
        self.global_map_sub: Subscription[OccupancyGrid] = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("global_map_topic").get_parameter_value().string_value,
            self.map_callback,
            map_qos,
        )

        self.static_tf_sub: Subscription[TFMessage] = self.create_subscription(
            TFMessage,
            "/tf_static",
            self.store_transforms,
            10,
        )

        # ----- Publishers -----
        self.point_pub: Publisher[PointStamped] = self.create_publisher(
            PointStamped,
            self.get_parameter("goal_marker_topic").get_parameter_value().string_value,
            10,
        )

        self.get_logger().info("RobotNetwork node initialized")

    def map_callback(self, msg: OccupancyGrid):
        self.current_global_map = msg

    def store_transforms(self, msg: TFMessage):
        for transform in msg.transforms:
            if transform.child_frame_id not in self.static_transforms:
                self.static_transforms[transform.child_frame_id] = transform
                self.get_logger().info(
                    f"Received new static transform for {transform.child_frame_id}"
                )

    def process_goals(self):
        for robot_name, network in self.optimizer_networks.items():
            if self.use_local_maps:
                robot_map: OccupancyGrid = network.robot_watcher.get_local_map()
            else:
                robot_map: OccupancyGrid = self.current_global_map

            if self.train_network:
                network.train()
                reward: float = network.train_network(robot_map)
                self.get_logger().info(
                    f"Trained network for {robot_name} with reward: {reward}"
                )
            else:
                network.eval()
                network.train_network(robot_map)

    def discover_robots(self):
        topics: list[tuple[str, list[str, str]]] = self.get_topic_names_and_types()

        for topic, types in topics:
            if (topic.endswith("/odom")) and ("nav_msgs/msg/Odometry" in types):
                robot_name = topic.split("/")[1]
                if robot_name not in self.robots:
                    # Create robot watcher
                    robot_watcher = RobotWatcher(robot_name)
                    rclpy.spin_once(
                        robot_watcher, timeout_sec=1.0
                    )  # Allow watcher to initialize
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
        point_msg.header.frame_id = self.global_frame_idd
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0

        self.point_publisher.publish(point_msg)

    def spin_robots(self):
        for watcher_node in self.robots.values():
            rclpy.spin_once(watcher_node, timeout_sec=1.0)

    def apply_transform(
        self, robot_name: str, point: list[float, float]
    ) -> list[float, float]:
        return point  # Placeholder for actual transform logic


if __name__ == "__main__":
    rclpy.init()
    robot_network_node = RobotNetwork()
    while rclpy.ok():
        robot_network_node.discover_robots()  # Initial robot discovery
        rclpy.spin(robot_network_node)

    robot_network_node.destroy_node()
    rclpy.shutdown()
