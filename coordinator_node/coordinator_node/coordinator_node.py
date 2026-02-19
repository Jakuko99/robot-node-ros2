import rclpy
from scipy.spatial.transform import Rotation
import numpy as np
from nav_msgs.msg import OccupancyGrid
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

from coordinator_node.reinforcement_network import ReinforcementSwarmNetwork


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__("coordinator_node")

        # ----- Parameters -----
        self.declare_parameter("train_network", False)
        self.declare_parameter("network_model_path", "")
        self.declare_parameter("global_map_topic", "global_map")
        self.declare_parameter("goal_marker_topic", "mapping_goals")
        self.declare_parameter("global_frame_id", "global_map")
        self.declare_parameter("goal_process_interval", 2.0)

        # ----- Variables -----
        self.current_global_map: OccupancyGrid = None
        self.robots: list[str] = []
        self.optimizer_network: ReinforcementSwarmNetwork = ReinforcementSwarmNetwork()
        self.train_network: bool = (
            self.get_parameter("train_network").get_parameter_value().bool_value
        )
        self.global_frame_id: str = (
            self.get_parameter("global_frame_id").get_parameter_value().string_value
        )
        self.network_model_path: str = (
            self.get_parameter("network_model_path").get_parameter_value().string_value
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
            self.save_model,
        )
        # ros2 service call /save_model std_srvs/srv/Trigger

        # ----- Timers -----
        self.discovery_timer: Timer = self.create_timer(
            5.0, callback=self.discover_robots
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

        self.get_logger().info("CoordinatorNode node initialized")

    def map_callback(self, msg: OccupancyGrid):
        self.current_global_map = msg

    def store_transforms(self, msg: TFMessage):
        transform: TransformStamped = None  # typeset helper
        for transform in msg.transforms:
            if transform.child_frame_id not in self.static_transforms:
                self.static_transforms[transform.child_frame_id] = transform
                self.get_logger().info(
                    f"Received new static transform for {transform.child_frame_id}"
                )

    def process_goals(self):
        if self.optimizer_network is not None:
            robot_map: OccupancyGrid = self.current_global_map

            if self.train_network:
                self.optimizer_network.train()
                reward: float = self.optimizer_network.train_network(robot_map)
                self.get_logger().info(f"Trained network with reward: {reward}")
            else:
                self.optimizer_network.eval()
                self.optimizer_network.eval_network(robot_map)

    def save_model(self, request: Trigger.Request, response: Trigger.Response):
        """Save all robot network models."""
        try:
            if self.optimizer_network is not None:
                request_inner = Trigger.Request()
                response_inner = Trigger.Response()
                self.optimizer_network.save_model(request_inner, response_inner)
                if not response_inner.success:
                    response.success = False
                    response.message = f"Failed to save model: {response_inner.message}"
                    return response

            response.success = True
            response.message = f"Saved model for coordinator node"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error saving model: {str(e)}"
            return response

    def discover_robots(self):
        nodes: list[str] = self.get_node_names()

    def publish_point(self, x: float, y: float):
        point_msg: PointStamped = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.global_frame_id
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0

        self.point_pub.publish(point_msg)

    def apply_transform(
        self, robot_name: str, point: list[float, float]
    ) -> list[float, float]:
        robot_trainsform: TransformStamped = self.static_transforms.get(
            f"{robot_name}_map"
        )

        if robot_trainsform is None:
            self.get_logger().warn(
                f"No static transform found for {robot_name}_map, cannot transform point"
            )
            return point

        # transform from global frame to robot local frame using ros2 builtin methods
        try:
            # Extract translation and rotation
            translation = robot_trainsform.transform.translation
            rotation = robot_trainsform.transform.rotation

            # Convert quaternion to yaw angle
            quat: list[float] = [rotation.x, rotation.y, rotation.z, rotation.w]
            yaw = Rotation.from_quat(quat).as_euler("xyz")[2]

            # Create 2D transformation matrix
            cos_yaw: float = np.cos(yaw)
            sin_yaw: float = np.sin(yaw)
            transform_matrix: np.ndarray = np.array(
                [
                    [cos_yaw, -sin_yaw, translation.x],
                    [sin_yaw, cos_yaw, translation.y],
                    [0, 0, 1],
                ]
            )

            # Transform point
            point_homogeneous: np.ndarray = np.array([point[0], point[1], 1])
            transformed_point: np.ndarray = transform_matrix @ point_homogeneous

            return transformed_point[:2].tolist()
        except Exception as e:
            self.get_logger().error(
                f"Error applying transform for {robot_name}: {str(e)}"
            )
            return point


def main():
    rclpy.init()
    coordinator_node_node = CoordinatorNode()
    while rclpy.ok():
        coordinator_node_node.discover_robots()  # Initial robot discovery
        rclpy.spin(coordinator_node_node)

    coordinator_node_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
