import rclpy
from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.task import Future
from rclpy.timer import Timer
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node, Publisher, Subscription
from nav_msgs.msg import OccupancyGrid, Odometry
from action_msgs.msg import GoalStatus
from rclpy.action.client import ClientGoalHandle
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from time import time
import math
import numpy as np

from robot_node.decision_network import DecisionNetwork, FeedbackLayer


class RobotNode(Node):
    def __init__(self):
        super().__init__(f"robot_node")

        # ----- Parameters ------
        self.declare_parameter("robot_name", "kris_robot1")
        self.declare_parameter("pheromone_map_topic", "pheromone_map")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("goal_process_interval", 5.0)
        self.declare_parameter("goal_topic", "goal_pose")

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

        # ----- Variables -----
        self.goal_future: Future = None
        self.goal_handle: ClientGoalHandle = None
        self.last_odom: Odometry = None
        self.last_goal: PoseStamped = None
        self.current_map: OccupancyGrid = None
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self.moving: bool = False
        self.action_server_connected: bool = False
        self.goal_publish_time: float = time()
        map_qos: QoSProfile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )  # correct map QoS profile

        # ----- Timers -----

        # ----- Clients -----
        self.nav_client: ActionClient = ActionClient(
            self, NavigateToPose, f"{self.namespace}/navigate_to_pose"
        )

        # ----- Sevices -----

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

        # ----- Publishers -----
        self.goal_pub: Publisher[PoseStamped] = self.create_publisher(
            PoseStamped, f"{self.namespace}/{self.goal_topic}", 10
        )

        # ----- Initialization -----
        self.get_logger().info("Attempting to connect to action server")
        result: bool = self.nav_client.wait_for_server(timeout_sec=10.0)

        if result:
            self.get_logger().info("Successfully connected to action server")
            self.action_server_connected = True
        else:
            self.get_logger().error("Failed to connect to action server within timeout")

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.theta = 2.0 * math.atan2(qz, qw)

    def map_callback(self, msg: OccupancyGrid):
        pass

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

        if not self.action_server_connected:
            self.get_logger().warn(
                "Published goal, but action server is not connected, cannot receive completion from nav stack!"
            )
            self.goal_pub.publish(goal_pose)

        else:
            self.goal_future = self.nav_client.send_goal_async(NavigateToPose.Goal(pose=goal_pose))
            self.goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info(f"Published new goal [{x}, {y}, {theta}]")
            self.goal_publish_time = time()
            self.moving = True

    def goal_response_callback(self, future: Future):
        self.goal_handle: ClientGoalHandle = future.result()  # goal result
        result_future: Future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future: Future):
        result: ClientGoalHandle = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:  # SUCCEEDED
            self.get_logger().info(f"Goal completed successfully")
        elif result.status in [
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
        ]:  # ABORTED or CANCELED
            self.get_logger().warn(f"Goal was abandoned")
        else:
            self.get_logger().error(f"Goal failed with status {result.status}")

        self.moving = False  # goal is done, so we are no longer moving

    def robot_discovery_callback(self):
        nodes: list[str] = self.get_node_names()
        topics: list[str] = [topic[0] for topic in self.get_topic_names_and_types()]

    @property
    def is_moving(self) -> bool:
        return self.moving


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
