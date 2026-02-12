from rclpy.action import ActionClient
from rclpy.task import Future
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node, Publisher, Subscription
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseStamped
import math


class RobotWatcher(Node):
    def __init__(self, robot_name: str):
        super().__init__(f"robot_watcher_{robot_name}")
        self.namespace: str = robot_name

        self.nav_client: ActionClient = ActionClient(
            self, NavigateToPose, f"{self.namespace}/navigate_to_pose"
        )
        self.send_goal_future: Future = None
        self.get_logger().info(
            f"Waiting for navigation action server for {self.namespace}..."
        )
        result: bool = self.nav_client.wait_for_server(
            timeout_sec=10.0
        )  # Wait for the action server to be available

        if result:
            self.get_logger().info(
                f"Connected to navigation action server for {self.namespace}"
            )
        else:
            self.get_logger().error(
                f"Failed to connect to navigation action server for {self.namespace}"
            )

        # ----- Subscribers -----
        self.odom_subscriber: Subscription[Odometry] = self.create_subscription(
            Odometry,
            f"{self.namespace}/odom",
            self.odom_callback,
            10,
        )

        # Create QoS profile for map subscription to match typical map publishers
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_subscriber: Subscription[OccupancyGrid] = self.create_subscription(
            OccupancyGrid,
            f"{self.namespace}/map",
            self.map_callback,
            map_qos,
        )

        self.cmd_vel_subscriber: Subscription[Twist] = self.create_subscription(
            Twist,
            f"{self.namespace}/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        # ---- Publishers -----
        self.goal_publisher: Publisher[PoseStamped] = self.create_publisher(
            PoseStamped, f"{self.namespace}/goal_pose", 10
        )

        # ----- Variables -----
        self.current_map: OccupancyGrid = None
        self.last_odom: Odometry = None
        self.last_cmd_vel: Twist = None
        self.last_goal: PoseStamped = None
        self.map_frame_id: str = f"{self.namespace}_map"
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self.moving: bool = False
        self.linear_x: float = 0.0
        self.angular_z: float = 0.0

        self.get_logger().info(f"RobotWatcher initialized for robot: {self.namespace}")

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z

        self.linear_x = msg.twist.twist.linear.x
        self.angular_z = msg.twist.twist.angular.z

    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel = msg

    def publish_goal(self, x: float, y: float, theta: float = 0.0):
        msg: PoseStamped = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        if theta == 0.0:
            dx = x - self.x
            dy = y - self.y
            theta = math.atan2(dy, dx)

        # Convert theta to quaternion for orientation
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.last_goal = msg
        # self.goal_publisher.publish(msg)

        self.send_goal_future: Future = self.nav_client.send_goal_async(
            NavigateToPose.Goal(pose=msg)
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(
            f"Published new goal for {self.namespace} [{x}, {y}, {theta}]"
        )
        self.moving = True

    def goal_response_callback(self, future: Future):
        result = future.result()  # get result of sending the goal
        result_future = result.get_result_async()
        result_future.add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future: Future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:  # SUCCEEDED
            self.get_logger().info(f"Goal completed successfully for {self.namespace}")
        elif result.status == GoalStatus.STATUS_ABORTED:  # ABORTED
            self.get_logger().warn(f"Goal was aborted for {self.namespace}")
        else:
            self.get_logger().warn(
                f"Goal failed with status {result.status} for {self.namespace}"
            )
        self.moving = False  # goal is done, so we are no longer moving

    @property
    def is_moving(self) -> bool:
        return self.moving

    def get_local_map(self) -> OccupancyGrid:
        if self.current_map:
            return self.current_map

        self.get_logger().warn(
            f"No map received yet for {self.namespace}",
            throttle_duration_sec=10.0,  # Only warn every 10 seconds
        )
        return OccupancyGrid()
