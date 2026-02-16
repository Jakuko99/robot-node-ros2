from rcloy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node, Publisher, Subscription
from nav_msgs.msg import OccupancyGrid, Odometry
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)  # correct map QoS profile
import math


class RobotWatcher(Node):
    def __init__(self, namespace: str):
        super.__init__(f"watcher_{namespace}")

        # ----- Variables -----
        self.namespace: str = namespace
        self.goal_future: Future = None
        self.last_odom: Odometry = None
        self.last_cmd_vel: Twist = None
        self.last_goal: PoseStamped = None
        self.current_map: OccupancyGrid = None
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self.moving: bool = False
        self.action_server_connected: bool = False
        map_qos: QoSProfile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ----- Clients -----
        self.nav_client: ActionClient = ActionClient(
            self, NavigateToPose, f"{self.namespace}/navigate_to_pose"
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
            f"{self.namespace}/map",
            self.map_callback,
            map_qos,
        )

        # self.cmd_vel_sub: Subscription[Twist] = self.create_subscription(
        #     Twist,
        #     f"{self.namespace}/cmd_vel",
        #     self.cmd_vel_callback,
        #     10,
        # )

        # ----- Publishers -----
        self.goal_pub: Publisher[PoseStamped] = self.create_publisher(
            PoseStamped, f"{self.namespace}/goal_pose", 10
        )

        # ----- Initialization -----
        self.get_logger().info("Attempting to connect to action server")
        result: bool = self.nav_client.wait_for_server(timeout=10.0)

        if result:
            self.get_logger().info("Successfully connected to action server")
            self.action_server_connected = True
        else:
            self.get_logger().error("Failed to connect to action server within timeout")

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z  # Simplified for 2D orientation

    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg

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
            self.goal_future = self.nav_client.send_goal_async(
                NavigateToPose.Goal(pose=goal_pose)
            )
            self.goal_future.add_done_callback(self.goal_response_callback)
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

        self.get_logger().warn(f"No map received yet for {self.namespace}, returning empty map")
        return OccupancyGrid()