from rclpy.node import Node, Publisher, Subscription
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped


class RobotWatcher(Node):
    def __init__(self, robot_name: str):
        super().__init__(f"robot_watcher_{robot_name}")
        self.namespace = robot_name

        # ----- Subscribers -----
        self.odom_subscriber: Subscription[Odometry] = self.create_subscription(
            Odometry,
            f"{self.namespace}/odom",
            self.odom_callback,
            10,
        )

        self.map_subscriber: Subscription[OccupancyGrid] = self.create_subscription(
            OccupancyGrid,
            f"{self.namespace}/map",
            self.map_callback,
            10,
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

        self.get_logger().info(f"RobotWatcher initialized for robot: {self.namespace}")

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z

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
        msg.pose.orientation.z = theta

        self.last_goal = msg
        self.goal_publisher.publish(msg)
        self.get_logger().info(f"Published new goal for {self.namespace}")
