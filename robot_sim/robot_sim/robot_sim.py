import rclpy
from rclpy.node import Node, Publisher
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from tf2_ros import TransformBroadcaster
import math
import random


class WheeledRobotSimulator(Node):
    def __init__(self):
        super().__init__("robot")
        self.declare_parameter("robot_description", "robot.urdf")

        # Publishers and subscribers
        self.velocity_subscriber = self.create_subscription(
            Twist, "cmd_vel", self.velocity_callback, 10
        )
        self.laser_publisher = self.create_publisher(LaserScan, "scan", 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # add joint published
        self.joint_publisher = self.create_publisher(JointState, "joint_states", 10)

        self.pub: Publisher = self.create_publisher(String, "robot_description", 10)
        robot_msg = String()
        robot_msg.data = (
            self.get_parameter("robot_description").get_parameter_value().string_value
        )
        self.pub.publish(robot_msg)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation in radians
        self.v_linear = 0.0
        self.v_angular = 0.0

        # Laser sensor parameters
        self.laser_ranges = [1.0] * 6  # Simulated distances
        self.laser_angles = [-np.pi / 3, -np.pi / 6, 0, np.pi / 6, np.pi / 3, np.pi / 2]

        # Timers for updating movement and sensors
        self.create_timer(0.1, self.update_robot_state)  # Update at 10Hz
        self.create_timer(0.5, self.publish_laser_scan)  # Publish scan at 2Hz

    def velocity_callback(self, msg: Twist):
        """Callback for velocity commands."""
        self.v_linear = msg.linear.x
        self.v_angular = msg.angular.z

    def update_robot_state(self):
        """Update the robot's position based on velocity commands."""
        dt = 0.1  # Time step
        self.theta += self.v_angular * dt
        self.x += self.v_linear * np.cos(self.theta) * dt
        self.y += self.v_linear * np.sin(self.theta) * dt
        self.get_logger().info(
            f"Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}"
        )

        self.publish_odometry()
        self.publish_tf()

    def publish_laser_scan(self):
        """Publish fake laser scan data."""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "base_link"
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 5.235
        scan_msg.angle_increment = 1.047
        scan_msg.range_min = 0.1
        scan_msg.range_max = 4.0
        scan_msg.ranges = [random.random() * 10] * 6

        self.laser_publisher.publish(scan_msg)
        self.get_logger().info(f"Published laser scan: {self.laser_ranges}")

    def publish_odometry(self):
        """Publish odometry data."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        odom_msg.twist.twist.linear.x = self.v_linear
        odom_msg.twist.twist.angular.z = self.v_angular

        self.odom_publisher.publish(odom_msg)
        self.get_logger().info(
            f"Published odometry: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}"
        )

    def publish_tf(self):
        """Publish transform from odom to base_link."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        quat = self.euler_to_quaternion(0, 0, self.theta)
        t.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        self.tf_broadcaster.sendTransform(t)

        robot_msg = String()
        robot_msg.data = (
            self.get_parameter("robot_description").get_parameter_value().string_value
        )
        self.pub.publish(robot_msg)

        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ["lwheel", "rhweel"]
        joint_msg.position = [self.x, self.y]
        joint_msg.velocity = [self.v_linear, self.v_angular]
        joint_msg.position
        self.joint_publisher.publish(joint_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to a quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return (x, y, z, w)


def main(args=None):
    rclpy.init(args=args)
    node = WheeledRobotSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
