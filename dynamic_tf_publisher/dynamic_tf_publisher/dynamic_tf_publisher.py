import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import numpy as np


class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__("dynamic_tf_publisher_node")
        self.static_publisher = self.create_publisher(TFMessage, "/tf_static", 10)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("child_frame_id", "dynamic_frame")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)

        self.publish_timer = self.create_timer(
            1.0 / self.get_parameter("publish_rate").get_parameter_value().double_value,
            self.publish_tf,
        )
        self.get_logger().info("Dynamic TF Publisher Node has been started.")

    def publish_tf(self):
        tf_msg = TFMessage()
        tranform = TransformStamped()
        tranform.header.stamp = self.get_clock().now().to_msg()
        tranform.header.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        tranform.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )
        tranform.transform.translation.x = (
            self.get_parameter("x").get_parameter_value().double_value
        )
        tranform.transform.translation.y = (
            self.get_parameter("y").get_parameter_value().double_value
        )
        tranform.transform.translation.z = (
            self.get_parameter("z").get_parameter_value().double_value
        )
        tranform.transform.rotation.x = 0.0
        tranform.transform.rotation.y = 0.0
        tranform.transform.rotation.z = 0.0
        tranform.transform.rotation.w = 1.0

        tf_msg.transforms.append(tranform)
        self.static_publisher.publish(tf_msg)
        self.get_logger().debug("Published dynamic TF.")


def main(args=None):
    rclpy.init(args=args)
    tf_publisher = DynamicTFPublisher()
    while rclpy.ok():
        rclpy.spin(tf_publisher)

    tf_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
