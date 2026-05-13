import rclpy
from rclpy.node import Node, Subscription, Publisher, Timer
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import numpy as np
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
import cv2
import time

from map_merger.aco_creator import ACOCreator
from sim_srvs.srv import SimulationOutput, ExplorationStatus

map_qos: QoSProfile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)  # map QoS profile to ensure we get the latest map


class MapSubscription:
    def __init__(self, robot_name: str, node: "MapMerger", topic_name: str, primary: bool = False):
        self.robot_name: str = robot_name
        self.node: "MapMerger" = node
        self.topic_name: str = topic_name
        self.primary: bool = primary

        self.subscription: Subscription[OccupancyGrid] = node.create_subscription(
            OccupancyGrid, topic_name, self.map_callback, map_qos
        )
        self.update_subscription: Subscription[OccupancyGridUpdate] = node.create_subscription(
            OccupancyGridUpdate,
            topic_name + "_updates",
            self.update_map_callback,
            map_qos,
        )
        self.odom_sub: Subscription[Odometry] = node.create_subscription(
            Odometry, topic_name.replace("/map", "/odom"), self.odom_callback, 10
        )

        self.map_data: OccupancyGrid = None
        self.map_position_x: float = 0.0
        self.map_position_y: float = 0.0

        self.map_width: int = 0
        self.map_height: int = 0
        self.map_resolution: float = 0.0

        self.robot_position_x: float = 0.0
        self.robot_position_y: float = 0.0
        self.robot_orientation: float = 0.0

    def map_callback(self, msg: OccupancyGrid):
        self.map_data: OccupancyGrid = msg
        self.map_position_x = msg.info.origin.position.x
        self.map_position_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution

        if self.primary:
            self.node.aco_creator.update_local_map(msg)

    def update_map_callback(self, msg: OccupancyGridUpdate):
        pass

    def odom_callback(self, msg: Odometry):
        self.robot_position_x = msg.pose.pose.position.x
        self.robot_position_y = msg.pose.pose.position.y
        self.robot_orientation = msg.pose.pose.orientation.z

        if self.primary:
            self.node.aco_creator.update_odom(msg)


class MapMerger(Node):
    def __init__(self):
        super().__init__("map_merger_node")
        self.declare_parameter("robot_name", "robot")
        self.robot_name: str = self.get_parameter("robot_name").get_parameter_value().string_value
        self.declare_parameter("merge_topic_name", "global_map")
        self.merge_topic_name: str = (
            f"/{self.robot_name}/{self.get_parameter('merge_topic_name').get_parameter_value().string_value}"
        )
        self.declare_parameter("map_frame_id", "map")
        self.map_frame_id: str = (
            self.get_parameter("map_frame_id").get_parameter_value().string_value
        )

        self.publisher: Publisher[OccupancyGrid] = self.create_publisher(
            OccupancyGrid, self.merge_topic_name, map_qos
        )
        self.static_subscription: Subscription[TFMessage] = self.create_subscription(
            TFMessage, "/tf_static", self.tf_callback, 10
        )

        self.create_service(
            SimulationOutput,
            f"/{self.robot_name}/export_map",
            self.save_map_callback,
        )
        self.create_service(
            ExplorationStatus,
            f"/{self.robot_name}/exploration_progress",
            self.exploration_status_callback,
        )

        self.static_transforms: dict[str, TFMessage] = dict()
        self.map_subscriptions: dict[str, MapSubscription] = {}
        self.aco_creator: ACOCreator = ACOCreator(self.robot_name, self)
        self.last_merge_confidence: float = 0.0
        self.last_merge_conflicted_ratio: float = 0.0
        self.last_merge_observed_ratio: float = 0.0
        self.map_subscriptions[self.robot_name] = MapSubscription(
            self.robot_name, self, f"/{self.robot_name}/map", primary=True
        )

        self.merge_timer: Timer = self.create_timer(5.0, callback=self.merge_maps_v2)

    def save_map_callback(
        self, request: SimulationOutput.Request, response: SimulationOutput.Response
    ) -> SimulationOutput.Response:
        if self.aco_creator.global_map:
            try:
                width = self.aco_creator.global_map.info.width
                height = self.aco_creator.global_map.info.height

                map_array = np.array(self.aco_creator.global_map.data, dtype=np.int8).reshape(
                    (height, width)
                )
                map_image = np.zeros((height, width, 3), dtype=np.uint8)
                map_image[map_array == 0] = [255, 255, 255]
                map_image[map_array == 100] = [0, 0, 0]
                map_image[map_array == -1] = [127, 127, 127]
                map_image[(map_array >= 10) & (map_array < 100)] = [255, 0, 0]
                map_image[map_array == -10] = [0, 0, 255]
                map_image[map_array == 110] = [0, 255, 0]
                cv2.imwrite(
                    f"export/{self.robot_name}_map-{request.id if request.id else int(time.time())}.png",
                    map_image,
                )
                response.success = True
                response.message = f"Map saved successfully as {self.robot_name}_map.png"
                self.get_logger().info(response.message)

            except Exception as e:
                response.success = False
                response.message = f"Failed to save map: {e}"
                self.get_logger().error(f"Error saving map: {e}")

        else:
            response.success = False
            response.message = "No map data available to save."

        return response

    def exploration_status_callback(
        self, request: ExplorationStatus.Request, response: ExplorationStatus.Response
    ) -> ExplorationStatus.Response:
        if self.aco_creator.global_map:
            explored_cells: int = sum(
                1 for cell in self.aco_creator.global_map.data if (cell >= 0 and cell <= 100)
            )
            overplapped_cells: int = sum(
                1 for cell in self.aco_creator.global_map.data if cell >= 10 and cell < 100
            )
            total_cells: int = len(self.aco_creator.global_map.data)

            response.success = True
            response.explore_ratio = (explored_cells / total_cells) if total_cells > 0 else 0
            response.map_height = self.aco_creator.global_map.info.height
            response.map_width = self.aco_creator.global_map.info.width
            response.overlap_ratio = (
                (overplapped_cells / explored_cells) if explored_cells > 0 else 0
            )
            response.message = f"Exploration ratio: {response.explore_ratio:.2f}, Overlap ratio: {response.overlap_ratio:.2f}"

        else:
            response.success = False
            self.get_logger().warn("Cannot determine exploration status: No map data available.")

        return response

    def tf_callback(self, msg: TFMessage):
        transform: TransformStamped  # type hint for transform in msg.transforms
        for transform in msg.transforms:
            if transform.child_frame_id not in self.static_transforms:
                self.static_transforms[transform.child_frame_id] = transform
                self.get_logger().info(
                    f"Received new static transform for {transform.child_frame_id}"
                )
            if transform.child_frame_id.replace("_map", "") == self.robot_name:
                self.aco_creator.update_transform(transform)

        self.discover_robots()

    def discover_robots(self):
        topics = self.get_topic_names_and_types()

        for topic, types in topics:
            if (
                topic.endswith("/map")
                and "nav_msgs/msg/OccupancyGrid" in types
                and "costmap" not in topic
            ):
                robot_name: str = topic.split("/")[1]

                if robot_name not in self.map_subscriptions and not robot_name == self.robot_name:
                    topic_name: str = f"/{robot_name}/map"
                    self.map_subscriptions[robot_name] = MapSubscription(
                        robot_name, self, topic_name
                    )
                    self.get_logger().info(
                        f"Subscribed to map topic for robot {robot_name} at {topic_name}"
                    )

    def merge_maps_v1(self):
        local_maps: dict[str, OccupancyGrid] = {
            sub.map_data.header.frame_id: sub.map_data
            for sub in self.map_subscriptions.values()
            if sub.map_data is not None
        }

        if len(local_maps) == len(self.map_subscriptions) and len(local_maps) > 1:
            min_x: float = min([m.info.origin.position.x for m in local_maps.values()])
            min_y: float = min([m.info.origin.position.y for m in local_maps.values()])
            max_x: float = max(
                [
                    m.info.origin.position.x + m.info.width * m.info.resolution
                    for m in local_maps.values()
                ]
            )
            max_y: float = max(
                [
                    m.info.origin.position.y + m.info.height * m.info.resolution
                    for m in local_maps.values()
                ]
            )

            merged_map: OccupancyGrid = OccupancyGrid()
            merged_map.header.frame_id = self.map_frame_id
            merged_map.header.stamp = self.get_clock().now().to_msg()
            merged_map.info.origin.position.x = min_x
            merged_map.info.origin.position.y = min_y
            merged_map.info.resolution = min([m.info.resolution for m in local_maps.values()])
            merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution)) + 10
            merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution)) + 10

            merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)

            for map in local_maps.values():
                static_tf: TransformStamped = self.get_static_transform(map.header.frame_id)
                for y in range(map.info.height):
                    for x in range(map.info.width):
                        index: int = x + y * map.info.width
                        merged_x: int = int(
                            np.floor(
                                (
                                    map.info.origin.position.x
                                    + static_tf.transform.translation.x
                                    + x * map.info.resolution
                                    - min_x
                                )
                                / merged_map.info.resolution
                            )
                        )
                        merged_y: int = int(
                            np.floor(
                                (
                                    map.info.origin.position.y
                                    + static_tf.transform.translation.y
                                    + y * map.info.resolution
                                    - min_y
                                )
                                / merged_map.info.resolution
                            )
                        )
                        try:
                            merged_i: int = merged_x + merged_y * merged_map.info.width
                            if (
                                map.data[index] == 0
                                and merged_map.data[merged_i] >= 0
                                and merged_map.data[merged_i] < 89  # cap value to 90
                            ):
                                merged_map.data[merged_i] += 10

                            elif map.data[index] == 0:
                                merged_map.data[merged_i] = 0

                            elif map.data[index] != -1:
                                merged_map.data[merged_i] = map.data[index]

                        except IndexError:
                            self.get_logger().warn(
                                f"Index error for merged map at ({merged_x}, {merged_y}) with map {map.header.frame_id} at ({x}, {y})"
                            )
                            continue

            updated_map: OccupancyGrid = self.aco_creator.update_global_map(merged_map)
            self.publisher.publish(updated_map)
            self.get_logger().info(
                f"Published merged map with frame_id {updated_map.header.frame_id}"
            )

        self.discover_robots()

    def merge_maps_v2(self):
        local_maps: dict[str, OccupancyGrid] = {
            sub.map_data.header.frame_id: sub.map_data
            for sub in self.map_subscriptions.values()
            if sub.map_data is not None
        }

        if len(local_maps) == len(self.map_subscriptions) and len(local_maps) > 1:
            min_x: float = min([m.info.origin.position.x for m in local_maps.values()])
            min_y: float = min([m.info.origin.position.y for m in local_maps.values()])
            max_x: float = max(
                [
                    m.info.origin.position.x + m.info.width * m.info.resolution
                    for m in local_maps.values()
                ]
            )
            max_y: float = max(
                [
                    m.info.origin.position.y + m.info.height * m.info.resolution
                    for m in local_maps.values()
                ]
            )

            merged_map: OccupancyGrid = OccupancyGrid()
            merged_map.header.frame_id = self.map_frame_id
            merged_map.header.stamp = self.get_clock().now().to_msg()
            merged_map.info.origin.position.x = min_x
            merged_map.info.origin.position.y = min_y
            merged_map.info.resolution = min([m.info.resolution for m in local_maps.values()])
            merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution)) + 10
            merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution)) + 10

            map_size: int = merged_map.info.width * merged_map.info.height
            merged_map.data = [-1] * map_size

            free_votes = np.zeros(map_size, dtype=np.int16)
            occupied_votes = np.zeros(map_size, dtype=np.int16)
            occupied_sum = np.zeros(map_size, dtype=np.int32)

            for map in local_maps.values():
                static_tf: TransformStamped = self.get_static_transform(map.header.frame_id)
                for y in range(map.info.height):
                    for x in range(map.info.width):
                        index: int = x + y * map.info.width
                        merged_x: int = int(
                            np.floor(
                                (
                                    map.info.origin.position.x
                                    + static_tf.transform.translation.x
                                    + x * map.info.resolution
                                    - min_x
                                )
                                / merged_map.info.resolution
                            )
                        )
                        merged_y: int = int(
                            np.floor(
                                (
                                    map.info.origin.position.y
                                    + static_tf.transform.translation.y
                                    + y * map.info.resolution
                                    - min_y
                                )
                                / merged_map.info.resolution
                            )
                        )
                        try:
                            merged_i: int = merged_x + merged_y * merged_map.info.width
                            if map.data[index] == -1:
                                continue

                            if map.data[index] == 0:
                                free_votes[merged_i] += 1
                            else:
                                occupied_votes[merged_i] += 1
                                occupied_sum[merged_i] += int(map.data[index])

                        except IndexError:
                            self.get_logger().warn(
                                f"Index error for merged map at ({merged_x}, {merged_y}) with map {map.header.frame_id} at ({x}, {y})"
                            )
                            continue

            total_votes = free_votes + occupied_votes
            observed_mask = total_votes > 0
            observed_cells = int(np.count_nonzero(observed_mask))

            if observed_cells > 0:
                conflicted_mask = np.logical_and(free_votes > 0, occupied_votes > 0)
                conflicted_cells = int(np.count_nonzero(conflicted_mask))

                dominant_votes = np.maximum(free_votes, occupied_votes)
                self.last_merge_confidence = float(
                    np.sum(dominant_votes[observed_mask] / total_votes[observed_mask])
                    / observed_cells
                )
                self.last_merge_conflicted_ratio = float(conflicted_cells / observed_cells)
                self.last_merge_observed_ratio = float(observed_cells / map_size)

                for i in np.where(observed_mask)[0]:
                    free_count = int(free_votes[i])
                    occupied_count = int(occupied_votes[i])

                    if occupied_count == 0:
                        # Keep legacy overlap visualization for free-space-only observations.
                        merged_map.data[i] = min(89, max(0, (free_count - 1) * 10))
                        continue

                    if free_count == 0:
                        merged_map.data[i] = int(round(occupied_sum[i] / occupied_count))
                        continue

                    total_count = free_count + occupied_count
                    occ_probability = occupied_count / total_count
                    confidence = max(free_count, occupied_count) / total_count
                    uncertainty = 1.0 - confidence

                    # Conservative safety bias: ambiguous cells are nudged toward occupied.
                    uncertainty_bias = int(round(15.0 * uncertainty))
                    probabilistic_value = int(round(occ_probability * 100.0)) + uncertainty_bias
                    merged_map.data[i] = min(99, max(1, probabilistic_value))
            else:
                self.last_merge_confidence = 0.0
                self.last_merge_conflicted_ratio = 0.0
                self.last_merge_observed_ratio = 0.0

            updated_map: OccupancyGrid = self.aco_creator.update_global_map(merged_map)
            self.publisher.publish(updated_map)
            self.get_logger().info(
                "Published merged map with frame_id "
                f"{updated_map.header.frame_id}; "
                f"merge_confidence={self.last_merge_confidence:.3f}, "
                f"conflicted_ratio={self.last_merge_conflicted_ratio:.3f}, "
                f"observed_ratio={self.last_merge_observed_ratio:.3f}"
            )

        self.discover_robots()

    def get_static_transform(self, frame_id: str) -> TransformStamped:
        return self.static_transforms.get(frame_id, TransformStamped())


def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MapMerger()
    while rclpy.ok():
        merge_map_node.discover_robots()
        rclpy.spin(merge_map_node)

    merge_map_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
