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
from transforms3d._gohlketransforms import compose_matrix, euler_from_quaternion

from map_merger.aco_creator import ACOCreator

map_qos: QoSProfile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)  # map QoS profile to ensure we get the latest map


class MapSubscription:
    def __init__(
        self, robot_name: str, node: "MapMerger", topic_name: str, primary: bool = False
    ):
        self.robot_name: str = robot_name
        self.node: "MapMerger" = node
        self.topic_name: str = topic_name
        self.primary: bool = primary

        self.subscription: Subscription[OccupancyGrid] = node.create_subscription(
            OccupancyGrid, topic_name, self.map_callback, map_qos
        )
        self.update_subscription: Subscription[OccupancyGridUpdate] = (
            node.create_subscription(
                OccupancyGridUpdate,
                topic_name + "_updates",
                self.update_map_callback,
                map_qos,
            )
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
        self.robot_name: str = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )
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

        self.static_transforms: dict[str, TFMessage] = dict()
        self.map_subscriptions: dict[str, MapSubscription] = {}
        self.aco_creator: ACOCreator = ACOCreator(self.robot_name, self)
        self.map_subscriptions[self.robot_name] = MapSubscription(
            self.robot_name, self, f"/{self.robot_name}/map", primary=True
        )

        self.merge_timer: Timer = self.create_timer(5.0, callback=self.merge_maps)

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

                if (
                    robot_name not in self.map_subscriptions
                    and not robot_name == self.robot_name
                ):
                    topic_name: str = f"/{robot_name}/map"
                    self.map_subscriptions[robot_name] = MapSubscription(
                        robot_name, self, topic_name
                    )
                    self.get_logger().info(
                        f"Subscribed to map topic for robot {robot_name} at {topic_name}"
                    )

    def merge_maps(self):
        local_maps = {
            sub.map_data.header.frame_id: sub.map_data
            for sub in self.map_subscriptions.values()
            if sub.map_data is not None
        }

        if len(local_maps) == len(self.map_subscriptions) and len(local_maps) > 1:
            loc_to_glob_mat: dict[str, np.ndarray] = {
                frame_id: compose_matrix(
                    translate=np.array(
                        [
                            -tf.transform.translation.x,
                            -tf.transform.translation.y,
                            -tf.transform.translation.z,
                        ]
                    ),
                    angles=euler_from_quaternion(
                        [
                            tf.transform.rotation.x,
                            tf.transform.rotation.y,
                            tf.transform.rotation.z,
                            tf.transform.rotation.w,
                        ]
                    ),
                )
                for frame_id, tf in self.static_transforms.items()
            }

            s_dict: dict[str, np.ndarray] = {
                frame_id: loc_to_glob_mat[frame_id]
                @ np.array(
                    [
                        map_data.info.origin.position.x,
                        map_data.info.origin.position.y,
                        0,
                        1,
                    ]
                )
                for frame_id, map_data in local_maps.items()
            }

            t_dict: dict[str, np.ndarray] = {
                frame_id: loc_to_glob_mat[frame_id]
                @ np.array(
                    [
                        map_data.info.origin.position.x
                        - map_data.info.width * map_data.info.resolution,
                        map_data.info.origin.position.y
                        - map_data.info.height * map_data.info.resolution,
                        0,
                        1,
                    ]
                )
                for frame_id, map_data in local_maps.items()
            }

            u_dict: dict[str, np.ndarray] = {
                frame_id: loc_to_glob_mat[frame_id]
                @ np.array(
                    [
                        map_data.info.origin.position.x,
                        map_data.info.origin.position.y
                        - map_data.info.height * map_data.info.resolution,
                        0,
                        1,
                    ]
                )
                for frame_id, map_data in local_maps.items()
            }

            v_dict: dict[str, np.ndarray] = {
                frame_id: loc_to_glob_mat[frame_id]
                @ np.array(
                    [
                        map_data.info.origin.position.x
                        - map_data.info.width * map_data.info.resolution,
                        map_data.info.origin.position.y,
                        0,
                        1,
                    ]
                )
                for frame_id, map_data in local_maps.items()
            }

            min_x = min(
                [s[0] for s in s_dict.values()]
                + [t[0] for t in t_dict.values()]
                + [u[0] for u in u_dict.values()]
                + [v[0] for v in v_dict.values()]
            )
            max_x = max(
                [s[0] for s in s_dict.values()]
                + [t[0] for t in t_dict.values()]
                + [u[0] for u in u_dict.values()]
                + [v[0] for v in v_dict.values()]
            )
            min_y = min(
                [s[1] for s in s_dict.values()]
                + [t[1] for t in t_dict.values()]
                + [u[1] for u in u_dict.values()]
                + [v[1] for v in v_dict.values()]
            )
            max_y = max(
                [s[1] for s in s_dict.values()]
                + [t[1] for t in t_dict.values()]
                + [u[1] for u in u_dict.values()]
                + [v[1] for v in v_dict.values()]
            )

            merged_map = OccupancyGrid()

            merged_map.info.resolution = local_maps[
                list(local_maps.keys())[0]
            ].info.resolution
            merged_map.info.width = int((max_x - min_x) / merged_map.info.resolution)
            merged_map.info.height = int((max_y - min_y) / merged_map.info.resolution)
            merged_map.info.origin.position.x = min(
                [m.info.origin.position.x for m in local_maps.values()]
            ) + max(
                [tf.transform.translation.x for tf in self.static_transforms.values()]
            )
            merged_map.info.origin.position.y = min(
                [m.info.origin.position.y for m in local_maps.values()]
            ) + max(
                [tf.transform.translation.y for tf in self.static_transforms.values()]
            )
            merged_map.info.origin.position.z = 0.0
            merged_map.info.origin.orientation.w = 1.0
            merged_map.header.frame_id = self.map_frame_id
            merged_map.header.stamp = self.get_clock().now().to_msg()

            print(
                f"Merged map size: {merged_map.info.height} x {merged_map.info.width}"
            )
            self.get_logger().warning(
                f"Merged map origin: x={merged_map.info.origin.position.x}, y={merged_map.info.origin.position.y}"
            )

            merged_data = np.full(
                (merged_map.info.height, merged_map.info.width), -1, dtype=np.int8
            )

            known_count = np.zeros(
                (merged_map.info.height, merged_map.info.width), dtype=np.int16
            )
            conflict_count = np.zeros_like(known_count)

            for frame_id, map_data in local_maps.items():
                print(
                    f"frame: {frame_id} size: {map_data.info.height} x {map_data.info.width}"
                )

                offset_x = int(
                    (s_dict[frame_id][0] - min_x) / merged_map.info.resolution
                )
                offset_y = int(
                    (s_dict[frame_id][1] - min_y) / merged_map.info.resolution
                )

                local_data = (
                    np.array(map_data.data)
                    .reshape((map_data.info.height, map_data.info.width))
                    .astype(np.int16)
                )

                print(f"Offset: x={offset_x}, y={offset_y}")

                ys = slice(offset_y, offset_y + local_data.shape[0])
                xs = slice(offset_x, offset_x + local_data.shape[1])

                local_known = local_data != -1
                merged_known = merged_data[ys, xs] != -1

                # Count overlaps
                overlap = local_known & merged_known
                conflict = overlap & (local_data != merged_data[ys, xs])

                conflict_count[ys, xs][conflict] += 1
                known_count[ys, xs][local_known] += 1

                # Where only local has data, write it directly.
                # Where both have data:
                #   - obstacles keep 100
                #   - free cells accumulate pheromone (treat 0 as 1 unit so
                #     repeated exploration yields higher values for ACO)
                patch = merged_data[ys, xs].astype(np.int16)
                only_local = local_known & ~merged_known
                both_known = local_known & merged_known
                both_obstacle = both_known & ((local_data == 100) | (patch == 100))
                both_free = both_known & ~both_obstacle

                patch[only_local] = local_data[only_local]
                patch[both_obstacle] = 100
                pheromone_existing = np.where(patch == 0, 1, patch)
                pheromone_new = np.where(local_data == 0, 1, local_data)
                patch[both_free] = np.clip(
                    pheromone_existing[both_free] + pheromone_new[both_free] + 10, 0, 90
                )
                merged_data[ys, xs] = patch.astype(np.int8)

            merged_map.data = merged_data.flatten().tolist()

            pheromone_map: OccupancyGrid = self.aco_creator.update_global_map(
                merged_map
            )
            self.publisher.publish(pheromone_map)

            self.get_logger().info(
                f"Published merged map. "
                f"({merged_map.info.width} x {merged_map.info.height}) | "
            )

            for sub in self.map_subscriptions.values():
                sub.map_data = None  # reset maps after merging

        self.discover_robots()


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
