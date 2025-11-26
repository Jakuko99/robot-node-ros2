import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import numpy as np
from transforms3d._gohlketransforms import compose_matrix, euler_from_quaternion


class MapSubscription:
    def __init__(self, robot_name: str, node: Node, topic_name: str):
        self.robot_name: str = robot_name
        self.node: Node = node
        self.topic_name: str = topic_name

        self.subscription = node.create_subscription(
            OccupancyGrid, topic_name, self.map_callback, 10
        )
        self.update_subscription = node.create_subscription(
            OccupancyGridUpdate, topic_name + "_updates", self.update_map_callback, 10
        )
        self.map_data: OccupancyGrid = None
        self.map_position_x: float = 0.0
        self.map_position_y: float = 0.0

        self.map_width: int = 0
        self.map_height: int = 0
        self.map_resolution: float = 0.0

    def map_callback(self, msg: OccupancyGrid):
        self.map_data: OccupancyGrid = msg
        self.map_position_x = msg.info.origin.position.x
        self.map_position_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution

        self.node.merge_maps()

    def update_map_callback(self, msg):
        pass


class MapMerger(Node):
    def __init__(self):
        super().__init__("map_merger_node")
        self.publisher = self.create_publisher(OccupancyGrid, "/global_map", 10)
        self.static_subscription = self.create_subscription(
            TFMessage, "/tf_static", self.tf_callback, 10
        )
        self.static_transforms: dict[str, TFMessage] = dict()
        self.map_subscriptions: dict[str, MapSubscription] = {}

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            if transform.child_frame_id not in self.static_transforms:
                self.static_transforms[transform.child_frame_id] = transform
                print(f"Received new static transform for {transform.child_frame_id}")

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

                if robot_name not in self.map_subscriptions:
                    topic_name: str = f"/{robot_name}/map"
                    self.map_subscriptions[robot_name] = MapSubscription(
                        robot_name, self, topic_name
                    )
                    self.get_logger().info(
                        f"Subscribed to map topic for robot {robot_name} at {topic_name}"
                    )

    def merge_maps(self):
        local_maps = [
            sub.map_data
            for sub in self.map_subscriptions.values()
            if sub.map_data is not None
        ]

        if len(local_maps) == len(self.map_subscriptions) and len(local_maps) > 1:
            width, height = max([map.info.width for map in local_maps]), max(
                [map.info.height for map in local_maps]
            )

            merged_map: OccupancyGrid = OccupancyGrid()
            merged_map.info.resolution = local_maps[0].info.resolution
            merged_map.info.width = width
            merged_map.info.height = height
            merged_map.info.origin.position.x = min(
                [map.info.origin.position.x for map in local_maps]
            )
            merged_map.info.origin.position.y = min(
                [map.info.origin.position.y for map in local_maps]
            )
            merged_map.info.origin.position.z = 0.0
            merged_map.info.origin.orientation.w = 1.0
            merged_map.header.frame_id = "global_map"
            merged_map.header.stamp = self.get_clock().now().to_msg()

            merged_data = np.full(
                (merged_map.info.height, merged_map.info.width), -1, dtype=np.int8
            )

            for map in local_maps:
                transform: TransformStamped = self.static_transforms.get(
                    f"{map.header.frame_id}"
                )

                print(
                    f"transform for {map.header.frame_id}: {transform.transform.translation.x}, {transform.transform.translation.y}"
                )
                print(
                    f"map origin: {map.info.origin.position.x}, {map.info.origin.position.y}"
                )

                transform_to_origin_x = (
                    transform.transform.translation.x - map.info.origin.position.x
                )
                transform_to_origin_y = (
                    transform.transform.translation.y - map.info.origin.position.y
                )

                offset_x = int(
                    merged_map.info.origin.position.x
                    - transform_to_origin_x / merged_map.info.resolution
                )
                offset_y = int(
                    merged_map.info.origin.position.y
                    - transform_to_origin_y / merged_map.info.resolution
                )
                print(f"Offset for {map.header.frame_id}: x={offset_x}, y={offset_y}")

                merged_data[
                    offset_y : (offset_y + map.info.height),
                    offset_x : (offset_x + map.info.width),
                ] = np.array(map.data).reshape((map.info.height, map.info.width))


            # glob2loc1_tf: TransformStamped = self.static_transforms.get(
            #     "kris_robot1_map"
            # )
            # glob2loc2_tf: TransformStamped = self.static_transforms.get(
            #     "kris_robot2_map"
            # )

            # loc1_to_glob_mat = compose_matrix(
            #     translate=np.array(
            #         [
            #             -glob2loc1_tf.transform.translation.x,
            #             -glob2loc1_tf.transform.translation.y,
            #             -glob2loc1_tf.transform.translation.z,
            #         ]
            #     ),
            #     angles=euler_from_quaternion(
            #         [
            #             glob2loc1_tf.transform.rotation.x,
            #             glob2loc1_tf.transform.rotation.y,
            #             glob2loc1_tf.transform.rotation.z,
            #             glob2loc1_tf.transform.rotation.w,
            #         ]
            #     ),
            # )

            # loc2_to_glob_mat = compose_matrix(
            #     translate=np.array(
            #         [
            #             -glob2loc2_tf.transform.translation.x,
            #             -glob2loc2_tf.transform.translation.y,
            #             -glob2loc2_tf.transform.translation.z,
            #         ]
            #     ),
            #     angles=euler_from_quaternion(
            #         [
            #             glob2loc2_tf.transform.rotation.x,
            #             glob2loc2_tf.transform.rotation.y,
            #             glob2loc2_tf.transform.rotation.z,
            #             glob2loc2_tf.transform.rotation.w,
            #         ]
            #     ),
            # )

            # map1 = local_maps[0]
            # map2 = local_maps[1]

            # s1 = loc1_to_glob_mat @ np.array(
            #     [-map1.info.origin.position.x, -map1.info.origin.position.y, 0, 1]
            # )
            # s2 = loc2_to_glob_mat @ np.array(
            #     [-map2.info.origin.position.x, -map2.info.origin.position.y, 0, 1]
            # )

            # t1 = loc1_to_glob_mat @ np.array(
            #     [
            #         -map1.info.origin.position.x
            #         + map1.info.width * map1.info.resolution,
            #         -map1.info.origin.position.y
            #         + map1.info.height * map1.info.resolution,
            #         0,
            #         1,
            #     ]
            # )

            # t2 = loc2_to_glob_mat @ np.array(
            #     [
            #         -map2.info.origin.position.x
            #         + map2.info.width * map2.info.resolution,
            #         -map2.info.origin.position.y
            #         + map2.info.height * map2.info.resolution,
            #         0,
            #         1,
            #     ]
            # )

            # xmin = min(s1[0], s2[0], t1[0], t2[0])
            # xmax = max(s1[0], s2[0], t1[0], t2[0])
            # ymin = min(s1[1], s2[1], t1[1], t2[1])
            # ymax = max(s1[1], s2[1], t1[1], t2[1])

            # merged_map = OccupancyGrid()
            # min_x, min_y = xmin, ymin
            # max_x, max_y = xmax, ymax

            # merged_map.info.resolution = local_maps[0].info.resolution
            # merged_map.info.width = int((max_x - min_x) / merged_map.info.resolution)
            # merged_map.info.height = int((max_y - min_y) / merged_map.info.resolution)
            # merged_map.info.origin.position.x = min_x
            # merged_map.info.origin.position.y = min_y
            # merged_map.info.origin.position.z = 0.0
            # merged_map.info.origin.orientation.w = 1.0
            # merged_map.header.frame_id = "global_map"
            # merged_map.header.stamp = self.get_clock().now().to_msg()

            # print(
            #     f"Merged map size: {merged_map.info.width} x {merged_map.info.height}"
            # )
            # print(
            #     f"map1: {map1.info.width} x {map1.info.height}, map2: {map2.info.width} x {map2.info.height}"
            # )

            # merged_data = np.full(
            #     (merged_map.info.height, merged_map.info.width), -1, dtype=np.int8
            # )

            # offset_x1 = int((s1[0] - min_x) / merged_map.info.resolution)
            # offset_y1 = int((s1[1] - min_y) / merged_map.info.resolution)

            # print(f"Offset1: x={offset_x1}, y={offset_y1}")

            # merged_data[
            #     offset_y1 : (offset_y1 + map1.info.height),
            #     offset_x1 : (offset_x1 + map1.info.width),
            # ] = np.array(map1.data).reshape((map1.info.height, map1.info.width))

            # offset_x2 = int((s2[0] - min_x) / merged_map.info.resolution)
            # offset_y2 = int((s2[1] - min_y) / merged_map.info.resolution)
            # merged_data[
            #     offset_y2 : (offset_y2 + map2.info.height),
            #     offset_x2 : (offset_x2 + map2.info.width),
            # ] = np.array(map2.data).reshape((map2.info.height, map2.info.width))

            merged_map.data = merged_data.flatten().tolist()
            self.publisher.publish(merged_map)
            self.get_logger().info(
                f"Published merged map. ({merged_map.info.width} x {merged_map.info.height})"
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
