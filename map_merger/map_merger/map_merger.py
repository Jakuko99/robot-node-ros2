import rclpy
import numpy as np
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from transforms3d._gohlketransforms import compose_matrix, euler_from_quaternion


class MapSubscription:
    def __init__(self, robot_name: str, topic_name: str, parent: "MapMerger"):
        self.robot_name: str = robot_name
        self.parent: "MapMerger" = parent
        self.topic_name: str = topic_name

        self.subscription = parent.create_subscription(
            OccupancyGrid, topic_name, self.map_callback, 10
        )
        self.update_subscription = parent.create_subscription(
            OccupancyGridUpdate, topic_name + "_updates", self.update_map_callback, 10
        )
        self.map_data: OccupancyGrid = None

    def map_callback(self, msg: OccupancyGrid):
        self.map_data: OccupancyGrid = msg
        self.parent.merge_maps()  # call parent node method to merge maps

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
                self.get_logger().info(
                    f"Received new static transform for {transform.child_frame_id}"
                )

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
                        robot_name=robot_name,
                        topic_name=topic_name,
                        parent=self,
                    )

                    self.get_logger().info(
                        f"Subscribed to map topic for robot {robot_name} at {topic_name}"
                    )

    def merge_maps(self):
        local_maps: dict[str, OccupancyGrid] = {
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

            min_x: float = min(
                [s[0] for s in s_dict.values()]
                + [t[0] for t in t_dict.values()]
                + [u[0] for u in u_dict.values()]
                + [v[0] for v in v_dict.values()]
            )
            max_x: float = max(
                [s[0] for s in s_dict.values()]
                + [t[0] for t in t_dict.values()]
                + [u[0] for u in u_dict.values()]
                + [v[0] for v in v_dict.values()]
            )
            min_y: float = min(
                [s[1] for s in s_dict.values()]
                + [t[1] for t in t_dict.values()]
                + [u[1] for u in u_dict.values()]
                + [v[1] for v in v_dict.values()]
            )
            max_y: float = max(
                [s[1] for s in s_dict.values()]
                + [t[1] for t in t_dict.values()]
                + [u[1] for u in u_dict.values()]
                + [v[1] for v in v_dict.values()]
            )

            merged_map: OccupancyGrid = OccupancyGrid()

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
            merged_map.header.frame_id = "global_map"
            merged_map.header.stamp = self.get_clock().now().to_msg()

            # print(
            #     f"Merged map size: {merged_map.info.height} x {merged_map.info.width}"
            # )

            self.get_logger().debug(
                f"Merged map origin: x={merged_map.info.origin.position.x}, y={merged_map.info.origin.position.y}"
            )

            merged_data = np.full(
                (merged_map.info.height, merged_map.info.width), -1, dtype=np.int8
            )

            for frame_id, map_data in local_maps.items():
                # print(
                #     f"frame: {frame_id} size: {map_data.info.height} x {map_data.info.width}"
                # )

                offset_x: int = int(
                    (s_dict[frame_id][0] - min_x) / merged_map.info.resolution
                )
                offset_y: int = int(
                    (s_dict[frame_id][1] - min_y) / merged_map.info.resolution
                )

                local_data: np.ndarray = np.array(map_data.data).reshape(
                    (map_data.info.height, map_data.info.width)
                )

                self.get_logger().debug(
                    f"Offset: x={offset_x}, y={offset_y} frame={frame_id}"
                )

                merged_data[
                    offset_y : local_data.shape[0] + offset_y,
                    offset_x : local_data.shape[1] + offset_x,
                ] = np.where(
                    local_data != -1,
                    local_data,
                    merged_data[
                        offset_y : local_data.shape[0] + offset_y,
                        offset_x : local_data.shape[1] + offset_x,
                    ],
                )

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
