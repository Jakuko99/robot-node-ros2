import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
from time import sleep


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
        self.map_subscriptions: dict[str, MapSubscription] = {}

    def discover_robots(self):
        topics = self.get_topic_names_and_types()

        robot_namespaces = set()
        for topic, types in topics:
            if (
                topic.endswith("/map")
                and "nav_msgs/msg/OccupancyGrid" in types
                and "costmap" not in topic
            ):
                robot_name = topic.split("/")[1]
                robot_namespaces.add(robot_name)

        for robot_name in robot_namespaces:
            if robot_name not in self.map_subscriptions:
                topic_name = f"/{robot_name}/map"
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
            self.get_logger().info(f"Merging {len(local_maps)} maps.")
            merged_map: OccupancyGrid = OccupancyGrid()

            min_x = min(map.info.origin.position.x for map in local_maps)
            min_y = min(map.info.origin.position.y for map in local_maps)

            merged_map.info.resolution = local_maps[0].info.resolution
            
            # Calculate the maximum extent in x and y directions
            max_x = max(map.info.origin.position.x + map.info.width * map.info.resolution for map in local_maps)
            max_y = max(map.info.origin.position.y + map.info.height * map.info.resolution for map in local_maps)
            
            merged_map.info.width = int((max_x - min_x) / merged_map.info.resolution)
            merged_map.info.height = int((max_y - min_y) / merged_map.info.resolution)
            merged_map.info.origin.position.x = min_x
            merged_map.info.origin.position.y = min_y
            merged_map.info.origin.position.z = 0.0
            merged_map.info.origin.orientation.w = 1.0
            merged_map.header.frame_id = "global_map"
            merged_map.header.stamp = self.get_clock().now().to_msg()

            merged_data = np.full(
                (merged_map.info.height, merged_map.info.width), -1, dtype=np.int8
            )

            for local_map in local_maps:
                offset_x = int(
                    (local_map.info.origin.position.x - min_x)
                    / local_map.info.resolution
                ) // 2
                offset_y = int(
                    (local_map.info.origin.position.y - min_y)
                    / local_map.info.resolution
                ) // 2

                local_data: np.ndarray = np.array(
                    local_map.data, dtype=np.int8
                ).reshape((local_map.info.height, local_map.info.width))
                # local_data = np.pad(
                #     local_data,
                #     (
                #         (0, merged_map.info.height - local_map.info.height),
                #         (0, merged_map.info.width - local_map.info.width),
                #     ),
                #     mode="constant",
                #     constant_values=-1,
                # )

                print(f"Offset for map: ({offset_x}, {offset_y})")
                print(f"Local map size: ({local_data.shape[1]}, {local_data.shape[0]})")
                print(
                    f"Merged map size: ({merged_map.info.width}, {merged_map.info.height})"
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


# ziskat minima v x a y , nasledne vypocitat posunutie map a velkost vyslednej mapy a potom cez numpy offsetnut druhu mapu aby boli zarovnane


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
