from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.impl.rcutils_logger import RcutilsLogger
import numpy as np
import math


class ACOCreator:
    def __init__(self, robot_name: str, parent: object):
        self.robot_name = robot_name
        self.parent = parent
        self.logger: RcutilsLogger = self.parent.get_logger()

        self.current_map: OccupancyGrid = None
        self.global_map: OccupancyGrid = None
        self.current_odom: Odometry = None
        self.static_transform: TransformStamped = None

    def update_local_map(self, new_map: OccupancyGrid):
        self.current_map = new_map

    def update_global_map(self, new_global_map: OccupancyGrid) -> OccupancyGrid:
        self.global_map = (
            self.merge_maps(self.global_map, new_global_map) if self.global_map else new_global_map
        )
        self.global_map = self.find_frontiers(self.global_map)
        self.global_map = self.find_lower_trust_points(self.global_map)

        map_data: np.ndarray = np.array(self.global_map.data, dtype=np.int8).reshape(
            (self.global_map.info.height, self.global_map.info.width)
        )

        if self.current_odom and self.static_transform and self.current_map:
            try:
                position: tuple[float, float] = self.pos_to_map_index(
                    self.global_map,
                    (
                        self.current_odom.pose.pose.position.x,
                        self.current_odom.pose.pose.position.y,
                    ),
                )
                map_data = self._set_region(map_data, *position[::-1], width=3, value=25)

            except ValueError:
                self.logger.warn(
                    "Current position is out of global map bounds. Skipping position marking."
                )

            self.global_map.data = np.array(map_data).flatten().tolist()

        return self.global_map

    def update_odom(self, new_odom: Odometry):
        self.current_odom = new_odom

    def update_transform(self, new_transform: TransformStamped):
        self.static_transform = new_transform

    # ----- Helper methods -----
    @staticmethod
    def _set_region(
        arr: np.ndarray,
        x: int,
        y: int,
        value: int,
        width: int = 5,
    ) -> np.ndarray:
        """
        Set square region of given width in arr to value, starting at (x, y) as top-left corner.
        """
        arr[x : x + width, y : y + width] = value
        return arr

    @staticmethod
    def _get_region(
        arr: np.ndarray,
        x: int,
        y: int,
        width: int = 5,
    ) -> np.ndarray:
        """
        Get square region of given width in arr, starting at (x, y) as top-left corner.
        """
        return arr[x : x + width, y : y + width]

    def pos_to_map_index(
        self,
        map: OccupancyGrid,
        pos: tuple[float, float],
        transform: TransformStamped = None,
    ) -> tuple[int, int]:
        """
        Return index of cells, that correspond to current odometry position
        """
        if transform is None:
            transform = self.static_transform

        if pos[0] < map.info.origin.position.x or pos[1] < map.info.origin.position.y:
            raise ValueError("Position is out of map bounds")

        m_res: float = map.info.resolution
        return (
            math.floor(
                abs(
                    (pos[0] + transform.transform.translation.x - map.info.origin.position.x)
                    / m_res
                )
            ),
            math.floor(
                abs(
                    (pos[1] + transform.transform.translation.y - map.info.origin.position.y)
                    / m_res
                )
            ),
        )

    @staticmethod
    def merge_maps(map1: OccupancyGrid, map2: OccupancyGrid) -> OccupancyGrid:
        maps: list[OccupancyGrid] = [map1, map2]

        min_x: float = min([m.info.origin.position.x for m in maps])
        min_y: float = min([m.info.origin.position.y for m in maps])
        max_x: float = max(
            [m.info.origin.position.x + m.info.width * m.info.resolution for m in maps]
        )
        max_y: float = max(
            [m.info.origin.position.y + m.info.height * m.info.resolution for m in maps]
        )

        merged_map: OccupancyGrid = OccupancyGrid()
        merged_map.header.frame_id = maps[0].header.frame_id
        merged_map.header.stamp = maps[0].header.stamp
        merged_map.info.origin.position.x = min_x
        merged_map.info.origin.position.y = min_y
        merged_map.info.resolution = min([m.info.resolution for m in maps])
        merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
        merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))

        merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)

        for map in maps:
            for y in range(map.info.height):
                for x in range(map.info.width):
                    try:
                        index: int = x + y * map.info.width
                        merged_x: int = int(
                            np.floor(
                                (map.info.origin.position.x + x * map.info.resolution - min_x)
                                / merged_map.info.resolution
                            )
                        )
                        merged_y: int = int(
                            np.floor(
                                (map.info.origin.position.y + y * map.info.resolution - min_y)
                                / merged_map.info.resolution
                            )
                        )
                        merged_i: int = merged_x + merged_y * merged_map.info.width

                        if merged_map.data[merged_i] > 0 and merged_map.data[merged_i] < 100:
                            continue

                        elif map.data[index] != -1:
                            merged_map.data[merged_i] = map.data[index]

                    except IndexError:
                        return map1

        return merged_map

    @staticmethod
    def find_frontiers(map: OccupancyGrid) -> OccupancyGrid:
        """
        Find frontiers on the map and mark their locations with a specific value (e.g., 110) in the map data.
        """
        map_data: np.ndarray = np.array(map.data, dtype=np.int8).reshape(
            (map.info.height, map.info.width)
        )

        for y in range(map.info.height):
            for x in range(map.info.width):
                if map_data[y, x] == 0:  # Free cell
                    # Check neighbors
                    neighbors = [(y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)]
                    for ny, nx in neighbors:
                        if 0 <= ny < map.info.height and 0 <= nx < map.info.width:
                            if map_data[ny, nx] == -1:  # Unknown neighbor
                                index: int = x + y * map.info.width
                                map.data[index] = 110  # Mark frontier cell

        return map

    @staticmethod
    def find_lower_trust_points(map: OccupancyGrid) -> OccupancyGrid:
        """
        Find lower trust points on the map and mark their locations with a specific value (e.g., -10) in the map data.
        Lower trust points are random occupied cells that are next to free cells.
        """
        map_data: np.ndarray = np.array(map.data, dtype=np.int8).reshape(
            (map.info.height, map.info.width)
        )

        for y in range(map.info.height):
            for x in range(map.info.width):
                if map_data[y, x] == 100:  # occupied cell
                    # Check neighbors
                    neighbors = [
                        (y - 1, x),
                        (y + 1, x),
                        (y, x - 1),
                        (y, x + 1),
                        (y - 1, x - 1),
                        (y - 1, x + 1),
                        (y + 1, x - 1),
                        (y + 1, x + 1),
                    ]
                    empty_neighbors = 0
                    for ny, nx in neighbors:
                        if 0 <= ny < map.info.height and 0 <= nx < map.info.width:
                            if map_data[ny, nx] == 0:  # Free neighbor
                                empty_neighbors += 1

                    if empty_neighbors >= 4:
                        index: int = x + y * map.info.width
                        map.data[index] = -10  # Mark lower trust point

        return map
