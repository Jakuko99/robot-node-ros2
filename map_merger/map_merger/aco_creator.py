from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
import math


class ACOCreator:
    def __init__(self, robot_name: str, parent: object):
        self.robot_name = robot_name
        self.merger_node = parent

        self.current_map: OccupancyGrid = None
        self.global_map: OccupancyGrid = None
        self.current_odom: Odometry = None
        self.global_transform: TransformStamped = None

    def update_local_map(self, new_map: OccupancyGrid):
        self.current_map = new_map

    def update_global_map(self, new_global_map: OccupancyGrid) -> OccupancyGrid:
        self.global_map = new_global_map

        map_data: np.ndarray = np.array(self.global_map.data, dtype=np.int8).reshape(
            (self.global_map.info.height, self.global_map.info.width)
        )

        if self.current_odom and self.global_transform and self.current_map:
            position: tuple[float, float] = self.pos_to_map_index(
                self.global_map,
                (
                    self.current_odom.pose.pose.position.y,
                    self.current_odom.pose.pose.position.x,
                ),
            )

            map_data = self._set_region(map_data, *position, width=10, value=20)
            self.global_map.data = map_data.flatten().tolist()

        return self.global_map

    def update_odom(self, new_odom: Odometry):
        self.current_odom = new_odom

    def update_transform(self, new_transform: TransformStamped):
        self.global_transform = new_transform

    # ----- Helper methods -----
    @staticmethod
    def _set_region(
        arr: np.ndarray,
        x: int,
        y: int,
        width: int,
        value: int,
    ) -> np.ndarray:
        """
        Set square region of given width in arr to value, starting at (x, y) as top-left corner.
        """
        arr[x : x + width, y : y + width] = value
        return arr

    @staticmethod
    def pos_to_map_index(
        map: OccupancyGrid, pos: tuple[float, float]
    ) -> tuple[int, int]:
        """
        Return index of cells, that correspond to current odometry position
        """
        m_res: float = map.info.resolution
        return (
            math.floor((pos[0] - map.info.origin.position.x) / m_res),
            math.floor((pos[1] - map.info.origin.position.y) / m_res),
        )
