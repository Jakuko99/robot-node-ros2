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
        self.map_data: np.ndarray = None
        self.current_odom: Odometry = None
        self.static_transform: TransformStamped = None

    def update_local_map(self, new_map: OccupancyGrid):
        self.current_map = new_map

    def update_global_map(self, new_global_map: OccupancyGrid) -> OccupancyGrid:
        # if self.global_map is None:
        self.global_map = new_global_map
        self.map_data: np.ndarray = np.array(self.global_map.data, dtype=np.int8).reshape(
            (self.global_map.info.height, self.global_map.info.width)
        )

        # else: - not working well, needs more thought
        #     new_data: np.ndarray = np.array(new_global_map.data, dtype=np.int8).reshape(
        #         (new_global_map.info.height, new_global_map.info.width)
        #     )
        #     self.map_data: np.ndarray = np.array(self.map_data, dtype=np.int8).reshape(
        #         (new_global_map.info.height, new_global_map.info.width)
        #     )

        #     # Replace values in map_data with new_data where new_data is -1, 100, or 0
        #     mask = (new_data == -1) | (new_data == 100) | (new_data == 0)
        #     self.map_data[mask] = new_data[mask]

        if self.current_odom and self.static_transform and self.current_map:
            position: tuple[float, float] = self.pos_to_map_index(
                self.global_map,
                (
                    self.current_odom.pose.pose.position.y,
                    self.current_odom.pose.pose.position.x,
                ),
            )
            self.map_data = self._set_region(
                self.map_data, position[1], position[0], width=2, value=25
            )

            # for i in range(110): # gradient test
            #     self.map_data = self._set_region(self.map_data, 0, i, width=5, value=i)

            self.global_map.data = np.array(self.map_data).flatten().tolist()

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
        width: int,
        value: int,
    ) -> np.ndarray:
        """
        Set square region of given width in arr to value, starting at (x, y) as top-left corner.
        """
        arr[x : x + width, y : y + width] = value
        return arr

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

        m_res: float = map.info.resolution
        return (
            math.floor(
                (pos[0] + transform.transform.translation.x - map.info.origin.position.x) / m_res
            ),
            math.floor(
                (pos[1] + transform.transform.translation.y - map.info.origin.position.y) / m_res
            ),
        )
