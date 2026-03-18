from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import math


class DecisionNetwork:
    def __init__(self, robot_name: str, pheromone_decay: float):
        self.robot_name = robot_name
        self.pheromone_decay = pheromone_decay
        self.feedback_layer = FeedbackLayer(self)

        self.current_map: OccupancyGrid = None
        self.current_odom: Odometry = None

    def update_state(self, map: OccupancyGrid, odom: Odometry):
        self.current_map = map
        self.current_odom = odom

        # self.feedback_layer.save_state(map, odom)

    def generate_goal(self) -> tuple[float, float]:
        if not self.current_map:
            return None

        clusters: list[tuple[int, int]] = self.find_clusters(
            self.current_map, value=110, cluster_size=5
        )
        if clusters:
            return self.map_index_to_pos(
                self.current_map, clusters[0]
            )  # Return position of the first cluster found

        return None

    @staticmethod
    def find_clusters(
        map: OccupancyGrid,
        value: int,
        cluster_size: int = 5,
    ) -> list[tuple[int, int]]:
        map_data: np.ndarray = np.array(map.data, dtype=np.int8).reshape(
            (map.info.height, map.info.width)
        )
        clusters: list[tuple[int, int]] = []
        for i in range(map.info.height):
            for j in range(map.info.width):
                if map_data[i, j] == value:
                    cluster_count = 0
                    for di in range(-cluster_size, cluster_size + 1):
                        for dj in range(-cluster_size, cluster_size + 1):
                            ni, nj = i + di, j + dj
                            if (
                                0 <= ni < map.info.height
                                and 0 <= nj < map.info.width
                                and map_data[ni, nj] == value
                            ):
                                cluster_count += 1
                    if cluster_count >= (2 * cluster_size + 1) ** 2 // 2:
                        clusters.append((i, j))
        return clusters

    @staticmethod
    def pos_to_map_index(
        map: OccupancyGrid,
        pos: tuple[float, float],
    ) -> tuple[int, int]:
        """
        Return index of cells, that correspond to current odometry position
        """
        if pos[0] < map.info.origin.position.x or pos[1] < map.info.origin.position.y:
            raise ValueError("Position is out of map bounds")

        m_res: float = map.info.resolution

        return (
            math.floor(abs((pos[0] - map.info.origin.position.x) / m_res)),
            math.floor(abs((pos[1] - map.info.origin.position.y) / m_res)),
        )

    @staticmethod
    def map_index_to_pos(
        map: OccupancyGrid,
        index: tuple[int, int],
    ) -> tuple[float, float]:
        """
        Return position of cell with given index
        """
        m_res: float = map.info.resolution
        return (  # TODO: problem here
            map.info.origin.position.x + (index[0] * m_res),
            map.info.origin.position.y + (index[1] * m_res),
        )


class FeedbackLayer:
    def __init__(self, network: DecisionNetwork):
        self.network: DecisionNetwork = network
        self.current_state: OccupancyGrid = None
        self.current_odom: Odometry = None

    def save_state(self, map: OccupancyGrid, odom: Odometry):
        self.current_state = map
        self.current_odom = odom

    def calculate_reward(
        self,
        new_map: OccupancyGrid,
        new_odom: Odometry,
        local_map: OccupancyGrid,
    ) -> float:
        total_reward: float = 0.0

        # Criterion 1: Ratio of explored to unexplored cells
        old_map_data: np.ndarray = np.array(self.current_state.data).reshape(
            self.current_state.info.height, self.current_state.info.width
        )
        old_ratio: float = np.sum((old_map_data >= 0) & (old_map_data < 100)) / old_map_data.size

        new_map_data: np.ndarray = np.array(new_map.data).reshape(
            new_map.info.height, new_map.info.width
        )
        new_ratio: float = np.sum((new_map_data >= 0) & (new_map_data < 100)) / new_map_data.size

        total_reward += (new_ratio - old_ratio) * 100.0  # Scale reward

        # Criterion 2: Traveled distance to information gain from exploration
        old_position: np.array = np.array(
            [self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y]
        )
        new_position: np.array = np.array(
            [new_odom.pose.pose.position.x, new_odom.pose.pose.position.y]
        )
        distance_traveled: float = np.linalg.norm(new_position - old_position)
        information_gain: float = (new_ratio - old_ratio) * 100.0

        total_reward += information_gain - (distance_traveled * 0.1)  # Penalize excessive movement

        # Criterion 3: Contribution to total coverage of the map per robot
        local_map_data: np.ndarray = np.array(local_map.data).reshape(
            local_map.info.height, local_map.info.width
        )
        local_explored: int = np.sum((local_map_data >= 0) & (local_map_data < 100))
        total_explored: int = np.sum((new_map_data >= 0) & (new_map_data < 100))

        total_reward += (local_explored / total_explored) * 20.0  # Scale reward

        return total_reward
