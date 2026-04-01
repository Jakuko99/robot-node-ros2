from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import torch.nn as nn
from torch.optim import Adam
import torch
from random import random
import math


EXPLOITATION_RATIO: float = 0.6  # Probability of choosing the best action vs exploring
LEARNING_RATE: float = 0.02
REWARD_EPSILON: float = 1e-6


class DecisionNetwork:
    def __init__(self, robot_name: str, pheromone_decay: float):
        self.robot_name = robot_name
        self.pheromone_decay = pheromone_decay
        self.feedback_layer = FeedbackLayer(self)

        self.current_map: OccupancyGrid = None
        self.input_map: np.ndarray = None
        self.current_odom: Odometry = None

        self.construct_nn(hidden_sizes=[32], obs_dim=5, n_acts=8)

    def update_state(self, map: OccupancyGrid, odom: Odometry):
        self.current_map = map
        self.input_map = self.transform_map(map)
        self.current_odom = odom

        if random() > EXPLOITATION_RATIO:
            self.feedback_layer.save_state(map, odom)

    def generate_goal(self) -> tuple[float, float]:
        if not self.current_map or not self.current_odom:
            return None

    def construct_nn(self, hidden_sizes: list[int], obs_dim: int, n_acts: int):
        self.logits_net = self.mlp(sizes=[obs_dim] + hidden_sizes + [n_acts])
        self.optimizer = Adam(self.logits_net.parameters(), lr=LEARNING_RATE)

    def get_policy(self, obs: np.ndarray) -> torch.Tensor:
        obs_tensor = torch.from_numpy(obs.astype(np.float32).reshape(-1)).float()
        if obs_tensor.numel() != 5:
            raise ValueError(f"Expected observation with 5 features, got {obs_tensor.numel()}")
        logits = self.logits_net(obs_tensor)
        return torch.softmax(logits, dim=0)

    def compute_loss(self, obs: np.ndarray, action: int, reward: float) -> torch.Tensor:
        policy = self.get_policy(obs)
        log_prob = torch.log(policy[action].clamp(min=REWARD_EPSILON))
        loss = -log_prob * reward  # REINFORCE loss
        return loss

    def update_policy(self, obs: np.ndarray, action: int, reward: float):
        loss = self.compute_loss(obs, action, reward)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        return float(loss.item())

    @staticmethod
    def mlp(sizes, activation=nn.Tanh, output_activation=nn.Identity):
        # Build a feedforward neural network.
        layers = []
        for j in range(len(sizes) - 1):
            act = activation if j < len(sizes) - 2 else output_activation
            layers += [nn.Linear(sizes[j], sizes[j + 1]), act()]
        return nn.Sequential(*layers)

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
        row = math.floor((pos[1] - map.info.origin.position.y) / m_res)
        col = math.floor((pos[0] - map.info.origin.position.x) / m_res)
        if row < 0 or row >= map.info.height or col < 0 or col >= map.info.width:
            raise ValueError("Position is out of map bounds")

        return (row, col)

    @staticmethod
    def map_index_to_pos(
        map: OccupancyGrid,
        index: tuple[int, int],
    ) -> tuple[float, float]:
        """
        Return position of cell with given index
        """
        row, col = index
        if row < 0 or row >= map.info.height or col < 0 or col >= map.info.width:
            raise ValueError("Map index is out of map bounds")

        m_res: float = map.info.resolution
        return (
            map.info.origin.position.x + ((col + 0.5) * m_res),
            map.info.origin.position.y + ((row + 0.5) * m_res),
        )

    @staticmethod
    def transform_map(map: OccupancyGrid) -> np.ndarray:
        map_data: np.ndarray = np.array(map.data, dtype=np.int8).reshape(
            (map.info.height, map.info.width)
        )

        out = np.zeros((map.info.height, map.info.width, 5), dtype=np.int8)
        # occupancy map is defined in binary as follows:
        out[:, :, 0] = (map_data[:, :] == -1).astype(np.int8)  # explored - 0, unexplored - 1
        out[:, :, 1] = (map_data[:, :] == 100).astype(np.int8)  # free - 0, occupied - 1
        out[:, :, 2] = ((map_data[:, :] >= 10) & (map_data[:, :] < 100)).astype(
            np.int8
        )  # overlap - 1
        out[:, :, 3] = (map_data[:, :] == -10).astype(np.int8)  # agent - 1
        out[:, :, 4] = (map_data[:, :] == 110).astype(np.int8)  # others agent - 1

        return out

    @staticmethod
    def get_local_patch(map: OccupancyGrid, odom: Odometry, patch_size: int = 5) -> OccupancyGrid:
        local_map = OccupancyGrid()

        if map and odom:
            local_map.info.resolution = map.info.resolution
            local_map.header.frame_id = map.header.frame_id
            local_map.header.stamp = map.header.stamp
            local_map.info.origin.position.x = (
                odom.pose.pose.position.x - patch_size * map.info.resolution
            )
            local_map.info.origin.position.y = (
                odom.pose.pose.position.y - patch_size * map.info.resolution
            )
            local_map.info.width = patch_size * 2 + 1
            local_map.info.height = patch_size * 2 + 1
            local_map.data = [-1] * (local_map.info.width * local_map.info.height)

            center_index = DecisionNetwork.pos_to_map_index(
                map, (odom.pose.pose.position.x, odom.pose.pose.position.y)
            )

            for i in range(-patch_size, patch_size + 1):
                for j in range(-patch_size, patch_size + 1):
                    global_i = center_index[0] + i
                    global_j = center_index[1] + j

                    if 0 <= global_i < map.info.height and 0 <= global_j < map.info.width:
                        local_index = (i + patch_size) * local_map.info.width + (j + patch_size)
                        global_index = global_i * map.info.width + global_j
                        local_map.data[local_index] = map.data[global_index]

        return local_map


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

        # total_reward += (new_ratio - old_ratio) * 100.0  # Scale reward

        # Criterion 2: Traveled distance to information gain from exploration
        old_position: np.array = np.array(
            [self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y]
        )
        new_position: np.array = np.array(
            [new_odom.pose.pose.position.x, new_odom.pose.pose.position.y]
        )
        distance_traveled: float = np.linalg.norm(new_position - old_position)
        information_gain: float = (new_ratio - old_ratio) * 100.0

        total_reward += information_gain / (distance_traveled * 0.1)  # Penalize excessive movement

        # # Criterion 3: Contribution to total coverage of the map per robot
        # local_map_data: np.ndarray = np.array(local_map.data).reshape(
        #     local_map.info.height, local_map.info.width
        # )
        # local_explored: int = np.sum((local_map_data >= 0) & (local_map_data < 100))
        # total_explored: int = np.sum((new_map_data >= 0) & (new_map_data < 100))

        # total_reward += (local_explored / total_explored) * 20.0  # Scale reward

        return float(total_reward)
