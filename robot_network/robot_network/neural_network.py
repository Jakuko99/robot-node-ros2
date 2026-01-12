import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torchvision import datasets, transforms
import matplotlib.pyplot as plt
from math import exp, sqrt
import numpy as np

from nav_msgs.msg import OccupancyGrid

BATCH_SIZE = 64
NUM_EPOCHS = 10
LEARNING_RATE = 0.001


class RobotSwarmOptimizerNetwork(nn.Module):
    """
    Network inputs:
    - Robot's current positions (x, y, theta)
    - Global map data merged from robots (occupancy grid) - dynamic size
    - Points of interest (unexplored areas, targets)

    Network outputs:
    - Next goal position (x, y)
    - Mapping priorities for unexplored areas
    """

    def __init__(self, train: bool = False, model_path: str = ""):
        super(RobotSwarmOptimizerNetwork, self).__init__()
        self._train: bool = train
        self.model_path: str = model_path

    def reward_criterion(
        self,
        position: tuple[float, float],
        state: OccupancyGrid,
        action: tuple[float, float],
        next_state: OccupancyGrid,
        frontiers: list[tuple[float, float]],
    ) -> int:
        """
        Reward exploration actions that extend the known map by reaching frontiers.
        """

        reward = 0.0

        # Frontier proximity reward
        if frontiers:
            fx, fy = zip(*frontiers)
            fx = np.array(fx)
            fy = np.array(fy)

            gx, gy = action
            dists = np.sqrt((fx - gx) ** 2 + (fy - gy) ** 2)
            min_dist = np.min(dists)

            # Encourage goals near frontiers
            sigma = 5.0
            reward += exp(-min_dist / sigma)

        # Actual map expansion reward
        before = state.data
        after = next_state.data

        newly_explored = np.logical_and(before == -1, after != -1)
        delta_unknown = int(np.sum(newly_explored))

        total_unknown_before = int(np.sum(before == -1))
        if total_unknown_before > 0:
            reward += 5.0 * (delta_unknown / total_unknown_before)

        # Frontier reduction reward - Fewer frontiers means exploration progress
        reward += 0.5 * (len(frontiers) - self._count_frontiers(next_state))

        # Motion cost penalty
        px, py = position
        ax, ay = action
        travel_cost = sqrt((ax - px) ** 2 + (ay - py) ** 2)
        reward -= 0.1 * travel_cost

        # Failure penalty
        if delta_unknown == 0:
            reward -= 0.5

        return int(round(reward))

    def _count_frontiers(self, grid: OccupancyGrid) -> int:
        h, w = grid.info.height, grid.info.width
        data = np.array(grid.data).reshape((h, w))
        frontier_count = 0

        for i in range(1, h - 1):
            for j in range(1, w - 1):
                if data[i, j] != 0:  # not free
                    continue

                neighborhood = data[i - 1 : i + 2, j - 1 : j + 2]
                if np.any(neighborhood == -1):
                    frontier_count += 1

        return frontier_count
