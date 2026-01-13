import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torchvision import datasets, transforms
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from math import exp, sqrt, ceil
from collections import deque
import numpy as np

from nav_msgs.msg import OccupancyGrid
from robot_network.robot_watcher import RobotWatcher

# ----- Hyperparameters -----
BATCH_SIZE = 64
NUM_EPOCHS = 10
LEARNING_RATE = 0.001

# ----- Frontier Detection Parameters -----
MIN_SAFETY_MARGIN = 0.5
MIN_CLUSTER_SIZE = 5


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
        self.previous_state: OccupancyGrid = None
        self.robots: dict[str, RobotWatcher] = {}

        self.model = nn.Sequential(
            nn.Linear(100, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 2),  # Output: next goal (x, y)
        )
        self.optimizer = optim.Adam(self.model.parameters(), lr=LEARNING_RATE)
        self.loss_fn = nn.MSELoss()

    def add_robot(self, robot_name: str, robot_watcher: RobotWatcher):
        if robot_name not in self.robots:
            self.robots[robot_name] = robot_watcher

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.model(x)

    def train_network(
        self,
        grid: OccupancyGrid,
    ) -> float:
        if not self._train:
            return None

        if self.previous_state is None:
            self.previous_state = grid
            return None

        robot_positions: list[tuple[float, float]] = [
            (watcher.x, watcher.y) for watcher in self.robots.values()
        ]
        reward: float = 0.0

        # Prepare training data
        inputs = []
        targets = []
        for pos in robot_positions:
            input_tensor = torch.tensor(pos, dtype=torch.float32)
            inputs.append(input_tensor)
            next_goal = pos
            target_tensor = torch.tensor(next_goal, dtype=torch.float32)
            targets.append(target_tensor)

        inputs_tensor = torch.stack(inputs)
        targets_tensor = torch.stack(targets)
        self.optimizer.zero_grad()
        outputs = self.forward(inputs_tensor)
        loss = self.loss_fn(outputs, targets_tensor)
        loss.backward()
        self.optimizer.step()
        self.previous_state = grid

        # compute reward
        for i, pos in enumerate(robot_positions):
            action = outputs[i].detach().numpy()
            reward = self.reward_criterion(pos, self.previous_state, action, grid)
            reward += reward

        # send goals to robots
        for i, (_, watcher) in enumerate(self.robots.items()):
            goal = outputs[i].detach().numpy()
            watcher.publish_goal(goal[0], goal[1])

        return reward

    def reward_criterion(
        self,
        position: tuple[float, float],
        state: OccupancyGrid,
        action: tuple[float, float],
        next_state: OccupancyGrid,
    ) -> float:
        """
        Reward exploration actions that extend the known map by reaching frontiers.
        """

        reward: float = 0.0
        frontiers: list[tuple[float, float]] = self._get_frontiers(state)

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

    def _count_frontiers(
        self,
        grid: OccupancyGrid,
        cluster_distance: float = 0.5,
    ) -> int:
        if not grid:
            return 0

        width: int = int(grid.info.width)
        height: int = int(grid.info.height)
        res: float = grid.info.resolution
        origin_x: float = grid.info.origin.position.x
        origin_y: float = grid.info.origin.position.y

        grid: np.ndarray = np.asarray(grid.data, dtype=np.int16).reshape(
            (height, width)
        )

        # Step 1: frontier cell detection (vectorized)
        free: np.ndarray = grid == 0
        unknown: np.ndarray = grid == -1
        occupied: np.ndarray = grid > 50
        # Pad for neighbor checks
        pad_unknown: np.ndarray = np.pad(
            unknown, 1, mode="constant", constant_values=False
        )
        pad_occupied: np.ndarray = np.pad(
            occupied, 1, mode="constant", constant_values=True
        )

        adjacent_unknown: np.ndarray = np.zeros_like(free, dtype=bool)
        adjacent_occupied: np.ndarray = np.zeros_like(free, dtype=bool)

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                adjacent_unknown |= pad_unknown[
                    1 + dy : 1 + dy + height, 1 + dx : 1 + dx + width
                ]
                adjacent_occupied |= pad_occupied[
                    1 + dy : 1 + dy + height, 1 + dx : 1 + dx + width
                ]

        frontier_mask = free & adjacent_unknown & (~adjacent_occupied)

        if not np.any(frontier_mask):
            return 0

        # Grid → world coordinates
        ys, xs = np.nonzero(frontier_mask)
        wx: int = origin_x + (xs + 0.5) * res
        wy: int = origin_y + (ys + 0.5) * res

        frontier_cells: np.ndarray = np.column_stack((xs, ys, wx, wy))

        # Step 2: safety check (vectorized per cell neighborhood)
        radius_cells: int = int(ceil(MIN_SAFETY_MARGIN / res))
        pad_occ: np.ndarray = np.pad(
            occupied, radius_cells, mode="constant", constant_values=True
        )

        safe_mask: np.ndarray = np.ones(len(frontier_cells), dtype=bool)

        for i, (x, y, _, _) in enumerate(frontier_cells):
            x = int(x)
            y = int(y)
            region = pad_occ[y : y + 2 * radius_cells + 1, x : x + 2 * radius_cells + 1]
            if np.any(region):
                safe_mask[i] = False

        safe_frontiers = frontier_cells[safe_mask]

        if len(safe_frontiers) == 0:
            return 0

        # Step 3: clustering using KD-Tree (fast)
        points: np.ndarray = safe_frontiers[:, 2:4]  # world coords only
        tree = cKDTree(points)

        visited: np.ndarray = np.zeros(len(points), dtype=bool)
        frontier_count: int = 0

        for i in range(len(points)):
            if visited[i]:
                continue

            # BFS using KD-Tree neighbors
            queue = deque([i])
            visited[i] = True
            cluster_size: int = 0

            while queue:
                idx = queue.popleft()
                cluster_size += 1

                neighbors = tree.query_ball_point(points[idx], cluster_distance)

                for n in neighbors:
                    if not visited[n]:
                        visited[n] = True
                        queue.append(n)

            if cluster_size >= MIN_CLUSTER_SIZE:
                frontier_count += 1

        return frontier_count

    def _get_frontiers(
        self,
        grid: OccupancyGrid,
        cluster_distance: float = 0.5,
    ) -> list[tuple[float, float]]:
        """
        Returns a list of frontier centroids [(x, y), ...]
        """

        if grid is None:
            return []

        width = int(grid.info.width)
        height = int(grid.info.height)
        res = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        # Convert map to NumPy grid
        grid = np.asarray(grid.data, dtype=np.int16).reshape((height, width))

        free = grid == 0
        unknown = grid == -1
        occupied = grid > 50

        # Step 1: frontier detection (vectorized)
        pad_unknown = np.pad(unknown, 1, mode="constant", constant_values=False)
        pad_occupied = np.pad(occupied, 1, mode="constant", constant_values=True)

        adjacent_unknown = np.zeros_like(free, dtype=bool)
        adjacent_occupied = np.zeros_like(free, dtype=bool)

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                adjacent_unknown |= pad_unknown[
                    1 + dy : 1 + dy + height, 1 + dx : 1 + dx + width
                ]
                adjacent_occupied |= pad_occupied[
                    1 + dy : 1 + dy + height, 1 + dx : 1 + dx + width
                ]

        frontier_mask = free & adjacent_unknown & (~adjacent_occupied)

        if not np.any(frontier_mask):
            return []

        # Grid → world coordinates
        ys, xs = np.nonzero(frontier_mask)
        wx = origin_x + (xs + 0.5) * res
        wy = origin_y + (ys + 0.5) * res

        frontier_cells = np.column_stack((xs, ys, wx, wy))

        # Step 2: safety check
        radius_cells = int(ceil(MIN_SAFETY_MARGIN / res))
        pad_occ = np.pad(occupied, radius_cells, mode="constant", constant_values=True)

        safe_mask = np.ones(len(frontier_cells), dtype=bool)

        for i, (x, y, _, _) in enumerate(frontier_cells):
            x = int(x)
            y = int(y)
            region = pad_occ[y : y + 2 * radius_cells + 1, x : x + 2 * radius_cells + 1]
            if np.any(region):
                safe_mask[i] = False

        safe_frontiers = frontier_cells[safe_mask]

        if len(safe_frontiers) == 0:
            return []

        # Step 3: clustering with KD-Tree
        points = safe_frontiers[:, 2:4]  # world coordinates
        tree = cKDTree(points)

        visited = np.zeros(len(points), dtype=bool)
        frontiers = []

        for i in range(len(points)):
            if visited[i]:
                continue

            queue = deque([i])
            visited[i] = True
            cluster_indices = []

            while queue:
                idx = queue.popleft()
                cluster_indices.append(idx)

                neighbors = tree.query_ball_point(points[idx], cluster_distance)

                for n in neighbors:
                    if not visited[n]:
                        visited[n] = True
                        queue.append(n)

            if len(cluster_indices) >= MIN_CLUSTER_SIZE:
                # Compute centroid
                centroid = points[cluster_indices].mean(axis=0)
                frontiers.append((float(centroid[0]), float(centroid[1])))

        return frontiers
