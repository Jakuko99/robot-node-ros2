from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import math
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Categorical


@dataclass
class MapMeta:
    resolution: float
    origin_x: float
    origin_y: float


@dataclass
class OccupancyGridData:
    """ROS-agnostic occupancy map container.

    grid: 2D array using ROS occupancy semantics (-1 unknown, 0 free, >0 occupied).
    """

    grid: np.ndarray
    meta: MapMeta


@dataclass
class RobotKinematics:
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class SwarmObservation:
    map_data: OccupancyGridData
    robot_states: Dict[str, RobotKinematics]


@dataclass
class ActionOutput:
    action_indices: Dict[str, int]
    target_points: Dict[str, Tuple[float, float]]
    frontier_centroids: torch.Tensor
    local_states: torch.Tensor
    global_state: torch.Tensor
    action_masks: torch.Tensor
    log_probs: torch.Tensor
    value: torch.Tensor


@dataclass
class RolloutStep:
    local_states: torch.Tensor
    global_state: torch.Tensor
    action_masks: torch.Tensor
    actions: torch.Tensor
    old_log_probs: torch.Tensor
    value: torch.Tensor
    reward: float
    done: float


class RolloutBuffer:
    def __init__(self):
        self.steps: List[RolloutStep] = []

    def add(self, step: RolloutStep) -> None:
        self.steps.append(step)

    def clear(self) -> None:
        self.steps.clear()

    def __len__(self) -> int:
        return len(self.steps)


class MultiAgentActorCritic(nn.Module):
    """CTDE actor-critic with decentralized actors and centralized critic."""

    def __init__(
        self,
        local_state_dim: int,
        global_state_dim: int,
        max_frontier_clusters: int,
        hidden_dim: int = 256,
    ):
        super().__init__()
        self.max_frontier_clusters = max_frontier_clusters

        self.actor_backbone = nn.Sequential(
            nn.Linear(local_state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.actor_head = nn.Linear(hidden_dim, max_frontier_clusters)

        self.critic = nn.Sequential(
            nn.Linear(global_state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
        )

    def actor_logits(
        self,
        local_state: torch.Tensor,
        action_mask: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        features = self.actor_backbone(local_state)
        logits = self.actor_head(features)
        if action_mask is not None:
            logits = logits.masked_fill(action_mask <= 0.0, -1e9)
        return logits

    def critic_value(self, global_state: torch.Tensor) -> torch.Tensor:
        return self.critic(global_state).squeeze(-1)


class RLCoordinator:
    """MAPPO-style coordinator: centralized critic, decentralized actors."""

    def __init__(
        self,
        num_agents: int,
        local_state_dim: int,
        global_state_dim: int,
        max_frontier_clusters: int,
        lr: float = 3e-4,
        gamma: float = 0.99,
        gae_lambda: float = 0.95,
        clip_epsilon: float = 0.2,
        entropy_coef: float = 0.01,
        value_coef: float = 0.5,
        device: str = "cpu",
    ):
        self.num_agents = num_agents
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_epsilon = clip_epsilon
        self.entropy_coef = entropy_coef
        self.value_coef = value_coef
        self.device = torch.device(device)

        self.model = MultiAgentActorCritic(
            local_state_dim=local_state_dim,
            global_state_dim=global_state_dim,
            max_frontier_clusters=max_frontier_clusters,
        ).to(self.device)
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=lr)

    def select_actions(
        self,
        local_states: torch.Tensor,
        global_state: torch.Tensor,
        action_masks: Optional[torch.Tensor] = None,
        deterministic: bool = False,
    ) -> Dict[str, torch.Tensor]:
        local_states = local_states.to(self.device)
        global_state = global_state.to(self.device)
        action_masks = (
            action_masks.to(self.device) if action_masks is not None else None
        )

        logits = self.model.actor_logits(local_states, action_masks)
        distribution = Categorical(logits=logits)

        if deterministic:
            actions = torch.argmax(logits, dim=-1)
        else:
            actions = distribution.sample()

        log_probs = distribution.log_prob(actions)
        entropy = distribution.entropy()
        value = self.model.critic_value(global_state.unsqueeze(0)).squeeze(0)

        return {
            "actions": actions,
            "log_probs": log_probs,
            "entropy": entropy,
            "value": value,
        }

    def compute_reward(
        self,
        coverage_gain: float,
        overlap_ratio: float,
        travel_distance: float,
        collision: bool,
        coverage_weight: float = 5.0,
        overlap_weight: float = 2.0,
        energy_weight: float = 0.1,
        collision_penalty: float = 10.0,
    ) -> float:
        reward = 0.0
        reward += coverage_weight * coverage_gain
        reward -= overlap_weight * overlap_ratio
        reward -= energy_weight * travel_distance
        if collision:
            reward -= collision_penalty
        return float(reward)

    def compute_gae(
        self,
        rewards: torch.Tensor,
        values: torch.Tensor,
        dones: torch.Tensor,
        last_value: float,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        gae = 0.0
        advantages = torch.zeros_like(rewards)
        for timestep in reversed(range(len(rewards))):
            next_value = (
                last_value if timestep == len(rewards) - 1 else values[timestep + 1]
            )
            non_terminal = 1.0 - dones[timestep]
            delta = (
                rewards[timestep]
                + self.gamma * next_value * non_terminal
                - values[timestep]
            )
            gae = delta + self.gamma * self.gae_lambda * non_terminal * gae
            advantages[timestep] = gae
        returns = advantages + values
        return returns, advantages

    def update_from_rollout(
        self,
        buffer: RolloutBuffer,
        last_value: float,
        epochs: int = 4,
    ) -> Dict[str, float]:
        if len(buffer) == 0:
            return {"actor_loss": 0.0, "critic_loss": 0.0, "entropy": 0.0}

        local_states = torch.stack([step.local_states for step in buffer.steps]).to(
            self.device
        )
        global_states = torch.stack([step.global_state for step in buffer.steps]).to(
            self.device
        )
        action_masks = torch.stack([step.action_masks for step in buffer.steps]).to(
            self.device
        )
        actions = torch.stack([step.actions for step in buffer.steps]).to(self.device)
        old_log_probs = torch.stack([step.old_log_probs for step in buffer.steps]).to(
            self.device
        )
        values = torch.stack([step.value for step in buffer.steps]).to(self.device)
        rewards = torch.tensor(
            [step.reward for step in buffer.steps],
            dtype=torch.float32,
            device=self.device,
        )
        dones = torch.tensor(
            [step.done for step in buffer.steps],
            dtype=torch.float32,
            device=self.device,
        )

        returns, advantages = self.compute_gae(rewards, values, dones, last_value)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        last_actor_loss = 0.0
        last_critic_loss = 0.0
        last_entropy = 0.0

        for _ in range(epochs):
            logits = self.model.actor_logits(
                local_states.reshape(-1, local_states.shape[-1]),
                action_masks.reshape(-1, action_masks.shape[-1]),
            )
            distribution = Categorical(logits=logits)

            flat_actions = actions.reshape(-1)
            new_log_probs = distribution.log_prob(flat_actions).reshape(
                actions.shape[0], actions.shape[1]
            )
            entropy = (
                distribution.entropy()
                .reshape(actions.shape[0], actions.shape[1])
                .mean(dim=1)
            )

            adv_agent = advantages.unsqueeze(-1).expand_as(new_log_probs)
            ratio = torch.exp(new_log_probs - old_log_probs)
            unclipped = ratio * adv_agent
            clipped = (
                torch.clamp(ratio, 1.0 - self.clip_epsilon, 1.0 + self.clip_epsilon)
                * adv_agent
            )
            actor_loss = -torch.min(unclipped, clipped).mean()

            critic_values = self.model.critic_value(global_states)
            critic_loss = F.mse_loss(critic_values, returns)

            entropy_bonus = entropy.mean()
            loss = (
                actor_loss
                + self.value_coef * critic_loss
                - self.entropy_coef * entropy_bonus
            )

            self.optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(self.model.parameters(), 0.5)
            self.optimizer.step()

            last_actor_loss = float(actor_loss.item())
            last_critic_loss = float(critic_loss.item())
            last_entropy = float(entropy_bonus.item())

        return {
            "actor_loss": last_actor_loss,
            "critic_loss": last_critic_loss,
            "entropy": last_entropy,
        }


class SwarmExplorer:
    """ROS-agnostic swarm exploration and MARL coordinator.

    RobotNode integration contract:
    - Provide `SwarmObservation` for each step.
    - Call `select_frontier_actions(...)` to get per-robot frontier targets.
    - Publish goals in RobotNode.
    - After environment transition, call `training_loop_step(...)`.
    """

    def __init__(
        self,
        robot_names: List[str],
        local_patch_size: int = 32,
        max_frontier_clusters: int = 20,
        collision_distance: float = 0.35,
        device: str = "cpu",
    ):
        self.robot_names = robot_names
        self.num_agents = len(robot_names)
        self.local_patch_size = local_patch_size
        self.max_frontier_clusters = max_frontier_clusters
        self.collision_distance = collision_distance

        local_state_dim = (
            3 + 1 + 1 + (2 * self.max_frontier_clusters) + (2 * self.num_agents)
        )
        global_state_dim = self.num_agents * local_state_dim

        self.rl = RLCoordinator(
            num_agents=self.num_agents,
            local_state_dim=local_state_dim,
            global_state_dim=global_state_dim,
            max_frontier_clusters=self.max_frontier_clusters,
            device=device,
        )

        self.rollout_buffer = RolloutBuffer()

    # ------------------------ Message -> Tensor --------------------------
    def occupancy_grid_to_numpy(self, grid_data: OccupancyGridData) -> np.ndarray:
        return np.asarray(grid_data.grid, dtype=np.int16)

    def normalize_grid(self, grid: np.ndarray) -> np.ndarray:
        normalized = np.zeros_like(grid, dtype=np.float32)
        normalized[grid == -1] = 0.5
        normalized[grid == 0] = 0.0
        normalized[grid > 0] = 1.0
        return normalized

    def compute_unknown_ratio(self, grid: np.ndarray) -> float:
        return float(np.mean(grid == -1))

    def compute_map_entropy(self, grid: np.ndarray) -> float:
        probabilities = np.array(
            [
                np.mean(grid == -1),
                np.mean(grid == 0),
                np.mean(grid > 0),
            ],
            dtype=np.float64,
        )
        probabilities = probabilities[probabilities > 0.0]
        if probabilities.size == 0:
            return 0.0
        return float(-np.sum(probabilities * np.log(probabilities + 1e-12)))

    def detect_frontier_cells(self, grid: np.ndarray) -> np.ndarray:
        free = grid == 0
        unknown = grid == -1

        height, width = grid.shape
        padded_unknown = np.pad(unknown, 1, mode="constant", constant_values=False)
        adjacent_unknown = np.zeros_like(free, dtype=bool)

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                adjacent_unknown |= padded_unknown[
                    1 + dy : 1 + dy + height,
                    1 + dx : 1 + dx + width,
                ]

        frontier_mask = free & adjacent_unknown
        return np.argwhere(frontier_mask)

    def cluster_frontiers(
        self,
        frontier_cells: np.ndarray,
        meta: MapMeta,
        min_cluster_size: int = 4,
        max_centroids: Optional[int] = None,
    ) -> np.ndarray:
        if frontier_cells.size == 0:
            return np.zeros((0, 2), dtype=np.float32)

        frontier_set = {tuple(cell.tolist()) for cell in frontier_cells}
        visited = set()
        centroids: List[Tuple[float, float]] = []

        neighbor_offsets = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]

        for seed in frontier_set:
            if seed in visited:
                continue

            queue = [seed]
            visited.add(seed)
            cluster = []

            while queue:
                current = queue.pop()
                cluster.append(current)
                for dr, dc in neighbor_offsets:
                    neighbor = (current[0] + dr, current[1] + dc)
                    if neighbor in frontier_set and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)

            if len(cluster) >= min_cluster_size:
                cluster_arr = np.array(cluster, dtype=np.float32)
                centroid_row_col = cluster_arr.mean(axis=0)
                centroids.append(
                    (
                        float(meta.origin_x + centroid_row_col[1] * meta.resolution),
                        float(meta.origin_y + centroid_row_col[0] * meta.resolution),
                    )
                )

        centroid_array = np.array(centroids, dtype=np.float32)
        if max_centroids is not None and len(centroid_array) > max_centroids:
            centroid_array = centroid_array[:max_centroids]
        return centroid_array

    def occupancy_grid_to_tensor(
        self, map_data: OccupancyGridData
    ) -> Dict[str, torch.Tensor]:
        grid = self.occupancy_grid_to_numpy(map_data)
        normalized = self.normalize_grid(grid)
        unknown_ratio = self.compute_unknown_ratio(grid)
        entropy = self.compute_map_entropy(grid)
        frontier_cells = self.detect_frontier_cells(grid)
        centroids = self.cluster_frontiers(
            frontier_cells,
            meta=map_data.meta,
            max_centroids=self.max_frontier_clusters,
        )

        return {
            "grid": torch.tensor(
                normalized, dtype=torch.float32, device=self.rl.device
            ),
            "unknown_ratio": torch.tensor(
                [unknown_ratio], dtype=torch.float32, device=self.rl.device
            ),
            "map_entropy": torch.tensor(
                [entropy], dtype=torch.float32, device=self.rl.device
            ),
            "frontier_cells": torch.tensor(
                frontier_cells, dtype=torch.float32, device=self.rl.device
            ),
            "frontier_centroids": torch.tensor(
                centroids, dtype=torch.float32, device=self.rl.device
            ),
        }

    def odometry_to_tensor(self, state: RobotKinematics) -> torch.Tensor:
        return torch.tensor(
            [state.x, state.y, state.yaw, state.vx, state.vy],
            dtype=torch.float32,
            device=self.rl.device,
        )

    def other_robots_to_tensors(
        self,
        robot_states: Dict[str, RobotKinematics],
    ) -> Tuple[Dict[str, torch.Tensor], torch.Tensor]:
        positions = np.array(
            [[robot_states[name].x, robot_states[name].y] for name in self.robot_names],
            dtype=np.float32,
        )
        pairwise_diff = positions[:, None, :] - positions[None, :, :]
        distances = np.linalg.norm(pairwise_diff, axis=-1)

        relative_map = {}
        for index, name in enumerate(self.robot_names):
            relative_map[name] = torch.tensor(
                pairwise_diff[index], dtype=torch.float32, device=self.rl.device
            )

        distance_matrix = torch.tensor(
            distances, dtype=torch.float32, device=self.rl.device
        )
        return relative_map, distance_matrix

    # ---------------------- State Construction ---------------------------
    def extract_local_patch(
        self,
        normalized_grid: np.ndarray,
        state: RobotKinematics,
        meta: MapMeta,
    ) -> np.ndarray:
        center_col = int((state.x - meta.origin_x) / meta.resolution)
        center_row = int((state.y - meta.origin_y) / meta.resolution)
        half = self.local_patch_size // 2

        row_min = max(center_row - half, 0)
        row_max = min(center_row + half, normalized_grid.shape[0])
        col_min = max(center_col - half, 0)
        col_max = min(center_col + half, normalized_grid.shape[1])

        patch = (
            np.ones((self.local_patch_size, self.local_patch_size), dtype=np.float32)
            * 0.5
        )
        cropped = normalized_grid[row_min:row_max, col_min:col_max]

        insert_row = (self.local_patch_size - cropped.shape[0]) // 2
        insert_col = (self.local_patch_size - cropped.shape[1]) // 2
        patch[
            insert_row : insert_row + cropped.shape[0],
            insert_col : insert_col + cropped.shape[1],
        ] = cropped
        return patch

    def _pad_frontier_centroids(self, centroids: torch.Tensor) -> torch.Tensor:
        padded = torch.zeros(
            (self.max_frontier_clusters, 2), dtype=torch.float32, device=self.rl.device
        )
        valid_count = min(centroids.shape[0], self.max_frontier_clusters)
        if valid_count > 0:
            padded[:valid_count] = centroids[:valid_count]
        return padded

    def build_state_tensors(
        self, observation: SwarmObservation
    ) -> Dict[str, torch.Tensor]:
        if any(name not in observation.robot_states for name in self.robot_names):
            raise ValueError(
                "Missing robot states for one or more robots in SwarmObservation"
            )

        map_tensors = self.occupancy_grid_to_tensor(observation.map_data)
        map_grid_np = self.occupancy_grid_to_numpy(observation.map_data)
        normalized_np = self.normalize_grid(map_grid_np)

        relative_positions, distance_matrix = self.other_robots_to_tensors(
            observation.robot_states
        )

        centroids = map_tensors["frontier_centroids"]
        padded_centroids = self._pad_frontier_centroids(centroids)

        local_state_list = []
        action_mask_list = []

        valid_centroid_count = min(centroids.shape[0], self.max_frontier_clusters)
        valid_mask = torch.zeros(
            self.max_frontier_clusters, dtype=torch.float32, device=self.rl.device
        )
        if valid_centroid_count > 0:
            valid_mask[:valid_centroid_count] = 1.0

        for name in self.robot_names:
            state = observation.robot_states[name]
            local_patch = self.extract_local_patch(
                normalized_np, state, observation.map_data.meta
            )
            _ = torch.tensor(local_patch, dtype=torch.float32, device=self.rl.device)

            pose_features = torch.tensor(
                [state.x, state.y, state.yaw],
                dtype=torch.float32,
                device=self.rl.device,
            )
            rel_features = relative_positions[name].reshape(-1)

            local_state = torch.cat(
                [
                    pose_features,
                    map_tensors["unknown_ratio"],
                    map_tensors["map_entropy"],
                    padded_centroids.reshape(-1),
                    rel_features,
                ]
            )
            local_state_list.append(local_state)
            action_mask_list.append(valid_mask.clone())

        local_states = torch.stack(local_state_list, dim=0)
        action_masks = torch.stack(action_mask_list, dim=0)
        global_state = local_states.reshape(-1)

        return {
            "local_states": local_states,
            "global_state": global_state,
            "action_masks": action_masks,
            "frontier_centroids": padded_centroids,
            "distance_matrix": distance_matrix,
            "map_entropy": map_tensors["map_entropy"],
            "unknown_ratio": map_tensors["unknown_ratio"],
        }

    # -------------------- Decision / Execution API -----------------------
    def select_frontier_actions(
        self,
        observation: SwarmObservation,
        deterministic: bool = False,
    ) -> ActionOutput:
        state_tensors = self.build_state_tensors(observation)
        if torch.all(state_tensors["action_masks"] <= 0.0):
            empty_actions = {name: -1 for name in self.robot_names}
            empty_targets = {name: (0.0, 0.0) for name in self.robot_names}
            zero = torch.zeros(
                self.num_agents, dtype=torch.float32, device=self.rl.device
            )
            return ActionOutput(
                action_indices=empty_actions,
                target_points=empty_targets,
                frontier_centroids=state_tensors["frontier_centroids"],
                local_states=state_tensors["local_states"],
                global_state=state_tensors["global_state"],
                action_masks=state_tensors["action_masks"],
                log_probs=zero,
                value=torch.tensor(0.0, device=self.rl.device),
            )

        rollout = self.rl.select_actions(
            local_states=state_tensors["local_states"],
            global_state=state_tensors["global_state"],
            action_masks=state_tensors["action_masks"],
            deterministic=deterministic,
        )

        actions = rollout["actions"].detach().cpu().numpy().tolist()
        centroids = state_tensors["frontier_centroids"].detach().cpu().numpy()

        action_indices: Dict[str, int] = {}
        target_points: Dict[str, Tuple[float, float]] = {}
        for robot_index, robot_name in enumerate(self.robot_names):
            frontier_index = int(actions[robot_index])
            target_x, target_y = centroids[frontier_index].tolist()
            action_indices[robot_name] = frontier_index
            target_points[robot_name] = (float(target_x), float(target_y))

        return ActionOutput(
            action_indices=action_indices,
            target_points=target_points,
            frontier_centroids=state_tensors["frontier_centroids"],
            local_states=state_tensors["local_states"],
            global_state=state_tensors["global_state"],
            action_masks=state_tensors["action_masks"],
            log_probs=rollout["log_probs"].detach(),
            value=rollout["value"].detach(),
        )

    # ------------------------- Reward utilities --------------------------
    def compute_coverage_gain(
        self, prev_unknown_ratio: float, next_unknown_ratio: float
    ) -> float:
        return max(prev_unknown_ratio - next_unknown_ratio, 0.0)

    def compute_frontier_overlap_ratio(self, assigned_frontiers: List[int]) -> float:
        valid = [index for index in assigned_frontiers if index >= 0]
        if not valid:
            return 0.0
        unique_count = len(set(valid))
        return 1.0 - (unique_count / len(valid))

    def compute_travel_distance(
        self,
        prev_observation: SwarmObservation,
        target_points: Dict[str, Tuple[float, float]],
    ) -> float:
        total = 0.0
        for robot_name in self.robot_names:
            current_state = prev_observation.robot_states[robot_name]
            target_xy = target_points.get(
                robot_name, (current_state.x, current_state.y)
            )
            total += float(
                math.hypot(
                    target_xy[0] - current_state.x, target_xy[1] - current_state.y
                )
            )
        return total

    def detect_collision(self, next_observation: SwarmObservation) -> bool:
        positions = np.array(
            [
                [
                    next_observation.robot_states[name].x,
                    next_observation.robot_states[name].y,
                ]
                for name in self.robot_names
            ],
            dtype=np.float32,
        )
        pairwise = positions[:, None, :] - positions[None, :, :]
        distance_matrix = np.linalg.norm(pairwise, axis=-1)
        np.fill_diagonal(distance_matrix, np.inf)
        return bool(np.any(distance_matrix < self.collision_distance))

    def compute_transition_reward(
        self,
        prev_observation: SwarmObservation,
        next_observation: SwarmObservation,
        action_output: ActionOutput,
    ) -> float:
        prev_unknown = self.compute_unknown_ratio(
            self.occupancy_grid_to_numpy(prev_observation.map_data)
        )
        next_unknown = self.compute_unknown_ratio(
            self.occupancy_grid_to_numpy(next_observation.map_data)
        )
        coverage_gain = self.compute_coverage_gain(prev_unknown, next_unknown)
        overlap_ratio = self.compute_frontier_overlap_ratio(
            list(action_output.action_indices.values())
        )
        travel_distance = self.compute_travel_distance(
            prev_observation, action_output.target_points
        )
        collision = self.detect_collision(next_observation)

        base_reward = self.rl.compute_reward(
            coverage_gain=coverage_gain,
            overlap_ratio=overlap_ratio,
            travel_distance=travel_distance,
            collision=collision,
        )

        distance_bonus = self._compute_frontier_proximity_bonus(
            prev_observation, action_output
        )

        return base_reward + distance_bonus

    def _compute_frontier_proximity_bonus(
        self,
        prev_observation: SwarmObservation,
        action_output: ActionOutput,
    ) -> float:
        """Compute reward bonus for choosing frontiers closer to robots.

        Prefers frontiers near current robot positions over distant ones.
        Uses exponential decay: bonus = sum(exp(-scaled_distance))
        """
        bonus = 0.0
        fronstier_centroids = action_output.frontier_centroids.detach().cpu().numpy()

        for robot_index, robot_name in enumerate(self.robot_names):
            action_idx = action_output.action_indices.get(robot_name, -1)
            if action_idx < 0 or action_idx >= len(fronstier_centroids):
                continue

            robot_state = prev_observation.robot_states[robot_name]
            target_centroid = fronstier_centroids[action_idx]

            dist = float(
                math.hypot(
                    target_centroid[0] - robot_state.x,
                    target_centroid[1] - robot_state.y,
                )
            )

            if dist > 0.1:
                scaled_dist = dist / 10.0
                prox_bonus = float(math.exp(-scaled_dist))
                bonus += 2.0 * prox_bonus

        return bonus

    # --------------------------- Training loop ---------------------------
    def add_transition(
        self,
        prev_observation: SwarmObservation,
        next_observation: SwarmObservation,
        action_output: ActionOutput,
        done: bool = False,
    ) -> float:
        reward = self.compute_transition_reward(
            prev_observation, next_observation, action_output
        )

        actions_tensor = torch.tensor(
            [action_output.action_indices[name] for name in self.robot_names],
            dtype=torch.long,
            device=self.rl.device,
        )

        self.rollout_buffer.add(
            RolloutStep(
                local_states=action_output.local_states.detach(),
                global_state=action_output.global_state.detach(),
                action_masks=action_output.action_masks.detach(),
                actions=actions_tensor,
                old_log_probs=action_output.log_probs.detach(),
                value=action_output.value.detach(),
                reward=float(reward),
                done=float(done),
            )
        )
        return float(reward)

    def update_policy(
        self,
        next_observation: Optional[SwarmObservation],
        done: bool,
        epochs: int = 4,
    ) -> Dict[str, float]:
        if len(self.rollout_buffer) == 0:
            return {"actor_loss": 0.0, "critic_loss": 0.0, "entropy": 0.0}

        if done or next_observation is None:
            last_value = 0.0
        else:
            next_state = self.build_state_tensors(next_observation)
            with torch.no_grad():
                last_value_tensor = self.rl.model.critic_value(
                    next_state["global_state"].unsqueeze(0)
                )
            last_value = float(last_value_tensor.squeeze(0).item())

        metrics = self.rl.update_from_rollout(
            self.rollout_buffer, last_value=last_value, epochs=epochs
        )
        self.rollout_buffer.clear()
        return metrics

    def training_loop_step(
        self,
        prev_observation: SwarmObservation,
        next_observation: SwarmObservation,
        action_output: ActionOutput,
        done: bool = False,
        update_every: int = 64,
        epochs: int = 4,
    ) -> Dict[str, float]:
        """One integrated training step for RobotNode-driven control loops.

        Typical cycle in RobotNode:
        1) action_output = explorer.select_frontier_actions(obs_t)
        2) RobotNode sends goals and waits for next map/odom update
        3) metrics = explorer.training_loop_step(obs_t, obs_t1, action_output, done)
        """
        reward = self.add_transition(
            prev_observation, next_observation, action_output, done=done
        )

        metrics = {
            "reward": reward,
            "buffer_size": float(len(self.rollout_buffer)),
            "actor_loss": 0.0,
            "critic_loss": 0.0,
            "entropy": 0.0,
        }

        if done or len(self.rollout_buffer) >= update_every:
            update_metrics = self.update_policy(
                next_observation=next_observation, done=done, epochs=epochs
            )
            metrics.update(update_metrics)

        return metrics

    def train(
        self,
        epochs: int = 4,
    ) -> Dict[str, float]:
        """Direct PPO training on accumulated experiences.

        Exposes the underlying PPO update for external batch-based training.
        Useful for centralized training from multi-environment rollouts.
        """
        if len(self.rollout_buffer) == 0:
            return {"actor_loss": 0.0, "critic_loss": 0.0, "entropy": 0.0}

        last_value = 0.0
        metrics = self.rl.update_from_rollout(
            self.rollout_buffer, last_value=last_value, epochs=epochs
        )
        self.rollout_buffer.clear()
        return metrics
