from nav_msgs.msg import OccupancyGrid, Odometry
from collections import deque
import numpy as np
import torch.nn as nn
from torch.optim import Adam
from torch.distributions import Categorical
from std_srvs.srv import Trigger
import torch
from random import random
import math
import copy

from robot_node.data_logger import DataLogger
from robot_node.utils import pos_to_map_index

EXPLOITATION_RATIO: float = 0.7  # More use of learned policy, less random overlap revisits
LEARNING_RATE: float = 1e-3  # Slightly stabler updates
REWARD_EPSILON: float = 1e-6
GAMMA: float = 0.995  # More long-term planning
ENTROPY_COEF: float = 0.01  # Less random exploration
ENTROPY_COEF_MIN: float = 0.005
TARGET_ENTROPY_RATIO: float = 0.55
VALUE_COEF: float = 0.35
ACTION_COUNT: int = 8
PATCH_RADIUS: int = 6  # Larger local patch for overlap context
MIN_GOAL_DISTANCE_M: float = 2.0
OBS_DIM: int = (PATCH_RADIUS * 2 + 1) ** 2
REWARD_NORMALIZATION_BETA: float = 0.2
REWARD_NORMALIZATION_CLIP: float = 5.0
REWARD_NORMALIZATION_EPS: float = 1e-5
CHECKPOINT_SCORE_KEY: str = "avg_reward"
CHECKPOINT_ROLLING_WINDOW: int = 20
MIN_TRAINING_BATCH_SIZE: int = 4
MIN_REWARD_DISTANCE_M: float = 1.0
REWARD_CLIP_ABS: float = 8.0
TARGET_ENTROPY: float = TARGET_ENTROPY_RATIO * math.log(ACTION_COUNT)


def layer_init(layer: nn.Module, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer


class DecisionNetwork(nn.Module):
    def __init__(self, robot_name: str, pheromone_decay: float, train: bool = True, parent=None):
        super().__init__()
        self.robot_name = robot_name
        self.pheromone_decay = pheromone_decay
        self.train_enabled = train
        self.feedback_layer = FeedbackLayer(self)
        self.device: torch.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.data_logger = DataLogger()

        if parent:
            self.logger = parent.get_logger()

        self.current_map: OccupancyGrid = None
        self.input_map: np.ndarray = None
        self.current_odom: Odometry = None

        # PPO actor-critic architecture with separate networks for policy and value estimation.
        self.critic = nn.Sequential(
            layer_init(nn.Linear(OBS_DIM, 96)),
            nn.SiLU(),
            layer_init(nn.Linear(96, 96)),
            nn.SiLU(),
            layer_init(nn.Linear(96, 64)),
            nn.SiLU(),
            layer_init(nn.Linear(64, 1), std=1.0),
        )
        self.actor = nn.Sequential(
            layer_init(nn.Linear(OBS_DIM, 64)),
            nn.SiLU(),
            layer_init(nn.Linear(64, 64)),
            nn.SiLU(),
            layer_init(nn.Linear(64, 64)),
            nn.SiLU(),
            (
                layer_init(nn.Linear(64, ACTION_COUNT), std=0.01)
                if ACTION_COUNT == 1
                else layer_init(nn.Linear(64, ACTION_COUNT), std=np.sqrt(0.1))
            ),
        )
        self.optimizer = Adam(self.parameters(), lr=LEARNING_RATE)
        self.reward_mean: float = 0.0
        self.reward_variance: float = 1.0
        self.reward_normalization_count: int = 0
        self.best_checkpoint_state: dict[str, torch.Tensor] = None
        self.best_checkpoint_score: float = float("-inf")
        self.latest_training_score: float = float("-inf")
        self.checkpoint_scores: deque[float] = deque(maxlen=CHECKPOINT_ROLLING_WINDOW)
        self.training_steps: int = 0  # For exploration decay
        self.to(self.device)

        # ----- Rollout storage -----
        self.rollout_obs: list[torch.Tensor] = []
        self.rollout_actions: list[int] = []
        self.rollout_rewards: list[float] = []
        self.rollout_raw_rewards: list[float] = []

        # Pending transition for delayed reward assignment
        self.pending_obs: torch.Tensor = None
        self.pending_action: int = None

    def get_value(self, x):
        x = self._obs_to_tensor(x)
        return self.critic(x)

    def get_action_and_value(self, x, action=None):
        x = self._obs_to_tensor(x)
        logits = self.actor(x)
        probs = Categorical(logits=logits)
        if action is None:
            action = probs.sample()
        return action, probs.log_prob(action), probs.entropy(), self.critic(x)

    def update_state(self, map: OccupancyGrid, odom: Odometry):
        self.current_map = map
        self.input_map = self.transform_map(map)
        self.current_odom = odom

    def generate_goal(self) -> tuple[float, float]:
        if not self.current_map or not self.current_odom:
            return None

        if self.train_enabled:
            # Finalize reward for previously issued action once a new state is available.
            self._finalize_pending_transition()

        obs_vec = self.extract_observation(self.current_map, self.current_odom)
        obs_tensor = self._obs_to_tensor(obs_vec)

        logits = self.actor(obs_tensor)
        policy = torch.softmax(logits, dim=-1)

        # Decay exploration over time: higher exploitation as training progresses
        exploration_ratio = EXPLOITATION_RATIO
        if self.train_enabled:
            # Gradual increase in exploitation from 0.7 to 0.9 over 100k steps
            max_steps = 100000.0
            exploration_decay = min(0.2, (self.training_steps / max_steps) * 0.2)
            exploration_ratio = min(0.9, EXPLOITATION_RATIO + exploration_decay)

        if random() < exploration_ratio:
            # Try higher-probability actions first, then fall back to the rest.
            candidate_actions = torch.argsort(policy, descending=True).tolist()
        else:
            sampled_order = torch.multinomial(policy, num_samples=ACTION_COUNT, replacement=False)
            candidate_actions = sampled_order.tolist()

        action_int = None
        goal = None
        for cand_action in candidate_actions:
            candidate_goal = self.action_to_goal(cand_action, self.current_odom, self.current_map)
            if candidate_goal is not None:
                action_int = int(cand_action)
                goal = candidate_goal
                break

        if goal is None or action_int is None:
            # No valid frontier found in any direction; stay in exploration mode
            return None

        action_tensor = torch.tensor(action_int, dtype=torch.int64, device=self.device)
        action, _, _, _ = self.get_action_and_value(obs_tensor, action_tensor)

        if self.train_enabled:
            # Save transition context and source state for delayed reward computation.
            self.pending_obs = obs_tensor.detach().cpu()
            self.pending_action = int(action.item())
            # Capture "old" state exactly at action commit time.
            self.feedback_layer.save_state(self.current_map, self.current_odom)
        return goal

    def train_epoch(self):
        if not self.train_enabled:
            return None

        # Finalize potential pending transition before optimization.
        self._finalize_pending_transition()

        if len(self.rollout_rewards) < MIN_TRAINING_BATCH_SIZE:
            return None

        obs_batch = torch.stack(self.rollout_obs).to(self.device)
        action_batch = torch.as_tensor(
            self.rollout_actions,
            dtype=torch.int64,
            device=self.device,
        )
        rewards = torch.tensor(self.rollout_rewards, dtype=torch.float32, device=self.device)
        raw_rewards = torch.tensor(
            self.rollout_raw_rewards, dtype=torch.float32, device=self.device
        )

        logits = self.actor(obs_batch)
        dist = Categorical(logits=logits)
        log_probs = dist.log_prob(action_batch)
        entropies = dist.entropy()
        values: torch.Tensor = self.critic(obs_batch).squeeze(-1)

        returns = torch.zeros_like(rewards)
        running_return = torch.tensor(0.0, device=self.device)
        for t in range(len(rewards) - 1, -1, -1):
            running_return = rewards[t] + GAMMA * running_return
            returns[t] = running_return

        advantages: torch.Tensor = returns - values

        if advantages.numel() > 1:
            advantages = (advantages - advantages.mean()) / (advantages.std(unbiased=False) + 1e-8)

        # Clip advantages to prevent extreme policy updates
        clipped_advantages = torch.clamp(advantages, -5.0, 5.0)

        policy_loss: torch.Tensor = -(log_probs * clipped_advantages.detach()).mean()
        value_loss: torch.Tensor = torch.nn.functional.mse_loss(values, returns)
        entropy_bonus: torch.Tensor = entropies.mean()

        # Adaptive entropy coefficient based on target entropy
        entropy_value = float(entropy_bonus.detach().item())
        entropy_scale = 1.0
        if entropy_value > 1e-6:
            entropy_scale = max(0.5, min(2.0, TARGET_ENTROPY / (entropy_value + 1e-8)))
        entropy_coef = max(ENTROPY_COEF_MIN, ENTROPY_COEF * entropy_scale)

        # Normalize value loss to prevent it from dominating policy loss
        normalized_value_loss = (
            value_loss / (returns.std() + 1e-8) if returns.std() > 1e-6 else value_loss
        )
        loss: torch.Tensor = (
            policy_loss + VALUE_COEF * normalized_value_loss - entropy_coef * entropy_bonus
        )

        self.optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.parameters(), 1.0)
        self.optimizer.step()

        # Increment training step counter for exploration decay
        self.training_steps += 1

        metrics = {
            "loss": float(loss.item()),
            "policy_loss": float(policy_loss.item()),
            "value_loss": float(value_loss.item()),
            "entropy": float(entropy_bonus.item()),
            "entropy_coef": float(entropy_coef),
            "avg_reward": float(raw_rewards.mean().item()),
            "batch_size": len(self.rollout_rewards),
            "training_steps": self.training_steps,
        }
        self.data_logger.log_batch(metrics)
        self._checkpoint(metrics)
        self._clear_rollout()
        return metrics

    def extract_observation(
        self,
        map: OccupancyGrid,
        odom: Odometry,
        patch_radius: int = PATCH_RADIUS,
    ) -> np.ndarray:
        local_patch = self.get_local_patch(map, odom, patch_size=patch_radius)
        transformed = self.transform_map(local_patch).astype(np.float32)

        # Convert 5-channel occupancy encoding into a single scalar utility map.
        unexplored = transformed[:, :, 0]
        occupied = transformed[:, :, 1]
        overlap = transformed[:, :, 2]
        others = transformed[:, :, 4]
        utility = unexplored + 0.3 * overlap - occupied - 0.2 * others

        # Normalize utility to prevent extreme values and improve neural network training
        utility_max = np.abs(utility).max() + 1e-6
        utility = utility / utility_max

        obs_vec = utility.reshape(-1)

        # Keep NN input size stable even if patch radius/config drifts at runtime.
        if obs_vec.size < OBS_DIM:
            padded = np.zeros((OBS_DIM,), dtype=np.float32)
            padded[: obs_vec.size] = obs_vec
            return padded
        if obs_vec.size > OBS_DIM:
            return obs_vec[:OBS_DIM]
        return obs_vec

    @staticmethod
    def action_to_goal(
        action: int,
        odom: Odometry,
        map: OccupancyGrid,
        step_cells: int = 3,
    ) -> tuple[float, float]:
        if odom is None or map is None:
            return None

        if action < 0 or action >= ACTION_COUNT:
            raise ValueError(f"Action must be in [0, {ACTION_COUNT - 1}], got {action}")

        angle = (2.0 * math.pi * action) / ACTION_COUNT
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)

        step = max(step_cells, 1) * map.info.resolution
        step = max(step, MIN_GOAL_DISTANCE_M)

        goal_x = odom.pose.pose.position.x + (cos_angle * step)
        goal_y = odom.pose.pose.position.y + (sin_angle * step)

        min_x = map.info.origin.position.x
        min_y = map.info.origin.position.y
        max_x = min_x + (map.info.width - 1) * map.info.resolution
        max_y = min_y + (map.info.height - 1) * map.info.resolution

        clamped_goal_x = float(np.clip(goal_x, min_x, max_x))
        clamped_goal_y = float(np.clip(goal_y, min_y, max_y))

        goal_distance = math.hypot(
            clamped_goal_x - odom.pose.pose.position.x,
            clamped_goal_y - odom.pose.pose.position.y,
        )

        # If clipping collapses the target near current pose (often near map edges), skip this action.
        if goal_distance < (MIN_GOAL_DISTANCE_M * 0.5):
            return None

        # Frontier detection: verify path has unexplored or free space ahead
        if not DecisionNetwork._has_frontier_in_direction(
            map,
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            cos_angle,
            sin_angle,
            sample_distance=3.0,
        ):
            return None

        return (clamped_goal_x, clamped_goal_y)

    @staticmethod
    def _has_frontier_in_direction(
        map: OccupancyGrid,
        robot_x: float,
        robot_y: float,
        cos_angle: float,
        sin_angle: float,
        sample_distance: float = 1.5,
    ) -> bool:
        """Check if there's unexplored or free space ahead in the given direction."""
        # Sample points along the ray to detect frontiers (boundaries between explored/unexplored)
        map_data: np.ndarray = np.array(map.data, dtype=np.int8).reshape(
            (map.info.height, map.info.width)
        )

        samples = 3
        for i in range(1, samples + 1):
            sample_dist = sample_distance * i
            sample_x = robot_x + cos_angle * sample_dist
            sample_y = robot_y + sin_angle * sample_dist

            try:
                idx = pos_to_map_index(map, [sample_x, sample_y])
                if idx is None:
                    return True  # Out of bounds but in valid direction

                cell_value = map_data[idx[1], idx[0]]
                # Prefer unexplored (-1) and free (0) over occupied (100)
                if cell_value == -1 or cell_value == 0:
                    return True
                # Penalize occupied cells; continue searching
                if cell_value >= 50:  # Likely occupied
                    continue
            except (IndexError, ValueError):
                # Out of bounds
                return True

        return False

    def _obs_to_tensor(self, obs) -> torch.Tensor:
        if isinstance(obs, torch.Tensor):
            return obs.to(self.device, dtype=torch.float32).view(-1)
        return torch.as_tensor(obs, dtype=torch.float32, device=self.device).view(-1)

    def _finalize_pending_transition(self):
        if not self.train_enabled:
            return

        if (
            self.pending_obs is None
            or self.pending_action is None
            or not self.feedback_layer.has_state()
            or self.current_map is None
            or self.current_odom is None
        ):
            return

        local_map = self.get_local_patch(self.current_map, self.current_odom, PATCH_RADIUS)
        reward = self.feedback_layer.calculate_reward(
            self.current_map,
            self.current_odom,
            local_map,
        )
        if not np.isfinite(reward):
            reward = 0.0
        normalized_reward = self._normalize_reward(reward)

        self.rollout_obs.append(self.pending_obs)
        self.rollout_actions.append(self.pending_action)
        self.rollout_rewards.append(float(normalized_reward))
        self.rollout_raw_rewards.append(float(reward))

        self.pending_obs = None
        self.pending_action = None

    def _clear_rollout(self):
        self.rollout_obs.clear()
        self.rollout_actions.clear()
        self.rollout_rewards.clear()
        self.rollout_raw_rewards.clear()

    def _checkpoint(self, metrics: dict[str, float] | None = None) -> float:
        score = float("-inf")
        if metrics is not None:
            score = float(metrics.get(CHECKPOINT_SCORE_KEY, float("-inf")))
        self.checkpoint_scores.append(score)

        rolling_score = float(np.mean(self.checkpoint_scores))
        self.latest_training_score = rolling_score

        if rolling_score >= self.best_checkpoint_score:
            self.best_checkpoint_score = rolling_score
            self.best_checkpoint_state = copy.deepcopy(self.state_dict())

        return rolling_score

    def _select_state_to_save(self) -> tuple[dict[str, torch.Tensor], str]:
        current_score = self.latest_training_score
        if self.best_checkpoint_state is None or current_score >= self.best_checkpoint_score:
            return (copy.deepcopy(self.state_dict()), "current")
        return (self.best_checkpoint_state, "checkpoint")

    @staticmethod
    def _soft_update(source_network: nn.Module, target_network: nn.Module, tau: float):
        for source_parameter, target_parameter in zip(
            source_network.parameters(), target_network.parameters()
        ):
            target_parameter.data.mul_(1.0 - tau)
            target_parameter.data.add_(tau * source_parameter.data)

    def _normalize_reward(self, reward: float) -> float:
        self.reward_normalization_count += 1

        if self.reward_normalization_count == 1:
            self.reward_mean = reward
            self.reward_variance = 1.0
        else:
            delta = reward - self.reward_mean
            self.reward_mean += REWARD_NORMALIZATION_BETA * delta
            self.reward_variance = (
                1.0 - REWARD_NORMALIZATION_BETA
            ) * self.reward_variance + REWARD_NORMALIZATION_BETA * (delta**2)

        normalized_reward = (reward - self.reward_mean) / math.sqrt(
            self.reward_variance + REWARD_NORMALIZATION_EPS
        )
        return float(
            np.clip(normalized_reward, -REWARD_NORMALIZATION_CLIP, REWARD_NORMALIZATION_CLIP)
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

            try:
                center_index = pos_to_map_index(
                    map, (odom.pose.pose.position.x, odom.pose.pose.position.y)
                )
            except ValueError:
                return local_map

            if center_index is None:
                return local_map

            try:
                for i in range(-patch_size, patch_size + 1):
                    for j in range(-patch_size, patch_size + 1):
                        global_i = center_index[0] + i
                        global_j = center_index[1] + j

                        if 0 <= global_i < map.info.height and 0 <= global_j < map.info.width:
                            local_index = (i + patch_size) * local_map.info.width + (j + patch_size)
                            global_index = global_i * map.info.width + global_j
                            local_map.data[local_index] = map.data[global_index]

            except (IndexError, TypeError):
                return local_map

        return local_map

    def save_model(self, path: str) -> bool:
        state_to_save, save_label = self._select_state_to_save()
        torch.save(state_to_save, path)
        self.logger.info(f"Model saved to {path} from {save_label}")
        return True

    def load_model(self, request: Trigger.Request, response: Trigger.Response, path: str):
        try:
            self.load_state_dict(torch.load(path, map_location=self.device))
            self.to(self.device)
            response.success = True
            response.message = "Model loaded successfully."

        except RuntimeError as e:
            response.message = f"Error loading model from {path}: {e}"
            response.success = False

        except FileNotFoundError:
            response.message = f"Model file not found at {path}!"
            response.success = False

        return response

    def load_model_from_path(self, path: str) -> bool:
        try:
            self.load_state_dict(torch.load(path, map_location=self.device))
            self.to(self.device)
            self.logger.info(f"Model loaded successfully from {path}")
            return True

        except RuntimeError as e:
            self.logger.error(f"Error loading model from {path}: {e}")
            return False

        except FileNotFoundError:
            self.logger.error(f"Model file not found at {path}!")
            return False


class FeedbackLayer:
    def __init__(self, network: DecisionNetwork):
        self.network: DecisionNetwork = network
        self.current_state: OccupancyGrid = None
        self.current_odom: Odometry = None

    def save_state(self, map: OccupancyGrid, odom: Odometry):
        # Keep snapshots so later callbacks cannot overwrite reward baseline semantics.
        self.current_state = copy.deepcopy(map)
        self.current_odom = copy.deepcopy(odom)

    def has_state(self) -> bool:
        return self.current_state is not None and self.current_odom is not None

    def calculate_reward(
        self,
        new_map: OccupancyGrid,
        new_odom: Odometry,
        local_map: OccupancyGrid = None,
    ) -> float:
        if not self.has_state() or new_map is None or new_odom is None:
            return 0.0

        if local_map is None:
            local_map = self.network.get_local_patch(new_map, new_odom, PATCH_RADIUS)

        total_reward: float = 0.0

        # Criterion 1: Ratio of explored to unexplored cells
        old_map_data: np.ndarray = np.array(self.current_state.data, dtype=np.int16).reshape(
            self.current_state.info.height, self.current_state.info.width
        )
        new_map_data: np.ndarray = np.array(new_map.data, dtype=np.int16).reshape(
            new_map.info.height, new_map.info.width
        )

        old_known: int = np.sum((old_map_data >= 0) & (old_map_data <= 100))
        new_known: int = np.sum((new_map_data >= 0) & (new_map_data <= 100))
        map_cells: float = float(max(new_map_data.size, 1))
        information_gain: float = (new_known - old_known) / map_cells

        # Criterion 2: Traveled distance to information gain from exploration
        old_position: np.array = np.array(
            [self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y],
            dtype=np.float32,
        )
        new_position: np.array = np.array(
            [new_odom.pose.pose.position.x, new_odom.pose.pose.position.y],
            dtype=np.float32,
        )
        distance_traveled: float = float(np.linalg.norm(new_position - old_position))

        # Reward efficient exploration (gain per meter) instead of rewarding longer travel.
        travel_norm: float = max(distance_traveled, MIN_REWARD_DISTANCE_M)
        exploration_efficiency: float = information_gain / travel_norm
        total_reward += 3.0 * information_gain + 1.5 * exploration_efficiency

        # Criterion 3: Penalize travel through overlap areas to encourage efficient coverage
        local_map_data: np.ndarray = np.array(local_map.data, dtype=np.int16).reshape(
            local_map.info.height, local_map.info.width
        )
        local_cells: float = float(max(local_map_data.size, 1))
        overlap_ratio: float = np.sum((local_map_data >= 10) & (local_map_data < 100)) / local_cells

        # Stronger discouragement of already-explored-by-others areas.
        total_reward -= 2.5 * overlap_ratio

        # Criterion 4: Penalty for staying still
        if distance_traveled < (MIN_REWARD_DISTANCE_M * 0.5):
            total_reward -= 0.5

        return float(total_reward)
