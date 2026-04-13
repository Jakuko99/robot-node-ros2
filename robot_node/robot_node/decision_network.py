from nav_msgs.msg import OccupancyGrid, Odometry
from collections import deque
import numpy as np
import torch.nn as nn
import torch.nn.functional as F
from torch.optim import Adam
from torch.distributions import Categorical
from std_srvs.srv import Trigger
import torch
import random
import math
import copy

# TODO: allow loading hyperparameters from config file or ROS params

ACTOR_LR: float = 3e-4
CRITIC_LR: float = 3e-4
ALPHA_LR: float = 3e-4
REWARD_EPSILON: float = 1e-6
GAMMA: float = 0.99
ACTION_COUNT: int = 8
PATCH_RADIUS: int = 4
MIN_GOAL_DISTANCE_M: float = 0.8
OBS_DIM: int = (PATCH_RADIUS * 2 + 1) ** 2
REPLAY_BUFFER_SIZE: int = 50000
BATCH_SIZE: int = 32
UPDATES_PER_TRAIN: int = 4
MIN_REPLAY_SIZE: int = 4
TARGET_TAU: float = 0.005
INITIAL_ALPHA: float = 0.2
TARGET_ENTROPY: float = 0.8 * math.log(ACTION_COUNT)
REWARD_NORMALIZATION_BETA: float = 0.01
REWARD_NORMALIZATION_CLIP: float = 5.0
REWARD_NORMALIZATION_EPS: float = 1e-6


def layer_init(layer: nn.Module, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer


def build_mlp(input_dim: int, output_dim: int, output_std: float = 1.0) -> nn.Sequential:
    return nn.Sequential(
        layer_init(nn.Linear(input_dim, 128)),
        nn.SiLU(),
        layer_init(nn.Linear(128, 128)),
        nn.SiLU(),
        layer_init(nn.Linear(128, output_dim), std=output_std),
    )


class DecisionNetwork(nn.Module):
    def __init__(self, robot_name: str, pheromone_decay: float, train: bool = True, parent=None):
        super().__init__()
        self.robot_name = robot_name
        self.pheromone_decay = pheromone_decay
        self.train_enabled = train
        self.feedback_layer = FeedbackLayer(self)
        self.device: torch.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        if parent:
            self.logger = parent.get_logger()

        self.current_map: OccupancyGrid = None
        self.input_map: np.ndarray = None
        self.current_odom: Odometry = None

        self.actor = build_mlp(OBS_DIM, ACTION_COUNT, output_std=0.01)
        self.q1 = build_mlp(OBS_DIM, ACTION_COUNT, output_std=1.0)
        self.q2 = build_mlp(OBS_DIM, ACTION_COUNT, output_std=1.0)
        self.target_q1 = copy.deepcopy(self.q1)
        self.target_q2 = copy.deepcopy(self.q2)
        self.log_alpha = nn.Parameter(torch.tensor(math.log(INITIAL_ALPHA), dtype=torch.float32))

        if self.train_enabled:
            self.actor_optimizer = Adam(self.actor.parameters(), lr=ACTOR_LR)
            self.critic_optimizer = Adam(
                list(self.q1.parameters()) + list(self.q2.parameters()), lr=CRITIC_LR
            )
            self.alpha_optimizer = Adam([self.log_alpha], lr=ALPHA_LR)

        for target_network in (self.target_q1, self.target_q2):
            for parameter in target_network.parameters():
                parameter.requires_grad = False

        self.replay_buffer: deque[tuple[torch.Tensor, int, float, torch.Tensor, float]] = deque(
            maxlen=REPLAY_BUFFER_SIZE
        )
        self.reward_mean: float = 0.0
        self.reward_variance: float = 1.0
        self.reward_normalization_count: int = 0
        self.to(self.device)

        # ----- Rollout storage -----
        # Pending transition for delayed reward assignment
        self.pending_obs: torch.Tensor = None
        self.pending_action: int = None

    def get_value(self, x):
        x = self._obs_to_tensor(x)
        q1 = self.q1(x)
        q2 = self.q2(x)
        return torch.min(q1, q2)

    def get_action_and_value(self, x, action=None):
        x = self._obs_to_tensor(x)
        logits = self.actor(x)
        if logits.dim() == 1:
            logits = logits.unsqueeze(0)
        q1_values = self.q1(x)
        q2_values = self.q2(x)
        if q1_values.dim() == 1:
            q1_values = q1_values.unsqueeze(0)
        if q2_values.dim() == 1:
            q2_values = q2_values.unsqueeze(0)
        probs = Categorical(logits=logits)
        if action is None:
            action = probs.sample()
        action_tensor = action.long()
        if action_tensor.dim() == 0:
            action_tensor = action_tensor.unsqueeze(0)
        q1 = q1_values.gather(-1, action_tensor.view(-1, 1)).squeeze(-1)
        q2 = q2_values.gather(-1, action_tensor.view(-1, 1)).squeeze(-1)

        if q1.numel() == 1:
            q1 = q1.squeeze(0)
        if q2.numel() == 1:
            q2 = q2.squeeze(0)
        return action, probs.log_prob(action), probs.entropy(), q1, q2

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

        valid_actions: list[int] = []
        valid_goals: dict[int, tuple[float, float]] = {}
        for cand_action in range(ACTION_COUNT):
            candidate_goal = self.action_to_goal(cand_action, self.current_odom, self.current_map)
            if candidate_goal is not None:
                valid_actions.append(cand_action)
                valid_goals[cand_action] = candidate_goal

        if len(valid_actions) == 0:
            return None

        valid_probs = policy[valid_actions]
        valid_probs = valid_probs / valid_probs.sum().clamp_min(REWARD_EPSILON)
        chosen_index = int(torch.multinomial(valid_probs, num_samples=1).item())
        action_int = valid_actions[chosen_index]
        goal = valid_goals[action_int]

        action_tensor = torch.tensor(action_int, dtype=torch.int64, device=self.device)
        action, _, _, _, _ = self.get_action_and_value(obs_tensor, action_tensor)

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

        if len(self.replay_buffer) < MIN_REPLAY_SIZE:
            return None

        batch_size = min(BATCH_SIZE, len(self.replay_buffer))
        updates = min(UPDATES_PER_TRAIN, max(1, len(self.replay_buffer) // batch_size))

        metrics = {
            "loss": 0.0,
            "actor_loss": 0.0,
            "critic_loss": 0.0,
            "alpha_loss": 0.0,
            "entropy": 0.0,
            "alpha": 0.0,
            "avg_reward": 0.0,
            "avg_raw_reward": 0.0,
            "batch_size": batch_size,
            "updates": updates,
        }

        for _ in range(updates):
            batch = random.sample(self.replay_buffer, batch_size)
            obs_batch = torch.stack([item[0] for item in batch]).to(self.device)
            action_batch = torch.as_tensor(
                [item[1] for item in batch], dtype=torch.int64, device=self.device
            )
            reward_batch = torch.as_tensor(
                [item[2] for item in batch], dtype=torch.float32, device=self.device
            )
            raw_reward_batch = torch.as_tensor(
                [item[5] for item in batch], dtype=torch.float32, device=self.device
            )
            next_obs_batch = torch.stack([item[3] for item in batch]).to(self.device)
            done_batch = torch.as_tensor(
                [item[4] for item in batch], dtype=torch.float32, device=self.device
            )

            with torch.no_grad():
                next_logits = self.actor(next_obs_batch)
                next_log_probs = torch.log_softmax(next_logits, dim=-1)
                next_probs = next_log_probs.exp()
                next_q1 = self.target_q1(next_obs_batch)
                next_q2 = self.target_q2(next_obs_batch)
                next_min_q = torch.min(next_q1, next_q2)
                alpha = self.log_alpha.exp()
                next_v = (next_probs * (next_min_q - alpha * next_log_probs)).sum(dim=-1)
                target_q = reward_batch + (1.0 - done_batch) * GAMMA * next_v

            current_q1 = self.q1(obs_batch).gather(1, action_batch.unsqueeze(-1)).squeeze(-1)
            current_q2 = self.q2(obs_batch).gather(1, action_batch.unsqueeze(-1)).squeeze(-1)
            critic_loss = F.mse_loss(current_q1, target_q) + F.mse_loss(current_q2, target_q)

            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            nn.utils.clip_grad_norm_(list(self.q1.parameters()) + list(self.q2.parameters()), 5.0)
            self.critic_optimizer.step()

            logits = self.actor(obs_batch)
            log_probs = torch.log_softmax(logits, dim=-1)
            probs = log_probs.exp()
            min_q = torch.min(self.q1(obs_batch), self.q2(obs_batch)).detach()
            alpha = self.log_alpha.exp().detach()
            actor_loss = (probs * (alpha * log_probs - min_q)).sum(dim=-1).mean()

            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            nn.utils.clip_grad_norm_(self.actor.parameters(), 5.0)
            self.actor_optimizer.step()

            entropy = -(probs * log_probs).sum(dim=-1).mean()
            alpha_loss = -(self.log_alpha * (entropy.detach() - TARGET_ENTROPY)).mean()

            self.alpha_optimizer.zero_grad()
            alpha_loss.backward()
            self.alpha_optimizer.step()

            self._soft_update(self.q1, self.target_q1, TARGET_TAU)
            self._soft_update(self.q2, self.target_q2, TARGET_TAU)

            metrics["loss"] += float((critic_loss + actor_loss + alpha_loss).item())
            metrics["actor_loss"] += float(actor_loss.item())
            metrics["critic_loss"] += float(critic_loss.item())
            metrics["alpha_loss"] += float(alpha_loss.item())
            metrics["entropy"] += float(entropy.item())
            metrics["alpha"] += float(self.log_alpha.exp().item())
            metrics["avg_reward"] += float(reward_batch.mean().item())
            metrics["avg_raw_reward"] += float(raw_reward_batch.mean().item())

        for key in (
            "loss",
            "actor_loss",
            "critic_loss",
            "alpha_loss",
            "entropy",
            "alpha",
            "avg_reward",
            "avg_raw_reward",
        ):
            metrics[key] /= updates

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
        step = max(step_cells, 1) * map.info.resolution
        step = max(step, MIN_GOAL_DISTANCE_M)

        goal_x = odom.pose.pose.position.x + (math.cos(angle) * step)
        goal_y = odom.pose.pose.position.y + (math.sin(angle) * step)

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

        return (clamped_goal_x, clamped_goal_y)

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

        current_obs = self.extract_observation(self.current_map, self.current_odom)
        next_obs = self._obs_to_tensor(current_obs).detach().cpu()

        local_map = self.get_local_patch(self.current_map, self.current_odom, PATCH_RADIUS)
        reward = self.feedback_layer.calculate_reward(
            self.current_map,
            self.current_odom,
            local_map,
        )
        if not np.isfinite(reward):
            reward = 0.0
        normalized_reward = self._normalize_reward(float(reward))

        self.replay_buffer.append(
            (self.pending_obs, self.pending_action, normalized_reward, next_obs, 0.0, float(reward))
        )

        self.pending_obs = None
        self.pending_action = None

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

    def save_model(self, request: Trigger.Request, response: Trigger.Response, path: str):
        torch.save(self.state_dict(), path)
        response.success = True
        response.message = f"Model saved successfully to {path}."
        return response

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

        # total_reward += (new_ratio - old_ratio) * 1.0  # Scale reward

        # Criterion 2: Traveled distance to information gain from exploration
        old_position: np.array = np.array(
            [self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y]
        )
        new_position: np.array = np.array(
            [new_odom.pose.pose.position.x, new_odom.pose.pose.position.y]
        )
        distance_traveled: float = np.linalg.norm(new_position - old_position)
        information_gain: float = new_ratio - old_ratio

        total_reward += information_gain / max(
            distance_traveled * 0.1,
            REWARD_EPSILON,
        )  # Penalize excessive movement

        # Criterion 3: Contribution to total coverage of the map per robot
        # if local_map:
        # local_map_data: np.ndarray = np.array(local_map.data).reshape(
        #     local_map.info.height, local_map.info.width
        # )
        # local_explored: int = np.sum((local_map_data >= 0) & (local_map_data < 100))
        # total_explored: int = np.sum((new_map_data >= 0) & (new_map_data < 100))

        # total_reward += (local_explored / total_explored) * 20.0  # Scale reward

        # Criterion 4: Penalize travel through overlap areas to encourage efficient coverage
        current_loc: OccupancyGrid = self.network.get_local_patch(new_map, new_odom, PATCH_RADIUS)
        current_loc_data: np.ndarray = np.array(current_loc.data).reshape(
            current_loc.info.height, current_loc.info.width
        )
        overlap_cells: int = np.sum((current_loc_data >= 10) & (current_loc_data < 100))
        total_reward -= overlap_cells * 0.1  # Penalize overlap to encourage spreading out

        return float(total_reward)
