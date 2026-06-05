from collections import deque
import copy
import math
from random import random

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Categorical
from torch.optim import Adam
from nav_msgs.msg import OccupancyGrid, Odometry
from std_srvs.srv import Trigger

from robot_node.data_logger import DataLogger
from robot_node.swarm_coordination import (
    TEAM_CONTEXT_DIM,
    MonotonicValueMixer,
    summarize_map,
)
from robot_node.utils import pos_to_map_index

EXPLOITATION_RATIO: float = 0.7  # More use of learned policy, less random overlap revisits
LEARNING_RATE: float = 3e-4  # Lower variance during swarm updates
REWARD_EPSILON: float = 1e-6
GAMMA: float = 0.995  # More long-term planning
ENTROPY_COEF: float = 0.01  # Less random exploration
ENTROPY_COEF_MIN: float = 0.02
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
MIN_TRAINING_BATCH_SIZE: int = 8
MIN_REWARD_DISTANCE_M: float = 1.0
REWARD_CLIP_ABS: float = 8.0
TARGET_ENTROPY: float = TARGET_ENTROPY_RATIO * math.log(ACTION_COUNT)
MAX_LOGIT_MAGNITUDE: float = 8.0
REPLAY_BUFFER_SIZE: int = 128
REPLAY_WARMUP_SIZE: int = 64
OFF_POLICY_BATCH_SIZE: int = 32
OFF_POLICY_UPDATES_PER_EPOCH: int = 4
TARGET_CRITIC_TAU: float = 0.01
TEAM_CONTEXT_TAU: float = 0.02
REWARD_WEIGHT_NAMES: tuple[str] = (
    "information_gain",
    "exploration_efficiency",
    "coverage_gain",
    "frontier_gain",
    "overlap_growth",
    "occupied_growth",
    "team_redundancy_penalty",
    "crowding_penalty",
    "frontier_penalty",
    "stagnation_penalty",
    "nearest_teammate",
    "local_unknown_ratio",
)
REWARD_WEIGHT_PRIOR: np.ndarray = np.array(
    [3.0, 1.5, 4.0, 1.25, 2.0, 0.5, 1.5, 1.0, 0.75, 0.5, 0.25, 0.1],
    dtype=np.float32,
)
REWARD_WEIGHT_BOUNDS: np.ndarray = np.array(
    [
        [0.5, 6.0],
        [0.5, 4.0],
        [0.5, 8.0],
        [0.5, 4.0],
        [1.5, 5.0],
        [0.5, 3.0],
        [1.0, 3.0],
        [1.0, 3.0],
        [0.5, 3.0],
        [0.5, 3.0],
        [0.5, 1.0],
        [0.5, 0.5],
    ],
    dtype=np.float32,
)
REWARD_BO_WARMUP: int = 12
REWARD_BO_UPDATE_PERIOD: int = 6
REWARD_BO_HISTORY_SIZE: int = 96
REWARD_BO_CANDIDATES: int = 48
REWARD_BO_POOL_SIZE: int = 64
REWARD_BO_NOISE: float = 1e-4
REWARD_BO_UCB: float = 1.25
REWARD_WEIGHT_SMOOTHING: float = 0.25
REWARD_WEIGHT_PRIOR_PENALTY: float = 0.04
REWARD_BO_RNG_SEED: int = 13


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
        self.team_critic = nn.Sequential(
            layer_init(nn.Linear(TEAM_CONTEXT_DIM, 32)),
            nn.SiLU(),
            layer_init(nn.Linear(32, 32)),
            nn.SiLU(),
            layer_init(nn.Linear(32, 1), std=1.0),
        )
        self.value_mixer = MonotonicValueMixer(TEAM_CONTEXT_DIM)
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
        self.target_critic = copy.deepcopy(self.critic)
        self.target_team_critic = copy.deepcopy(self.team_critic)
        self.target_value_mixer = copy.deepcopy(self.value_mixer)
        self.target_critic.to(self.device)
        self.target_team_critic.to(self.device)
        self.target_value_mixer.to(self.device)
        self.target_critic.eval()
        self.target_team_critic.eval()
        self.target_value_mixer.eval()
        self.reward_mean: float = 0.0
        self.reward_variance: float = 1.0
        self.reward_normalization_count: int = 0
        self.best_checkpoint_state: dict[str, torch.Tensor] = None
        self.best_checkpoint_score: float = float("-inf")
        self.latest_training_score: float = float("-inf")
        self.checkpoint_scores: deque[float] = deque(maxlen=CHECKPOINT_ROLLING_WINDOW)
        self.training_steps: int = 0  # For exploration decay
        self.team_context: torch.Tensor = torch.zeros(TEAM_CONTEXT_DIM, dtype=torch.float32)
        self.pending_team_context: torch.Tensor = None
        self.to(self.device)

        # ----- Rollout storage -----
        self.rollout_obs: list[torch.Tensor] = []
        self.rollout_actions: list[int] = []
        self.rollout_rewards: list[float] = []
        self.rollout_raw_rewards: list[float] = []
        self.rollout_team_contexts: list[torch.Tensor] = []
        self.rollout_reward_components: list[dict[str, float]] = []
        self.replay_buffer: deque[
            tuple[torch.Tensor, int, float, float, torch.Tensor, float, torch.Tensor, torch.Tensor]
        ] = deque(maxlen=REPLAY_BUFFER_SIZE)

        # Pending transition for delayed reward assignment
        self.pending_obs: torch.Tensor = None
        self.pending_action: int = None

    def get_value(self, x):
        x = self._obs_to_tensor(x).unsqueeze(0)
        return self._estimate_value(x, self._team_context_tensor().unsqueeze(0))

    def get_action_and_value(self, x, action=None):
        x = self._obs_to_tensor(x)
        logits = torch.clamp(self.actor(x), -MAX_LOGIT_MAGNITUDE, MAX_LOGIT_MAGNITUDE)
        probs = Categorical(logits=logits)
        if action is None:
            action = probs.sample()
        value = self._estimate_value(x.unsqueeze(0), self._team_context_tensor().unsqueeze(0))
        return action, probs.log_prob(action), probs.entropy(), value

    def update_state(self, map: OccupancyGrid, odom: Odometry):
        self.current_map = map
        self.input_map = self.transform_map(map)
        self.current_odom = odom

    def update_team_context(self, team_context):
        self.team_context = self._team_context_tensor(team_context).detach().cpu()

    def _team_context_tensor(self, team_context=None) -> torch.Tensor:
        if team_context is None:
            team_context = self.team_context

        if isinstance(team_context, torch.Tensor):
            tensor = team_context.to(self.device, dtype=torch.float32).view(-1)
        else:
            tensor = torch.as_tensor(team_context, dtype=torch.float32, device=self.device).view(-1)

        if tensor.numel() < TEAM_CONTEXT_DIM:
            padded = torch.zeros(TEAM_CONTEXT_DIM, dtype=torch.float32, device=self.device)
            padded[: tensor.numel()] = tensor
            tensor = padded
        elif tensor.numel() > TEAM_CONTEXT_DIM:
            tensor = tensor[:TEAM_CONTEXT_DIM]

        return torch.clamp(tensor, -1.0, 1.0)

    def _estimate_value(
        self,
        obs_batch: torch.Tensor,
        team_context_batch: torch.Tensor | None = None,
        use_target: bool = False,
    ) -> torch.Tensor:
        critic = self.target_critic if use_target else self.critic
        team_critic = self.target_team_critic if use_target else self.team_critic
        value_mixer = self.target_value_mixer if use_target else self.value_mixer

        if team_context_batch is None:
            team_context_batch = (
                self._team_context_tensor().unsqueeze(0).repeat(obs_batch.shape[0], 1)
            )
        elif team_context_batch.dim() == 1:
            team_context_batch = team_context_batch.unsqueeze(0)

        local_value = critic(obs_batch).squeeze(-1)
        team_value = team_critic(team_context_batch).squeeze(-1)
        return value_mixer(local_value, team_value, team_context_batch)

    def generate_goal(self) -> tuple[float, float]:
        if not self.current_map or not self.current_odom:
            return None

        if self.train_enabled:
            # Finalize reward for previously issued action once a new state is available.
            self._finalize_pending_transition()

        obs_vec = self.extract_observation(self.current_map, self.current_odom)
        obs_tensor = self._obs_to_tensor(obs_vec)

        logits = torch.clamp(self.actor(obs_tensor), -MAX_LOGIT_MAGNITUDE, MAX_LOGIT_MAGNITUDE)
        policy = torch.softmax(logits, dim=-1)

        if self.train_enabled and len(self.replay_buffer) < REPLAY_WARMUP_SIZE:
            # Warm up replay with purely random actions before relying on policy.
            candidate_actions = np.random.permutation(ACTION_COUNT).tolist()
        else:
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
                sampled_order = torch.multinomial(
                    policy, num_samples=ACTION_COUNT, replacement=False
                )
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
            self.pending_team_context = self._team_context_tensor().detach().cpu()
            self.pending_action = int(action.item())
            # Capture "old" state exactly at action commit time.
            self.feedback_layer.save_state(self.current_map, self.current_odom)
        return goal

    def train_epoch(self):
        if not self.train_enabled:
            return None

        # Finalize potential pending transition before optimization.
        self._finalize_pending_transition()

        update_metrics: list[dict[str, float]] = []
        current_batch_size = 0
        avg_reward = 0.0

        if len(self.rollout_rewards) >= MIN_TRAINING_BATCH_SIZE:
            update_metrics.append(self._run_on_policy_update())
            current_batch_size = len(self.rollout_rewards)
            avg_reward = float(np.mean(self.rollout_raw_rewards))

        off_policy_updates = 0
        if len(self.replay_buffer) >= OFF_POLICY_BATCH_SIZE:
            for _ in range(OFF_POLICY_UPDATES_PER_EPOCH):
                update_metrics.append(self._run_off_policy_update())
                off_policy_updates += 1

        if not update_metrics:
            return None

        loss_mean = float(np.mean([m["loss"] for m in update_metrics]))
        policy_loss_mean = float(np.mean([m["policy_loss"] for m in update_metrics]))
        value_loss_mean = float(np.mean([m["value_loss"] for m in update_metrics]))
        entropy_mean = float(np.mean([m["entropy"] for m in update_metrics]))
        entropy_coef_mean = float(np.mean([m["entropy_coef"] for m in update_metrics]))
        if self.rollout_reward_components:
            coverage_gain_mean = float(
                np.mean([item.get("coverage_gain", 0.0) for item in self.rollout_reward_components])
            )
            frontier_gain_mean = float(
                np.mean([item.get("frontier_gain", 0.0) for item in self.rollout_reward_components])
            )
            overlap_growth_mean = float(
                np.mean(
                    [item.get("overlap_growth", 0.0) for item in self.rollout_reward_components]
                )
            )
            crowding_penalty_mean = float(
                np.mean(
                    [item.get("crowding_penalty", 0.0) for item in self.rollout_reward_components]
                )
            )
            redundancy_penalty_mean = float(
                np.mean(
                    [item.get("redundancy_penalty", 0.0) for item in self.rollout_reward_components]
                )
            )
            nearest_teammate_mean = float(
                np.mean(
                    [item.get("nearest_teammate", 0.0) for item in self.rollout_reward_components]
                )
            )
            team_context_mean = float(
                np.mean(
                    [item.get("team_context_mean", 0.0) for item in self.rollout_reward_components]
                )
            )
        else:
            coverage_gain_mean = 0.0
            frontier_gain_mean = 0.0
            overlap_growth_mean = 0.0
            crowding_penalty_mean = 0.0
            redundancy_penalty_mean = 0.0
            nearest_teammate_mean = 0.0
            team_context_mean = 0.0

        # Increment training step counter for exploration decay
        self.training_steps += 1

        metrics = {
            "loss": loss_mean,
            "policy_loss": policy_loss_mean,
            "value_loss": value_loss_mean,
            "entropy": entropy_mean,
            "entropy_coef": entropy_coef_mean,
            "avg_reward": avg_reward,
            "batch_size": current_batch_size,
            "training_steps": self.training_steps,
            "replay_size": int(len(self.replay_buffer)),
            "off_policy_updates": int(off_policy_updates),
            "coverage_gain": coverage_gain_mean,
            "frontier_gain": frontier_gain_mean,
            "overlap_growth": overlap_growth_mean,
            "crowding_penalty": crowding_penalty_mean,
            "redundancy_penalty": redundancy_penalty_mean,
            "nearest_teammate": nearest_teammate_mean,
            "team_context_mean": team_context_mean,
        }
        self.data_logger.log_batch(metrics)
        self._checkpoint(metrics)
        self._clear_rollout()
        return metrics

    def _run_on_policy_update(self) -> dict[str, float]:
        obs_batch = torch.stack(self.rollout_obs).to(self.device)
        team_context_batch = torch.stack(self.rollout_team_contexts).to(self.device)
        action_batch = torch.as_tensor(
            self.rollout_actions,
            dtype=torch.int64,
            device=self.device,
        )
        rewards = torch.tensor(self.rollout_rewards, dtype=torch.float32, device=self.device)

        logits = torch.clamp(self.actor(obs_batch), -MAX_LOGIT_MAGNITUDE, MAX_LOGIT_MAGNITUDE)
        dist = Categorical(logits=logits)
        log_probs = dist.log_prob(action_batch)
        entropies = dist.entropy()
        values: torch.Tensor = self._estimate_value(obs_batch, team_context_batch)

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
        value_loss: torch.Tensor = torch.nn.functional.smooth_l1_loss(values, returns)
        entropy_bonus: torch.Tensor = entropies.mean()
        entropy_coef: float = self._compute_adaptive_entropy_coef(entropy_bonus)

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
        self._soft_update(self.critic, self.target_critic, TARGET_CRITIC_TAU)
        self._soft_update(self.team_critic, self.target_team_critic, TEAM_CONTEXT_TAU)
        self._soft_update(self.value_mixer, self.target_value_mixer, TEAM_CONTEXT_TAU)

        return {
            "loss": float(loss.item()),
            "policy_loss": float(policy_loss.item()),
            "value_loss": float(value_loss.item()),
            "entropy": float(entropy_bonus.item()),
            "entropy_coef": float(entropy_coef),
        }

    def _run_off_policy_update(self) -> dict[str, float]:
        batch_size = min(OFF_POLICY_BATCH_SIZE, len(self.replay_buffer))
        sample_indices = np.random.choice(len(self.replay_buffer), size=batch_size, replace=False)
        sampled_transitions = [self.replay_buffer[int(i)] for i in sample_indices]

        obs_batch = torch.stack([transition[0] for transition in sampled_transitions]).to(
            self.device
        )
        action_batch = torch.as_tensor(
            [transition[1] for transition in sampled_transitions],
            dtype=torch.int64,
            device=self.device,
        )
        reward_batch = torch.as_tensor(
            [transition[2] for transition in sampled_transitions],
            dtype=torch.float32,
            device=self.device,
        )
        next_obs_batch = torch.stack([transition[4] for transition in sampled_transitions]).to(
            self.device
        )
        done_batch = torch.as_tensor(
            [transition[5] for transition in sampled_transitions],
            dtype=torch.float32,
            device=self.device,
        )
        team_context_batch = torch.stack([transition[6] for transition in sampled_transitions]).to(
            self.device
        )
        next_team_context_batch = torch.stack(
            [transition[7] for transition in sampled_transitions]
        ).to(self.device)

        with torch.no_grad():
            next_values = self._estimate_value(
                next_obs_batch,
                next_team_context_batch,
                use_target=True,
            )
            td_targets = reward_batch + GAMMA * (1.0 - done_batch) * next_values

        logits = torch.clamp(self.actor(obs_batch), -MAX_LOGIT_MAGNITUDE, MAX_LOGIT_MAGNITUDE)
        dist = Categorical(logits=logits)
        log_probs = dist.log_prob(action_batch)
        entropy_bonus = dist.entropy().mean()

        values = self._estimate_value(obs_batch, team_context_batch)
        advantages = td_targets - values
        if advantages.numel() > 1:
            advantages = (advantages - advantages.mean()) / (advantages.std(unbiased=False) + 1e-8)

        policy_loss: torch.Tensor = -(log_probs * advantages.detach()).mean()
        value_loss: torch.Tensor = torch.nn.functional.smooth_l1_loss(values, td_targets)
        entropy_coef: float = self._compute_adaptive_entropy_coef(entropy_bonus)
        loss: torch.Tensor = policy_loss + VALUE_COEF * value_loss - entropy_coef * entropy_bonus

        self.optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.parameters(), 1.0)
        self.optimizer.step()
        self._soft_update(self.critic, self.target_critic, TARGET_CRITIC_TAU)
        self._soft_update(self.team_critic, self.target_team_critic, TEAM_CONTEXT_TAU)
        self._soft_update(self.value_mixer, self.target_value_mixer, TEAM_CONTEXT_TAU)

        return {
            "loss": float(loss.item()),
            "policy_loss": float(policy_loss.item()),
            "value_loss": float(value_loss.item()),
            "entropy": float(entropy_bonus.item()),
            "entropy_coef": float(entropy_coef),
        }

    @staticmethod
    def _compute_adaptive_entropy_coef(entropy_bonus: torch.Tensor) -> float:
        entropy_value = float(entropy_bonus.detach().item())
        entropy_scale = 1.0
        if entropy_value > 1e-6:
            entropy_scale = max(0.5, min(2.0, TARGET_ENTROPY / (entropy_value + 1e-8)))
        return max(ENTROPY_COEF_MIN, ENTROPY_COEF * entropy_scale)

    def extract_observation(
        self,
        map: OccupancyGrid,
        odom: Odometry,
        patch_radius: int = PATCH_RADIUS,
    ) -> np.ndarray:
        local_patch = self.get_local_patch(map, odom, patch_size=patch_radius)
        transformed = self.transform_map(local_patch).astype(np.float32)

        # Convert multi-channel occupancy cues into a compact utility map for the actor.
        unexplored = transformed[:, :, 0]
        occupied = transformed[:, :, 1]
        overlap = transformed[:, :, 2]
        others = transformed[:, :, 4]
        frontier_mask = self._frontier_mask(local_patch)
        utility = unexplored + 0.6 * frontier_mask + 0.25 * overlap - 1.0 * occupied - 0.35 * others

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

    def _frontier_mask(self, local_patch: OccupancyGrid) -> np.ndarray:
        if local_patch is None or local_patch.data is None:
            return np.zeros((1, 1), dtype=np.float32)

        map_data = np.asarray(local_patch.data, dtype=np.int16).reshape(
            (local_patch.info.height, local_patch.info.width)
        )
        unknown = map_data < 0
        free = (map_data >= 0) & (map_data < 50)
        adjacent_free = np.zeros_like(free)
        adjacent_free[1:, :] |= free[:-1, :]
        adjacent_free[:-1, :] |= free[1:, :]
        adjacent_free[:, 1:] |= free[:, :-1]
        adjacent_free[:, :-1] |= free[:, 1:]
        frontier = unknown & adjacent_free
        return frontier.astype(np.float32)

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
        next_obs = self._obs_to_tensor(
            self.extract_observation(self.current_map, self.current_odom)
        )
        current_team_context = self._team_context_tensor().detach().cpu()
        pending_team_context = (
            self.pending_team_context.clone().detach()
            if self.pending_team_context is not None
            else current_team_context
        )
        reward = self.feedback_layer.calculate_reward(
            self.current_map,
            self.current_odom,
            local_map,
            team_context=current_team_context,
        )
        if not np.isfinite(reward):
            reward = 0.0
        normalized_reward = self._normalize_reward(reward)

        self.rollout_obs.append(self.pending_obs)
        self.rollout_actions.append(self.pending_action)
        self.rollout_rewards.append(float(normalized_reward))
        self.rollout_raw_rewards.append(float(reward))
        self.rollout_team_contexts.append(pending_team_context)
        self.rollout_reward_components.append(dict(self.feedback_layer.last_reward_components))
        self.replay_buffer.append(
            (
                self.pending_obs.clone().detach().cpu(),
                int(self.pending_action),
                float(normalized_reward),
                float(reward),
                next_obs.detach().cpu(),
                0.0,
                pending_team_context.clone().detach().cpu(),
                current_team_context.clone().detach().cpu(),
            )
        )

        self.pending_obs = None
        self.pending_action = None
        self.pending_team_context = None

    def _clear_rollout(self):
        self.rollout_obs.clear()
        self.rollout_actions.clear()
        self.rollout_rewards.clear()
        self.rollout_raw_rewards.clear()
        self.rollout_team_contexts.clear()
        self.rollout_reward_components.clear()

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
            self.load_state_dict(torch.load(path, map_location=self.device), strict=False)
            self.target_critic.load_state_dict(self.critic.state_dict())
            self.target_team_critic.load_state_dict(self.team_critic.state_dict())
            self.target_value_mixer.load_state_dict(self.value_mixer.state_dict())
            self.target_critic.to(self.device)
            self.target_team_critic.to(self.device)
            self.target_value_mixer.to(self.device)
            self.target_critic.eval()
            self.target_team_critic.eval()
            self.target_value_mixer.eval()
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
            self.load_state_dict(
                torch.load(path, map_location=self.device),
                strict=False,
            )
            self.target_critic.load_state_dict(self.critic.state_dict())
            self.target_team_critic.load_state_dict(self.team_critic.state_dict())
            self.target_value_mixer.load_state_dict(self.value_mixer.state_dict())
            self.target_critic.to(self.device)
            self.target_team_critic.to(self.device)
            self.target_value_mixer.to(self.device)
            self.target_critic.eval()
            self.target_team_critic.eval()
            self.target_value_mixer.eval()
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
        self.last_reward_components: dict[str, float] = {}
        self.reward_weight_names: tuple[str] = REWARD_WEIGHT_NAMES
        self.reward_weight_prior: np.ndarray = REWARD_WEIGHT_PRIOR.copy()
        self.reward_weight_bounds: np.ndarray = REWARD_WEIGHT_BOUNDS.copy()
        self.reward_weights: np.ndarray = self.reward_weight_prior.copy()
        self._reward_feature_history: deque[np.ndarray] = deque(maxlen=REWARD_BO_HISTORY_SIZE)
        self._reward_value_history: deque[float] = deque(maxlen=REWARD_BO_HISTORY_SIZE)
        self._reward_call_count: int = 0
        self._reward_rng = np.random.default_rng(REWARD_BO_RNG_SEED)

    def save_state(self, map: OccupancyGrid, odom: Odometry):
        # Keep snapshots so later callbacks cannot overwrite reward baseline semantics.
        self.current_state = copy.deepcopy(map)
        self.current_odom = copy.deepcopy(odom)

    def has_state(self) -> bool:
        return self.current_state is not None and self.current_odom is not None

    def _clip_reward_weights(self, weights: np.ndarray) -> np.ndarray:
        lower_bounds = self.reward_weight_bounds[:, 0]
        upper_bounds = self.reward_weight_bounds[:, 1]
        return np.clip(weights, lower_bounds, upper_bounds).astype(np.float32, copy=False)

    def _reward_feature_vector(
        self,
        information_gain: float,
        exploration_efficiency: float,
        coverage_gain: float,
        frontier_gain: float,
        overlap_growth: float,
        occupied_growth: float,
        team_redundancy_base: float,
        crowding_base: float,
        frontier_gap_base: float,
        stagnation_flag: float,
        nearest_teammate_bonus: float,
        local_unknown_ratio: float,
    ) -> np.ndarray:
        return np.array(
            [
                information_gain,
                exploration_efficiency,
                coverage_gain,
                frontier_gain,
                -overlap_growth,
                -occupied_growth,
                -team_redundancy_base,
                -crowding_base,
                -frontier_gap_base,
                -stagnation_flag,
                nearest_teammate_bonus,
                local_unknown_ratio,
            ],
            dtype=np.float32,
        )

    def _reward_model_score(self, weights: np.ndarray) -> float:
        if not self._reward_feature_history:
            return float("-inf")

        feature_matrix = np.vstack(self._reward_feature_history).astype(np.float32, copy=False)
        raw_rewards = feature_matrix @ weights.astype(np.float32, copy=False)
        transformed_rewards = np.tanh(raw_rewards / 6.0)
        mean_reward = float(np.mean(transformed_rewards))
        stability_penalty = float(np.std(transformed_rewards))
        distance_penalty = float(np.linalg.norm(weights - self.reward_weight_prior))
        prior_scale = float(max(np.linalg.norm(self.reward_weight_prior), 1.0))
        return (
            mean_reward
            - 0.15 * stability_penalty
            - REWARD_WEIGHT_PRIOR_PENALTY * (distance_penalty / prior_scale)
        )

    @staticmethod
    def _squared_exponential_kernel(
        x_a: np.ndarray,
        x_b: np.ndarray,
        length_scale: np.ndarray,
        signal_variance: float,
    ) -> np.ndarray:
        scaled_a = x_a / length_scale
        scaled_b = x_b / length_scale
        diff = scaled_a[:, None, :] - scaled_b[None, :, :]
        squared_distance = np.sum(diff * diff, axis=-1)
        return signal_variance * np.exp(-0.5 * squared_distance)

    def _gp_predict(
        self,
        x_train: np.ndarray,
        y_train: np.ndarray,
        x_test: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        if x_train.shape[0] == 0:
            return (
                np.zeros((x_test.shape[0],), dtype=np.float32),
                np.ones((x_test.shape[0],), dtype=np.float32),
            )

        y_mean = float(np.mean(y_train))
        centered_targets = y_train - y_mean
        length_scale = np.maximum(
            0.35 * (self.reward_weight_bounds[:, 1] - self.reward_weight_bounds[:, 0]),
            1e-3,
        ).astype(np.float32)
        signal_variance = float(max(np.var(y_train), 1e-3))
        kernel_train = self._squared_exponential_kernel(
            x_train,
            x_train,
            length_scale,
            signal_variance,
        )
        kernel_train.flat[:: kernel_train.shape[0] + 1] += REWARD_BO_NOISE

        try:
            chol = np.linalg.cholesky(kernel_train)
            alpha = np.linalg.solve(chol.T, np.linalg.solve(chol, centered_targets))
            kernel_cross = self._squared_exponential_kernel(
                x_train,
                x_test,
                length_scale,
                signal_variance,
            )
            mean = y_mean + kernel_cross.T @ alpha
            v = np.linalg.solve(chol, kernel_cross)
            kernel_test = self._squared_exponential_kernel(
                x_test,
                x_test,
                length_scale,
                signal_variance,
            )
            variance = np.clip(np.diag(kernel_test) - np.sum(v * v, axis=0), 1e-8, None)
            return mean.astype(np.float32, copy=False), np.sqrt(variance).astype(
                np.float32, copy=False
            )
        except np.linalg.LinAlgError:
            fallback_mean = np.full((x_test.shape[0],), y_mean, dtype=np.float32)
            fallback_std = np.full((x_test.shape[0],), np.std(y_train) + 1e-3, dtype=np.float32)
            return fallback_mean, fallback_std

    def _sample_reward_weight_candidates(self, candidate_count: int) -> np.ndarray:
        lower_bounds = self.reward_weight_bounds[:, 0]
        upper_bounds = self.reward_weight_bounds[:, 1]
        span = np.maximum(upper_bounds - lower_bounds, 1e-3)
        center = self.reward_weights

        candidates: list[np.ndarray] = [self.reward_weight_prior.copy(), center.copy()]
        for _ in range(max(candidate_count - len(candidates), 0)):
            if self._reward_rng.random() < 0.5:
                sample = self._reward_rng.uniform(lower_bounds, upper_bounds)
            else:
                sample = center + self._reward_rng.normal(0.0, 0.2, size=center.shape) * span
            candidates.append(self._clip_reward_weights(np.asarray(sample, dtype=np.float32)))

        return np.vstack(candidates)

    def _maybe_optimize_reward_weights(self):
        if len(self._reward_feature_history) < REWARD_BO_WARMUP:
            return

        if self._reward_call_count % REWARD_BO_UPDATE_PERIOD != 0:
            return

        candidate_pool = self._sample_reward_weight_candidates(REWARD_BO_CANDIDATES)
        candidate_scores = np.asarray(
            [self._reward_model_score(candidate) for candidate in candidate_pool],
            dtype=np.float32,
        )

        x_train = candidate_pool.astype(np.float32, copy=False)
        y_train = candidate_scores.astype(np.float32, copy=False)

        proposal_pool = self._sample_reward_weight_candidates(REWARD_BO_POOL_SIZE)
        posterior_mean, posterior_std = self._gp_predict(x_train, y_train, proposal_pool)
        acquisition = posterior_mean + REWARD_BO_UCB * posterior_std
        best_index = int(np.argmax(acquisition))
        proposal = proposal_pool[best_index]
        proposal_score = float(self._reward_model_score(proposal))

        current_score = self._reward_model_score(self.reward_weights)
        if proposal_score <= current_score:
            return

        blended_weights = (
            1.0 - REWARD_WEIGHT_SMOOTHING
        ) * self.reward_weights + REWARD_WEIGHT_SMOOTHING * proposal
        self.reward_weights = self._clip_reward_weights(blended_weights)

    def calculate_reward(
        self,
        new_map: OccupancyGrid,
        new_odom: Odometry,
        local_map: OccupancyGrid = None,
        team_context: torch.Tensor | np.ndarray | None = None,
    ) -> float:
        if not self.has_state() or new_map is None or new_odom is None:
            return 0.0

        if local_map is None:
            local_map = self.network.get_local_patch(new_map, new_odom, PATCH_RADIUS)

        old_summary = summarize_map(self.current_state)
        new_summary = summarize_map(new_map)

        old_map_data: np.ndarray = np.array(self.current_state.data, dtype=np.int16).reshape(
            self.current_state.info.height, self.current_state.info.width
        )
        new_map_data: np.ndarray = np.array(new_map.data, dtype=np.int16).reshape(
            new_map.info.height, new_map.info.width
        )

        old_known: int = np.sum(old_map_data >= 0)
        new_known: int = np.sum(new_map_data >= 0)
        map_cells: float = float(max(new_map_data.size, 1))
        information_gain: float = (new_known - old_known) / map_cells

        coverage_gain = float(new_summary[0] - old_summary[0])
        frontier_gain = float(new_summary[6] - old_summary[6])
        overlap_growth = float(new_summary[4] - old_summary[4])
        occupied_growth = float(new_summary[3] - old_summary[3])

        old_position: np.array = np.array(
            [self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y],
            dtype=np.float32,
        )
        new_position: np.array = np.array(
            [new_odom.pose.pose.position.x, new_odom.pose.pose.position.y],
            dtype=np.float32,
        )
        distance_traveled: float = float(np.linalg.norm(new_position - old_position))

        travel_norm: float = max(distance_traveled, MIN_REWARD_DISTANCE_M)
        exploration_efficiency: float = information_gain / travel_norm
        total_reward: float = 3.0 * information_gain + 1.5 * exploration_efficiency

        team_vector = self.network._team_context_tensor(team_context).detach().cpu().numpy()
        teammate_pressure = float(team_vector[4])
        nearest_teammate = float(team_vector[5])
        spread_ratio = float(team_vector[6])

        local_map_data: np.ndarray = np.array(local_map.data, dtype=np.int16).reshape(
            local_map.info.height, local_map.info.width
        )
        local_cells: float = float(max(local_map_data.size, 1))
        local_overlap_ratio = float(
            np.sum((local_map_data >= 10) & (local_map_data < 100)) / local_cells
        )
        local_unknown_ratio = float(np.sum(local_map_data < 0) / local_cells)

        team_redundancy_base = local_overlap_ratio + max(0.0, -coverage_gain)
        crowding_base = teammate_pressure + max(0.0, 0.25 - spread_ratio)
        frontier_gap_base = max(0.0, 0.1 - frontier_gain)
        stagnation_flag = float(distance_traveled < (MIN_REWARD_DISTANCE_M * 0.5))

        weight_vector = self._clip_reward_weights(self.reward_weights)
        team_redundancy_penalty = weight_vector[6] * team_redundancy_base
        crowding_penalty = weight_vector[7] * crowding_base
        frontier_penalty = weight_vector[8] * frontier_gap_base
        stagnation_penalty = weight_vector[9] * stagnation_flag
        total_reward = float(
            weight_vector[0] * information_gain
            + weight_vector[1] * exploration_efficiency
            + weight_vector[2] * coverage_gain
            + weight_vector[3] * frontier_gain
            - weight_vector[4] * overlap_growth
            - weight_vector[5] * occupied_growth
            - team_redundancy_penalty
            - crowding_penalty
            - frontier_penalty
            - stagnation_penalty
        )

        if nearest_teammate > 0.0:
            total_reward += weight_vector[10] * min(nearest_teammate, 1.0)

        total_reward += weight_vector[11] * local_unknown_ratio

        reward_feature_vector = self._reward_feature_vector(
            information_gain=information_gain,
            exploration_efficiency=exploration_efficiency,
            coverage_gain=coverage_gain,
            frontier_gain=frontier_gain,
            overlap_growth=overlap_growth,
            occupied_growth=occupied_growth,
            team_redundancy_base=team_redundancy_base,
            crowding_base=crowding_base,
            frontier_gap_base=frontier_gap_base,
            stagnation_flag=stagnation_flag,
            nearest_teammate_bonus=min(nearest_teammate, 1.0) if nearest_teammate > 0.0 else 0.0,
            local_unknown_ratio=local_unknown_ratio,
        )

        self._reward_feature_history.append(reward_feature_vector)
        self._reward_value_history.append(float(total_reward))
        self._reward_call_count += 1
        self._maybe_optimize_reward_weights()

        self.last_reward_components = {
            "information_gain": float(information_gain),
            "coverage_gain": float(coverage_gain),
            "frontier_gain": float(frontier_gain),
            "overlap_growth": float(overlap_growth),
            "occupied_growth": float(occupied_growth),
            "exploration_efficiency": float(exploration_efficiency),
            "team_redundancy_penalty": float(team_redundancy_penalty),
            "crowding_penalty": float(crowding_penalty),
            "frontier_penalty": float(frontier_penalty),
            "stagnation_penalty": float(stagnation_penalty),
            "nearest_teammate": float(nearest_teammate),
            "team_context_mean": float(np.mean(team_vector)),
            "local_unknown_ratio": float(local_unknown_ratio),
            "local_overlap_ratio": float(local_overlap_ratio),
            "reward_weight_mean": float(np.mean(weight_vector)),
        }

        return float(total_reward)
