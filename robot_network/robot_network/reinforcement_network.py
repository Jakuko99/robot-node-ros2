import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from scipy.spatial import cKDTree
from math import exp, sqrt, ceil
from collections import deque
import numpy as np

from nav_msgs.msg import OccupancyGrid
from std_srvs.srv._trigger import Trigger_Request, Trigger_Response
from robot_network.robot_watcher import RobotWatcher

# ----- Hyperparameters -----
BATCH_SIZE = 32
NUM_EPOCHS = 10
LEARNING_RATE = 0.0003
GAMMA = 0.99  # Discount factor for PPO
LAMBDA = 0.95  # GAE parameter
CLIP_EPSILON = 0.2  # PPO clipping parameter
VALUE_COEF = 0.5  # Value loss coefficient
ENTROPY_COEF = 0.01  # Entropy bonus coefficient
MAX_REGENERATION_ATTEMPTS = 3  # Maximum attempts to regenerate output if reward is zero
NO_PROGRESS_PENALTY = -2.0  # Penalty for no exploration progress

# ----- Frontier Detection Parameters -----
MIN_SAFETY_MARGIN = 0.5
MIN_CLUSTER_SIZE = 5

# ----- Transformer Parameters -----
D_MODEL = 256  # Embedding dimension
N_HEADS = 8  # Number of attention heads
N_LAYERS = 3  # Number of transformer layers
D_FF = 1024  # Feedforward dimension
MAX_ROBOTS = 20  # Maximum number of robots
MAX_FRONTIERS = 100  # Maximum number of frontiers


class PositionalEncoding(nn.Module):
    """
    3D positional embedding for robot and frontier positions.
    Encodes x, y coordinates into high-dimensional space.
    """

    def __init__(self, d_model: int, max_len: int = 5000):
        super(PositionalEncoding, self).__init__()
        self.d_model = d_model

        # Create positional encoding matrix
        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(
            torch.arange(0, d_model, 2).float() * (-np.log(10000.0) / d_model)
        )

        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        self.register_buffer("pe", pe)

    def forward(self, x: torch.Tensor, positions: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: Tensor of shape (batch, seq_len, d_model)
            positions: Tensor of shape (batch, seq_len, 2) containing (x, y) coordinates

        Returns:
            Tensor with positional encoding added
        """
        batch_size, seq_len, _ = x.shape

        # Normalize positions to [0, max_len)
        pos_normalized = (positions * 100).long().clamp(0, self.pe.size(0) - 1)

        # Get positional encodings
        pos_enc = self.pe[pos_normalized[:, :, 0]] + self.pe[pos_normalized[:, :, 1]]

        return x + pos_enc


class TransformerEncoder(nn.Module):
    """
    Transformer encoder for processing robot and frontier sequences.
    Processes layers sequentially as mentioned in notes.
    """

    def __init__(self, d_model: int, n_heads: int, n_layers: int, d_ff: int):
        super(TransformerEncoder, self).__init__()

        self.layers = nn.ModuleList(
            [
                nn.TransformerEncoderLayer(
                    d_model=d_model,
                    nhead=n_heads,
                    dim_feedforward=d_ff,
                    dropout=0.1,
                    batch_first=True,
                )
                for _ in range(n_layers)
            ]
        )

        self.norm = nn.LayerNorm(d_model)

    def forward(self, x: torch.Tensor, mask: torch.Tensor = None) -> torch.Tensor:
        """
        Process sequences through transformer layers sequentially.
        Each layer looks at the output of the previous layer.

        Args:
            x: Input tensor (batch, seq_len, d_model)
            mask: Attention mask (optional)

        Returns:
            Encoded tensor (batch, seq_len, d_model)
        """
        for layer in self.layers:
            x = layer(x, src_key_padding_mask=mask)

        return self.norm(x)


class AttentionCriterion(nn.Module):
    """
    Attention-based criterion for robot-frontier assignment.
    Uses cross-attention to determine which frontier each robot should target.
    """

    def __init__(self, d_model: int, n_heads: int):
        super(AttentionCriterion, self).__init__()

        self.cross_attention = nn.MultiheadAttention(
            embed_dim=d_model, num_heads=n_heads, batch_first=True
        )

        self.fc = nn.Sequential(
            nn.Linear(d_model, d_model // 2),
            nn.ReLU(),
            nn.Linear(d_model // 2, 1),
        )

    def forward(
        self, robot_features: torch.Tensor, frontier_features: torch.Tensor
    ) -> torch.Tensor:
        """
        Compute attention scores between robots and frontiers.

        Args:
            robot_features: (batch, n_robots, d_model)
            frontier_features: (batch, n_frontiers, d_model)

        Returns:
            Assignment scores: (batch, n_robots, n_frontiers)
        """
        # Cross-attention: robots attend to frontiers
        attended_features, attention_weights = self.cross_attention(
            query=robot_features, key=frontier_features, value=frontier_features
        )

        # Compute assignment scores
        scores = self.fc(attended_features)  # (batch, n_robots, 1)

        return attention_weights  # (batch, n_robots, n_frontiers)


class PPOActor(nn.Module):
    """
    PPO Actor network that outputs goal assignments for robots.
    """

    def __init__(self, d_model: int):
        super(PPOActor, self).__init__()

        self.fc = nn.Sequential(
            nn.Linear(d_model, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 2),  # Output: (x, y) goal position
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: Robot features (batch, n_robots, d_model)

        Returns:
            Goal positions (batch, n_robots, 2)
        """
        return self.fc(x)


class PPOCritic(nn.Module):
    """
    PPO Critic network that estimates state value.
    """

    def __init__(self, d_model: int):
        super(PPOCritic, self).__init__()

        self.fc = nn.Sequential(
            nn.Linear(d_model, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 1),  # Output: state value
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: State features (batch, d_model)

        Returns:
            State value (batch, 1)
        """
        return self.fc(x)


class ReinforcementSwarmNetwork(nn.Module):
    """
    Reinforcement Learning network using Transformer + PPO for robot swarm coordination.

    Based on notes.txt:
    - Transformer architecture with sequential layer processing
    - 3D embedding using positional encoding
    - Goal assignment with backtracking prevention
    - PPO (Proximal Policy Optimization) method
    - Attention criterion for frontier assignment

    Network inputs:
    - Robot positions (x, y, theta)
    - Global merged occupancy grid
    - Frontier positions from unexplored areas

    Network outputs:
    - Goal assignments for each robot (x, y)
    - State value estimate
    """

    def __init__(
        self,
        train: bool = False,
        model_path: str = "",
        trained_model_path: str = "",
        parent=None,
    ):
        super(ReinforcementSwarmNetwork, self).__init__()

        self._train: bool = train
        self.model_path: str = model_path
        self.trained_model_path: str = trained_model_path
        self.previous_state: OccupancyGrid = None
        self.previous_goals: dict[str, tuple[float, float]] = {}  # Track previous goals
        self.robots: dict[str, RobotWatcher] = {}
        self.parent = parent

        # Embedding layers
        self.robot_embedding = nn.Linear(3, D_MODEL)  # (x, y, theta) -> d_model
        self.frontier_embedding = nn.Linear(2, D_MODEL)  # (x, y) -> d_model

        # Positional encoding
        self.pos_encoding = PositionalEncoding(D_MODEL)

        # Transformer encoder
        self.robot_encoder = TransformerEncoder(D_MODEL, N_HEADS, N_LAYERS, D_FF)
        self.frontier_encoder = TransformerEncoder(D_MODEL, N_HEADS, N_LAYERS, D_FF)

        # Attention-based assignment
        self.attention_criterion = AttentionCriterion(D_MODEL, N_HEADS)

        # PPO Actor-Critic
        self.actor = PPOActor(D_MODEL)
        self.critic = PPOCritic(D_MODEL)

        # Optimizers
        self.actor_optimizer = optim.Adam(
            list(self.robot_embedding.parameters())
            + list(self.frontier_embedding.parameters())
            + list(self.pos_encoding.parameters())
            + list(self.robot_encoder.parameters())
            + list(self.frontier_encoder.parameters())
            + list(self.attention_criterion.parameters())
            + list(self.actor.parameters()),
            lr=LEARNING_RATE,
        )
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=LEARNING_RATE)

        # Experience buffer
        self.states = []
        self.actions = []
        self.rewards = []
        self.values = []
        self.log_probs = []
        self.dones = []

        # Training statistics
        self.consecutive_zero_rewards = 0
        self.total_training_reward = 0.0
        self.training_steps = 0

        # Load pretrained model if provided
        if self.trained_model_path and self.trained_model_path != "":
            try:
                self.load_model(self.trained_model_path)
                if parent:
                    parent.get_logger().info(
                        f"Loaded pretrained model from {self.trained_model_path}"
                    )
            except Exception as e:
                if parent:
                    parent.get_logger().warn(
                        f"Failed to load model from {self.trained_model_path}: {str(e)}"
                    )

    def save_model(
        self, request: Trigger_Request, response: Trigger_Response
    ) -> Trigger_Response:
        try:
            torch.save(
                {
                    "robot_embedding": self.robot_embedding.state_dict(),
                    "frontier_embedding": self.frontier_embedding.state_dict(),
                    "robot_encoder": self.robot_encoder.state_dict(),
                    "frontier_encoder": self.frontier_encoder.state_dict(),
                    "attention_criterion": self.attention_criterion.state_dict(),
                    "actor": self.actor.state_dict(),
                    "critic": self.critic.state_dict(),
                },
                self.model_path,
            )
        except Exception as e:
            response.success = False
            response.message = f"Failed to save model: {str(e)}"
            return response

        response.success = True
        response.message = f"Model saved to {self.model_path}"
        return response

    def load_model(self, path: str):
        """Load pretrained model weights."""
        checkpoint = torch.load(path)
        self.robot_embedding.load_state_dict(checkpoint["robot_embedding"])
        self.frontier_embedding.load_state_dict(checkpoint["frontier_embedding"])
        self.robot_encoder.load_state_dict(checkpoint["robot_encoder"])
        self.frontier_encoder.load_state_dict(checkpoint["frontier_encoder"])
        self.attention_criterion.load_state_dict(checkpoint["attention_criterion"])
        self.actor.load_state_dict(checkpoint["actor"])
        self.critic.load_state_dict(checkpoint["critic"])

    def add_robot(self, robot_name: str, robot_watcher: RobotWatcher):
        if robot_name not in self.robots:
            self.robots[robot_name] = robot_watcher
            self.previous_goals[robot_name] = None

    def forward(
        self,
        robot_positions: torch.Tensor,
        frontier_positions: torch.Tensor,
        robot_mask: torch.Tensor = None,
        frontier_mask: torch.Tensor = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Forward pass through the network.

        Args:
            robot_positions: (batch, n_robots, 3) - (x, y, theta)
            frontier_positions: (batch, n_frontiers, 2) - (x, y)
            robot_mask: (batch, n_robots) - mask for padded robots
            frontier_mask: (batch, n_frontiers) - mask for padded frontiers

        Returns:
            goal_positions: (batch, n_robots, 2)
            state_value: (batch, 1)
            attention_weights: (batch, n_robots, n_frontiers)
        """
        batch_size = robot_positions.shape[0]

        # Embed robots and frontiers
        robot_embedded = self.robot_embedding(
            robot_positions
        )  # (batch, n_robots, d_model)
        frontier_embedded = self.frontier_embedding(
            frontier_positions
        )  # (batch, n_frontiers, d_model)

        # Add positional encoding
        robot_pos_2d = robot_positions[:, :, :2]  # (x, y) only
        robot_embedded = self.pos_encoding(robot_embedded, robot_pos_2d)
        frontier_embedded = self.pos_encoding(frontier_embedded, frontier_positions)

        # Encode through transformers (sequential layer processing)
        robot_features = self.robot_encoder(robot_embedded, robot_mask)
        frontier_features = self.frontier_encoder(frontier_embedded, frontier_mask)

        # Attention-based frontier assignment
        attention_weights = self.attention_criterion(robot_features, frontier_features)

        # Select frontiers for each robot based on attention
        # (batch, n_robots, n_frontiers) @ (batch, n_frontiers, 2) -> (batch, n_robots, 2)
        goal_positions = torch.bmm(attention_weights, frontier_positions)

        # Compute state value (use mean of robot features as state representation)
        state_repr = robot_features.mean(dim=1)  # (batch, d_model)
        state_value = self.critic(state_repr)

        return goal_positions, state_value, attention_weights

    def select_action(
        self,
        robot_positions: np.ndarray,
        frontiers: list[tuple[float, float]],
        add_exploration_noise: bool = False,
    ) -> tuple[list[tuple[float, float]], torch.Tensor, torch.Tensor]:
        """
        Select actions (goals) for robots with backtracking prevention.

        Args:
            robot_positions: List of (x, y, theta) for each robot
            frontiers: List of frontier centroids (x, y)
            add_exploration_noise: If True, add noise to encourage exploration (used when regenerating)

        Returns:
            goals: List of (x, y) goals for each robot
            log_prob: Log probability of the action
            value: State value estimate
        """
        if len(robot_positions) == 0 or len(frontiers) == 0:
            return [], None, None

        # Prepare tensors
        robot_tensor = torch.tensor(robot_positions, dtype=torch.float32).unsqueeze(0)
        frontier_tensor = torch.tensor(frontiers, dtype=torch.float32).unsqueeze(0)

        # Forward pass
        with torch.no_grad() if not self._train else torch.enable_grad():
            goal_positions, state_value, attention_weights = self.forward(
                robot_tensor, frontier_tensor
            )

            # Add exploration noise if regenerating due to zero reward
            if add_exploration_noise:
                noise = torch.randn_like(goal_positions) * 0.5
                goal_positions = goal_positions + noise

        # Convert to list of goals
        goals = goal_positions[0].detach().numpy().tolist()

        # Backtracking prevention: check if goal is too close to previous goal
        robot_names = list(self.robots.keys())
        for i, robot_name in enumerate(robot_names[: len(goals)]):
            prev_goal = self.previous_goals.get(robot_name)
            if prev_goal is not None:
                current_goal = goals[i]
                distance = sqrt(
                    (current_goal[0] - prev_goal[0]) ** 2
                    + (current_goal[1] - prev_goal[1]) ** 2
                )

                # If new goal is too close to previous goal, find alternative
                if distance < 0.5:  # Threshold for backtracking
                    # Find next best frontier from attention weights
                    attn = attention_weights[0, i].detach().numpy()
                    sorted_indices = np.argsort(attn)[::-1]

                    for idx in sorted_indices[1:]:  # Skip the top choice
                        alternative_goal = frontiers[idx]
                        alt_distance = sqrt(
                            (alternative_goal[0] - prev_goal[0]) ** 2
                            + (alternative_goal[1] - prev_goal[1]) ** 2
                        )
                        if alt_distance >= 0.5:
                            goals[i] = alternative_goal
                            break

            # Update previous goal
            self.previous_goals[robot_name] = tuple(goals[i])

        # Compute log probability (simplified for continuous actions)
        log_prob = -torch.nn.functional.mse_loss(
            goal_positions, goal_positions.detach()
        )

        return goals, log_prob, state_value

    def compute_gae(
        self, rewards: list[float], values: list[torch.Tensor], dones: list[bool]
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """
        Compute Generalized Advantage Estimation (GAE).

        Args:
            rewards: List of rewards
            values: List of value estimates
            dones: List of done flags

        Returns:
            advantages: GAE advantages
            returns: Discounted returns
        """
        advantages = []
        gae = 0

        # Convert values to scalars, handling None values
        values_scalar = [v.item() if v is not None else 0.0 for v in values]
        next_value = 0

        for t in reversed(range(len(rewards))):
            delta = rewards[t] + GAMMA * next_value * (1 - dones[t]) - values_scalar[t]
            gae = delta + GAMMA * LAMBDA * (1 - dones[t]) * gae
            advantages.insert(0, gae)
            next_value = values_scalar[t]

        advantages = torch.tensor(advantages, dtype=torch.float32)
        returns = advantages + torch.tensor(values_scalar, dtype=torch.float32)

        return advantages, returns

    def train_network(self, grid: OccupancyGrid) -> float:
        """
        Train the network using PPO algorithm or run inference.

        Args:
            grid: Current occupancy grid

        Returns:
            reward: Exploration reward (training) or None (inference)
        """
        if self.previous_state is None:
            self.previous_state = grid
            return None

        # Get frontiers
        frontiers = self._get_frontiers(grid)

        if len(frontiers) == 0 or len(self.robots) == 0:
            self.previous_state = grid
            return None

        # Prepare robot positions
        robot_positions = []
        robot_names = []
        for name, watcher in self.robots.items():
            robot_positions.append([watcher.x, watcher.y, watcher.theta])
            robot_names.append(name)

        # Attempt to generate actions with good reward
        best_goals = None
        best_reward = float("-inf")
        best_log_prob = None
        best_value = None

        for attempt in range(MAX_REGENERATION_ATTEMPTS):
            # Add exploration noise for regeneration attempts after the first
            add_noise = attempt > 0
            goals, log_prob, value = self.select_action(
                robot_positions, frontiers, add_noise
            )

            if goals is None:
                continue

            # Calculate potential reward for these goals
            if self._train:
                # Calculate exploration reward
                current_explored = np.sum(np.array(grid.data) != -1)
                previous_explored = np.sum(np.array(self.previous_state.data) != -1)
                exploration_progress = current_explored - previous_explored

                # Reward based on percentage increase in explored area
                total_cells = len(grid.data)
                reward = float(exploration_progress) / max(total_cells, 1) * 100.0

                # Apply negative reward for no progress
                if exploration_progress <= 0:
                    reward = NO_PROGRESS_PENALTY
                    # Increase penalty for consecutive zero rewards
                    reward -= self.consecutive_zero_rewards * 0.5

                # Additional reward shaping for better exploration
                reward += self._compute_potential_reward(
                    robot_positions, goals, frontiers, grid
                )

                # Keep track of best attempt
                if reward > best_reward:
                    best_reward = reward
                    best_goals = goals
                    best_log_prob = log_prob
                    best_value = value

                # If we got a positive reward, use these goals
                if reward > 0:
                    break
            else:
                # In inference mode, use first attempt
                best_goals = goals
                best_log_prob = log_prob
                best_value = value
                break

        # Use the best goals found
        goals = best_goals
        log_prob = best_log_prob
        value = best_value
        reward = best_reward if self._train else None

        if goals is None:
            self.previous_state = grid
            return None

        # Publish goals to robots
        for i, robot_name in enumerate(robot_names):
            if i < len(goals):
                watcher = self.robots[robot_name]
                goal = goals[i]

                if not watcher.is_moving:
                    transformed_action = self.parent.apply_transform(
                        watcher.namespace, list(goal)
                    )
                    watcher.publish_goal(*transformed_action)
                    self.parent.publish_point(*transformed_action)

        # Training mode: store experience and update policy
        if self._train:
            # Track consecutive zero rewards
            if reward <= 0:
                self.consecutive_zero_rewards += 1
            else:
                self.consecutive_zero_rewards = 0

            # Update training statistics
            self.total_training_reward += reward
            self.training_steps += 1

            # Log progress periodically
            if self.parent and self.training_steps % 10 == 0:
                avg_reward = self.total_training_reward / self.training_steps
                self.parent.get_logger().info(
                    f"Training step {self.training_steps}: reward={reward:.3f}, "
                    f"avg_reward={avg_reward:.3f}, consecutive_zeros={self.consecutive_zero_rewards}"
                )

            # Store experience (detach tensors to avoid graph reuse)
            self.states.append((robot_positions, frontiers))
            self.actions.append(goals)
            self.rewards.append(reward)
            self.values.append(value.detach() if value is not None else None)
            if log_prob is not None:
                self.log_probs.append(log_prob.detach())
            self.dones.append(False)

            # Train after collecting enough experiences
            if len(self.rewards) >= BATCH_SIZE:
                self._update_policy()

            self.previous_state = grid
            return reward
        else:
            # Inference mode: just return None
            self.previous_state = grid
            return None

    def _update_policy(self):
        """
        Update policy using PPO algorithm.
        """
        if len(self.rewards) == 0:
            return

        # Compute advantages and returns
        advantages, returns = self.compute_gae(self.rewards, self.values, self.dones)

        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        # PPO update
        for epoch in range(NUM_EPOCHS):
            # For each experience in the buffer
            for i in range(len(self.states)):
                robot_positions, frontiers = self.states[i]
                old_action = self.actions[i]

                # Prepare tensors (fresh tensors for each forward pass)
                robot_tensor = torch.tensor(
                    robot_positions, dtype=torch.float32
                ).unsqueeze(0)
                frontier_tensor = torch.tensor(
                    frontiers, dtype=torch.float32
                ).unsqueeze(0)
                old_action_tensor = torch.tensor(
                    old_action, dtype=torch.float32
                ).unsqueeze(0)

                # Forward pass (creates fresh computation graph)
                new_action, new_value, _ = self.forward(robot_tensor, frontier_tensor)

                # Compute ratios for PPO
                new_log_prob = -F.mse_loss(new_action, old_action_tensor)

                # Use detached old_log_prob (no graph attached)
                if i < len(self.log_probs):
                    old_log_prob = self.log_probs[i]  # Already detached when stored
                else:
                    old_log_prob = new_log_prob.detach()  # Detach if not available

                ratio = torch.exp(new_log_prob - old_log_prob)

                # PPO clipped objective
                surr1 = ratio * advantages[i]
                surr2 = (
                    torch.clamp(ratio, 1.0 - CLIP_EPSILON, 1.0 + CLIP_EPSILON)
                    * advantages[i]
                )
                actor_loss = -torch.min(surr1, surr2).mean()

                # Value loss
                value_loss = F.mse_loss(new_value.squeeze(), returns[i])

                # Entropy bonus (encourage exploration)
                entropy = -new_log_prob
                entropy_loss = -ENTROPY_COEF * entropy

                # Total loss
                total_loss = actor_loss + VALUE_COEF * value_loss + entropy_loss

                # Update networks
                self.actor_optimizer.zero_grad()
                self.critic_optimizer.zero_grad()
                total_loss.backward()  # Fresh graph each time, safe to backward
                torch.nn.utils.clip_grad_norm_(self.parameters(), max_norm=0.5)
                self.actor_optimizer.step()
                self.critic_optimizer.step()

        # Clear experience buffer
        self.states.clear()
        self.actions.clear()
        self.rewards.clear()
        self.values.clear()
        self.log_probs.clear()
        self.dones.clear()

    def reward_criterion(
        self,
        position: tuple[float, float],
        state: OccupancyGrid,
        action: tuple[float, float],
        next_state: OccupancyGrid,
    ) -> float:
        """
        Compute reward based on exploration progress.

        Args:
            position: Robot position (x, y)
            state: Previous occupancy grid
            action: Selected goal (x, y)
            next_state: Current occupancy grid

        Returns:
            reward: Exploration reward (negative if no progress)
        """
        reward = 0.0

        # Calculate exploration percentage increase
        before_explored = np.sum(np.array(state.data) != -1)
        after_explored = np.sum(np.array(next_state.data) != -1)
        total_cells = len(state.data)

        exploration_increase = (after_explored - before_explored) / max(total_cells, 1)

        # Strong penalty for no progress, reward for exploration
        if exploration_increase > 0:
            reward += exploration_increase * 100.0  # Scale to reasonable range
        else:
            # Apply negative reward for no progress
            reward += NO_PROGRESS_PENALTY

        # Additional penalty if exploration is decreasing (should not happen but just in case)
        if exploration_increase < 0:
            reward += exploration_increase * 50.0  # Additional penalty

        # Frontier proximity reward (only if making progress)
        if exploration_increase > 0:
            frontiers = self._get_frontiers(state)
            if frontiers:
                fx, fy = zip(*frontiers)
                fx = np.array(fx)
                fy = np.array(fy)

                gx, gy = action
                dists = np.sqrt((fx - gx) ** 2 + (fy - gy) ** 2)
                min_dist = np.min(dists)

                sigma = 5.0
                reward += exp(-min_dist / sigma)

        # Frontier reduction reward (more frontiers explored is good)
        frontier_reduction = self._count_frontiers(state) - self._count_frontiers(
            next_state
        )
        if frontier_reduction > 0:
            reward += 0.5 * frontier_reduction
        elif frontier_reduction < 0:
            # Penalty if frontiers increased (might indicate backtracking)
            reward -= 0.3 * abs(frontier_reduction)

        # Motion cost penalty (encourage efficient movement)
        px, py = position
        ax, ay = action
        travel_cost = sqrt((ax - px) ** 2 + (ay - py) ** 2)
        reward -= 0.1 * travel_cost

        return reward

    def _count_frontiers(
        self, grid: OccupancyGrid, cluster_distance: float = 0.5
    ) -> int:
        """Count number of frontier clusters in the map."""
        if not grid:
            return 0

        width = int(grid.info.width)
        height = int(grid.info.height)
        res = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        grid_data = np.asarray(grid.data, dtype=np.int16).reshape((height, width))

        # Step 1: frontier cell detection (vectorized)
        free = grid_data == 0
        unknown = grid_data == -1
        occupied = grid_data > 50

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
            return 0

        # Grid → world coordinates
        ys, xs = np.nonzero(frontier_mask)
        wx = origin_x + (xs + 0.5) * res
        wy = origin_y + (ys + 0.5) * res

        frontier_cells = np.column_stack((xs, ys, wx, wy))

        # Step 2: safety check (vectorized per cell neighborhood)
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
            return 0

        # Step 3: clustering using KD-Tree
        points = safe_frontiers[:, 2:4]  # world coords only
        tree = cKDTree(points)

        visited = np.zeros(len(points), dtype=bool)
        frontier_count = 0

        for i in range(len(points)):
            if visited[i]:
                continue

            queue = deque([i])
            visited[i] = True
            cluster_size = 0

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
        self, grid: OccupancyGrid, cluster_distance: float = 0.5
    ) -> list[tuple[float, float]]:
        """
        Returns a list of frontier centroids [(x, y), ...] from the occupancy grid.
        """
        if grid is None:
            return []

        width = int(grid.info.width)
        height = int(grid.info.height)
        res = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        grid_data = np.asarray(grid.data, dtype=np.int16).reshape((height, width))

        free = grid_data == 0
        unknown = grid_data == -1
        occupied = grid_data > 50

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

    def _compute_potential_reward(
        self,
        robot_positions: list[list[float]],
        goals: list[tuple[float, float]],
        frontiers: list[tuple[float, float]],
        grid: OccupancyGrid,
    ) -> float:
        """
        Compute potential-based reward shaping to encourage better exploration.

        Args:
            robot_positions: Current robot positions
            goals: Selected goals for robots
            frontiers: Available frontier centroids
            grid: Current occupancy grid

        Returns:
            Shaped reward value
        """
        reward = 0.0

        # Reward for goal diversity (robots should spread out)
        if len(goals) > 1:
            goal_distances = []
            for i in range(len(goals)):
                for j in range(i + 1, len(goals)):
                    dist = sqrt(
                        (goals[i][0] - goals[j][0]) ** 2
                        + (goals[i][1] - goals[j][1]) ** 2
                    )
                    goal_distances.append(dist)
            if goal_distances:
                avg_dist = np.mean(goal_distances)
                reward += avg_dist * 0.1  # Encourage spreading out

        # Reward for targeting far frontiers (encourage exploration)
        for robot_pos, goal in zip(robot_positions, goals):
            dist_to_goal = sqrt(
                (goal[0] - robot_pos[0]) ** 2 + (goal[1] - robot_pos[1]) ** 2
            )
            # Small reward for distance (encourages exploring further areas)
            reward += min(dist_to_goal * 0.05, 0.5)

        # Penalty if multiple robots target the same frontier
        goal_set = set()
        duplicates = 0
        for goal in goals:
            goal_tuple = (round(goal[0], 2), round(goal[1], 2))
            if goal_tuple in goal_set:
                duplicates += 1
            goal_set.add(goal_tuple)
        reward -= duplicates * 0.5

        return reward
