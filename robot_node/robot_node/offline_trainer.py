#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch.distributions import Categorical

from robot_node.decision_network import (
    ACTION_COUNT,
    ENTROPY_COEF,
    OBS_DIM,
    REWARD_NORMALIZATION_CLIP,
    REWARD_NORMALIZATION_EPS,
    VALUE_COEF,
    DecisionNetwork,
)


@dataclass
class OfflineTransition:
    observation: np.ndarray
    action: int
    reward: float
    done: bool
    episode_id: int
    step_id: int


@dataclass
class OfflineDataset:
    observations: torch.Tensor
    actions: torch.Tensor
    returns: torch.Tensor
    normalized_rewards: torch.Tensor
    rewards: torch.Tensor


@dataclass
class EpochMetrics:
    epoch: int
    loss: float
    policy_loss: float
    value_loss: float
    entropy: float
    avg_reward: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Offline trainer for robot_node DecisionNetwork using DataCollector exports.",
    )
    parser.add_argument(
        "--dataset",
        nargs="+",
        required=True,
        help="One or more dataset files (.jsonl or .json), or directories containing them.",
    )
    parser.add_argument(
        "--output-model",
        default="export/offline_trained_model.pth",
        help="Path where the trained model state_dict will be written.",
    )
    parser.add_argument("--epochs", type=int, default=40, help="Number of training epochs.")
    parser.add_argument("--batch-size", type=int, default=128, help="Mini-batch size.")
    parser.add_argument(
        "--gamma",
        type=float,
        default=0.99,
        help="Discount factor used to compute Monte Carlo returns.",
    )
    parser.add_argument(
        "--learning-rate",
        type=float,
        default=3e-4,
        help="Learning rate override for offline optimization.",
    )
    parser.add_argument(
        "--value-coef",
        type=float,
        default=VALUE_COEF,
        help="Value loss coefficient.",
    )
    parser.add_argument(
        "--entropy-coef",
        type=float,
        default=ENTROPY_COEF,
        help="Entropy bonus coefficient.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=7,
        help="Random seed for reproducibility.",
    )
    parser.add_argument(
        "--checkpoint-dir",
        default="export/checkpoints",
        help="Directory used for periodic and best checkpoints.",
    )
    parser.add_argument(
        "--checkpoint-every",
        type=int,
        default=5,
        help="Save a periodic checkpoint every N epochs.",
    )
    return parser.parse_args()


def expand_dataset_paths(raw_paths: list[str]) -> list[Path]:
    output: list[Path] = []
    for raw in raw_paths:
        path = Path(raw)
        if path.is_dir():
            output.extend(sorted(path.glob("*.jsonl")))
            output.extend(sorted(path.glob("*.json")))
            continue

        if path.is_file():
            output.append(path)

    unique_paths = sorted(set(output))
    if not unique_paths:
        raise FileNotFoundError(
            "No dataset files found. Pass valid .jsonl/.json files or directories."
        )
    return unique_paths


def iter_records(path: Path):
    if path.suffix.lower() == ".jsonl":
        with path.open("r", encoding="utf-8") as file_handle:
            for line in file_handle:
                line = line.strip()
                if not line:
                    continue
                yield json.loads(line)
        return

    if path.suffix.lower() == ".json":
        with path.open("r", encoding="utf-8") as file_handle:
            data = json.load(file_handle)
        if isinstance(data, list):
            for item in data:
                if isinstance(item, dict):
                    yield item
        return


def ensure_obs_dim(observation: Any) -> np.ndarray | None:
    try:
        obs = np.asarray(observation, dtype=np.float32).reshape(-1)
    except Exception:
        return None

    if obs.size == 0:
        return None

    if obs.size < OBS_DIM:
        padded = np.zeros((OBS_DIM,), dtype=np.float32)
        padded[: obs.size] = obs
        return padded

    if obs.size > OBS_DIM:
        return obs[:OBS_DIM]

    return obs


def infer_action_from_goal(record: dict[str, Any]) -> int | None:
    goal = record.get("goal")
    odom = record.get("odom")
    if not isinstance(goal, dict) or not isinstance(odom, dict):
        return None

    if "x" not in goal or "y" not in goal or "x" not in odom or "y" not in odom:
        return None

    dx = float(goal["x"]) - float(odom["x"])
    dy = float(goal["y"]) - float(odom["y"])
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return None

    angle = math.atan2(dy, dx)
    if angle < 0.0:
        angle += 2.0 * math.pi

    sector = (2.0 * math.pi) / ACTION_COUNT
    action = int(round(angle / sector)) % ACTION_COUNT
    return action


def read_transitions(dataset_paths: list[Path]) -> list[OfflineTransition]:
    transitions: list[OfflineTransition] = []

    for path_index, path in enumerate(dataset_paths):
        for row_index, record in enumerate(iter_records(path)):
            obs = ensure_obs_dim(record.get("observation"))
            if obs is None:
                continue

            action_raw = record.get("action")
            action: int | None
            if isinstance(action_raw, int):
                action = action_raw
            else:
                action = infer_action_from_goal(record)

            if action is None or action < 0 or action >= ACTION_COUNT:
                continue

            reward_raw = record.get("reward", 0.0)
            reward = float(reward_raw) if reward_raw is not None else 0.0
            if not math.isfinite(reward):
                reward = 0.0
            reward = float(np.clip(reward, -REWARD_NORMALIZATION_CLIP, REWARD_NORMALIZATION_CLIP))

            done = bool(record.get("done", False))
            episode_id = int(record.get("episode_id", path_index))
            step_id = int(record.get("step_id", row_index))

            transitions.append(
                OfflineTransition(
                    observation=obs,
                    action=action,
                    reward=reward,
                    done=done,
                    episode_id=episode_id,
                    step_id=step_id,
                )
            )

    transitions.sort(key=lambda item: (item.episode_id, item.step_id))
    return transitions


def compute_discounted_returns(transitions: list[OfflineTransition], gamma: float) -> np.ndarray:
    rewards = np.asarray([t.reward for t in transitions], dtype=np.float32)
    dones = np.asarray([t.done for t in transitions], dtype=np.bool_)

    returns = np.zeros_like(rewards, dtype=np.float32)
    running_return = 0.0

    # Reverse pass with done-mask resets to keep episodes separated.
    for index in range(len(transitions) - 1, -1, -1):
        if dones[index]:
            running_return = rewards[index]
        else:
            running_return = rewards[index] + gamma * running_return
        returns[index] = running_return

    return returns


def build_dataset(transitions: list[OfflineTransition], gamma: float) -> OfflineDataset:
    observations = np.stack([t.observation for t in transitions], axis=0)
    actions = np.asarray([t.action for t in transitions], dtype=np.int64)
    rewards = np.asarray([t.reward for t in transitions], dtype=np.float32)

    reward_mean = float(rewards.mean())
    reward_std = float(rewards.std())
    normalized_rewards = (rewards - reward_mean) / max(reward_std, REWARD_NORMALIZATION_EPS)
    normalized_rewards = np.clip(
        normalized_rewards,
        -REWARD_NORMALIZATION_CLIP,
        REWARD_NORMALIZATION_CLIP,
    ).astype(np.float32)

    train_transitions = [
        OfflineTransition(
            observation=transition.observation,
            action=transition.action,
            reward=float(normalized_rewards[index]),
            done=transition.done,
            episode_id=transition.episode_id,
            step_id=transition.step_id,
        )
        for index, transition in enumerate(transitions)
    ]
    returns = compute_discounted_returns(train_transitions, gamma=gamma)

    return OfflineDataset(
        observations=torch.as_tensor(observations, dtype=torch.float32),
        actions=torch.as_tensor(actions, dtype=torch.int64),
        returns=torch.as_tensor(returns, dtype=torch.float32),
        normalized_rewards=torch.as_tensor(normalized_rewards, dtype=torch.float32),
        rewards=torch.as_tensor(rewards, dtype=torch.float32),
    )


def train_offline(
    network: DecisionNetwork,
    dataset: OfflineDataset,
    epochs: int,
    batch_size: int,
    value_coef: float,
    entropy_coef: float,
    checkpoint_dir: Path,
    checkpoint_every: int,
) -> tuple[EpochMetrics, Path]:
    sample_count = int(dataset.observations.shape[0])
    if sample_count == 0:
        raise ValueError("No samples available for training")

    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    best_metrics: EpochMetrics | None = None
    best_checkpoint_path = checkpoint_dir / "offline_best.pth"

    for epoch in range(1, epochs + 1):
        permutation = torch.randperm(sample_count, device=network.device)
        epoch_loss = 0.0
        epoch_policy_loss = 0.0
        epoch_value_loss = 0.0
        epoch_entropy = 0.0
        epoch_reward = 0.0
        sample_counter = 0
        batch_counter = 0

        for start in range(0, sample_count, batch_size):
            end = min(start + batch_size, sample_count)
            indices = permutation[start:end]

            obs = dataset.observations[indices].to(network.device)
            actions = dataset.actions[indices].to(network.device)
            returns = dataset.returns[indices].to(network.device)
            raw_rewards = dataset.rewards[indices].to(network.device)

            logits = network.actor(obs)
            dist = Categorical(logits=logits)
            log_probs = dist.log_prob(actions)
            entropy = dist.entropy().mean()
            values = network.critic(obs).squeeze(-1)

            advantages = returns - values.detach()
            if advantages.numel() > 1:
                advantages = (advantages - advantages.mean()) / (
                    advantages.std(unbiased=False) + 1e-8
                )

            policy_loss = -(log_probs * advantages).mean()
            value_loss = torch.nn.functional.mse_loss(values, returns)
            loss = policy_loss + value_coef * value_loss - entropy_coef * entropy

            network.optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(network.parameters(), 1.0)
            network.optimizer.step()

            epoch_loss += float(loss.item())
            epoch_policy_loss += float(policy_loss.item())
            epoch_value_loss += float(value_loss.item())
            epoch_entropy += float(entropy.item())
            epoch_reward += float(raw_rewards.sum().item())
            sample_counter += int(raw_rewards.numel())
            batch_counter += 1

        avg_loss = epoch_loss / max(batch_counter, 1)
        avg_policy = epoch_policy_loss / max(batch_counter, 1)
        avg_value = epoch_value_loss / max(batch_counter, 1)
        avg_entropy = epoch_entropy / max(batch_counter, 1)
        avg_reward = epoch_reward / max(sample_counter, 1)
        metrics = EpochMetrics(
            epoch=epoch,
            loss=avg_loss,
            policy_loss=avg_policy,
            value_loss=avg_value,
            entropy=avg_entropy,
            avg_reward=avg_reward,
        )

        if best_metrics is None or metrics.loss < best_metrics.loss:
            best_metrics = metrics
            _save_checkpoint(
                path=best_checkpoint_path,
                network=network,
                metrics=metrics,
                kind="best",
            )

        if checkpoint_every > 0 and (epoch % checkpoint_every == 0 or epoch == epochs):
            periodic_checkpoint_path = checkpoint_dir / f"offline_epoch_{epoch:03d}.pth"
            _save_checkpoint(
                path=periodic_checkpoint_path,
                network=network,
                metrics=metrics,
                kind="periodic",
            )

        print(
            f"epoch={epoch:03d} "
            f"loss={avg_loss:.5f} "
            f"policy={avg_policy:.5f} "
            f"value={avg_value:.5f} "
            f"entropy={avg_entropy:.5f} "
            f"avg_reward={avg_reward:.5f}"
        )

    if best_metrics is None:
        raise RuntimeError("Training completed without producing metrics")

    return (best_metrics, best_checkpoint_path)


def _save_checkpoint(
    path: Path,
    network: DecisionNetwork,
    metrics: EpochMetrics,
    kind: str,
) -> None:
    payload: dict[str, Any] = {
        "model_state_dict": network.state_dict(),
        "optimizer_state_dict": network.optimizer.state_dict(),
        "epoch": int(metrics.epoch),
        "metrics": {
            "loss": float(metrics.loss),
            "policy_loss": float(metrics.policy_loss),
            "value_loss": float(metrics.value_loss),
            "entropy": float(metrics.entropy),
            "avg_reward": float(metrics.avg_reward),
        },
        "checkpoint_kind": kind,
    }
    torch.save(payload, path)


def train_model(
    dataset_dir: str,
    output_model: str,
    epochs: int,
    batch_size: int,
    gamma: float,
    learning_rate: float,
    value_coef: float,
    entropy_coef: float,
    checkpoint_dir: str,
    checkpoint_every: int,
):
    dataset_paths = expand_dataset_paths([dataset_dir])
    transitions = read_transitions(dataset_paths)
    if not transitions:
        raise ValueError("No valid transitions found in supplied datasets")

    dataset = build_dataset(transitions, gamma=gamma)
    average_reward = float(dataset.rewards.mean().item())
    print(
        f"Loaded {len(transitions)} transitions from {len(dataset_paths)} file(s); "
        f"average reward={average_reward:.5f}"
    )

    network = DecisionNetwork(robot_name="offline_trainer", pheromone_decay=0.0, train=True)
    network.optimizer = torch.optim.Adam(network.parameters(), lr=learning_rate)
    network.train()

    checkpoint_dir = Path(checkpoint_dir)
    best_metrics, best_checkpoint_path = train_offline(
        network=network,
        dataset=dataset,
        epochs=epochs,
        batch_size=batch_size,
        value_coef=value_coef,
        entropy_coef=entropy_coef,
        checkpoint_dir=checkpoint_dir,
        checkpoint_every=checkpoint_every,
    )
    print(
        f"Best checkpoint: epoch={best_metrics.epoch} "
        f"loss={best_metrics.loss:.5f} path={best_checkpoint_path}"
    )

    output_path = Path(output_model)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(network.state_dict(), output_path)
    print(f"Saved trained model to {output_path}")


def main() -> int:
    args = parse_args()

    if args.epochs <= 0:
        raise ValueError("--epochs must be > 0")
    if args.batch_size <= 0:
        raise ValueError("--batch-size must be > 0")
    if not (0.0 <= args.gamma <= 1.0):
        raise ValueError("--gamma must be in [0, 1]")
    if args.checkpoint_every < 0:
        raise ValueError("--checkpoint-every must be >= 0")

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    dataset_paths = expand_dataset_paths(args.dataset)
    transitions = read_transitions(dataset_paths)
    if not transitions:
        raise ValueError("No valid transitions found in supplied datasets")

    dataset = build_dataset(transitions, gamma=args.gamma)
    average_reward = float(dataset.rewards.mean().item())
    print(
        f"Loaded {len(transitions)} transitions from {len(dataset_paths)} file(s); "
        f"average reward={average_reward:.5f}"
    )

    network = DecisionNetwork(robot_name="offline_trainer", pheromone_decay=0.0, train=True)
    network.optimizer = torch.optim.Adam(network.parameters(), lr=args.learning_rate)
    network.train()

    checkpoint_dir = Path(args.checkpoint_dir)
    best_metrics, best_checkpoint_path = train_offline(
        network=network,
        dataset=dataset,
        epochs=args.epochs,
        batch_size=args.batch_size,
        value_coef=args.value_coef,
        entropy_coef=args.entropy_coef,
        checkpoint_dir=checkpoint_dir,
        checkpoint_every=args.checkpoint_every,
    )
    print(
        f"Best checkpoint: epoch={best_metrics.epoch} "
        f"loss={best_metrics.loss:.5f} path={best_checkpoint_path}"
    )

    output_path = Path(args.output_model)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(network.state_dict(), output_path)
    print(f"Saved trained model to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

# offline_trainer --dataset export/offline_dataset_kris_robot1_1.jsonl --epochs 40 --checkpoint-dir export/checkpoints --checkpoint-every 2
