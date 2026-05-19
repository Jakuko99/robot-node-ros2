from __future__ import annotations

from typing import Any, Iterable
import math

import numpy as np
import torch
import torch.nn as nn
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry


TEAM_CONTEXT_DIM: int = 8


def _ensure_2d_map(data: np.ndarray, height: int, width: int) -> np.ndarray:
    if height <= 0 or width <= 0:
        return np.zeros((1, 1), dtype=np.int16)
    try:
        return data.reshape((height, width))
    except ValueError:
        return np.zeros((height, width), dtype=np.int16)


def _get_map_array(map_payload: OccupancyGrid | dict[str, Any] | None) -> tuple[np.ndarray, int, int]:
    if map_payload is None:
        return (np.zeros((1, 1), dtype=np.int16), 1, 1)

    if isinstance(map_payload, OccupancyGrid):
        height = int(map_payload.info.height)
        width = int(map_payload.info.width)
        data = np.asarray(map_payload.data, dtype=np.int16)
        return (_ensure_2d_map(data, height, width), height, width)

    height = int(map_payload.get("height", 0) or 0)
    width = int(map_payload.get("width", 0) or 0)
    data = np.asarray(map_payload.get("data", []), dtype=np.int16)
    return (_ensure_2d_map(data, height, width), height, width)


def _frontier_fraction(map_data: np.ndarray) -> float:
    if map_data.size == 0:
        return 0.0

    unknown = map_data == -1
    free = (map_data >= 0) & (map_data < 50)
    if not np.any(unknown) or not np.any(free):
        return 0.0

    adjacent_free = np.zeros_like(free)
    adjacent_free[1:, :] |= free[:-1, :]
    adjacent_free[:-1, :] |= free[1:, :]
    adjacent_free[:, 1:] |= free[:, :-1]
    adjacent_free[:, :-1] |= free[:, 1:]

    frontier = unknown & adjacent_free
    return float(frontier.mean())


def summarize_map(map_payload: OccupancyGrid | dict[str, Any] | None) -> np.ndarray:
    map_data, _, _ = _get_map_array(map_payload)

    known_ratio = float(np.mean(map_data >= 0))
    unknown_ratio = float(np.mean(map_data < 0))
    free_ratio = float(np.mean((map_data >= 0) & (map_data < 50)))
    occupied_ratio = float(np.mean(map_data >= 90))
    overlap_ratio = float(np.mean((map_data >= 10) & (map_data < 100)))
    teammate_ratio = float(np.mean((map_data == -10) | (map_data == 110)))
    frontier_ratio = _frontier_fraction(map_data)
    explored_balance = float(np.clip(known_ratio - unknown_ratio, -1.0, 1.0))

    summary = np.array(
        [
            known_ratio,
            unknown_ratio,
            free_ratio,
            occupied_ratio,
            overlap_ratio,
            teammate_ratio,
            frontier_ratio,
            explored_balance,
        ],
        dtype=np.float32,
    )
    return np.clip(summary, -1.0, 1.0)


def _pose_to_xy(goal: PoseStamped | dict[str, Any] | None) -> tuple[float, float] | None:
    if goal is None:
        return None

    if isinstance(goal, PoseStamped):
        return (float(goal.pose.position.x), float(goal.pose.position.y))

    if isinstance(goal, dict) and "x" in goal and "y" in goal:
        return (float(goal["x"]), float(goal["y"]))

    return None


def build_team_context(
    map_payload: OccupancyGrid | dict[str, Any] | None,
    odom: Odometry | dict[str, Any] | None,
    other_goals: dict[str, PoseStamped] | None = None,
    extra_features: Iterable[float] | None = None,
) -> np.ndarray:
    summary = summarize_map(map_payload)

    robot_position = None
    if isinstance(odom, Odometry):
        robot_position = np.array(
            [odom.pose.pose.position.x, odom.pose.pose.position.y], dtype=np.float32
        )
    elif isinstance(odom, dict) and "x" in odom and "y" in odom:
        robot_position = np.array([float(odom["x"]), float(odom["y"])], dtype=np.float32)

    goal_distances: list[float] = []
    if other_goals and robot_position is not None:
        for goal in other_goals.values():
            xy = _pose_to_xy(goal)
            if xy is None:
                continue
            goal_distances.append(float(np.linalg.norm(robot_position - np.asarray(xy, dtype=np.float32))))

    teammate_count = float(len(goal_distances))
    count_norm = float(np.clip(teammate_count / 8.0, 0.0, 1.0))
    nearest_distance = float(min(goal_distances)) if goal_distances else 0.0
    mean_distance = float(np.mean(goal_distances)) if goal_distances else 0.0

    if isinstance(map_payload, OccupancyGrid):
        diag = math.hypot(
            max(int(map_payload.info.width), 1) * float(map_payload.info.resolution),
            max(int(map_payload.info.height), 1) * float(map_payload.info.resolution),
        )
    elif isinstance(map_payload, dict):
        width = max(int(map_payload.get("width", 1) or 1), 1)
        height = max(int(map_payload.get("height", 1) or 1), 1)
        resolution = float(map_payload.get("resolution", 1.0) or 1.0)
        diag = math.hypot(width * resolution, height * resolution)
    else:
        diag = 1.0

    diag = max(diag, 1e-6)
    nearest_norm = float(np.clip(nearest_distance / diag, 0.0, 1.0))
    spread_norm = float(np.clip(mean_distance / diag, 0.0, 1.0))

    context = np.zeros((TEAM_CONTEXT_DIM,), dtype=np.float32)
    context[: summary.size] = summary[:TEAM_CONTEXT_DIM]
    context[4] = count_norm
    context[5] = nearest_norm
    context[6] = spread_norm
    context[7] = float(summary[1])

    if extra_features is not None:
        extra_vector = np.asarray(list(extra_features), dtype=np.float32).reshape(-1)
        if extra_vector.size > 0:
            limit = min(extra_vector.size, TEAM_CONTEXT_DIM - 2)
            blended = np.clip(extra_vector[:limit], -1.0, 1.0)
            context[-limit:] = 0.7 * context[-limit:] + 0.3 * blended

    return np.clip(context, -1.0, 1.0)


def build_team_context_from_record(record: dict[str, Any]) -> np.ndarray:
    local_map = record.get("local_map") or {}
    odom = record.get("odom") or {}
    metadata = record.get("metadata") or {}

    if isinstance(metadata, dict):
        team_context = metadata.get("team_context")
        if isinstance(team_context, (list, tuple, np.ndarray)):
            vector = np.asarray(team_context, dtype=np.float32).reshape(-1)
            if vector.size > 0:
                if vector.size < TEAM_CONTEXT_DIM:
                    padded = np.zeros((TEAM_CONTEXT_DIM,), dtype=np.float32)
                    padded[: vector.size] = vector
                    vector = padded
                elif vector.size > TEAM_CONTEXT_DIM:
                    vector = vector[:TEAM_CONTEXT_DIM]
                return np.clip(vector, -1.0, 1.0)

    extra_features: list[float] = []
    if isinstance(metadata, dict):
        extra_features.append(float(metadata.get("other_goal_count", 0.0) or 0.0))
        extra_features.append(float(metadata.get("moving", 0.0) or 0.0))
        extra_features.append(float(metadata.get("goal_timeout", 0.0) or 0.0))

    return build_team_context(
        map_payload=local_map if isinstance(local_map, dict) else None,
        odom=odom if isinstance(odom, dict) else None,
        other_goals=None,
        extra_features=extra_features,
    )


class MonotonicValueMixer(nn.Module):
    def __init__(self, context_dim: int = TEAM_CONTEXT_DIM):
        super().__init__()
        self.hyper_weights = nn.Sequential(
            nn.Linear(context_dim, 16),
            nn.SiLU(),
            nn.Linear(16, 2),
        )
        self.hyper_bias = nn.Sequential(
            nn.Linear(context_dim, 16),
            nn.SiLU(),
            nn.Linear(16, 1),
        )
        self.context_norm = nn.LayerNorm(context_dim)

    def forward(
        self,
        local_value: torch.Tensor,
        team_value: torch.Tensor,
        context: torch.Tensor,
    ) -> torch.Tensor:
        if context.dim() == 1:
            context = context.unsqueeze(0)

        context = self.context_norm(context)
        weights = torch.nn.functional.softplus(self.hyper_weights(context)) + 1e-3
        bias = self.hyper_bias(context).squeeze(-1)

        local_value = local_value.view(-1)
        team_value = team_value.view(-1)
        mixed = weights[:, 0] * local_value + weights[:, 1] * team_value + bias
        return mixed.squeeze(-1)
