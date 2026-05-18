from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import atan2
from pathlib import Path
from time import time
from typing import Any
import copy
import json
import math

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry

from robot_node.utils import pos_to_map_index

PATCH_RADIUS: int = 4
OBS_DIM: int = (PATCH_RADIUS * 2 + 1) ** 2


@dataclass
class CollectedTransition:
    transition_id: int
    episode_id: int
    step_id: int
    robot_name: str
    timestamp: float
    observation: list[float]
    local_map: dict[str, Any]
    odom: dict[str, float]
    action: int | None = None
    goal: dict[str, float] | None = None
    reward: float | None = None
    next_observation: list[float] | None = None
    next_local_map: dict[str, Any] | None = None
    next_odom: dict[str, float] | None = None
    done: bool = False
    success: bool | None = None
    metadata: dict[str, Any] = field(default_factory=dict)


class DataCollector:
    def __init__(
        self,
        robot_name: str,
        output_dir: str = "export",
        dataset_name: str = "offline_dataset",
    ):
        self.robot_name = robot_name
        self.output_dir = Path(output_dir)
        self.dataset_name = dataset_name
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.episode_id: int = 0
        self.step_id: int = 0
        self.transition_id: int = 0

        self.transitions: list[CollectedTransition] = []
        self._pending_transition: CollectedTransition | None = None

    @property
    def has_pending_transition(self) -> bool:
        return self._pending_transition is not None

    def start_episode(self, episode_id: int | None = None) -> int:
        if episode_id is None:
            self.episode_id += 1
        else:
            self.episode_id = episode_id

        self.step_id = 0
        self._pending_transition = None
        return self.episode_id

    def begin_transition(
        self,
        current_map: OccupancyGrid,
        current_odom: Odometry,
        action: int | None = None,
        goal: tuple[float, float] | PoseStamped | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> int:
        if self._pending_transition is not None:
            raise RuntimeError("A transition is already pending finalization")

        observation = self.extract_observation(current_map, current_odom).tolist()
        local_map = self._serialize_occupancy_grid(self.get_local_patch(current_map, current_odom))

        transition = CollectedTransition(
            transition_id=self.transition_id,
            episode_id=self.episode_id,
            step_id=self.step_id,
            robot_name=self.robot_name,
            timestamp=self._timestamp_from_map(current_map),
            observation=observation,
            local_map=local_map,
            odom=self._serialize_odometry(current_odom),
            action=action,
            goal=self._serialize_goal(goal),
            metadata=dict(metadata or {}),
        )

        self._pending_transition = transition
        return transition.transition_id

    def finalize_transition(
        self,
        next_map: OccupancyGrid,
        next_odom: Odometry,
        reward: float,
        done: bool = False,
        success: bool | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> int:
        if self._pending_transition is None:
            raise RuntimeError("No pending transition to finalize")

        pending = copy.deepcopy(self._pending_transition)
        pending.reward = float(reward) if math.isfinite(reward) else 0.0
        pending.done = done
        pending.success = success
        pending.next_observation = self.extract_observation(next_map, next_odom).tolist()
        pending.next_local_map = self._serialize_occupancy_grid(
            self.get_local_patch(next_map, next_odom)
        )
        pending.next_odom = self._serialize_odometry(next_odom)

        if metadata:
            pending.metadata.update(metadata)

        self.transitions.append(pending)
        self._pending_transition = None
        self.transition_id += 1
        self.step_id += 1
        return pending.transition_id

    def record_transition(
        self,
        current_map: OccupancyGrid,
        current_odom: Odometry,
        next_map: OccupancyGrid,
        next_odom: Odometry,
        reward: float,
        action: int | None = None,
        goal: tuple[float, float] | PoseStamped | None = None,
        done: bool = False,
        success: bool | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> int:
        self.begin_transition(
            current_map=current_map,
            current_odom=current_odom,
            action=action,
            goal=goal,
            metadata=metadata,
        )
        return self.finalize_transition(
            next_map=next_map,
            next_odom=next_odom,
            reward=reward,
            done=done,
            success=success,
            metadata=metadata,
        )

    def clear(self):
        self.transitions.clear()
        self._pending_transition = None

    def export_jsonl(self, path: str | None = None) -> str:
        output_path = (
            Path(path)
            if path is not None
            else self.output_dir / f"{self.dataset_name}_{self.robot_name}.jsonl"
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, "w", encoding="utf-8") as file_handle:
            for transition in self.transitions:
                file_handle.write(json.dumps(self._to_jsonable(asdict(transition))) + "\n")

        return str(output_path)

    def export_json(self, path: str | None = None) -> str:
        output_path = (
            Path(path)
            if path is not None
            else self.output_dir / f"{self.dataset_name}_{self.robot_name}.json"
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, "w", encoding="utf-8") as file_handle:
            json.dump(
                [self._to_jsonable(asdict(transition)) for transition in self.transitions],
                file_handle,
                indent=2,
            )

        return str(output_path)

    @staticmethod
    def load_jsonl(path: str) -> list[dict[str, Any]]:
        records: list[dict[str, Any]] = []
        with open(path, "r", encoding="utf-8") as file_handle:
            for line in file_handle:
                line = line.strip()
                if not line:
                    continue
                records.append(json.loads(line))
        return records

    @staticmethod
    def _timestamp_from_map(map_msg: OccupancyGrid | None) -> float:
        if map_msg is None:
            return time()

        stamp = getattr(map_msg.header, "stamp", None)
        if stamp is None:
            return time()

        if hasattr(stamp, "sec") and hasattr(stamp, "nanosec"):
            return float(stamp.sec) + float(stamp.nanosec) * 1e-9

        return time()

    @staticmethod
    def _serialize_goal(goal: tuple[float, float] | PoseStamped | None) -> dict[str, float] | None:
        if goal is None:
            return None

        if isinstance(goal, PoseStamped):
            return {
                "x": float(goal.pose.position.x),
                "y": float(goal.pose.position.y),
                "z": float(goal.pose.position.z),
            }

        return {"x": float(goal[0]), "y": float(goal[1])}

    @staticmethod
    def _serialize_odometry(odom: Odometry | None) -> dict[str, float]:
        if odom is None:
            return {}

        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        yaw = 2.0 * atan2(orientation.z, orientation.w)

        return {
            "x": float(position.x),
            "y": float(position.y),
            "z": float(position.z),
            "yaw": float(yaw),
        }

    @staticmethod
    def _serialize_occupancy_grid(map_msg: OccupancyGrid | None) -> dict[str, Any]:
        if map_msg is None:
            return {}

        width = int(map_msg.info.width)
        height = int(map_msg.info.height)
        data = np.asarray(map_msg.data, dtype=np.int8).reshape((height, width))

        return {
            "width": width,
            "height": height,
            "resolution": float(map_msg.info.resolution),
            "origin_x": float(map_msg.info.origin.position.x),
            "origin_y": float(map_msg.info.origin.position.y),
            "frame_id": str(map_msg.header.frame_id),
            "data": data.reshape(-1).astype(int).tolist(),
        }

    @staticmethod
    def _to_jsonable(value: Any) -> Any:
        if isinstance(value, dict):
            return {str(key): DataCollector._to_jsonable(item) for key, item in value.items()}

        if isinstance(value, list):
            return [DataCollector._to_jsonable(item) for item in value]

        if isinstance(value, tuple):
            return [DataCollector._to_jsonable(item) for item in value]

        if isinstance(value, np.ndarray):
            return value.tolist()

        if isinstance(value, (np.integer, np.floating)):
            return value.item()

        return value

    @staticmethod
    def layer_to_utility_map(local_map: OccupancyGrid) -> np.ndarray:
        transformed = DataCollector.transform_map(local_map).astype(np.float32)

        unexplored = transformed[:, :, 0]
        occupied = transformed[:, :, 1]
        overlap = transformed[:, :, 2]
        others = transformed[:, :, 4]
        utility = unexplored + 0.3 * overlap - occupied - 0.2 * others
        return utility

    @staticmethod
    def extract_observation(
        map_msg: OccupancyGrid,
        odom: Odometry,
        patch_radius: int = PATCH_RADIUS,
    ) -> np.ndarray:
        local_patch = DataCollector.get_local_patch(map_msg, odom, patch_size=patch_radius)
        utility = DataCollector.layer_to_utility_map(local_patch)
        obs_vec = utility.reshape(-1).astype(np.float32)

        if obs_vec.size < OBS_DIM:
            padded = np.zeros((OBS_DIM,), dtype=np.float32)
            padded[: obs_vec.size] = obs_vec
            return padded

        if obs_vec.size > OBS_DIM:
            return obs_vec[:OBS_DIM]

        return obs_vec

    @staticmethod
    def transform_map(map_msg: OccupancyGrid) -> np.ndarray:
        map_data: np.ndarray = np.array(map_msg.data, dtype=np.int8).reshape(
            (map_msg.info.height, map_msg.info.width)
        )

        out = np.zeros((map_msg.info.height, map_msg.info.width, 5), dtype=np.int8)
        out[:, :, 0] = (map_data[:, :] == -1).astype(np.int8)
        out[:, :, 1] = (map_data[:, :] == 100).astype(np.int8)
        out[:, :, 2] = ((map_data[:, :] >= 10) & (map_data[:, :] < 100)).astype(np.int8)
        out[:, :, 3] = (map_data[:, :] == -10).astype(np.int8)
        out[:, :, 4] = (map_data[:, :] == 110).astype(np.int8)

        return out

    @staticmethod
    def get_local_patch(
        map: OccupancyGrid,
        odom: Odometry,
        patch_size: int = PATCH_RADIUS,
        static_transform_x: float = 0.0,
        static_transform_y: float = 0.0,
    ) -> OccupancyGrid:
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

        for i in range(-patch_size, patch_size + 1):
            for j in range(-patch_size, patch_size + 1):
                global_i = center_index[0] + i
                global_j = center_index[1] + j

                if 0 <= global_i < map.info.height and 0 <= global_j < map.info.width:
                    local_index = (i + patch_size) * local_map.info.width + (j + patch_size)
                    global_index = global_i * map.info.width + global_j
                    local_map.data[local_index] = map.data[global_index]

        return local_map
