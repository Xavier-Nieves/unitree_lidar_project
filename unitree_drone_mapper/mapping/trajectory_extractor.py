"""Extract SLAM poses from bag files or saved trajectory JSON.

Used in post-processing to get the camera trajectory
for texture projection.
"""

import json
import numpy as np
from typing import List

from utils.logger import setup_logger

logger = setup_logger(__name__)


def load_trajectory_json(filepath: str) -> List[dict]:
    """Load trajectory from the JSON file saved by TrajectoryLogger.

    Args:
        filepath: Path to trajectory.json.

    Returns:
        List of pose dicts with position and orientation.
    """
    with open(filepath, "r") as f:
        data = json.load(f)

    poses = data.get("poses", [])
    logger.info(f"Loaded {len(poses)} poses from {filepath}")
    return poses


def load_trajectory_from_bag(bag_path: str, topic: str = "/Odometry") -> List[dict]:
    """Extract trajectory poses directly from a ROS 2 bag file.

    Args:
        bag_path: Path to bag directory.
        topic: Odometry topic name.

    Returns:
        List of pose dicts.
    """
    from utils.bag_tools import read_trajectory_from_bag
    return read_trajectory_from_bag(bag_path, topic)


def subsample_trajectory(poses: List[dict], min_distance: float = 0.5) -> List[dict]:
    """Subsample trajectory to reduce redundant poses.

    Keeps poses that are at least min_distance apart.

    Args:
        poses: Full trajectory.
        min_distance: Minimum distance between kept poses.

    Returns:
        Subsampled list of poses.
    """
    if not poses:
        return []

    subsampled = [poses[0]]

    for pose in poses[1:]:
        last_pos = np.array(subsampled[-1]["position"])
        curr_pos = np.array(pose["position"])
        dist = np.linalg.norm(curr_pos - last_pos)

        if dist >= min_distance:
            subsampled.append(pose)

    logger.info(f"Subsampled trajectory: {len(poses)} -> {len(subsampled)} poses")
    return subsampled


def interpolate_pose_at_time(poses: List[dict], target_time: float) -> dict:
    """Linearly interpolate a pose at a specific timestamp.

    Args:
        poses: Sorted list of poses with timestamps.
        target_time: Target timestamp.

    Returns:
        Interpolated pose dict.
    """
    if not poses:
        raise ValueError("Empty pose list")

    # Find bracketing poses
    for i in range(len(poses) - 1):
        t0 = poses[i]["timestamp"]
        t1 = poses[i + 1]["timestamp"]

        if t0 <= target_time <= t1:
            # Linear interpolation factor
            dt = t1 - t0
            alpha = (target_time - t0) / dt if dt > 0 else 0.0

            pos0 = np.array(poses[i]["position"])
            pos1 = np.array(poses[i + 1]["position"])
            position = pos0 + alpha * (pos1 - pos0)

            # Simple SLERP approximation for quaternion
            q0 = np.array(poses[i]["orientation"])
            q1 = np.array(poses[i + 1]["orientation"])
            orientation = _slerp(q0, q1, alpha)

            return {
                "timestamp": target_time,
                "position": position.tolist(),
                "orientation": orientation.tolist(),
            }

    # Outside range — return nearest
    if target_time <= poses[0]["timestamp"]:
        return poses[0]
    return poses[-1]


def _slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two quaternions."""
    dot = np.dot(q0, q1)

    if dot < 0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)

    theta = np.arccos(np.clip(dot, -1, 1))
    sin_theta = np.sin(theta)

    s0 = np.sin((1 - t) * theta) / sin_theta
    s1 = np.sin(t * theta) / sin_theta

    return s0 * q0 + s1 * q1
