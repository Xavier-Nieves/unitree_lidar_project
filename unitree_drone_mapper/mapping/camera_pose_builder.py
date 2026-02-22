"""Build camera poses from SLAM trajectory + LiDAR-to-camera extrinsic.

Takes the SLAM trajectory (LiDAR frame) and applies the calibrated
extrinsic transform to produce camera poses for texture projection.
"""

import numpy as np
import yaml
from typing import List

from utils.transforms import (
    pose_to_transform,
    build_transform_matrix,
    euler_to_rotation_matrix,
)
from utils.logger import setup_logger

logger = setup_logger(__name__)


def load_extrinsic(calibration_file: str) -> np.ndarray:
    """Load the LiDAR-to-camera extrinsic transform from calibration YAML.

    Args:
        calibration_file: Path to camera_calibration.yaml.

    Returns:
        4x4 homogeneous transform (LiDAR frame -> camera frame).
    """
    with open(calibration_file, "r") as f:
        cal = yaml.safe_load(f)

    ext = cal.get("extrinsic", {})
    translation = np.array(ext.get("translation", [0, 0, 0]))
    rpy = ext.get("rotation_rpy", [0, 0, 0])

    R = euler_to_rotation_matrix(rpy[0], rpy[1], rpy[2])
    T = build_transform_matrix(translation, R)

    logger.info(f"Loaded extrinsic: t={translation}, rpy={rpy}")
    return T


def build_camera_poses(
    slam_poses: List[dict],
    extrinsic: np.ndarray,
) -> List[dict]:
    """Transform SLAM poses to camera poses using the extrinsic calibration.

    Args:
        slam_poses: List of SLAM pose dicts (position + orientation in LiDAR frame).
        extrinsic: 4x4 LiDAR-to-camera transform.

    Returns:
        List of camera pose dicts in the same format.
    """
    camera_poses = []

    for pose in slam_poses:
        # Build 4x4 transform for this SLAM pose (world -> LiDAR)
        T_world_lidar = pose_to_transform(
            pose["position"],
            pose["orientation"],
        )

        # Apply extrinsic: world -> camera = (world -> LiDAR) @ (LiDAR -> camera)
        T_world_camera = T_world_lidar @ extrinsic

        # Extract position and rotation from the camera transform
        camera_pos = T_world_camera[:3, 3].tolist()

        # Convert rotation matrix back to quaternion
        from utils.transforms import rotation_matrix_to_quaternion
        camera_quat = rotation_matrix_to_quaternion(T_world_camera[:3, :3]).tolist()

        camera_poses.append({
            "timestamp": pose.get("timestamp"),
            "position": camera_pos,
            "orientation": camera_quat,
            "transform": T_world_camera.tolist(),
        })

    logger.info(f"Built {len(camera_poses)} camera poses from SLAM trajectory")
    return camera_poses
