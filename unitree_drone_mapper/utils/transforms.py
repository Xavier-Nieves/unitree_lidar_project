"""Coordinate frame math, quaternion operations, and transform utilities."""

import numpy as np
from typing import Tuple


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [w, x, y, z]."""
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert Euler angles (roll, pitch, yaw in radians) to 3x3 rotation matrix.

    Uses ZYX convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    return Rz @ Ry @ Rx


def build_transform_matrix(
    translation: np.ndarray, rotation: np.ndarray
) -> np.ndarray:
    """Build a 4x4 homogeneous transform from translation and rotation matrix.

    Args:
        translation: [x, y, z] array.
        rotation: 3x3 rotation matrix.

    Returns:
        4x4 homogeneous transform matrix.
    """
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Invert a 4x4 homogeneous transform matrix."""
    T_inv = np.eye(4)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def apply_transform(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to an Nx3 array of points.

    Args:
        T: 4x4 homogeneous transform.
        points: Nx3 array of 3D points.

    Returns:
        Nx3 array of transformed points.
    """
    ones = np.ones((points.shape[0], 1))
    pts_h = np.hstack([points, ones])  # Nx4
    transformed = (T @ pts_h.T).T
    return transformed[:, :3]


def pose_to_transform(
    position: Tuple[float, float, float],
    quaternion: Tuple[float, float, float, float],
) -> np.ndarray:
    """Convert a pose (position + quaternion) to a 4x4 transform.

    Args:
        position: (x, y, z) tuple.
        quaternion: (w, x, y, z) tuple.

    Returns:
        4x4 homogeneous transform matrix.
    """
    R = quaternion_to_rotation_matrix(np.array(quaternion))
    return build_transform_matrix(np.array(position), R)
