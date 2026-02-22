"""Direct equirectangular texture projection onto mesh.

Projects Insta360 X3 equirectangular images directly onto the 3D mesh
using known camera poses. No virtual camera decomposition needed —
works natively with 360-degree images.
"""

import numpy as np
import cv2
from typing import List, Optional

from utils.logger import setup_logger

logger = setup_logger(__name__)


def equirect_to_direction(u: float, v: float) -> np.ndarray:
    """Convert equirectangular UV coordinates to 3D direction vector.

    Args:
        u: Horizontal coordinate [0, 1] (0 = left, 1 = right).
        v: Vertical coordinate [0, 1] (0 = top, 1 = bottom).

    Returns:
        Unit 3D direction vector.
    """
    theta = (u - 0.5) * 2 * np.pi   # longitude: -pi to pi
    phi = (v - 0.5) * np.pi          # latitude: -pi/2 to pi/2

    x = np.cos(phi) * np.sin(theta)
    y = -np.sin(phi)
    z = np.cos(phi) * np.cos(theta)

    return np.array([x, y, z])


def direction_to_equirect(direction: np.ndarray) -> tuple:
    """Convert 3D direction to equirectangular UV coordinates.

    Args:
        direction: 3D direction vector (will be normalized).

    Returns:
        (u, v) tuple in [0, 1] range.
    """
    d = direction / np.linalg.norm(direction)

    theta = np.arctan2(d[0], d[2])  # longitude
    phi = np.arcsin(np.clip(-d[1], -1, 1))  # latitude

    u = theta / (2 * np.pi) + 0.5
    v = phi / np.pi + 0.5

    return (u, v)


def project_texture(
    mesh,
    images: List[str],
    camera_poses: List[dict],
    image_size: tuple = (5760, 2880),
    blend: bool = True,
) -> Optional[np.ndarray]:
    """Project equirectangular images onto mesh vertices.

    For each mesh vertex, find the closest camera pose, compute the
    direction from camera to vertex, look up the color in the
    equirectangular image at that direction, and assign it.

    Args:
        mesh: Open3D TriangleMesh.
        images: List of image file paths.
        camera_poses: List of camera pose dicts (matching images order).
        image_size: (width, height) of equirectangular images.
        blend: If True, blend colors from multiple overlapping images.

    Returns:
        Texture image as numpy array, or None on failure.
    """
    import open3d as o3d

    vertices = np.asarray(mesh.vertices)
    n_verts = len(vertices)

    if not images or not camera_poses:
        logger.error("No images or camera poses provided")
        return None

    logger.info(
        f"Projecting {len(images)} images onto {n_verts:,} vertices..."
    )

    # Initialize vertex colors
    colors = np.zeros((n_verts, 3), dtype=np.float64)
    weights = np.zeros(n_verts, dtype=np.float64)

    for idx, (img_path, pose) in enumerate(zip(images, camera_poses)):
        img = cv2.imread(img_path)
        if img is None:
            logger.warning(f"Could not read image: {img_path}")
            continue

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_h, img_w = img.shape[:2]

        # Camera position and rotation
        cam_pos = np.array(pose["position"])
        if "transform" in pose:
            T = np.array(pose["transform"])
            R_inv = T[:3, :3].T
        else:
            R_inv = np.eye(3)

        # For each vertex, compute direction from camera and look up color
        directions = vertices - cam_pos  # Nx3
        distances = np.linalg.norm(directions, axis=1)

        # Transform directions to camera frame
        cam_directions = (R_inv @ directions.T).T

        for i in range(n_verts):
            if distances[i] < 0.1:  # Skip very close points
                continue

            d = cam_directions[i]
            u, v = direction_to_equirect(d)

            # Clamp to image bounds
            px = int(np.clip(u * img_w, 0, img_w - 1))
            py = int(np.clip(v * img_h, 0, img_h - 1))

            color = img[py, px].astype(np.float64) / 255.0

            if blend:
                # Distance-weighted blending
                w = 1.0 / max(distances[i], 0.1)
                colors[i] += color * w
                weights[i] += w
            else:
                # Nearest camera wins
                if weights[i] == 0 or distances[i] < weights[i]:
                    colors[i] = color
                    weights[i] = distances[i]

        if (idx + 1) % 10 == 0:
            logger.info(f"  Processed {idx + 1}/{len(images)} images")

    # Normalize blended colors
    if blend:
        mask = weights > 0
        colors[mask] /= weights[mask, np.newaxis]

    # Apply colors to mesh
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    logger.info("Texture projection complete")

    return colors
