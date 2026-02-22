"""PCD/PLY/OBJ read/write helpers using Open3D."""

import os
import json
import numpy as np
from pathlib import Path

from utils.logger import setup_logger

logger = setup_logger(__name__)


def load_point_cloud(filepath: str):
    """Load a point cloud from PCD, PLY, or other supported format.

    Returns:
        Open3D PointCloud object.
    """
    import open3d as o3d

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Point cloud not found: {filepath}")

    pcd = o3d.io.read_point_cloud(filepath)
    logger.info(f"Loaded point cloud: {filepath} ({len(pcd.points):,} points)")
    return pcd


def save_point_cloud(pcd, filepath: str, write_ascii: bool = False):
    """Save a point cloud to file (.pcd, .ply, etc.)."""
    import open3d as o3d

    Path(filepath).parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(filepath, pcd, write_ascii=write_ascii)

    size_mb = os.path.getsize(filepath) / (1024 * 1024)
    logger.info(f"Saved point cloud: {filepath} ({len(pcd.points):,} pts, {size_mb:.2f} MB)")


def load_mesh(filepath: str):
    """Load a triangle mesh from OBJ, PLY, or STL.

    Returns:
        Open3D TriangleMesh object.
    """
    import open3d as o3d

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Mesh not found: {filepath}")

    mesh = o3d.io.read_triangle_mesh(filepath)
    logger.info(
        f"Loaded mesh: {filepath} "
        f"({len(mesh.vertices):,} verts, {len(mesh.triangles):,} tris)"
    )
    return mesh


def save_mesh(mesh, filepath: str, write_ascii: bool = True):
    """Save a triangle mesh to file (.obj, .ply, .stl)."""
    import open3d as o3d

    Path(filepath).parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(filepath, mesh, write_ascii=write_ascii)

    size_mb = os.path.getsize(filepath) / (1024 * 1024)
    logger.info(
        f"Saved mesh: {filepath} "
        f"({len(mesh.vertices):,} verts, {len(mesh.triangles):,} tris, {size_mb:.2f} MB)"
    )


def save_poses_json(poses: list, filepath: str):
    """Save a list of camera/SLAM poses to JSON."""
    Path(filepath).parent.mkdir(parents=True, exist_ok=True)

    serializable = []
    for pose in poses:
        entry = {}
        for k, v in pose.items():
            if isinstance(v, np.ndarray):
                entry[k] = v.tolist()
            else:
                entry[k] = v
        serializable.append(entry)

    with open(filepath, "w") as f:
        json.dump(serializable, f, indent=2)

    logger.info(f"Saved {len(poses)} poses to {filepath}")


def load_poses_json(filepath: str) -> list:
    """Load poses from a JSON file."""
    with open(filepath, "r") as f:
        poses = json.load(f)
    logger.info(f"Loaded {len(poses)} poses from {filepath}")
    return poses


def create_session_dirs(base_dir: str, session_id: str) -> dict:
    """Create the directory structure for a flight session.

    Returns:
        Dict with paths: session, raw, bags, images, metadata, processed, logs.
    """
    session_dir = os.path.join(base_dir, session_id)
    dirs = {
        "session": session_dir,
        "raw": os.path.join(session_dir, "raw"),
        "bags": os.path.join(session_dir, "raw", "bags"),
        "images": os.path.join(session_dir, "raw", "images"),
        "metadata": os.path.join(session_dir, "raw", "metadata"),
        "processed": os.path.join(session_dir, "processed"),
        "model_textured": os.path.join(session_dir, "processed", "model_textured"),
        "logs": os.path.join(session_dir, "logs"),
    }

    for path in dirs.values():
        Path(path).mkdir(parents=True, exist_ok=True)

    logger.info(f"Created session directory: {session_dir}")
    return dirs
