"""Statistical outlier removal and voxel downsampling for point clouds.

Migrated from the cleaning stage of auto_mesh_generator.py.
Operates on files — no ROS dependency required.
"""

import numpy as np
from utils.logger import setup_logger

logger = setup_logger(__name__)


def remove_outliers(pcd, nb_neighbors: int = 20, std_ratio: float = 2.0):
    """Statistical outlier removal.

    Args:
        pcd: Open3D PointCloud.
        nb_neighbors: Number of neighbors for statistical analysis.
        std_ratio: Standard deviation threshold.

    Returns:
        Cleaned Open3D PointCloud.
    """
    original_count = len(pcd.points)
    logger.info(f"Removing outliers (neighbors={nb_neighbors}, std={std_ratio})...")

    pcd_clean, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio,
    )

    removed = original_count - len(pcd_clean.points)
    percent = (removed / original_count) * 100 if original_count > 0 else 0
    logger.info(f"Removed {removed:,} outliers ({percent:.1f}%), {len(pcd_clean.points):,} remaining")

    return pcd_clean


def voxel_downsample(pcd, voxel_size: float = 0.02):
    """Voxel downsampling to reduce point density.

    Args:
        pcd: Open3D PointCloud.
        voxel_size: Voxel edge length in meters. 0 = skip.

    Returns:
        Downsampled Open3D PointCloud.
    """
    if voxel_size <= 0:
        return pcd

    original_count = len(pcd.points)
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)

    logger.info(
        f"Downsampled: {original_count:,} -> {len(pcd_down.points):,} points "
        f"(voxel={voxel_size}m)"
    )
    return pcd_down


def clean_point_cloud(pcd, config: dict):
    """Full cleaning pipeline: outlier removal + optional downsampling.

    Args:
        pcd: Open3D PointCloud.
        config: cloud_cleaning config dict from config.yaml.

    Returns:
        Cleaned Open3D PointCloud.
    """
    nb_neighbors = config.get("outlier_neighbors", 20)
    std_ratio = config.get("outlier_std_ratio", 2.0)
    voxel_size = config.get("voxel_size", 0.02)

    pcd = remove_outliers(pcd, nb_neighbors, std_ratio)
    pcd = voxel_downsample(pcd, voxel_size)

    return pcd
