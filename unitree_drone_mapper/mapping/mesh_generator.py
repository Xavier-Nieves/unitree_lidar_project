"""Normal estimation and Poisson surface reconstruction.

Migrated from the meshing stage of auto_mesh_generator.py.
Operates on files — no ROS dependency required.
"""

import numpy as np
import open3d as o3d

from utils.logger import setup_logger

logger = setup_logger(__name__)


def estimate_normals(pcd, radius: float = 0.1, max_nn: int = 30):
    """Estimate and orient surface normals.

    Args:
        pcd: Open3D PointCloud.
        radius: Search radius for normal estimation.
        max_nn: Maximum neighbors for normal estimation.

    Returns:
        PointCloud with normals.
    """
    logger.info(f"Computing normals (radius={radius}m, max_nn={max_nn})...")

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius,
            max_nn=max_nn,
        )
    )

    # Orient normals consistently
    pcd.orient_normals_consistent_tangent_plane(k=15)
    logger.info("Normals computed and oriented")

    return pcd


def poisson_reconstruction(pcd, depth: int = 9):
    """Poisson surface reconstruction from point cloud with normals.

    Args:
        pcd: Open3D PointCloud with normals.
        depth: Poisson octree depth (higher = more detail, slower).

    Returns:
        Open3D TriangleMesh.
    """
    logger.info(f"Poisson reconstruction (depth={depth})...")

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd,
        depth=depth,
        width=0,
        scale=1.1,
        linear_fit=False,
    )

    logger.info(
        f"Raw mesh: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} triangles"
    )

    # Remove low-density vertices (artifacts at edges)
    densities = np.asarray(densities)
    threshold = np.quantile(densities, 0.01)
    vertices_to_remove = densities < threshold
    mesh.remove_vertices_by_mask(vertices_to_remove)

    removed = vertices_to_remove.sum()
    logger.info(f"Removed {removed:,} low-density vertices")

    return mesh


def clean_mesh(mesh):
    """Post-process mesh: remove isolated components and degenerate geometry.

    Args:
        mesh: Open3D TriangleMesh.

    Returns:
        Cleaned mesh.
    """
    logger.info("Cleaning mesh...")

    # Keep only the largest connected component
    triangle_clusters, cluster_n_triangles, _ = mesh.cluster_connected_triangles()
    triangle_clusters = np.asarray(triangle_clusters)
    cluster_n_triangles = np.asarray(cluster_n_triangles)

    if len(cluster_n_triangles) > 1:
        largest = cluster_n_triangles.argmax()
        triangles_to_remove = triangle_clusters != largest
        mesh.remove_triangles_by_mask(triangles_to_remove)
        logger.info(f"Removed {triangles_to_remove.sum():,} isolated triangles")

    # Clean degenerate geometry
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()

    logger.info(
        f"Final mesh: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} triangles"
    )

    return mesh


def generate_mesh(pcd, config: dict):
    """Full mesh generation pipeline: normals + Poisson + cleanup.

    Args:
        pcd: Open3D PointCloud (already cleaned).
        config: meshing config dict from config.yaml.

    Returns:
        Open3D TriangleMesh.
    """
    depth = config.get("poisson_depth", 9)

    pcd = estimate_normals(pcd)
    mesh = poisson_reconstruction(pcd, depth=depth)
    mesh = clean_mesh(mesh)

    return mesh
