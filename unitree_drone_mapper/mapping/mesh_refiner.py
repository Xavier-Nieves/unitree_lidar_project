"""Seam reduction, color correction, and final mesh cleanup.

Post-processing step after texture projection to improve
visual quality of the final textured model.
"""

import numpy as np
import open3d as o3d

from utils.logger import setup_logger

logger = setup_logger(__name__)


def smooth_vertex_colors(mesh, iterations: int = 3):
    """Smooth vertex colors to reduce seams between projected images.

    Applies Laplacian smoothing to the color channel only.

    Args:
        mesh: Open3D TriangleMesh with vertex colors.
        iterations: Number of smoothing passes.

    Returns:
        Mesh with smoothed colors.
    """
    if not mesh.has_vertex_colors():
        logger.warning("Mesh has no vertex colors to smooth")
        return mesh

    logger.info(f"Smoothing vertex colors ({iterations} iterations)...")

    colors = np.asarray(mesh.vertex_colors).copy()
    triangles = np.asarray(mesh.triangles)
    n_verts = len(colors)

    # Build adjacency list
    adjacency = [set() for _ in range(n_verts)]
    for tri in triangles:
        for i in range(3):
            for j in range(3):
                if i != j:
                    adjacency[tri[i]].add(tri[j])

    # Iterative Laplacian smoothing on colors
    for iteration in range(iterations):
        new_colors = colors.copy()
        for i in range(n_verts):
            neighbors = list(adjacency[i])
            if neighbors:
                neighbor_colors = colors[neighbors]
                # Blend: 50% own color + 50% neighbor average
                avg = neighbor_colors.mean(axis=0)
                new_colors[i] = 0.5 * colors[i] + 0.5 * avg
        colors = new_colors

    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    logger.info("Color smoothing complete")

    return mesh


def correct_brightness(mesh, target_brightness: float = 0.5):
    """Normalize overall brightness of vertex colors.

    Args:
        mesh: Open3D TriangleMesh with vertex colors.
        target_brightness: Target average brightness [0, 1].

    Returns:
        Mesh with corrected brightness.
    """
    if not mesh.has_vertex_colors():
        return mesh

    colors = np.asarray(mesh.vertex_colors).copy()
    current_brightness = colors.mean()

    if current_brightness > 0:
        scale = target_brightness / current_brightness
        colors = np.clip(colors * scale, 0, 1)
        mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
        logger.info(
            f"Brightness corrected: {current_brightness:.3f} -> {target_brightness:.3f}"
        )

    return mesh


def smooth_mesh_geometry(mesh, iterations: int = 1):
    """Light Laplacian smoothing of mesh geometry.

    Args:
        mesh: Open3D TriangleMesh.
        iterations: Number of smoothing passes.

    Returns:
        Smoothed mesh.
    """
    logger.info(f"Smoothing mesh geometry ({iterations} iterations)...")
    mesh = mesh.filter_smooth_laplacian(
        number_of_iterations=iterations,
        lambda_filter=0.5,
    )
    return mesh


def refine_mesh(mesh, config: dict = None):
    """Full refinement pipeline: color smoothing + brightness correction.

    Args:
        mesh: Open3D TriangleMesh with vertex colors.
        config: Optional refinement config.

    Returns:
        Refined mesh.
    """
    mesh = smooth_vertex_colors(mesh, iterations=3)
    mesh = correct_brightness(mesh)

    logger.info(
        f"Refinement complete: {len(mesh.vertices):,} verts, "
        f"{len(mesh.triangles):,} tris"
    )

    return mesh
