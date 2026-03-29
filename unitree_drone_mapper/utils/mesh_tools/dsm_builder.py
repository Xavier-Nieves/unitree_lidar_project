"""
dsm_builder.py — Ball Pivoting Algorithm (BPA) mesh from non-ground points.

Takes classified non-ground points (buildings, trees, objects) and produces
a surface mesh that correctly preserves gaps between structures.

Algorithm: Ball Pivoting Algorithm (Bernardini et al. 1999, IEEE TVCG)
    A virtual ball of radius ρ rolls across the point cloud surface.
    A triangle forms only when the ball simultaneously touches three points
    without any other point falling inside the ball.
    The ball pivots around each edge, forming adjacent triangles.
    If the ball cannot reach between two points (gap too large),
    no triangle forms — the gap is preserved in the mesh.

Why BPA for non-ground (not Delaunay):
    Buildings and trees are fully 3D structures with vertical walls,
    overhangs, and complex geometry. Delaunay 2.5D only allows one Z
    value per XY position — it cannot represent a vertical wall.
    BPA works in full 3D space and handles arbitrary geometry.

Why BPA over Poisson for non-ground:
    Poisson fills gaps by solving a global function — it connects the
    left wall of building A to the right wall of building B through the
    air gap between them. BPA only connects points the ball can touch,
    so the gap between buildings remains a gap in the mesh.

Multi-scale radii:
    Three radii are used (r, 2r, 4r) to handle both fine detail (small r)
    and larger flat surfaces like rooftops (large r).

Dependencies: open3d, scipy
"""

import sys
import threading
import time
import numpy as np
from pathlib import Path

# ── Robust project root resolution ───────────────────────────────────────────
# This file lives at:
#   unitree_drone_mapper/utils/mesh_tools/dsm_builder.py
#
# parents[0] = .../utils/mesh_tools/
# parents[1] = .../utils/
# parents[2] = .../unitree_drone_mapper/        ← package, NOT project root
# parents[3] = .../unitree_lidar_project/       ← project root  ✓
#
# We add the project root so that `unitree_drone_mapper` is importable
# as a package regardless of the working directory at invocation time.
_PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from unitree_drone_mapper.config_loader import load_config as _load_config

# ── Performance limits ────────────────────────────────────────────────────────
_cfg = _load_config()

DEFAULT_MAX_PTS      = _cfg["dsm"]["DEFAULT_MAX_PTS"]
DEFAULT_MAX_PTS_FAST = _cfg["dsm"]["DEFAULT_MAX_PTS_FAST"]
MIN_RADIUS           = _cfg["dsm"]["MIN_RADIUS"]
MAX_RADIUS           = _cfg["dsm"]["MAX_RADIUS"]


# ── BPA progress spinner ──────────────────────────────────────────────────────

class _BPAProgress:
    """Background progress display for normal estimation and BPA stages."""

    BAR_WIDTH          = 30
    SEC_PER_1K_NORMALS = 0.08
    SEC_PER_1K_BPA     = 0.25

    def __init__(self, n_pts: int, stage: str = "normals"):
        self.n_pts   = n_pts
        self.stage   = stage
        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.t_start = None

    @property
    def _estimate_s(self) -> float:
        rate = (self.SEC_PER_1K_NORMALS if self.stage == "normals"
                else self.SEC_PER_1K_BPA)
        return max(5.0, (self.n_pts / 1000) * rate)

    def start(self, stage: str) -> None:
        self.stage   = stage
        self.t_start = time.time()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        sys.stdout.write("\r" + " " * 80 + "\r")
        sys.stdout.flush()

    def _run(self) -> None:
        est = self._estimate_s
        while not self._stop.is_set():
            elapsed  = time.time() - self.t_start
            fraction = min(elapsed / est, 0.99)
            filled   = int(self.BAR_WIDTH * fraction)
            bar      = "█" * filled + "░" * (self.BAR_WIDTH - filled)
            pct      = int(fraction * 100)
            eta      = max(0, est - elapsed)
            label    = "normals" if self.stage == "normals" else "BPA"
            sys.stdout.write(
                f"\r  [DSMBuilder] {label}  [{bar}] {pct:3d}%  "
                f"{elapsed:.0f}s elapsed  ~{eta:.0f}s remaining   "
            )
            sys.stdout.flush()
            self._stop.wait(0.5)


class DSMBuilder:
    """
    Ball Pivoting Algorithm mesh builder for non-ground objects.

    Parameters
    ----------
    radius : float or None
        Ball radius in metres. None = auto-estimate from point density.
        Clamped to [MIN_RADIUS, MAX_RADIUS] from config.

    radius_scale : tuple of float
        Multipliers applied to base radius for multi-scale BPA.

    nn_sample : int
        Points sampled for nearest-neighbour radius estimation.

    fast : bool
        Single radius, lower point cap.

    max_pts : int or None
        Hard cap before BPA. None = use config default.
    """

    def __init__(self,
                 radius:       float | None = None,
                 radius_scale: tuple        = (1.0, 2.0, 4.0),
                 nn_sample:    int          = 1000,
                 fast:         bool         = False,
                 max_pts:      int | None   = None):
        self.radius       = radius
        self.radius_scale = (1.0,) if fast else radius_scale
        self.nn_sample    = nn_sample
        self.fast         = fast
        self.max_pts      = (
            max_pts if max_pts is not None else
            DEFAULT_MAX_PTS_FAST if fast else DEFAULT_MAX_PTS
        )

    def build(self, nonground_pts: np.ndarray):
        """
        Build BPA surface mesh from non-ground points.

        Returns open3d.geometry.TriangleMesh, or None if insufficient points.
        """
        try:
            import open3d as o3d
        except ImportError:
            print("  [DSMBuilder] open3d not available — skipping DSM")
            return None

        if len(nonground_pts) < 10:
            print(f"  [DSMBuilder] Too few non-ground points "
                  f"({len(nonground_pts)}) — skipping DSM")
            return None

        t0     = time.time()
        pts    = self._apply_cap(nonground_pts)
        n_pts  = len(pts)
        radius = self.radius or self._estimate_radius(pts)
        radius = max(MIN_RADIUS, min(radius, MAX_RADIUS))
        radii  = [radius * s for s in self.radius_scale]

        print(f"  [DSMBuilder] BPA  radius={radius:.4f}m  "
              f"radii={[f'{r:.4f}' for r in radii]}  pts={n_pts:,}")

        spinner    = _BPAProgress(n_pts)
        pcd        = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))

        spinner.start("normals")
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radii[-1] * 2, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(100)
        spinner.stop()
        print(f"  [DSMBuilder] Normals done in {time.time() - t0:.1f}s")

        spinner.start("bpa")
        mesh = (o3d.geometry.TriangleMesh
                .create_from_point_cloud_ball_pivoting(
                    pcd, o3d.utility.DoubleVector(radii)))
        spinner.stop()

        mesh.remove_degenerate_triangles()
        mesh.remove_unreferenced_vertices()

        print(f"  [DSMBuilder] DSM: {len(np.asarray(mesh.triangles)):,} "
              f"triangles  ({time.time() - t0:.1f}s)")
        return mesh

    def _apply_cap(self, pts: np.ndarray) -> np.ndarray:
        if len(pts) <= self.max_pts:
            return pts
        mode = "FAST " if self.fast else ""
        print(f"  [DSMBuilder] {mode}Downsampling {len(pts):,} → "
              f"{self.max_pts:,} pts (BPA cap)")
        return pts[np.random.choice(len(pts), self.max_pts, replace=False)]

    def _estimate_radius(self, pts: np.ndarray) -> float:
        from scipy.spatial import KDTree
        n        = min(self.nn_sample, len(pts))
        idx      = np.random.choice(len(pts), n, replace=False)
        tree     = KDTree(pts[idx])
        dists, _ = tree.query(pts[idx], k=2)
        avg_nn   = float(np.median(dists[:, 1]))
        radius   = avg_nn * 2.5
        clamped  = max(MIN_RADIUS, min(radius, MAX_RADIUS))
        suffix   = f"  (clamped to {clamped:.4f}m)" if clamped != radius else ""
        print(f"  [DSMBuilder] Auto radius: median NN={avg_nn:.4f}m  "
              f"→ radius={radius:.4f}m{suffix}")
        return radius
