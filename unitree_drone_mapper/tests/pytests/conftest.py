"""Shared pytest fixtures for mesh_tools unit tests.

sys.path is extended here so that 'mesh_tools' is importable regardless
of which directory pytest is invoked from. pytest.ini sets testpaths to
this directory, meaning pytest always runs from the project root
(~/unitree_lidar_project), so we resolve utils/ relative to this file.

These fixtures produce synthetic point clouds that work without
any hardware, ROS, PDAL, or open3d.  All heavy-dep tests that
need open3d are skipped automatically when the library is absent.
"""

import sys
from pathlib import Path

# Add unitree_drone_mapper/utils/ to sys.path so test files can do:
#   from mesh_tools.ground_classifier import GroundClassifier
# without needing to know the working directory at invocation time.
_UTILS_DIR = Path(__file__).resolve().parents[2] / "utils"
if str(_UTILS_DIR) not in sys.path:
    sys.path.insert(0, str(_UTILS_DIR))

import numpy as np
import pytest


@pytest.fixture
def flat_cloud():
    """200×200 flat ground plane at Z≈0 with small Gaussian noise."""
    rng = np.random.default_rng(42)
    x = np.linspace(0, 10, 50)
    y = np.linspace(0, 10, 50)
    xx, yy = np.meshgrid(x, y)
    zz = rng.normal(0.0, 0.02, xx.shape)
    return np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()]).astype(np.float32)


@pytest.fixture
def mixed_cloud():
    """Flat ground at Z≈0 plus a box of objects at Z∈[1, 3]."""
    rng = np.random.default_rng(42)
    x = np.linspace(0, 10, 40)
    y = np.linspace(0, 10, 40)
    xx, yy = np.meshgrid(x, y)
    ground = np.column_stack([
        xx.ravel(), yy.ravel(),
        rng.normal(0.0, 0.02, xx.size),
    ]).astype(np.float32)
    obj_x = rng.uniform(3, 7, 300)
    obj_y = rng.uniform(3, 7, 300)
    obj_z = rng.uniform(1.0, 3.0, 300)
    objects = np.column_stack([obj_x, obj_y, obj_z]).astype(np.float32)
    return np.vstack([ground, objects])


@pytest.fixture
def tiny_cloud():
    """Minimal 5-point cloud — for boundary / too-few-points tests."""
    return np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.5, 0.5, 0.1],
    ], dtype=np.float32)
