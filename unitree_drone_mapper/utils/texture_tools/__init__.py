"""
texture_tools/ — Post-flight texture projection pipeline for DronePi.

Adds per-vertex RGB colour to the LiDAR mesh by projecting IMX477
camera frames onto mesh vertices using the drone's SLAM pose history.

Modules
-------
  camera_model.py       CameraModel — loads intrinsics and extrinsics,
                        projects 3-D points to pixel coordinates.

  pose_interpolator.py  PoseInterpolator — SLERP interpolation of the
                        SLAM pose history to match camera timestamps.

  texture_projector.py  TextureProjector — per-vertex colour accumulation
                        across all frames, with Z-buffer occlusion culling.

  texture_stage.py      TextureProjectionStage — Stage 7 interface called
                        by postprocess_mesh.py.

Public exports (convenience imports for postprocess_mesh.py)
------------------------------------------------------------
    from texture_tools import CameraModel, PoseInterpolator
    from texture_tools import TextureProjector, TextureProjectionStage

Architecture notes
------------------
  All classes are pure Python with no ROS dependency so they run
  identically during offline bag replay and in unit tests.
  Open3D is imported lazily inside TextureProjector.build() so
  camera_model.py and pose_interpolator.py remain importable on
  development machines that do not have Open3D installed.
"""

from .camera_model        import CameraModel
from .pose_interpolator   import PoseInterpolator
from .texture_projector   import TextureProjector
from .texture_stage       import TextureProjectionStage

__all__ = [
    "CameraModel",
    "PoseInterpolator",
    "TextureProjector",
    "TextureProjectionStage",
]
