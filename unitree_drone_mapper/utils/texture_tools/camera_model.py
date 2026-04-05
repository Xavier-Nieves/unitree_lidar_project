"""
camera_model.py — IMX477 camera intrinsic and extrinsic model.

Single source of truth for all camera geometry used by the texture
pipeline.  Reads config/camera_calibration.yaml once at construction
and exposes two methods used by TextureProjector:

  get_camera_world_pose(drone_world_pose)
      Applies the rigid body extrinsic transform to convert a SLAM
      drone pose (4×4 matrix, world frame) into the corresponding
      camera pose in the world frame.

  project_point(point_cam)
      Applies the pinhole projection model with the loaded intrinsics
      to convert a point in the camera frame into pixel coordinates.
      Returns None if the point is behind the camera or outside the
      image boundary.

Coordinate conventions
----------------------
  Extrinsic transform T_cam_body:
      Transforms a point expressed in the drone body frame into the
      camera frame.  Built from the YAML translation and rotation_rpy
      (roll/pitch/yaw in radians, ZYX convention) using scipy Rotation.

  Projection (pinhole model):
      u = fx * (X_c / Z_c) + cx
      v = fy * (Y_c / Z_c) + cy

      Points with Z_c <= 0 are behind the camera and are rejected.
      Points projecting outside [0, width) × [0, height) are rejected.

  Distortion:
      cv2.undistortPoints() is available via undistort_pixel() for
      callers that want lens correction before colour sampling.  The
      projector calls this when the --use-distortion flag is set.

Dependencies
------------
  numpy, scipy, pyyaml, opencv-python (cv2)
  All available in the dronepi conda environment.

Source
------
  Intrinsic calibration: ~/calibration/imx477_calibration.json
  (copied to config/camera_calibration.yaml after each calibration run)
  RMS reprojection error at time of capture: 1.99 px.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import yaml


# ── Default calibration path ──────────────────────────────────────────────────
# Resolved relative to this file so the class works regardless of the
# working directory at call time.
#
# File layout:
#   unitree_drone_mapper/
#     config/
#       camera_calibration.yaml   ← loaded here
#     utils/
#       texture_tools/
#         camera_model.py         ← this file
#
# parents[0] = .../texture_tools/
# parents[1] = .../utils/
# parents[2] = .../unitree_drone_mapper/
_DEFAULT_CALIB_PATH = (
    Path(__file__).resolve().parents[2]
    / "config"
    / "camera_calibration.yaml"
)


class CameraModel:
    """
    IMX477 camera model: intrinsics, extrinsics, and projection.

    Parameters
    ----------
    calibration_path : str or Path, optional
        Path to camera_calibration.yaml.
        Defaults to unitree_drone_mapper/config/camera_calibration.yaml.

    Raises
    ------
    FileNotFoundError
        If the calibration file does not exist at the resolved path.
    KeyError
        If a required field is missing from the YAML.
    ValueError
        If image dimensions, focal lengths, or principal point are
        not positive numbers.

    Attributes (read-only after construction)
    -----------------------------------------
    fx, fy : float          Focal lengths in pixels.
    cx, cy : float          Principal point in pixels.
    width, height : int     Image dimensions in pixels.
    dist_coeffs : np.ndarray  (5,) distortion coefficients [k1,k2,p1,p2,k3].
    T_cam_body : np.ndarray   (4,4) rigid transform, drone body → camera frame.
    """

    def __init__(self, calibration_path: str | Path | None = None) -> None:
        path = Path(calibration_path) if calibration_path else _DEFAULT_CALIB_PATH

        if not path.exists():
            raise FileNotFoundError(
                f"[CameraModel] Calibration file not found: {path}\n"
                f"  Run the calibration script and copy the result to:\n"
                f"  {_DEFAULT_CALIB_PATH}"
            )

        with open(path) as f:
            raw = yaml.safe_load(f)

        self._load_intrinsics(raw, path)
        self._load_extrinsics(raw, path)

        print(
            f"  [CameraModel] Loaded from {path.name}\n"
            f"    fx={self.fx:.2f}  fy={self.fy:.2f}  "
            f"cx={self.cx:.2f}  cy={self.cy:.2f}\n"
            f"    image={self.width}×{self.height}  "
            f"dist={[round(d, 4) for d in self.dist_coeffs.tolist()]}\n"
            f"    T_cam_body translation="
            f"{self.T_cam_body[:3, 3].round(4).tolist()}"
        )

    # ── Public API ─────────────────────────────────────────────────────────────

    def get_camera_world_pose(self, drone_world_pose: np.ndarray) -> np.ndarray:
        """
        Convert a drone body pose in the world frame to the camera world pose.

        The SLAM system (Point-LIO) publishes poses of the drone body frame.
        The camera is rigidly offset from the body frame by T_cam_body.
        Composing the two gives the camera pose in the world frame:

            T_cam_world = T_drone_world @ inv(T_cam_body)

        Equivalently, the camera world pose is:

            T_cam_world = drone_world_pose @ T_body_cam

        where T_body_cam = inv(T_cam_body).

        Parameters
        ----------
        drone_world_pose : np.ndarray, shape (4, 4)
            Homogeneous transform of the drone body in the world frame,
            as published by Point-LIO on /aft_mapped_to_init.

        Returns
        -------
        np.ndarray, shape (4, 4)
            Homogeneous transform of the camera in the world frame.
        """
        return drone_world_pose @ self._T_body_cam

    def project_point(
        self, point_cam: np.ndarray
    ) -> Optional[Tuple[int, int]]:
        """
        Project a point in the camera frame to pixel coordinates.

        Uses the pinhole model without distortion correction.  For
        distortion-corrected projection, undistort the pixel coordinates
        afterward with undistort_pixel().

        Parameters
        ----------
        point_cam : np.ndarray, shape (3,)
            Point expressed in the camera coordinate frame (X right,
            Y down, Z forward — OpenCV convention).

        Returns
        -------
        (u, v) : tuple of int
            Integer pixel coordinates, u along the image width axis,
            v along the image height axis.
        None
            If the point is behind the camera (Z <= 0) or projects
            outside the image boundary.
        """
        z = float(point_cam[2])
        if z <= 0.0:
            return None

        u = self.fx * (float(point_cam[0]) / z) + self.cx
        v = self.fy * (float(point_cam[1]) / z) + self.cy

        u_int = int(u)
        v_int = int(v)

        if 0 <= u_int < self.width and 0 <= v_int < self.height:
            return u_int, v_int
        return None

    def undistort_pixel(
        self, u: float, v: float
    ) -> Tuple[float, float]:
        """
        Apply lens distortion correction to a raw pixel coordinate.

        Uses cv2.undistortPoints() with the loaded distortion coefficients.
        Call this after project_point() when more accurate colour sampling
        is required (e.g. for close-range captures where barrel distortion
        is visible).

        Parameters
        ----------
        u, v : float
            Raw pixel coordinates from project_point().

        Returns
        -------
        (u_corr, v_corr) : tuple of float
            Distortion-corrected pixel coordinates in the same image space.
        """
        import cv2

        K = np.array([
            [self.fx, 0.0,     self.cx],
            [0.0,     self.fy, self.cy],
            [0.0,     0.0,     1.0   ],
        ], dtype=np.float64)

        pts = np.array([[[u, v]]], dtype=np.float64)
        corrected = cv2.undistortPoints(pts, K, self.dist_coeffs, P=K)
        u_c = float(corrected[0, 0, 0])
        v_c = float(corrected[0, 0, 1])
        return u_c, v_c

    def transform_points_to_camera(
        self, points_world: np.ndarray, camera_world_pose: np.ndarray
    ) -> np.ndarray:
        """
        Transform an array of world-frame points into the camera frame.

        Parameters
        ----------
        points_world : np.ndarray, shape (N, 3)
            Points in the world (SLAM map) frame.
        camera_world_pose : np.ndarray, shape (4, 4)
            Camera pose in the world frame, as returned by
            get_camera_world_pose().

        Returns
        -------
        np.ndarray, shape (N, 3)
            Points expressed in the camera frame.
        """
        # Invert camera world pose to get world→camera transform
        T_world_to_cam = self._invert_transform(camera_world_pose)

        # Homogeneous multiplication: (N, 4) @ (4, 4).T → (N, 4)
        ones      = np.ones((len(points_world), 1), dtype=np.float64)
        pts_h     = np.hstack([points_world.astype(np.float64), ones])
        pts_cam_h = (T_world_to_cam @ pts_h.T).T
        return pts_cam_h[:, :3]

    # ── Private helpers ────────────────────────────────────────────────────────

    def _load_intrinsics(self, raw: dict, path: Path) -> None:
        """Parse the camera: block from the YAML and validate values."""
        cam = raw.get("camera", {})

        self.fx     = float(cam["fx"])
        self.fy     = float(cam["fy"])
        self.cx     = float(cam["cx"])
        self.cy     = float(cam["cy"])
        self.width  = int(cam["image_width"])
        self.height = int(cam["image_height"])
        self.dist_coeffs = np.array(
            cam["dist_coeffs"], dtype=np.float64
        ).ravel()

        # Sanity checks — catch swapped YAML or unit errors early
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError(
                f"[CameraModel] Focal lengths must be positive "
                f"(got fx={self.fx}, fy={self.fy}) in {path}"
            )
        if not (0 < self.cx < self.width) or not (0 < self.cy < self.height):
            raise ValueError(
                f"[CameraModel] Principal point ({self.cx}, {self.cy}) "
                f"must be inside the image ({self.width}×{self.height}) in {path}"
            )
        if len(self.dist_coeffs) not in (4, 5, 8):
            raise ValueError(
                f"[CameraModel] dist_coeffs must have 4, 5, or 8 values "
                f"(got {len(self.dist_coeffs)}) in {path}"
            )

    def _load_extrinsics(self, raw: dict, path: Path) -> None:
        """
        Parse the extrinsic: block and build the 4×4 rigid transform
        T_cam_body (drone body frame → camera frame).

        The YAML stores:
          translation : [x, y, z]   metres, body frame origin to camera origin
          rotation_rpy: [r, p, y]   radians, ZYX Euler angles

        scipy.spatial.transform.Rotation is used for RPY → rotation matrix
        conversion because it handles gimbal lock correctly and is numerically
        stable for small angles.
        """
        from scipy.spatial.transform import Rotation

        ext = raw.get("extrinsic", {})

        translation = np.array(ext["translation"], dtype=np.float64).ravel()
        rpy         = np.array(ext["rotation_rpy"], dtype=np.float64).ravel()

        if len(translation) != 3:
            raise ValueError(
                f"[CameraModel] extrinsic.translation must have 3 values "
                f"(got {len(translation)}) in {path}"
            )
        if len(rpy) != 3:
            raise ValueError(
                f"[CameraModel] extrinsic.rotation_rpy must have 3 values "
                f"(got {len(rpy)}) in {path}"
            )

        # Build T_cam_body: transforms a point in body frame to camera frame
        R = Rotation.from_euler("xyz", rpy, degrees=False).as_matrix()

        self.T_cam_body = np.eye(4, dtype=np.float64)
        self.T_cam_body[:3, :3] = R
        self.T_cam_body[:3,  3] = translation

        # Pre-compute the inverse (body frame → camera → body frame is frequent)
        self._T_body_cam = self._invert_transform(self.T_cam_body)

    @staticmethod
    def _invert_transform(T: np.ndarray) -> np.ndarray:
        """
        Invert a 4×4 rigid body transform analytically.

        For a transform T = [R | t; 0 | 1], the inverse is:
            T_inv = [R.T | -R.T @ t; 0 | 1]

        This is numerically superior to np.linalg.inv() for rotation
        matrices because it exploits the orthogonality of R.
        """
        T_inv = np.eye(4, dtype=np.float64)
        R     = T[:3, :3]
        t     = T[:3,  3]
        T_inv[:3, :3] =  R.T
        T_inv[:3,  3] = -R.T @ t
        return T_inv
