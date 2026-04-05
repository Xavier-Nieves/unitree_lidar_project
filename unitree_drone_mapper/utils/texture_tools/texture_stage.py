"""
texture_stage.py — Stage 7 interface for the post-processing pipeline.

Wraps CameraModel, PoseInterpolator, and TextureProjector into a single
class that postprocess_mesh.py calls with the same interface pattern used
by all other pipeline stages (BagReader, MLSSmoother, etc.).

Usage in postprocess_mesh.py (Stage 7 — planned addition)
---------------------------------------------------------
    if camera_available and images_dir:
        from texture_tools.texture_stage import TextureProjectionStage

        log("[7/7] Projecting texture onto mesh...")
        texturer = TextureProjectionStage(
            calibration_path="config/camera_calibration.yaml"
        )
        textured_mesh = texturer.project(
            mesh      = combined_mesh,
            bag_path  = bag_path,
            pose_topic  = "/aft_mapped_to_init",
            image_topic = "/arducam/image_raw",
        )
        publisher.publish_textured(textured_mesh)
    else:
        log("[7/7] Camera not available — skipping texture projection")

Bag replay mode
---------------
  project() reads camera frames and SLAM poses directly from the recorded
  rosbag using the rosbags library (no live ROS needed).  This matches
  the offline post-processing model used by the rest of the pipeline.

Output
------
  Returns an open3d.geometry.TriangleMesh with vertex_colors populated.
  Saves a copy as textured_mesh.ply alongside the existing mesh_final.ply
  when save_output=True (default).

Dependencies
------------
  rosbags, open3d, numpy, opencv-python
  All available in the dronepi conda environment.
"""

from __future__ import annotations

import struct
import time
from pathlib import Path
from typing import Optional

import numpy as np

from .camera_model      import CameraModel
from .pose_interpolator import PoseInterpolator
from .texture_projector import TextureProjector


class TextureProjectionStage:
    """
    Stage 7 — texture projection orchestrator.

    Parameters
    ----------
    calibration_path : str or Path, optional
        Path to camera_calibration.yaml.  Defaults to the project-relative
        path resolved by CameraModel.
    use_distortion : bool
        Pass through to TextureProjector.  Default False.
    angle_weighting : bool
        Pass through to TextureProjector.  Default False.
    max_frames : int or None
        Maximum number of camera frames to process.  None = all frames.
        Useful for quick test runs during development.
    save_output : bool
        If True, write textured_mesh.ply to the bag directory on completion.
        Default True.
    """

    def __init__(
        self,
        calibration_path:  str | Path | None = None,
        use_distortion:    bool = False,
        angle_weighting:   bool = False,
        max_frames:        Optional[int] = None,
        save_output:       bool = True,
    ) -> None:
        self._calibration_path = calibration_path
        self._use_distortion   = use_distortion
        self._angle_weighting  = angle_weighting
        self._max_frames       = max_frames
        self._save_output      = save_output

    # ── Public API ─────────────────────────────────────────────────────────────

    def project(
        self,
        mesh,
        bag_path:    Path,
        pose_topic:  str = "/aft_mapped_to_init",
        image_topic: str = "/arducam/image_raw",
    ):
        """
        Project texture from bag frames onto the mesh.

        Parameters
        ----------
        mesh : open3d.geometry.TriangleMesh
            The LiDAR mesh to texture.  Vertex positions must be in the
            same world frame as the SLAM poses in the bag.
        bag_path : Path
            Path to the rosbag2 directory.
        pose_topic : str
            ROS topic name for SLAM odometry (nav_msgs/Odometry).
        image_topic : str
            ROS topic name for camera frames (sensor_msgs/Image).

        Returns
        -------
        open3d.geometry.TriangleMesh
            The input mesh with vertex_colors populated.

        Notes
        -----
        The method performs two passes over the bag:
          Pass 1 — Load all SLAM poses into PoseInterpolator.
          Pass 2 — Process each camera frame in order, projecting onto
                   the mesh and accumulating colour.

        Two passes are used rather than one interleaved pass because
        PoseInterpolator requires the bracketing poses on both sides of
        each camera timestamp.  A single forward pass would miss the
        right bracket for any camera frame that arrives before the next
        pose message.
        """
        t0 = time.time()

        camera = CameraModel(self._calibration_path)
        interp = PoseInterpolator()

        bag_path = Path(bag_path)
        if not bag_path.exists():
            raise FileNotFoundError(
                f"[TextureProjectionStage] Bag not found: {bag_path}"
            )

        # Pass 1 — load all SLAM poses
        print(f"  [TextureStage] Pass 1: loading SLAM poses from {pose_topic} ...")
        n_poses = self._load_poses(bag_path, pose_topic, interp)
        print(
            f"  [TextureStage] {n_poses:,} poses loaded  "
            f"({interp.time_range_s:.1f}s span)"
        )

        if n_poses < 2:
            print(
                "  [TextureStage] WARNING: fewer than 2 poses — "
                "cannot interpolate.  Check that the bag contains "
                f"the topic '{pose_topic}'."
            )
            return mesh

        # Initialise projector
        projector = TextureProjector(
            mesh            = mesh,
            camera          = camera,
            use_distortion  = self._use_distortion,
            angle_weighting = self._angle_weighting,
        )

        # Pass 2 — project each camera frame
        print(
            f"  [TextureStage] Pass 2: projecting frames from "
            f"{image_topic} ..."
        )
        n_frames, total_visible = self._project_frames(
            bag_path, image_topic, interp, projector, camera
        )
        elapsed = time.time() - t0

        print(
            f"  [TextureStage] {n_frames} frames processed  "
            f"avg_visible={total_visible // max(n_frames, 1):,}  "
            f"({elapsed:.1f}s total)"
        )

        # Finalise — normalise accumulators, apply grey to untextured vertices
        textured_mesh = projector.finalize()

        # Transfer vertex colours onto the original mesh topology
        import open3d as o3d
        mesh.vertex_colors = textured_mesh.vertex_colors

        # Save textured PLY alongside the existing mesh
        if self._save_output:
            out_path = bag_path / "textured_mesh.ply"
            o3d.io.write_triangle_mesh(str(out_path), mesh)
            print(f"  [TextureStage] Saved → {out_path}")

        return mesh

    # ── Private helpers ────────────────────────────────────────────────────────

    def _load_poses(
        self,
        bag_path:   Path,
        pose_topic: str,
        interp:     PoseInterpolator,
    ) -> int:
        """
        Read nav_msgs/Odometry messages from the bag and populate the
        PoseInterpolator buffer.

        Returns the number of poses loaded.
        """
        from rosbags.rosbag2  import Reader
        from rosbags.typesys  import Stores, get_typestore

        typestore = get_typestore(Stores.ROS2_HUMBLE)
        n = 0

        with Reader(str(bag_path)) as reader:
            connections = [
                c for c in reader.connections if c.topic == pose_topic
            ]
            if not connections:
                print(
                    f"  [TextureStage] Topic '{pose_topic}' not in bag.  "
                    f"Available: {[c.topic for c in reader.connections]}"
                )
                return 0

            for conn, ts_ns, raw in reader.messages(connections=connections):
                msg = typestore.deserialize_cdr(raw, conn.msgtype)
                pose_4x4 = self._odometry_to_matrix(msg)
                interp.add_pose(int(ts_ns), pose_4x4)
                n += 1

        return n

    def _project_frames(
        self,
        bag_path:    Path,
        image_topic: str,
        interp:      PoseInterpolator,
        projector:   TextureProjector,
        camera:      CameraModel,
    ):
        """
        Read sensor_msgs/Image messages from the bag and project each frame.

        Returns (n_frames_processed, total_visible_vertices).
        """
        from rosbags.rosbag2  import Reader
        from rosbags.typesys  import Stores, get_typestore

        typestore   = get_typestore(Stores.ROS2_HUMBLE)
        n_frames    = 0
        total_vis   = 0
        limit       = self._max_frames

        with Reader(str(bag_path)) as reader:
            connections = [
                c for c in reader.connections if c.topic == image_topic
            ]
            if not connections:
                print(
                    f"  [TextureStage] Topic '{image_topic}' not in bag.  "
                    f"Available: {[c.topic for c in reader.connections]}"
                )
                return 0, 0

            for conn, ts_ns, raw in reader.messages(connections=connections):
                if limit is not None and n_frames >= limit:
                    break

                msg      = typestore.deserialize_cdr(raw, conn.msgtype)
                frame_ts = (
                    int(msg.header.stamp.sec) * 1_000_000_000
                    + int(msg.header.stamp.nanosec)
                )

                # Look up interpolated drone pose at frame timestamp
                drone_pose = interp.get_pose_at(frame_ts)
                if drone_pose is None:
                    continue

                # Convert drone pose to camera world pose
                cam_pose = camera.get_camera_world_pose(drone_pose)

                # Decode image to BGR numpy array
                image = self._decode_image(msg)
                if image is None:
                    continue

                visible = projector.project_frame(image, cam_pose)
                total_vis += visible
                n_frames  += 1

                if n_frames % 10 == 0:
                    print(
                        f"\r  [TextureStage] Frame {n_frames}  "
                        f"visible={visible:,}     ",
                        end="", flush=True,
                    )

        print()
        return n_frames, total_vis

    @staticmethod
    def _odometry_to_matrix(msg) -> np.ndarray:
        """
        Convert a nav_msgs/Odometry message to a 4×4 homogeneous transform.

        Point-LIO publishes poses as position + quaternion in the
        pose.pose field of the Odometry message.  The quaternion uses
        the ROS convention [x, y, z, w].
        """
        from scipy.spatial.transform import Rotation

        p   = msg.pose.pose.position
        q   = msg.pose.pose.orientation

        R   = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T   = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3,  3] = [p.x, p.y, p.z]
        return T

    @staticmethod
    def _decode_image(msg) -> Optional[np.ndarray]:
        """
        Decode a sensor_msgs/Image message to a BGR numpy array.

        Handles bgr8 and rgb8 encodings, which are the two formats
        produced by ArducamNode in flight/arducam_node.py.
        Returns None for unsupported encodings with a warning.
        """
        encoding = msg.encoding.lower()
        raw      = bytes(msg.data)
        h, w     = int(msg.height), int(msg.width)

        if encoding == "bgr8":
            return np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3).copy()

        if encoding == "rgb8":
            rgb = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3).copy()
            import cv2
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if encoding == "mono8":
            gray = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
            import cv2
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        print(
            f"  [TextureStage] Unsupported image encoding '{encoding}' "
            f"— skipping frame"
        )
        return None
