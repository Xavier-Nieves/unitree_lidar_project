"""Subscribe to registered point clouds and accumulate/save them.

Runs alongside Point-LIO during flight to save the growing
point cloud map for post-processing.
"""

import numpy as np
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from utils.logger import setup_logger
from utils.ros_helpers import SENSOR_QOS

logger = setup_logger(__name__)


class CloudAccumulator(Node):
    """ROS 2 node that accumulates registered point clouds."""

    def __init__(self, output_dir: str, config: dict):
        """Initialize cloud accumulator.

        Args:
            output_dir: Directory to save point cloud files.
            config: SLAM config dict.
        """
        super().__init__("cloud_accumulator")

        self.output_dir = output_dir
        Path(output_dir).mkdir(parents=True, exist_ok=True)

        self.all_points = []
        self.frame_count = 0
        self.point_count = 0

        from sensor_msgs.msg import PointCloud2

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            "/cloud_registered",
            self._cloud_callback,
            SENSOR_QOS,
        )

        self.get_logger().info(f"Cloud accumulator started, saving to {output_dir}")

    def _cloud_callback(self, msg):
        """Accumulate points from each registered cloud message."""
        import sensor_msgs_py.point_cloud2 as pc2

        points = []
        for point in pc2.read_points(msg, skip_nans=True,
                                     field_names=("x", "y", "z")):
            points.append([point[0], point[1], point[2]])

        if points:
            self.all_points.extend(points)
            self.point_count = len(self.all_points)
            self.frame_count += 1

            if self.frame_count % 10 == 0:
                self.get_logger().info(
                    f"Frames: {self.frame_count} | Points: {self.point_count:,}"
                )

    def save(self, filename: str = "cloud_raw.pcd") -> Optional[str]:
        """Save the accumulated cloud to a PCD file.

        Returns:
            Path to saved file, or None if no points.
        """
        if not self.all_points:
            logger.warning("No points to save")
            return None

        import open3d as o3d

        filepath = f"{self.output_dir}/{filename}"
        points_array = np.array(self.all_points, dtype=np.float64)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_array)
        o3d.io.write_point_cloud(filepath, pcd)

        logger.info(f"Saved {self.point_count:,} points to {filepath}")
        return filepath

    def get_point_count(self) -> int:
        """Return current accumulated point count."""
        return self.point_count

    def get_frame_count(self) -> int:
        """Return number of cloud frames received."""
        return self.frame_count

    def clear(self):
        """Clear accumulated points."""
        self.all_points = []
        self.point_count = 0
        self.frame_count = 0
