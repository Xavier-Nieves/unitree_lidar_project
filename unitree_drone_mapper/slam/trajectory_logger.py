"""Subscribe to SLAM odometry and path topics, save trajectory poses.

Runs alongside Point-LIO during flight to log the SLAM trajectory
for post-processing (camera pose estimation, cloud registration).
"""

import json
import numpy as np
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from utils.logger import setup_logger
from utils.ros_helpers import SENSOR_QOS

logger = setup_logger(__name__)


class TrajectoryLogger(Node):
    """ROS 2 node that subscribes to odometry and saves poses."""

    def __init__(self, output_dir: str, config: dict):
        """Initialize trajectory logger.

        Args:
            output_dir: Directory to save trajectory files.
            config: SLAM config dict.
        """
        super().__init__("trajectory_logger")

        self.output_dir = output_dir
        Path(output_dir).mkdir(parents=True, exist_ok=True)

        self.poses = []
        self.pose_count = 0

        from nav_msgs.msg import Odometry, Path as PathMsg

        self.odom_sub = self.create_subscription(
            Odometry,
            "/Odometry",
            self._odom_callback,
            SENSOR_QOS,
        )

        self.path_sub = self.create_subscription(
            PathMsg,
            "/path",
            self._path_callback,
            SENSOR_QOS,
        )

        self._latest_path = None
        self.get_logger().info(f"Trajectory logger started, saving to {output_dir}")

    def _odom_callback(self, msg):
        """Record each odometry pose."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamp = msg.header.stamp

        pose = {
            "timestamp": stamp.sec + stamp.nanosec * 1e-9,
            "position": [p.x, p.y, p.z],
            "orientation": [q.w, q.x, q.y, q.z],
        }
        self.poses.append(pose)
        self.pose_count += 1

        if self.pose_count % 100 == 0:
            self.get_logger().info(f"Recorded {self.pose_count} poses")

    def _path_callback(self, msg):
        """Store latest full path from Point-LIO."""
        self._latest_path = msg

    def save(self) -> str:
        """Save all recorded poses to a JSON file.

        Returns:
            Path to the saved trajectory file.
        """
        filepath = f"{self.output_dir}/trajectory.json"

        with open(filepath, "w") as f:
            json.dump({
                "pose_count": len(self.poses),
                "poses": self.poses,
            }, f, indent=2)

        logger.info(f"Saved {len(self.poses)} poses to {filepath}")
        return filepath

    def get_latest_pose(self) -> Optional[dict]:
        """Get the most recent SLAM pose."""
        if self.poses:
            return self.poses[-1]
        return None

    def get_distance_traveled(self) -> float:
        """Compute total distance traveled from the trajectory."""
        if len(self.poses) < 2:
            return 0.0

        total = 0.0
        for i in range(1, len(self.poses)):
            p0 = np.array(self.poses[i - 1]["position"])
            p1 = np.array(self.poses[i]["position"])
            total += np.linalg.norm(p1 - p0)

        return total
