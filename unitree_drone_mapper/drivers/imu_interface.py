"""IMU data handling interface.

For the Unitree L1, the IMU is built into the LiDAR and published on
/unilidar/imu by the LiDAR driver. This module provides utilities for
reading and processing IMU data when needed independently.
"""

import numpy as np
from typing import Optional, Callable

from utils.logger import setup_logger

logger = setup_logger(__name__)


class IMUInterface:
    """Interface for subscribing to and processing IMU data."""

    def __init__(self, config: dict):
        """Initialize IMU interface.

        Args:
            config: IMU config dict from config.yaml (imu section).
        """
        self.source = config.get("source", "lidar_internal")
        self.topic = "/unilidar/imu"
        self._latest_data = None
        self._subscription = None
        self._callback: Optional[Callable] = None

    def setup_subscriber(self, node):
        """Create a ROS 2 subscription for IMU data.

        Args:
            node: ROS 2 node to attach the subscription to.
        """
        from sensor_msgs.msg import Imu
        from utils.ros_helpers import SENSOR_QOS

        self._subscription = node.create_subscription(
            Imu,
            self.topic,
            self._imu_callback,
            SENSOR_QOS,
        )
        logger.info(f"IMU subscriber created on {self.topic} (source: {self.source})")

    def _imu_callback(self, msg):
        """Store latest IMU message and forward to user callback."""
        self._latest_data = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "orientation": [
                msg.orientation.w,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
            ],
            "angular_velocity": [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            "linear_acceleration": [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
        }

        if self._callback:
            self._callback(self._latest_data)

    def set_callback(self, callback: Callable):
        """Register an external callback for IMU data."""
        self._callback = callback

    def get_latest(self) -> Optional[dict]:
        """Get the most recent IMU reading."""
        return self._latest_data

    def get_gravity_vector(self) -> np.ndarray:
        """Estimate gravity direction from latest accelerometer reading.

        Returns:
            Normalized gravity vector in sensor frame, or [0, 0, -9.81] default.
        """
        if self._latest_data:
            acc = np.array(self._latest_data["linear_acceleration"])
            norm = np.linalg.norm(acc)
            if norm > 0:
                return acc / norm * 9.81
        return np.array([0.0, 0.0, -9.81])
