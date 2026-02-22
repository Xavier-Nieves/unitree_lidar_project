"""Unitree L1 LiDAR launch/config wrapper.

Manages the Unitree L1 LiDAR ROS 2 driver node lifecycle.
The actual driver is the C++ unitree_lidar_ros2_node — this module
provides a Python interface to launch, configure, and monitor it.
"""

import subprocess
import signal
import os
from typing import Optional

from utils.logger import setup_logger

logger = setup_logger(__name__)


class LidarDriver:
    """Wrapper for launching and managing the Unitree L1 LiDAR driver."""

    def __init__(self, config: dict):
        """Initialize LiDAR driver manager.

        Args:
            config: LiDAR config dict from config.yaml (lidar section).
        """
        self.port = config.get("device", "/dev/ttyUSB0")
        self.range_min = config.get("range_min", 0.0)
        self.range_max = config.get("range_max", 50.0)
        self.cloud_scan_num = config.get("cloud_scan_num", 18)
        self.rotate_yaw_bias = config.get("rotate_yaw_bias", 0.0)
        self.range_scale = config.get("range_scale", 0.001)
        self._process: Optional[subprocess.Popen] = None

    def start(self) -> bool:
        """Launch the LiDAR driver node.

        Returns:
            True if started successfully.
        """
        if self._process and self._process.poll() is None:
            logger.warning("LiDAR driver already running")
            return True

        cmd = [
            "ros2", "run", "unitree_lidar_ros2", "unitree_lidar_ros2_node",
            "--ros-args",
            "-p", f"port:={self.port}",
            "-p", f"rotate_yaw_bias:={self.rotate_yaw_bias}",
            "-p", f"range_scale:={self.range_scale}",
            "-p", f"range_bias:=0.0",
            "-p", f"range_max:={self.range_max}",
            "-p", f"range_min:={self.range_min}",
            "-p", "cloud_frame:=unilidar_lidar",
            "-p", "cloud_topic:=unilidar/cloud",
            "-p", f"cloud_scan_num:={self.cloud_scan_num}",
            "-p", "imu_frame:=unilidar_imu",
            "-p", "imu_topic:=unilidar/imu",
        ]

        logger.info(f"Starting LiDAR driver on {self.port}")
        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            logger.info(f"LiDAR driver started (PID: {self._process.pid})")
            return True
        except FileNotFoundError:
            logger.error("unitree_lidar_ros2 package not found. Is it built and sourced?")
            return False

    def stop(self):
        """Stop the LiDAR driver node."""
        if self._process and self._process.poll() is None:
            logger.info("Stopping LiDAR driver...")
            os.killpg(os.getpgid(self._process.pid), signal.SIGINT)
            self._process.wait(timeout=10)
            logger.info("LiDAR driver stopped")
        self._process = None

    def is_running(self) -> bool:
        """Check if the LiDAR driver process is alive."""
        return self._process is not None and self._process.poll() is None

    def check_device(self) -> bool:
        """Check if the LiDAR device is physically connected."""
        exists = os.path.exists(self.port)
        if not exists:
            logger.warning(f"LiDAR device not found at {self.port}")
        return exists
