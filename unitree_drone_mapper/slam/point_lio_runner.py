"""Launch and manage the Point-LIO SLAM process.

Point-LIO is a C++ ROS 2 node that performs LiDAR-inertial odometry.
This module wraps its lifecycle — launching, monitoring, and stopping it.
"""

import subprocess
import signal
import os
from typing import Optional

from utils.logger import setup_logger

logger = setup_logger(__name__)


class PointLIORunner:
    """Manager for the Point-LIO SLAM process."""

    def __init__(self, config: dict):
        """Initialize Point-LIO runner.

        Args:
            config: SLAM config dict from config.yaml (slam section).
        """
        self.config_file = config.get("point_lio_config", "config/point_lio_params.yaml")
        self.use_imu_as_input = config.get("use_imu_as_input", False)
        self.prop_at_freq_of_imu = config.get("prop_at_freq_of_imu", True)
        self.point_filter_num = config.get("point_filter_num", 1)
        self.filter_size_surf = config.get("filter_size_surf", 0.1)
        self.filter_size_map = config.get("filter_size_map", 0.1)
        self._process: Optional[subprocess.Popen] = None

    def start(self, save_pcd: bool = True) -> bool:
        """Launch Point-LIO as a subprocess.

        Args:
            save_pcd: Whether Point-LIO should save PCD maps.

        Returns:
            True if started successfully.
        """
        if self._process and self._process.poll() is None:
            logger.warning("Point-LIO already running")
            return True

        cmd = [
            "ros2", "run", "point_lio", "pointlio_mapping",
            "--ros-args",
            "--params-file", self.config_file,
            "-p", f"use_imu_as_input:={'true' if self.use_imu_as_input else 'false'}",
            "-p", f"prop_at_freq_of_imu:={'true' if self.prop_at_freq_of_imu else 'false'}",
            "-p", "check_satu:=true",
            "-p", "init_map_size:=10",
            "-p", f"point_filter_num:={self.point_filter_num}",
            "-p", "space_down_sample:=true",
            "-p", f"filter_size_surf:={self.filter_size_surf}",
            "-p", f"filter_size_map:={self.filter_size_map}",
            "-p", "cube_side_length:=1000.0",
            "-p", "runtime_pos_log_enable:=false",
            "-p", f"pcd_save.pcd_save_en:={'true' if save_pcd else 'false'}",
        ]

        logger.info("Starting Point-LIO SLAM...")
        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            logger.info(f"Point-LIO started (PID: {self._process.pid})")
            return True
        except FileNotFoundError:
            logger.error(
                "Point-LIO package not found. "
                "Ensure point_lio is built and workspace is sourced."
            )
            return False

    def stop(self):
        """Gracefully stop the Point-LIO process."""
        if self._process and self._process.poll() is None:
            logger.info("Stopping Point-LIO...")
            os.killpg(os.getpgid(self._process.pid), signal.SIGINT)
            try:
                self._process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                logger.warning("Point-LIO did not stop gracefully, sending SIGKILL")
                os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
                self._process.wait(timeout=5)
            logger.info("Point-LIO stopped")
        self._process = None

    def is_running(self) -> bool:
        """Check if Point-LIO is currently running."""
        return self._process is not None and self._process.poll() is None

    def get_return_code(self) -> Optional[int]:
        """Get the exit code of Point-LIO (None if still running)."""
        if self._process:
            return self._process.poll()
        return None
