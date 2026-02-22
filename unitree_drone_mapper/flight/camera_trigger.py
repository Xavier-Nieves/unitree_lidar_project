"""Insta360 X3 camera trigger during flight.

Triggers photo captures at waypoints or at regular distance intervals.
Logs metadata (GPS/position, timestamp, pose) for each capture.
"""

import json
import time
from pathlib import Path
from typing import Optional, List

from drivers.insta360_driver import Insta360Driver
from utils.logger import setup_logger

logger = setup_logger(__name__)


class CameraTrigger:
    """Manages Insta360 X3 capture during flight missions."""

    def __init__(self, camera_driver: Insta360Driver, config: dict):
        """Initialize camera trigger.

        Args:
            camera_driver: Initialized Insta360Driver instance.
            config: Camera config dict from config.yaml.
        """
        self.camera = camera_driver
        self.capture_interval_m = config.get("camera", {}).get("capture_interval_m", 2.0)

        self._captures: List[dict] = []
        self._last_capture_position = None
        self._output_dir: Optional[str] = None
        self._metadata_file: Optional[str] = None

    def setup(self, images_dir: str, metadata_dir: str):
        """Configure output directories for this session.

        Args:
            images_dir: Directory to save captured images.
            metadata_dir: Directory to save capture metadata.
        """
        self._output_dir = images_dir
        Path(images_dir).mkdir(parents=True, exist_ok=True)
        Path(metadata_dir).mkdir(parents=True, exist_ok=True)
        self._metadata_file = f"{metadata_dir}/capture_log.json"
        logger.info(f"Camera trigger configured: images -> {images_dir}")

    def trigger_at_waypoint(self, waypoint_idx: int, position: list,
                            orientation: list = None) -> bool:
        """Capture a photo at a waypoint.

        Args:
            waypoint_idx: Current waypoint index.
            position: [x, y, z] position at capture time.
            orientation: [w, x, y, z] quaternion (optional).

        Returns:
            True if capture succeeded.
        """
        if not self._output_dir:
            logger.warning("Camera trigger not set up — call setup() first")
            return False

        filepath = self.camera.capture_and_download(self._output_dir)

        if filepath:
            capture_info = {
                "index": len(self._captures),
                "waypoint": waypoint_idx,
                "timestamp": time.time(),
                "position": position,
                "orientation": orientation,
                "filepath": filepath,
            }
            self._captures.append(capture_info)
            logger.info(
                f"Capture {len(self._captures)} at waypoint {waypoint_idx}: {filepath}"
            )
            return True

        logger.warning(f"Capture failed at waypoint {waypoint_idx}")
        return False

    def trigger_by_distance(self, current_position: list,
                            orientation: list = None) -> bool:
        """Trigger a capture if we've moved far enough since the last one.

        Args:
            current_position: [x, y, z] current position.
            orientation: [w, x, y, z] quaternion (optional).

        Returns:
            True if a capture was triggered.
        """
        import numpy as np

        if self._last_capture_position is None:
            self._last_capture_position = current_position
            return self.trigger_at_waypoint(-1, current_position, orientation)

        dist = np.linalg.norm(
            np.array(current_position[:2]) - np.array(self._last_capture_position[:2])
        )

        if dist >= self.capture_interval_m:
            self._last_capture_position = current_position
            return self.trigger_at_waypoint(-1, current_position, orientation)

        return False

    def save_metadata(self):
        """Save all capture metadata to JSON."""
        if self._metadata_file and self._captures:
            with open(self._metadata_file, "w") as f:
                json.dump({
                    "capture_count": len(self._captures),
                    "captures": self._captures,
                }, f, indent=2)
            logger.info(f"Saved {len(self._captures)} capture records")

    @property
    def capture_count(self) -> int:
        return len(self._captures)
