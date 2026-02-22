"""ArUco marker detection + precision landing controller.

Merges functionality from the original aruco_landing_node.py and
landing_controller.py into a single module. Detects ArUco markers
using the downward camera and guides the drone to land precisely
on the marker.
"""

import numpy as np
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from utils.logger import setup_logger
from utils.ros_helpers import SENSOR_QOS

logger = setup_logger(__name__)


class PrecisionLanding(Node):
    """ROS 2 node for ArUco-guided precision landing."""

    def __init__(self, px4_interface, config: dict):
        """Initialize precision landing.

        Args:
            px4_interface: PX4Interface node for sending velocity commands.
            config: Config dict.
        """
        super().__init__("precision_landing")

        self.px4 = px4_interface
        self.target_marker_id = 0  # ArUco marker ID to land on

        # PID gains for landing controller
        self.kp_xy = 0.5
        self.kp_z = 0.3
        self.landing_speed = 0.3  # m/s descent rate
        self.position_threshold = 0.1  # meters — close enough to center

        # State
        self._marker_detected = False
        self._marker_position = None  # [x, y] offset from center in meters
        self._marker_distance = None  # distance to marker

        # Subscribe to camera for ArUco detection
        from sensor_msgs.msg import Image

        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self._image_callback,
            SENSOR_QOS,
        )

        self.get_logger().info("Precision landing initialized")

    def _image_callback(self, msg):
        """Process camera image for ArUco detection."""
        try:
            import cv2
            from cv_bridge import CvBridge

            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Detect ArUco markers
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            params = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            corners, ids, rejected = detector.detectMarkers(frame)

            if ids is not None and self.target_marker_id in ids:
                idx = list(ids.flatten()).index(self.target_marker_id)
                marker_corners = corners[idx][0]

                # Calculate center of marker
                center_x = np.mean(marker_corners[:, 0])
                center_y = np.mean(marker_corners[:, 1])

                # Convert to offset from image center (normalized)
                h, w = frame.shape[:2]
                offset_x = (center_x - w / 2) / (w / 2)  # -1 to 1
                offset_y = (center_y - h / 2) / (h / 2)

                # Estimate distance from marker size
                marker_width = np.linalg.norm(marker_corners[0] - marker_corners[1])
                # Approximate: known marker size / apparent size * focal length
                self._marker_distance = 500.0 / max(marker_width, 1.0)

                self._marker_position = [offset_x, offset_y]
                self._marker_detected = True
            else:
                self._marker_detected = False

        except Exception as e:
            self.get_logger().error(f"ArUco detection error: {e}")

    def execute_landing(self) -> bool:
        """Execute one step of the precision landing controller.

        Call this in a loop during the landing phase.

        Returns:
            True when landing is complete (on the ground).
        """
        if not self._marker_detected or self._marker_position is None:
            # No marker visible — hold position
            return False

        offset_x, offset_y = self._marker_position

        # P controller for horizontal correction
        vx = -self.kp_xy * offset_y  # camera y -> drone forward
        vy = -self.kp_xy * offset_x  # camera x -> drone right

        # Descend if centered enough
        if abs(offset_x) < self.position_threshold and abs(offset_y) < self.position_threshold:
            vz = self.landing_speed  # NED: positive is down
        else:
            vz = 0.0  # Hold altitude while centering

        # Send velocity command (would need velocity setpoint in PX4)
        self.px4.goto_position(
            self.px4.position[0] + vx * 0.1,
            self.px4.position[1] + vy * 0.1,
            self.px4.position[2] + vz * 0.1,
        )

        # Check if we've landed (altitude near zero)
        if abs(self.px4.position[2]) < 0.3:
            logger.info("Precision landing complete!")
            return True

        return False

    @property
    def marker_detected(self) -> bool:
        return self._marker_detected

    @property
    def marker_offset(self) -> Optional[Tuple[float, float]]:
        if self._marker_position:
            return tuple(self._marker_position)
        return None
