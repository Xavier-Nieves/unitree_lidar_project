"""LiDAR-based collision avoidance for PX4.

Subscribes to the LiDAR point cloud, detects obstacles within a
configurable distance threshold, and publishes ObstacleDistance
messages to PX4 for path planning / emergency stop.
"""

import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node

from utils.logger import setup_logger
from utils.ros_helpers import SENSOR_QOS, RELIABLE_QOS

logger = setup_logger(__name__)

# 72 sectors around the drone (5 degrees each, 360/5 = 72)
NUM_SECTORS = 72
SECTOR_ANGLE = 360.0 / NUM_SECTORS
MAX_RANGE_CM = 5000  # 50 meters in centimeters


class CollisionAvoidance(Node):
    """ROS 2 node for LiDAR-based obstacle detection and PX4 integration."""

    def __init__(self, config: dict):
        """Initialize collision avoidance.

        Args:
            config: Flight config dict with collision parameters.
        """
        super().__init__("collision_avoidance")

        self.min_distance = config.get("flight", {}).get("collision_min_distance_m", 2.0)
        self.enabled = True

        # Obstacle sectors (distance to nearest obstacle per sector)
        self._sectors = np.full(NUM_SECTORS, MAX_RANGE_CM, dtype=np.float32)

        # Subscribe to LiDAR point cloud
        from sensor_msgs.msg import PointCloud2

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            "/cloud_registered",
            self._cloud_callback,
            SENSOR_QOS,
        )

        # Publish obstacle distances to PX4
        from px4_msgs.msg import ObstacleDistance

        self.obstacle_pub = self.create_publisher(
            ObstacleDistance,
            "/fmu/in/obstacle_distance",
            RELIABLE_QOS,
        )

        # Publish at 10 Hz
        self._timer = self.create_timer(0.1, self._publish_obstacles)

        self.get_logger().info(
            f"Collision avoidance initialized (min distance: {self.min_distance}m)"
        )

    def _cloud_callback(self, msg):
        """Process point cloud and update obstacle sectors."""
        import sensor_msgs_py.point_cloud2 as pc2

        # Reset sectors
        sectors = np.full(NUM_SECTORS, MAX_RANGE_CM, dtype=np.float32)

        for point in pc2.read_points(msg, skip_nans=True,
                                     field_names=("x", "y", "z")):
            x, y, z = point[0], point[1], point[2]

            # Only consider points at drone altitude (+/- 1m)
            if abs(z) > 1.0:
                continue

            # Calculate horizontal distance and angle
            dist = np.sqrt(x * x + y * y)
            if dist < 0.1:  # Skip points too close (LiDAR blind zone)
                continue

            angle = np.degrees(np.arctan2(y, x)) % 360.0
            sector_idx = int(angle / SECTOR_ANGLE) % NUM_SECTORS
            dist_cm = dist * 100.0  # Convert to centimeters

            # Keep minimum distance per sector
            if dist_cm < sectors[sector_idx]:
                sectors[sector_idx] = dist_cm

        self._sectors = sectors

    def _publish_obstacles(self):
        """Publish obstacle distance array to PX4."""
        if not self.enabled:
            return

        from px4_msgs.msg import ObstacleDistance

        msg = ObstacleDistance()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.frame = 0  # MAV_FRAME_BODY_FRD
        msg.sensor_type = 0  # MAV_DISTANCE_SENSOR_LASER
        msg.min_distance = 20  # 20 cm
        msg.max_distance = MAX_RANGE_CM
        msg.increment = SECTOR_ANGLE
        msg.distances = [int(d) for d in self._sectors]

        self.obstacle_pub.publish(msg)

    def get_closest_obstacle(self) -> float:
        """Get the distance to the closest obstacle in any direction.

        Returns:
            Distance in meters to closest obstacle.
        """
        return float(np.min(self._sectors)) / 100.0

    def is_obstacle_close(self) -> bool:
        """Check if any obstacle is within the minimum safety distance."""
        return self.get_closest_obstacle() < self.min_distance

    def enable(self):
        self.enabled = True
        logger.info("Collision avoidance enabled")

    def disable(self):
        self.enabled = False
        logger.info("Collision avoidance disabled")
