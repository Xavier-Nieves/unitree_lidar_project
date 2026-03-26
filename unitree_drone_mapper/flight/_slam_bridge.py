#!/usr/bin/env python3
"""_slam_bridge.py — Point-LIO odometry → MAVROS vision pose + odometry bridge.

Converts Point-LIO's /aft_mapped_to_init odometry output into two MAVROS
topics for full PX4 EKF2 fusion:

  /mavros/vision_pose/pose   (geometry_msgs/PoseStamped)
      Used by EKF2 for position and attitude fusion.
      Enables EKF2_EV_CTRL bits 0 (horizontal pos), 1 (vertical pos), 3 (yaw).

  /mavros/odometry/out       (nav_msgs/Odometry)
      Used by EKF2 for 3D velocity fusion (EKF2_EV_CTRL bit 2).
      Point-LIO's twist field contains body-frame linear velocity from IMU
      integration — publishing this eliminates the need for EKF2 to
      differentiate position to estimate velocity, improving dynamic
      stability during fast waypoint transitions.

Frame Convention
----------------
Point-LIO outputs poses in ENU (East-North-Up), which is the ROS standard.
MAVROS handles the ENU→NED conversion internally before forwarding to PX4.
Do NOT manually convert frames here.

PX4 Parameters Required
-----------------------
    EKF2_EV_CTRL  = 15   All four bits: pos + vertical + velocity + yaw
    EKF2_HGT_REF  = 3    Vision as primary height reference
    EKF2_EV_DELAY = 30   Tune based on observed Pi→Pixhawk latency (ms)
    EKF2_MAG_TYPE = 1    Automatic — SLAM yaw primary, magnetometer fallback

Launched as a subprocess by main.py (MODE 3) and drone_watchdog.py (MODE 2).

Dependencies
------------
    rclpy, nav_msgs, geometry_msgs (ROS 2 Jazzy)
    Point-LIO must be running and publishing /aft_mapped_to_init
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# ── QoS Profiles ──────────────────────────────────────────────────────────────

# Match Point-LIO's publisher QoS (BEST_EFFORT for sensor data)
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# MAVROS vision_pose subscriber expects RELIABLE QoS
VISION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class SLAMBridgeNode(Node):
    """Bridges Point-LIO odometry to MAVROS for full EKF2 vision fusion.

    Publishes two topics from one /aft_mapped_to_init subscription:

      /mavros/vision_pose/pose  — pose only (position + orientation)
          Enables EKF2_EV_CTRL bits 0, 1, 3: horizontal pos, height, yaw.

      /mavros/odometry/out      — full odometry including linear velocity
          Enables EKF2_EV_CTRL bit 2: 3D velocity fusion.
          EKF2 receives velocity directly from SLAM rather than having to
          differentiate position updates — improves dynamic flight stability.
    """

    def __init__(self):
        super().__init__("slam_to_mavros_bridge")

        # Publisher 1: Vision pose for position + attitude fusion
        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/mavros/vision_pose/pose",
            VISION_QOS,
        )

        # Publisher 2: Full odometry for 3D velocity fusion (EKF2_EV_CTRL bit 2)
        # MAVROS odometry plugin forwards this as MAVLink ODOMETRY message.
        # frame_id=odom, child_frame_id=base_link per MAVROS convention.
        self.odom_pub = self.create_publisher(
            Odometry,
            "/mavros/odometry/out",
            VISION_QOS,
        )

        # Subscriber: Point-LIO accumulated SLAM pose + body-frame velocity
        self.sub = self.create_subscription(
            Odometry,
            "/aft_mapped_to_init",
            self._callback,
            SENSOR_QOS,
        )

        self.msg_count     = 0
        self.last_log_time = self.get_clock().now()

        self.get_logger().info(
            "SLAM bridge started: /aft_mapped_to_init → "
            "/mavros/vision_pose/pose + /mavros/odometry/out"
        )
        self.get_logger().info(
            "EKF2_EV_CTRL=15: pos + height + 3D velocity + yaw fusion active"
        )

    def _callback(self, msg: Odometry) -> None:
        """Forward pose and velocity from Point-LIO to MAVROS."""

        # ── Pose-only message ─────────────────────────────────────────────────
        pose                 = PoseStamped()
        pose.header.stamp    = msg.header.stamp
        pose.header.frame_id = "map"
        pose.pose            = msg.pose.pose   # ENU pass-through, no conversion

        self.pose_pub.publish(pose)

        # ── Full odometry message (pose + linear velocity) ────────────────────
        odom                 = Odometry()
        odom.header.stamp    = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose            = msg.pose
        odom.twist           = msg.twist       # body-frame velocity from Point-LIO

        self.odom_pub.publish(odom)

        # ── Periodic diagnostics every 2 seconds ─────────────────────────────
        self.msg_count += 1
        now     = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9

        if elapsed >= 2.0:
            p    = msg.pose.pose.position
            v    = msg.twist.twist.linear
            rate = self.msg_count / elapsed
            self.get_logger().info(
                f"Bridge: {self.msg_count} frames ({rate:.1f} Hz)  "
                f"pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})  "
                f"vel=({v.x:.2f}, {v.y:.2f}, {v.z:.2f}) m/s"
            )
            self.msg_count     = 0
            self.last_log_time = now


def main():
    rclpy.init()
    node = SLAMBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SLAM bridge stopped")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
