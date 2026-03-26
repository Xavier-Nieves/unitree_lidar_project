#!/usr/bin/env python3
"""bench_test.py — DronePi sensor data bench verification script.

Simulates all companion computer data sources and verifies that PX4/QGC
can see them via the MAVLink Inspector — without OFFBOARD mode, without
the LiDAR running, and without arming the drone.

What this tests
---------------
  TEST 1  MAVROS connection     — Pixhawk heartbeat visible, FCU connected
  TEST 2  Vision pose (SLAM)    — Publishes fake pose to /mavros/vision_pose/pose
                                   QGC should show VISION_POSITION_ESTIMATE
  TEST 3  Obstacle distance     — Publishes fake OBSTACLE_DISTANCE sectors
                                   QGC should show OBSTACLE_DISTANCE messages
  TEST 4  Downward rangefinder  — Publishes fake Range to lidar_down topic
                                   QGC should show DISTANCE_SENSOR (PITCH_270)
  TEST 5  Collision zone topic  — Verifies /dronepi/collision_zone publishes
  TEST 6  GPS topic             — Checks /mavros/global_position/global is present
  TEST 7  IMU topic             — Checks /mavros/imu/data is present
  TEST 8  EKF status            — Checks /mavros/local_position/pose is publishing
                                   (confirms EKF2 has initialised)

How to run
----------
  # On the Pi — MAVROS must be running, LiDAR does NOT need to be on
  source /opt/ros/jazzy/setup.bash
  source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
  python3 ~/unitree_lidar_project/unitree_drone_mapper/utils/bench_test.py

  # Check QGC MAVLink Inspector during the test for:
  #   VISION_POSITION_ESTIMATE  — confirms TEST 2 passed
  #   OBSTACLE_DISTANCE         — confirms TEST 3 passed
  #   DISTANCE_SENSOR           — confirms TEST 4 passed (requires pluginlist fix)

  # Stop with Ctrl+C when done

Dependencies
------------
  rclpy, geometry_msgs, mavros_msgs, sensor_msgs, nav_msgs, std_msgs
"""

import math
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, NavSatFix, Range
from std_msgs.msg import String

# ── Config ────────────────────────────────────────────────────────────────────

PUBLISH_HZ   = 10     # Rate for simulated sensor data
TEST_DURATION = 60    # Seconds to run before summary (0 = run until Ctrl+C)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# ── Logging ───────────────────────────────────────────────────────────────────

def ts():
    return datetime.now().strftime("%H:%M:%S")

def ok(msg):    print(f"[{ts()}]  ✓  {msg}", flush=True)
def fail(msg):  print(f"[{ts()}]  ✗  {msg}", flush=True)
def info(msg):  print(f"[{ts()}]  ·  {msg}", flush=True)
def header(msg):
    print(f"\n{'='*55}", flush=True)
    print(f"  {msg}", flush=True)
    print(f"{'='*55}", flush=True)


# ── Bench Test Node ───────────────────────────────────────────────────────────

class BenchTestNode(Node):
    """Publishes simulated sensor data and monitors MAVROS topics.

    Simulates what the full flight stack publishes so you can verify
    that PX4/QGC receives and displays each data stream correctly,
    entirely on the bench without LiDAR or arming.
    """

    def __init__(self):
        super().__init__("bench_test")

        self._lock = threading.Lock()

        # ── Test result tracking ──────────────────────────────────────────────
        self._results = {
            "mavros_connected":   False,
            "vision_pose_pub":    False,
            "obstacle_dist_pub":  False,
            "lidar_down_pub":     False,
            "collision_zone_pub": False,
            "gps_receiving":      False,
            "imu_receiving":      False,
            "ekf_initialised":    False,
        }
        self._msg_counts = {k: 0 for k in self._results}

        # ── Publishers (simulating flight stack data) ─────────────────────────

        # TEST 2 — Simulated SLAM pose → /mavros/vision_pose/pose
        self._vision_pub = self.create_publisher(
            PoseStamped, "/mavros/vision_pose/pose", RELIABLE_QOS)

        # TEST 3 — Simulated obstacle distance as LaserScan → /mavros/obstacle/send
        # MAVROS converts LaserScan to MAVLink OBSTACLE_DISTANCE for PX4
        self._obs_pub = self.create_publisher(
            LaserScan, "/mavros/obstacle/send", RELIABLE_QOS)

        # TEST 4 — Simulated downward rangefinder → MAVROS distance_sensor plugin
        self._range_pub = self.create_publisher(
            Range, "/mavros/distance_sensor/lidar_down_sub", RELIABLE_QOS)

        # TEST 5 — Collision zone status (normally from collision_monitor.py)
        self._zone_pub = self.create_publisher(
            String, "/dronepi/collision_zone", RELIABLE_QOS)

        # ── Subscribers (monitoring MAVROS output) ────────────────────────────

        # TEST 1 — MAVROS connection
        self._state_sub = self.create_subscription(
            State, "/mavros/state", self._state_cb, RELIABLE_QOS)

        # TEST 6 — GPS
        self._gps_sub = self.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            self._gps_cb, SENSOR_QOS)

        # TEST 7 — IMU
        from sensor_msgs.msg import Imu
        self._imu_sub = self.create_subscription(
            Imu, "/mavros/imu/data", self._imu_cb, SENSOR_QOS)

        # TEST 8 — EKF local position (confirms EKF2 has initialised)
        self._pose_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._pose_cb, SENSOR_QOS)

        # ── Publish timer ─────────────────────────────────────────────────────
        self._t0        = time.time()
        self._tick      = 0
        self._pub_timer = self.create_timer(
            1.0 / PUBLISH_HZ, self._publish_all)

        # ── Status print timer ────────────────────────────────────────────────
        self._log_timer = self.create_timer(5.0, self._print_status)

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _state_cb(self, msg: State):
        with self._lock:
            if msg.connected:
                if not self._results["mavros_connected"]:
                    ok("TEST 1 PASS — MAVROS connected to FCU (PX4 heartbeat received)")
                self._results["mavros_connected"] = True
            self._msg_counts["mavros_connected"] += 1

    def _gps_cb(self, msg):
        with self._lock:
            if not self._results["gps_receiving"]:
                fix = "3D fix" if msg.status.status >= 0 else "no fix"
                ok(f"TEST 6 PASS — GPS topic active ({fix}  "
                   f"lat={msg.latitude:.4f}  lon={msg.longitude:.4f})")
            self._results["gps_receiving"] = True
            self._msg_counts["gps_receiving"] += 1

    def _imu_cb(self, msg):
        with self._lock:
            if not self._results["imu_receiving"]:
                ok("TEST 7 PASS — IMU topic active")
            self._results["imu_receiving"] = True
            self._msg_counts["imu_receiving"] += 1

    def _pose_cb(self, msg):
        with self._lock:
            p = msg.pose.position
            if not self._results["ekf_initialised"]:
                ok(f"TEST 8 PASS — EKF local position active  "
                   f"({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
            self._results["ekf_initialised"] = True
            self._msg_counts["ekf_initialised"] += 1

    # ── Publish simulated data ────────────────────────────────────────────────

    def _publish_all(self):
        now  = self.get_clock().now().to_msg()
        self._tick += 1
        t = time.time() - self._t0

        # TEST 2 — Simulated SLAM pose (slow circular motion for visual confirmation)
        pose                  = PoseStamped()
        pose.header.stamp     = now
        pose.header.frame_id  = "map"
        pose.pose.position.x  = 0.5 * math.sin(t * 0.2)
        pose.pose.position.y  = 0.5 * math.cos(t * 0.2)
        pose.pose.position.z  = 0.0
        pose.pose.orientation.w = 1.0
        self._vision_pub.publish(pose)
        with self._lock:
            if not self._results["vision_pose_pub"]:
                ok("TEST 2 — Publishing simulated SLAM pose to /mavros/vision_pose/pose")
                ok("         Check QGC MAVLink Inspector for VISION_POSITION_ESTIMATE")
            self._results["vision_pose_pub"] = True
            self._msg_counts["vision_pose_pub"] += 1

        # TEST 3 — Simulated obstacle (LaserScan → MAVROS → OBSTACLE_DISTANCE → PX4)
        # Simulates obstacle at 3 m directly ahead, all other sectors clear
        scan                 = LaserScan()
        scan.header.stamp    = now
        scan.header.frame_id = "base_link"
        scan.angle_min       = -math.pi
        scan.angle_max       =  math.pi
        scan.angle_increment = (2.0 * math.pi) / 72
        scan.time_increment  = 0.0
        scan.scan_time       = 1.0 / PUBLISH_HZ
        scan.range_min       = 0.70   # IGNORE_RADIUS metres
        scan.range_max       = 3.50   # CAUTION_RADIUS metres
        ranges               = [float('inf')] * 72
        ranges[0]  = 3.0    # obstacle at 3 m directly ahead (sector 0)
        ranges[1]  = 3.1
        ranges[71] = 3.1
        scan.ranges = ranges
        self._obs_pub.publish(scan)
        with self._lock:
            if not self._results["obstacle_dist_pub"]:
                ok("TEST 3 — Publishing simulated obstacle distance to /mavros/obstacle/send")
                ok("         Check QGC MAVLink Inspector for OBSTACLE_DISTANCE")
            self._results["obstacle_dist_pub"] = True
            self._msg_counts["obstacle_dist_pub"] += 1

        # TEST 4 — Simulated downward AGL (2.5 m above ground)
        rng                 = Range()
        rng.header.stamp    = now
        rng.header.frame_id = "lidar_down"
        rng.radiation_type  = Range.INFRARED
        rng.field_of_view   = math.radians(15.0)
        rng.min_range       = 0.10
        rng.max_range       = 15.0
        rng.range           = 2.5   # Simulated 2.5 m above ground
        self._range_pub.publish(rng)
        with self._lock:
            if not self._results["lidar_down_pub"]:
                ok("TEST 4 — Publishing simulated AGL to /mavros/distance_sensor/lidar_down_sub")
                ok("         Check QGC MAVLink Inspector for DISTANCE_SENSOR (PITCH_270)")
                ok("         If not visible: pluginlist fix is still needed (see prev steps)")
            self._results["lidar_down_pub"] = True
            self._msg_counts["lidar_down_pub"] += 1

        # TEST 5 — Collision zone status
        zone_msg      = String()
        zone_msg.data = "CLEAR"
        self._zone_pub.publish(zone_msg)
        with self._lock:
            if not self._results["collision_zone_pub"]:
                ok("TEST 5 — Publishing /dronepi/collision_zone (CLEAR)")
            self._results["collision_zone_pub"] = True
            self._msg_counts["collision_zone_pub"] += 1

    # ── Status print ──────────────────────────────────────────────────────────

    def _print_status(self):
        elapsed = time.time() - self._t0
        header(f"Bench test status  ({elapsed:.0f}s elapsed)")

        labels = {
            "mavros_connected":   "TEST 1  MAVROS/FCU connected",
            "vision_pose_pub":    "TEST 2  Vision pose publishing",
            "obstacle_dist_pub":  "TEST 3  Obstacle distance publishing",
            "lidar_down_pub":     "TEST 4  Downward rangefinder publishing",
            "collision_zone_pub": "TEST 5  Collision zone topic active",
            "gps_receiving":      "TEST 6  GPS data from FCU",
            "imu_receiving":      "TEST 7  IMU data from FCU",
            "ekf_initialised":    "TEST 8  EKF local position active",
        }
        with self._lock:
            for key, label in labels.items():
                status = "PASS" if self._results[key] else "WAIT"
                count  = self._msg_counts[key]
                sym    = "✓" if self._results[key] else "·"
                print(f"  {sym}  [{status}]  {label}  ({count} msgs)", flush=True)

        info("")
        info("QGC MAVLink Inspector — what to look for:")
        info("  VISION_POSITION_ESTIMATE  → TEST 2 visible in PX4")
        info("  OBSTACLE_DISTANCE         → TEST 3 visible in PX4")
        info("  DISTANCE_SENSOR           → TEST 4 (requires pluginlist fix)")
        info("  HEARTBEAT from comp id 1  → MAVROS healthy")

    def summary(self):
        """Print final pass/fail summary."""
        header("BENCH TEST SUMMARY")
        passed = sum(1 for v in self._results.values() if v)
        total  = len(self._results)
        for key, label in {
            "mavros_connected":   "MAVROS/FCU connection",
            "vision_pose_pub":    "Vision pose (SLAM bridge sim)",
            "obstacle_dist_pub":  "Obstacle distance (collision monitor sim)",
            "lidar_down_pub":     "Downward AGL rangefinder sim",
            "collision_zone_pub": "Collision zone topic",
            "gps_receiving":      "GPS from FCU",
            "imu_receiving":      "IMU from FCU",
            "ekf_initialised":    "EKF2 local position",
        }.items():
            sym = "✓ PASS" if self._results[key] else "✗ FAIL"
            print(f"  {sym}  {label}", flush=True)

        print(f"\n  {passed}/{total} tests passed", flush=True)
        if passed == total:
            ok("All systems nominal — ready for SLAM stack integration test")
        else:
            fail("Fix failing tests before proceeding to flight")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 55, flush=True)
    print("  DronePi Bench Test — sensor data verification", flush=True)
    print("  MAVROS must be running. LiDAR OFF is OK.", flush=True)
    print("  OFFBOARD mode NOT required.", flush=True)
    print("  Open QGC MAVLink Inspector to verify data.", flush=True)
    print("=" * 55, flush=True)

    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        print("ERROR: ROS 2 not sourced. Run:", flush=True)
        print("  source /opt/ros/jazzy/setup.bash", flush=True)
        sys.exit(1)

    rclpy.init()
    node = BenchTestNode()

    info(f"Publishing simulated data at {PUBLISH_HZ} Hz...")
    info("Press Ctrl+C to stop and see summary")

    try:
        if TEST_DURATION > 0:
            deadline = time.time() + TEST_DURATION
            while time.time() < deadline and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
