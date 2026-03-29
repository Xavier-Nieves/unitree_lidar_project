#!/usr/bin/env python3
"""flight_controller.py — Reusable MAVROS flight control primitives.

PURPOSE
-------
This module is a STANDALONE TEST UTILITY, used exclusively by manual
test scripts. It is NOT used by main.py, which implements its own
MainNode class internally with a superset of this functionality.

Callers
-------
    test_offboard_flight.py   — hover test
    (any future standalone test scripts)

NOT a caller
------------
    main.py                   — uses its own MainNode, never imports this
    drone_watchdog.py         — subprocess management only, no flight commands

Design rationale
----------------
FlightController owns one rclpy Node and one background spin thread.
All state is protected by a single threading.Lock. Methods are safe to
call from the main thread while the spin thread processes callbacks.

Usage
-----
    from flight_controller import FlightController

    fc = FlightController(node_name="my_test")
    fc.wait_for_connection(timeout=15.0)
    fc.wait_for_ekf(timeout=30.0, require_gps=False)

    hx, hy, hz = fc.get_position()
    fc.stream_setpoint(hx, hy, hz, yaw=0.0, duration=3.0)
    fc.arm()
    fc.set_mode("OFFBOARD")
    fc.fly_to(hx, hy, hz + 1.5, yaw=0.0, timeout=25.0)
    fc.set_mode("AUTO.LAND")
    fc.wait_for_disarm(timeout=30.0)
    fc.shutdown()
"""

import math
import threading
import time


# ── Constants ─────────────────────────────────────────────────────────────────

SETPOINT_HZ      = 20      # Hz — PX4 requires > 2 Hz to accept OFFBOARD
WP_TOLERANCE_M   = 0.5     # metres — arrival radius for fly_to()
EKF_STABLE_COUNT = 20      # consecutive Z readings within tolerance
EKF_Z_TOLERANCE  = 0.03    # metres — max Z drift per reading to count as stable


class FlightController:
    """MAVROS interface for standalone test scripts.

    Provides thread-safe access to drone state and control primitives.
    Owns one rclpy Node and one background spin thread for the lifetime
    of the instance. Call shutdown() when done.

    Parameters
    ----------
    node_name : str
        ROS 2 node name. Use a unique name per script to avoid conflicts.
    """

    def __init__(self, node_name: str = "flight_controller"):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from geometry_msgs.msg import PoseStamped
            from mavros_msgs.msg import State
            from mavros_msgs.srv import CommandBool, SetMode
        except ImportError as exc:
            raise RuntimeError(
                f"ROS 2 not sourced — cannot import rclpy: {exc}"
            )

        self._rclpy = rclpy
        self._PS    = PoseStamped
        self._lock  = threading.Lock()

        # State cache
        self._state   = State()
        self._pose    = None   # PoseStamped
        self._home_z  = None   # float — captured on first stable EKF reading

        rclpy.init()
        self._node = Node(node_name)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions
        self._node.create_subscription(
            State, "/mavros/state", self._state_cb, 10)
        self._node.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._pose_cb, sensor_qos)

        # Publishers
        self._sp_pub = self._node.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)

        # Service clients
        self._arm_client  = self._node.create_client(
            CommandBool, "/mavros/cmd/arming")
        self._mode_client = self._node.create_client(
            SetMode, "/mavros/set_mode")

        # Background spin thread
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg) -> None:
        with self._lock:
            self._state = msg

    def _pose_cb(self, msg) -> None:
        with self._lock:
            self._pose = msg

    # ── State Queries ──────────────────────────────────────────────────────────

    def is_connected(self) -> bool:
        with self._lock:
            return self._state.connected

    def is_armed(self) -> bool:
        with self._lock:
            return self._state.armed

    def get_mode(self) -> str:
        with self._lock:
            return self._state.mode

    def get_position(self) -> tuple:
        """Return current (x, y, z) in metres, ENU local frame."""
        with self._lock:
            if self._pose is None:
                return 0.0, 0.0, 0.0
            p = self._pose.pose.position
            return p.x, p.y, p.z

    def get_altitude(self) -> float:
        """Return current Z in metres."""
        return self.get_position()[2]

    def get_altitude_above_home(self) -> float:
        """Return Z relative to home position captured during wait_for_ekf()."""
        _, _, z = self.get_position()
        with self._lock:
            home_z = self._home_z or 0.0
        return z - home_z

    def get_yaw(self) -> float:
        """Return current yaw in radians from orientation quaternion."""
        with self._lock:
            if self._pose is None:
                return 0.0
            q = self._pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    # ── Initialization Helpers ─────────────────────────────────────────────────

    def wait_for_connection(self, timeout: float = 15.0) -> bool:
        """Block until MAVROS reports FCU connected, or timeout expires.

        Returns True if connected, False if timed out.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_connected():
                return True
            time.sleep(0.1)
        return False

    def wait_for_ekf(self,
                     timeout: float = 30.0,
                     require_gps: bool = False) -> bool:
        """Block until EKF Z output is stable, then capture home Z.

        Stability is defined as EKF_STABLE_COUNT consecutive Z readings
        each differing from the previous by less than EKF_Z_TOLERANCE metres.

        Parameters
        ----------
        timeout    : seconds to wait before returning False
        require_gps: if True, also waits for GPS fix (future use)

        Returns True if stable, False if timed out.
        """
        prev_z   = None
        stable   = 0
        deadline = time.time() + timeout

        while time.time() < deadline:
            _, _, z = self.get_position()
            if prev_z is not None:
                if abs(z - prev_z) < EKF_Z_TOLERANCE:
                    stable += 1
                else:
                    stable = 0
            prev_z = z

            if stable >= EKF_STABLE_COUNT:
                with self._lock:
                    self._home_z = z
                return True
            time.sleep(0.1)

        return False

    # ── Setpoint Control ───────────────────────────────────────────────────────

    def publish_setpoint(self,
                         x: float, y: float, z: float,
                         yaw: float = 0.0) -> None:
        """Publish a single local ENU setpoint immediately."""
        msg = self._PS()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._sp_pub.publish(msg)

    def stream_setpoint(self,
                        x: float, y: float, z: float,
                        yaw: float = 0.0,
                        duration: float = 3.0) -> None:
        """Continuously publish one setpoint for duration seconds.

        Used to establish a setpoint stream before switching to OFFBOARD,
        since PX4 requires an active stream before accepting the mode.
        """
        end = time.time() + duration
        while time.time() < end:
            self.publish_setpoint(x, y, z, yaw)
            time.sleep(1.0 / SETPOINT_HZ)

    def fly_to(self,
               x: float, y: float, z: float,
               yaw: float = 0.0,
               timeout: float = 30.0,
               tolerance: float = WP_TOLERANCE_M) -> bool:
        """Stream setpoints toward target until within tolerance or timeout.

        Publishes at SETPOINT_HZ to keep OFFBOARD mode active during travel.

        Returns True if target reached, False if timeout expired.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.publish_setpoint(x, y, z, yaw)

            cx, cy, cz = self.get_position()
            dist = math.sqrt(
                (cx - x) ** 2 + (cy - y) ** 2 + (cz - z) ** 2
            )
            print(f"\r  dist={dist:.2f}m  pos=({cx:.2f},{cy:.2f},{cz:.2f})",
                  end="", flush=True)

            if dist < tolerance:
                print()
                return True
            time.sleep(1.0 / SETPOINT_HZ)

        print()
        return False

    # ── Vehicle Commands ───────────────────────────────────────────────────────

    def arm(self) -> bool:
        """Send arm command. Returns True if service call succeeded."""
        from mavros_msgs.srv import CommandBool
        req       = CommandBool.Request()
        req.value = True
        future    = self._arm_client.call_async(req)
        self._rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=5.0)
        return bool(future.result() and future.result().success)

    def disarm(self) -> bool:
        """Send disarm command. Returns True if service call succeeded."""
        from mavros_msgs.srv import CommandBool
        req       = CommandBool.Request()
        req.value = False
        future    = self._arm_client.call_async(req)
        self._rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=5.0)
        return bool(future.result() and future.result().success)

    def set_mode(self, mode: str) -> bool:
        """Send mode change request. Returns True if mode_sent confirmed."""
        from mavros_msgs.srv import SetMode
        req             = SetMode.Request()
        req.custom_mode = mode
        future          = self._mode_client.call_async(req)
        self._rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=5.0)
        return bool(future.result() and future.result().mode_sent)

    # ── Wait Helpers ───────────────────────────────────────────────────────────

    def wait_for_arm(self, timeout: float = 10.0) -> bool:
        """Block until armed state confirmed, or timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_armed():
                return True
            time.sleep(0.1)
        return False

    def wait_for_disarm(self, timeout: float = 30.0) -> bool:
        """Block until disarmed state confirmed, or timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.is_armed():
                return True
            time.sleep(0.2)
        return False

    def wait_for_mode(self, mode: str, timeout: float = 10.0) -> bool:
        """Block until FCU reports the requested mode, or timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.get_mode() == mode:
                return True
            time.sleep(0.1)
        return False

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        """Destroy node and shut down rclpy. Call once when script exits."""
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


# ── Convenience factory ────────────────────────────────────────────────────────

def create_controller(node_name: str = "flight_controller") -> FlightController:
    """Return a new FlightController instance. Sugar for test scripts."""
    return FlightController(node_name=node_name)
