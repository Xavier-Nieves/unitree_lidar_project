"""flight/_flow_bridge.py — Hailo optical flow → MAVROS velocity bridge.

Runs in the Conda dronepi environment alongside the rest of the flight stack.
Subscribes to /hailo/optical_flow (published by hailo_flight_node in its own
env) and forwards valid, confidence-gated velocity estimates to
/mavros/odometry/out for EKF2 velocity fusion via EKF2_EV_CTRL bit 2.

This bridge is the integration point between the two isolated Python
environments — the Hailo env publishes the flow topic, the Conda env consumes
it. No shared imports, no shared venv.

EKF2 integration:
    /mavros/odometry/out → MAVROS → PX4 VISION_SPEED_ESTIMATE
    EKF2_EV_CTRL bit 2 must be set (included in the value 15 = 0b1111)
    EKF2_EV_DELAY must account for Hailo inference latency (~15–25ms)

Safety gates applied before forwarding:
    1. Confidence threshold — reject low-confidence flow estimates
    2. Latency guard — reject stale messages (> MAX_MSG_AGE_S old)
    3. Velocity magnitude clamp — reject physically implausible spikes
    4. Consecutive fallback counter — escalate to degraded mode flag
       after FALLBACK_ESCALATE_COUNT consecutive rejections

Environment: conda activate dronepi

Design notes:
    - The bridge does NOT modify the Hailo node's output — it is a
      transparent gate. If confidence is sufficient, the message passes
      through unchanged (with header timestamp preserved for EKF2 delay
      compensation). If not, it is silently dropped.
    - The repurposed twist.linear.z confidence field is read here and
      then discarded — the outgoing Odometry message uses standard fields.
    - FlowBridge is instantiated by main.py. It does not have its own
      main() — it is a module, not a standalone script.
"""

import threading
import time

# Constants
CONFIDENCE_THRESHOLD    = 0.50   # Reject flow output below this confidence
MAX_MSG_AGE_S           = 0.15   # Reject messages older than 150ms
MAX_VELOCITY_M_S        = 15.0   # Clamp — above this is physically implausible
FALLBACK_ESCALATE_COUNT = 30     # Consecutive rejections before degraded flag


class FlowBridge:
    """Gates Hailo optical flow and forwards to MAVROS odometry velocity.

    Args:
        node: A rclpy Node instance (from MainNode._node or a dedicated node).
              Subscriptions and publishers are created on this node.
        confidence_threshold: Minimum confidence to forward. Default 0.50.
        degraded_callback:    Optional callable() called when consecutive
                              fallback count exceeds FALLBACK_ESCALATE_COUNT.
                              Use to trigger EKFMonitor escalation.
    """

    def __init__(
        self,
        node,
        confidence_threshold: float = CONFIDENCE_THRESHOLD,
        degraded_callback=None,
    ):
        self._node                 = node
        self._conf_threshold       = confidence_threshold
        self._degraded_callback    = degraded_callback
        self._lock                 = threading.Lock()

        self._fallback_count       = 0
        self._total_received       = 0
        self._total_forwarded      = 0
        self._last_forwarded_t     = None
        self._degraded_mode        = False

        # Lazily imported ROS types — avoid import at module level so this
        # file can be imported in environments where rclpy is not installed
        self._odom_pub  = None
        self._flow_sub  = None

        self._setup_ros()

    # ── Public API ────────────────────────────────────────────────────────────

    def set_confidence_threshold(self, threshold: float) -> None:
        """Update the confidence gate threshold at runtime."""
        with self._lock:
            self._conf_threshold = max(0.0, min(1.0, threshold))

    def get_fallback_count(self) -> int:
        """Return the current consecutive rejected-frame counter."""
        with self._lock:
            return self._fallback_count

    def is_degraded(self) -> bool:
        """Return True if consecutive fallbacks have exceeded the escalation limit."""
        with self._lock:
            return self._degraded_mode

    def get_stats(self) -> dict:
        """Return forwarding statistics for logging and monitoring."""
        with self._lock:
            fwd = self._total_forwarded
            rcv = self._total_received
            return {
                "received":   rcv,
                "forwarded":  fwd,
                "pass_rate":  round(fwd / rcv, 3) if rcv > 0 else 0.0,
                "fallback_count": self._fallback_count,
                "degraded":   self._degraded_mode,
            }

    # ── Private ───────────────────────────────────────────────────────────────

    def _setup_ros(self) -> None:
        """Create subscriber and publisher on the shared node."""
        try:
            from geometry_msgs.msg import TwistStamped
            from nav_msgs.msg      import Odometry
            from rclpy.qos         import QoSProfile, ReliabilityPolicy, HistoryPolicy
        except ImportError as exc:
            print(f"[FlowBridge] ROS 2 import failed: {exc}")
            return

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._flow_sub  = self._node.create_subscription(
            TwistStamped,
            "/hailo/optical_flow",
            self._flow_callback,
            sensor_qos,
        )
        self._odom_pub  = self._node.create_publisher(
            Odometry,
            "/mavros/odometry/out",
            reliable_qos,
        )

    def _flow_callback(self, msg) -> None:
        """Process incoming optical flow message and forward if valid.

        The Hailo flight node encodes confidence in twist.linear.z.
        The outgoing Odometry message uses only the velocity fields —
        position fields are left at zero (MAVROS ignores them when
        EKF2_EV_CTRL only has velocity bit set).
        """
        with self._lock:
            self._total_received += 1

            vx         = float(msg.twist.linear.x)
            vy         = float(msg.twist.linear.y)
            confidence = float(msg.twist.linear.z)   # repurposed field

            # Gate 1: confidence threshold
            if confidence < self._conf_threshold:
                self._fallback_count += 1
                self._check_escalation()
                return

            # Gate 2: message age — reject stale Hailo outputs
            now        = self._node.get_clock().now()
            msg_time   = rclpy_time_from_header(msg.header, self._node)
            if msg_time is not None:
                age_s = (now - msg_time).nanoseconds / 1e9
                if age_s > MAX_MSG_AGE_S:
                    self._fallback_count += 1
                    self._check_escalation()
                    return

            # Gate 3: velocity magnitude clamp
            speed = (vx**2 + vy**2) ** 0.5
            if speed > MAX_VELOCITY_M_S:
                self._fallback_count += 1
                self._check_escalation()
                return

            # All gates passed — forward to MAVROS
            self._fallback_count  = 0
            self._degraded_mode   = False
            self._total_forwarded += 1
            self._last_forwarded_t = time.monotonic()

        self._forward_velocity(msg.header, vx, vy)

    def _forward_velocity(self, header, vx: float, vy: float) -> None:
        """Build and publish nav_msgs/Odometry with velocity fields populated."""
        if self._odom_pub is None:
            return

        try:
            from nav_msgs.msg import Odometry
        except ImportError:
            return

        odom = Odometry()
        # Preserve the original Hailo timestamp for EKF2 delay compensation
        odom.header          = header
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"

        # Velocity in body frame — EKF2 fuses these when EKF2_EV_CTRL bit 2 set
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0   # no vertical flow estimate

        # Covariance diagonal — tuned conservatively
        # Increase these values if EKF2 shows large velocity innovations
        odom.twist.covariance[0]  = 0.1   # vx variance (m/s)²
        odom.twist.covariance[7]  = 0.1   # vy variance
        odom.twist.covariance[14] = 0.5   # vz variance (not estimated — high)

        self._odom_pub.publish(odom)

    def _check_escalation(self) -> None:
        """Check if consecutive fallback count warrants degraded mode flag."""
        if (not self._degraded_mode and
                self._fallback_count >= FALLBACK_ESCALATE_COUNT):
            self._degraded_mode = True
            if self._degraded_callback is not None:
                try:
                    self._degraded_callback()
                except Exception:
                    pass


def rclpy_time_from_header(header, node):
    """Convert a std_msgs/Header stamp to rclpy Time. Returns None on failure."""
    try:
        from rclpy.time import Time
        return Time(
            seconds=header.stamp.sec,
            nanoseconds=header.stamp.nanosec,
            clock_type=node.get_clock().clock_type,
        )
    except Exception:
        return None
