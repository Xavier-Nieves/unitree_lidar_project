"""watchdog_core/mavros_reader.py — MAVROS ROS 2 interface.

Subscribes to:
  /mavros/state      — armed flag, flight mode, FCU connection status
  /mavros/rc/in      — RC channel values for toggle detection

Publishes to:
  /mavros/play_tune  — buzzer tunes (QBASIC Format 1)

RC Toggle
---------
  Rising-edge detection with debounce on CH6 (index 5 by default).
  Call check_toggle_pressed() to consume a pending toggle event.

Buzzer
------
  play_tune(tune_str) publishes directly to the Pixhawk buzzer.
  Convenience wrappers: beep_start / beep_stop / beep_ack / beep_error /
                        beep_processing / beep_done
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State, RCIn, PlayTuneV2

from .buzzer import (
    TUNE_FORMAT,
    TUNE_START, TUNE_STOP, TUNE_ACK, TUNE_ERROR,
    TUNE_PROCESSING, TUNE_DONE,
)

# ── RC Configuration ──────────────────────────────────────────────────────────

RC_TOGGLE_CHANNEL = 5      # 0-indexed: CH6 = index 5
RC_HIGH_THRESHOLD = 1700   # PWM above which button is considered pressed
RC_LOW_THRESHOLD  = 1300   # PWM below which button is considered released
RC_DEBOUNCE_MS    = 200    # Minimum ms between toggle events

ENABLE_BUZZER = True


# ── MavrosReader ──────────────────────────────────────────────────────────────

class MavrosReader:
    """ROS 2 interface for state monitoring, RC input, and buzzer output."""

    def __init__(self):
        self._armed       = False
        self._mode        = ""
        self._connected   = False
        self._rc_channels = []
        self._lock        = threading.Lock()

        # RC toggle edge detection
        self._button_was_high  = False
        self._last_toggle_time = 0.0
        self._toggle_pending   = False

        rclpy.init()
        self._node = Node("drone_watchdog")

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._node.create_subscription(
            State, "/mavros/state", self._state_cb, state_qos)
        self._node.create_subscription(
            RCIn, "/mavros/rc/in", self._rc_cb, sensor_qos)

        self._tune_pub = self._node.create_publisher(
            PlayTuneV2, "/mavros/play_tune", 10)

        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg: State) -> None:
        with self._lock:
            self._armed     = msg.armed
            self._mode      = msg.mode
            self._connected = msg.connected

    def _rc_cb(self, msg: RCIn) -> None:
        with self._lock:
            self._rc_channels = list(msg.channels)
            if len(self._rc_channels) <= RC_TOGGLE_CHANNEL:
                return

            ch_value       = self._rc_channels[RC_TOGGLE_CHANNEL]
            now            = time.time()
            button_is_high = ch_value > RC_HIGH_THRESHOLD
            button_is_low  = ch_value < RC_LOW_THRESHOLD

            # Rising-edge detection with debounce
            if button_is_high and not self._button_was_high:
                if (now - self._last_toggle_time) > (RC_DEBOUNCE_MS / 1000.0):
                    self._toggle_pending   = True
                    self._last_toggle_time = now

            if button_is_low:
                self._button_was_high = False
            elif button_is_high:
                self._button_was_high = True

    # ── Buzzer ────────────────────────────────────────────────────────────────

    def play_tune(self, tune: str) -> None:
        if not ENABLE_BUZZER:
            return
        msg        = PlayTuneV2()
        msg.format = TUNE_FORMAT
        msg.tune   = tune
        self._tune_pub.publish(msg)

    def beep_start(self):       self.play_tune(TUNE_START)
    def beep_stop(self):        self.play_tune(TUNE_STOP)
    def beep_ack(self):         self.play_tune(TUNE_ACK)
    def beep_error(self):       self.play_tune(TUNE_ERROR)
    def beep_processing(self):  self.play_tune(TUNE_PROCESSING)
    def beep_done(self):        self.play_tune(TUNE_DONE)

    # ── State Properties ──────────────────────────────────────────────────────

    @property
    def armed(self) -> bool:
        with self._lock: return self._armed

    @property
    def mode(self) -> str:
        with self._lock: return self._mode

    @property
    def connected(self) -> bool:
        with self._lock: return self._connected

    def get_rc_channel(self, channel: int) -> int:
        with self._lock:
            return self._rc_channels[channel] if channel < len(self._rc_channels) else 0

    def check_toggle_pressed(self) -> bool:
        """Consume and return a pending RC toggle event."""
        with self._lock:
            if self._toggle_pending:
                self._toggle_pending = False
                return True
            return False

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
