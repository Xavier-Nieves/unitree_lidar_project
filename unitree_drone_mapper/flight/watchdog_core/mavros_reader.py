"""watchdog_core/mavros_reader.py — MAVROS ROS 2 interface.

Subscribes to:
  /mavros/state      — armed flag, flight mode, FCU connection status
  /mavros/rc/in      — RC channel values for toggle detection

Publishes to:
  /mavros/play_tune  — buzzer tunes (QBASIC Format 1)

This module is transport-only for buzzer playback:
  play_tune(tune_str) publishes directly to the Pixhawk buzzer.
Higher-level sound meaning belongs in buzzer.py and the watchdog.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import PlayTuneV2, RCIn, State

from .buzzer import TUNE_FORMAT

# ── RC Configuration ──────────────────────────────────────────────────────────

RC_TOGGLE_CHANNEL = 5      # 0-indexed: CH6 = index 5
RC_HIGH_THRESHOLD = 1700   # PWM above which button is considered pressed
RC_LOW_THRESHOLD  = 1300   # PWM below which button is considered released
RC_DEBOUNCE_MS    = 200    # Minimum ms between toggle events

ENABLE_BUZZER = True


class MavrosReader:
    """ROS 2 interface for state monitoring, RC input, and buzzer output."""

    def __init__(self) -> None:
        self._armed = False
        self._mode = ""
        self._connected = False
        self._rc_channels: list[int] = []
        self._lock = threading.Lock()

        # RC toggle edge detection
        self._button_was_high = False
        self._last_toggle_time = 0.0
        self._toggle_pending = False

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

        self._node.create_subscription(State, "/mavros/state", self._state_cb, state_qos)
        self._node.create_subscription(RCIn, "/mavros/rc/in", self._rc_cb, sensor_qos)
        self._tune_pub = self._node.create_publisher(PlayTuneV2, "/mavros/play_tune", 10)

        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg: State) -> None:
        with self._lock:
            self._armed = msg.armed
            self._mode = msg.mode
            self._connected = msg.connected

    def _rc_cb(self, msg: RCIn) -> None:
        with self._lock:
            self._rc_channels = list(msg.channels)
            if len(self._rc_channels) <= RC_TOGGLE_CHANNEL:
                return

            ch_value = self._rc_channels[RC_TOGGLE_CHANNEL]
            now = time.time()
            button_is_high = ch_value > RC_HIGH_THRESHOLD
            button_is_low = ch_value < RC_LOW_THRESHOLD

            if button_is_high and not self._button_was_high:
                if (now - self._last_toggle_time) > (RC_DEBOUNCE_MS / 1000.0):
                    self._toggle_pending = True
                    self._last_toggle_time = now

            if button_is_low:
                self._button_was_high = False
            elif button_is_high:
                self._button_was_high = True

    # ── Buzzer ────────────────────────────────────────────────────────────────

    def play_tune(self, tune: str) -> None:
        if not ENABLE_BUZZER:
            return
        msg = PlayTuneV2()
        msg.format = TUNE_FORMAT
        msg.tune = tune
        self._tune_pub.publish(msg)

    # ── State Properties ──────────────────────────────────────────────────────

    @property
    def armed(self) -> bool:
        with self._lock:
            return self._armed

    @property
    def mode(self) -> str:
        with self._lock:
            return self._mode

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

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
