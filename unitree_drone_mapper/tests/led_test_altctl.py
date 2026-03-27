#!/usr/bin/env python3
"""
led_test_altctl.py — Quick bench test for NuttX shell LED control.

Forces cyan/blink on ALTCTL so you can verify the SERIAL_CONTROL
pipeline works indoors without GPS.

Triggers
--------
  ALTCTL   → cyan blink   (test target)
  any other mode → white on (neutral fallback)

Run
---
  python3 led_test_altctl.py

Watch the logs — if you see "Shell → led_control blink -c cyan" but the
LED doesn't change, the issue is in the NuttX binary name or device id.
If you don't even see the shell line, the MAVROS publisher isn't connecting.
"""

import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import Mavlink, State

# ── SERIAL_CONTROL constants ──────────────────────────────────────────────────

MAVLINK_MSG_ID_SERIAL_CONTROL = 126
SERIAL_CONTROL_DEV_SHELL      = 10
SERIAL_CONTROL_FLAG_RESPOND   = 0x04
MAVLINK2_MAGIC                = 253
MAVLINK_SYSID                 = 255
MAVLINK_COMPID                = 0

# ── Topic ─────────────────────────────────────────────────────────────────────
# MAVROS on this system uses the /uas1/ namespace.
# mavlink_sink  = publisher → Pixhawk  (we write here)
# mavlink_source = Pixhawk → ROS       (we don't touch this)

MAVLINK_SEND_TOPIC = "/uas1/mavlink_sink"

# ── Test config ───────────────────────────────────────────────────────────────

TEST_MODE   = "ALTCTL"
TEST_COLOR  = "cyan"
TEST_EFFECT = "blink"

FALLBACK_COLOR  = "white"
FALLBACK_EFFECT = "on"

RESEND_INTERVAL_S = 0.5


class LEDTestNode(Node):

    def __init__(self):
        super().__init__("led_test_altctl")

        self._current_mode = ""
        self._prev_mode    = ""

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            State, "/mavros/state", self._state_cb, state_qos)

        # QoS must be BEST_EFFORT to match /uas1/mavlink_sink's profile,
        # otherwise messages are silently dropped by the middleware.
        mavlink_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._mavlink_pub = self.create_publisher(
            Mavlink, MAVLINK_SEND_TOPIC, mavlink_qos)

        self.create_timer(RESEND_INTERVAL_S, self._timer_cb)

        self.get_logger().info("LED test node started.")
        self.get_logger().info(f"Watching for mode: {TEST_MODE}")
        self.get_logger().info(f"  → will send: led_control {TEST_EFFECT} -c {TEST_COLOR}")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg: State) -> None:
        self._current_mode = msg.mode

    def _timer_cb(self) -> None:
        mode = self._current_mode

        # Log every mode change so you can see exactly what PX4 is reporting
        if mode != self._prev_mode:
            self.get_logger().info(f"Mode changed: {self._prev_mode!r} → {mode!r}")
            self._prev_mode = mode

        if mode == TEST_MODE:
            self._set_led(TEST_COLOR, TEST_EFFECT)
        else:
            self._set_led(FALLBACK_COLOR, FALLBACK_EFFECT)

    # ── LED helpers ───────────────────────────────────────────────────────────

    def _set_led(self, color: str, effect: str) -> None:
        cmd = f"led_control {effect} -c {color}"
        self._send_shell(cmd)
        self.get_logger().debug(f"Shell → {cmd}")

    def _send_shell(self, cmd: str) -> None:
        cmd_bytes = (cmd + "\n").encode("ascii")
        count     = min(len(cmd_bytes), 70)
        data      = cmd_bytes[:70].ljust(70, b"\x00")

        payload = struct.pack(
            "<IHBBb70s",
            0,      # baudrate
            0,      # timeout
            SERIAL_CONTROL_DEV_SHELL,
            SERIAL_CONTROL_FLAG_RESPOND,
            count,
            data,
        )

        padded    = payload.ljust((len(payload) + 7) // 8 * 8, b"\x00")
        n_slots   = len(padded) // 8
        payload64 = list(struct.unpack(f"<{n_slots}Q", padded))

        msg                = Mavlink()
        msg.header.stamp   = self.get_clock().now().to_msg()
        msg.framing_status = Mavlink.FRAMING_OK
        msg.magic          = MAVLINK2_MAGIC
        msg.msgid          = MAVLINK_MSG_ID_SERIAL_CONTROL
        msg.sysid          = MAVLINK_SYSID
        msg.compid         = MAVLINK_COMPID
        msg.len            = n_slots
        msg.payload64      = payload64

        self._mavlink_pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LEDTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
