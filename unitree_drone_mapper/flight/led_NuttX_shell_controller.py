#!/usr/bin/env python3
"""
LED controller via NuttX shell tunneled through MAVLink SERIAL_CONTROL
Works on Pixhawk 6X where MAV_CMD_DO_LED_CONTROL is not supported
ROS 2 Jazzy + MAVROS
"""
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, Mavlink
from mavros_msgs.srv import CommandLong
from pymavlink.dialects.v20 import common as mavlink2
import struct
import time

# Colors available in NuttX led_control shell command
# Format: (shell_color_name, log_description)
MODE_LED = {
    "OFFBOARD":     "cyan",
    "POSCTL":       "green",
    "ALTCTL":       "blue",
    "STABILIZED":   "amber",
    "MANUAL":       "white",
    "AUTO.MISSION": "purple",
    "AUTO.RTL":     "red",
    "AUTO.LAND":    "yellow",
    "AUTO.TAKEOFF": "green",
}

DEFAULT_LED = "white"


class LEDShellController(Node):

    def __init__(self):
        super().__init__('px4_led_shell_controller')

        self.current_mode = ""
        self.prev_mode = ""

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        # Publisher for raw MAVLink messages (SERIAL_CONTROL)
        self.mavlink_pub = self.create_publisher(
            Mavlink,
            '/mavros/mavlink/send',
            10
        )

        # 2Hz timer — re-sends OFFBOARD to fight Commander override
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('LED Shell Controller started!')

    def state_callback(self, msg):
        self.current_mode = msg.mode

    def send_shell_command(self, cmd: str):
        """
        Tunnel a NuttX shell command via MAVLink SERIAL_CONTROL.
        Equivalent to typing in the NuttX shell directly.
        """
        # SERIAL_CONTROL message (id=126)
        # device=10 (SERIAL_CONTROL_DEV_SHELL), flags=0x4 (RESPOND), timeout=0
        cmd_bytes = (cmd + '\n').encode('ascii')

        # Pack as MAVLink SERIAL_CONTROL (msg id 126)
        # Fields: device(u8), flags(u8), timeout(u16), baudrate(u32), count(u8), data(70 bytes)
        device  = 10    # SERIAL_CONTROL_DEV_SHELL
        flags   = 0x04  # SERIAL_CONTROL_FLAG_RESPOND
        timeout = 0
        baud    = 0
        count   = len(cmd_bytes)

        # Pad data to 70 bytes
        data = cmd_bytes[:70].ljust(70, b'\x00')

        payload = struct.pack('<BBHIB70s', device, flags, timeout, baud, count, data)

        msg = Mavlink()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.framing_status = Mavlink.FRAMING_OK
        msg.magic = 253         # MAVLink 2 magic
        msg.len = len(payload)
        msg.msgid = 126         # SERIAL_CONTROL
        msg.payload64 = list(struct.unpack('<' + 'Q' * (len(payload) // 8 + 1),
                             payload.ljust((len(payload) // 8 + 1) * 8, b'\x00')))
        self.mavlink_pub.publish(msg)
        self.get_logger().debug(f'Shell cmd sent: {cmd}')

    def set_led_color(self, color: str, mode: str = "on"):
        """Send led_control command through NuttX shell"""
        # mode can be: on, blink, breathe, flash
        cmd = f'led_control {mode} -c {color}'
        self.send_shell_command(cmd)

    def timer_callback(self):
        mode = self.current_mode

        if mode != self.prev_mode:
            color = MODE_LED.get(mode, DEFAULT_LED)
            self.get_logger().info(f'Mode → {mode}  |  LED → {color}')

            if mode == "OFFBOARD":
                self.set_led_color(color, "breathe")   # Breathe cyan in offboard
            elif mode in ("AUTO.RTL", "AUTO.LAND"):
                self.set_led_color(color, "blink")     # Blink for critical modes
            else:
                self.set_led_color(color, "on")        # Solid for others

            self.prev_mode = mode

        # Keep re-sending OFFBOARD breathe to fight Commander
        elif mode == "OFFBOARD":
            self.set_led_color("cyan", "breathe")


def main(args=None):
    rclpy.init(args=args)
    node = LEDShellController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()