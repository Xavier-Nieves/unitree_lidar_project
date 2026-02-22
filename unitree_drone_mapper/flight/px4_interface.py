"""PX4 DDS offboard control — arm, disarm, mode switching, velocity commands.

Uses ROS 2 DDS (micro-XRCE-DDS) to communicate with PX4 directly,
bypassing MAVROS. This is the recommended approach for PX4 v1.14+.

Topics used:
  - /fmu/in/offboard_control_mode  (publish)
  - /fmu/in/trajectory_setpoint     (publish)
  - /fmu/in/vehicle_command          (publish)
  - /fmu/out/vehicle_status          (subscribe)
  - /fmu/out/vehicle_local_position  (subscribe)
  - /fmu/out/battery_status          (subscribe)
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node

from utils.logger import setup_logger
from utils.ros_helpers import RELIABLE_QOS, SENSOR_QOS

logger = setup_logger(__name__)


class PX4Interface(Node):
    """ROS 2 node for PX4 offboard control via DDS."""

    def __init__(self, config: dict):
        """Initialize PX4 interface.

        Args:
            config: PX4 config dict from config.yaml (px4 + flight sections).
        """
        super().__init__("px4_interface")

        self.takeoff_altitude = config.get("flight", {}).get("takeoff_altitude", 5.0)
        self.cruise_speed = config.get("flight", {}).get("cruise_speed", 2.0)
        self.rtl_battery = config.get("flight", {}).get("rtl_battery_percent", 25)

        # State
        self._armed = False
        self._mode = "UNKNOWN"
        self._position = [0.0, 0.0, 0.0]
        self._battery_percent = 100.0
        self._offboard_counter = 0

        # Publishers
        from px4_msgs.msg import (
            OffboardControlMode,
            TrajectorySetpoint,
            VehicleCommand,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", RELIABLE_QOS
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", RELIABLE_QOS
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", RELIABLE_QOS
        )

        # Subscribers
        from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, BatteryStatus

        self.status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status",
            self._status_callback, SENSOR_QOS
        )
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            self._position_callback, SENSOR_QOS
        )
        self.battery_sub = self.create_subscription(
            BatteryStatus, "/fmu/out/battery_status",
            self._battery_callback, SENSOR_QOS
        )

        # Heartbeat timer (offboard mode requires continuous setpoints)
        self._heartbeat_timer = self.create_timer(0.1, self._heartbeat)

        self.get_logger().info("PX4 DDS interface initialized")

    def _status_callback(self, msg):
        self._armed = msg.arming_state == 2  # ARMED
        nav_state_map = {0: "MANUAL", 4: "ALTITUDE", 14: "OFFBOARD", 5: "RTL"}
        self._mode = nav_state_map.get(msg.nav_state, f"MODE_{msg.nav_state}")

    def _position_callback(self, msg):
        self._position = [msg.x, msg.y, msg.z]

    def _battery_callback(self, msg):
        self._battery_percent = msg.remaining * 100.0

    def _heartbeat(self):
        """Send periodic offboard control mode messages."""
        from px4_msgs.msg import OffboardControlMode

        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def arm(self) -> bool:
        """Arm the vehicle.

        Returns:
            True if arm command sent successfully.
        """
        return self._send_command(
            command=176,  # MAV_CMD_COMPONENT_ARM_DISARM
            param1=1.0,   # arm
        )

    def disarm(self) -> bool:
        """Disarm the vehicle."""
        return self._send_command(
            command=176,
            param1=0.0,  # disarm
        )

    def set_offboard_mode(self) -> bool:
        """Switch to offboard flight mode."""
        return self._send_command(
            command=176,  # Placeholder — actual command depends on PX4 version
            param1=1.0,
            param2=6.0,  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        )

    def takeoff(self, altitude: Optional[float] = None) -> bool:
        """Command takeoff to specified altitude.

        Args:
            altitude: Target altitude in meters (default from config).
        """
        alt = altitude or self.takeoff_altitude
        logger.info(f"Takeoff to {alt}m")
        return self._send_command(
            command=22,  # MAV_CMD_NAV_TAKEOFF
            param7=alt,
        )

    def land(self) -> bool:
        """Command landing at current position."""
        logger.info("Landing...")
        return self._send_command(command=21)  # MAV_CMD_NAV_LAND

    def return_to_launch(self) -> bool:
        """Command return to launch."""
        logger.info("Return to launch")
        return self._send_command(command=20)  # MAV_CMD_NAV_RETURN_TO_LAUNCH

    def goto_position(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Send a position setpoint in local NED frame.

        Args:
            x, y, z: Target position in meters (NED frame, z is negative up).
            yaw: Target heading in radians.
        """
        from px4_msgs.msg import TrajectorySetpoint

        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _send_command(self, command: int, param1: float = 0.0, param2: float = 0.0,
                      param7: float = 0.0) -> bool:
        """Send a vehicle command to PX4."""
        from px4_msgs.msg import VehicleCommand

        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)
        return True

    # Properties
    @property
    def is_armed(self) -> bool:
        return self._armed

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def position(self) -> list:
        return self._position

    @property
    def battery_percent(self) -> float:
        return self._battery_percent

    def check_battery_rtl(self) -> bool:
        """Check if battery is low enough to trigger RTL.

        Returns:
            True if battery is below RTL threshold.
        """
        if self._battery_percent < self.rtl_battery:
            logger.warning(
                f"Battery low: {self._battery_percent:.1f}% "
                f"(threshold: {self.rtl_battery}%)"
            )
            return True
        return False
