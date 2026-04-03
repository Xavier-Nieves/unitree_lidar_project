#!/usr/bin/env python3
"""
rpi_health_node.py
Publishes Raspberry Pi system health metrics as a ROS 2 node.

Topic: /rpi/health  (std_msgs/String, JSON payload)
Rate:  2.0 s
"""

import json
import os
import subprocess
import sys
import time

try:
    import psutil
except ImportError:
    print("[rpi_health] ERROR: psutil not found — run: pip install psutil")
    sys.exit(1)

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    print("[rpi_health] ERROR: ROS 2 not sourced — source /opt/ros/jazzy/setup.bash")
    sys.exit(1)


def read_cpu_temp() -> float:
    thermal = "/sys/class/thermal/thermal_zone0/temp"
    if os.path.exists(thermal):
        try:
            with open(thermal, "r") as f:
                return round(int(f.read().strip()) / 1000.0, 1)
        except Exception:
            pass

    try:
        r = subprocess.run(
            ["vcgencmd", "measure_temp"],
            capture_output=True,
            text=True,
            timeout=1,
            check=False,
        )
        val = r.stdout.strip().replace("temp=", "").replace("'C", "").replace("°C", "")
        return round(float(val), 1)
    except Exception:
        pass

    return -1.0


def read_throttle() -> tuple[bool, str]:
    try:
        r = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True,
            text=True,
            timeout=1,
            check=False,
        )
        hex_val = r.stdout.strip().replace("throttled=", "").strip()
        bits = int(hex_val, 16)
        return bool(bits & 0xF), hex_val
    except Exception:
        return False, "unavailable"


def read_cpu_freq() -> float:
    try:
        freq = psutil.cpu_freq()
        if freq:
            return round(freq.current, 1)
    except Exception:
        pass

    freq_path = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
    if os.path.exists(freq_path):
        try:
            with open(freq_path, "r") as f:
                return round(int(f.read().strip()) / 1000.0, 1)
        except Exception:
            pass

    return -1.0


def collect_metrics() -> dict:
    mem = psutil.virtual_memory()
    disk = psutil.disk_usage("/")
    load = os.getloadavg()
    throttled, throttle_bits = read_throttle()

    return {
        "cpu_percent": round(psutil.cpu_percent(interval=0.1), 1),
        "cpu_temp": read_cpu_temp(),
        "cpu_freq_mhz": read_cpu_freq(),
        "cpu_count": psutil.cpu_count(logical=True),
        "mem_percent": round(mem.percent, 1),
        "mem_used_mb": round(mem.used / 1024 / 1024, 1),
        "mem_total_mb": round(mem.total / 1024 / 1024, 1),
        "disk_percent": round(disk.percent, 1),
        "disk_used_gb": round(disk.used / 1024**3, 2),
        "disk_total_gb": round(disk.total / 1024**3, 2),
        "throttled": throttled,
        "throttle_bits": throttle_bits,
        "uptime_s": int(time.time() - psutil.boot_time()),
        "load_avg_1m": round(load[0], 2),
        "load_avg_5m": round(load[1], 2),
        "timestamp": round(time.time(), 3),
    }


class RpiHealthNode(Node):
    def __init__(self, rate_s: float = 2.0):
        super().__init__("rpi_health")
        self._pub = self.create_publisher(String, "/rpi/health", 10)
        self._timer = self.create_timer(rate_s, self._publish)
        self.get_logger().info(
            f"rpi_health node started — publishing /rpi/health every {rate_s}s"
        )

    def _publish(self) -> None:
        try:
            metrics = collect_metrics()
            msg = String()
            msg.data = json.dumps(metrics)
            self._pub.publish(msg)
            self.get_logger().info(
                f"CPU:{metrics['cpu_percent']}%  "
                f"Temp:{metrics['cpu_temp']}C  "
                f"Mem:{metrics['mem_percent']}%  "
                f"Disk:{metrics['disk_percent']}%  "
                f"Throttled:{metrics['throttled']}"
            )
        except Exception as exc:
            self.get_logger().warn(f"Metric collection error: {exc}")


def main() -> None:
    rclpy.init()
    node = RpiHealthNode(rate_s=2.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
