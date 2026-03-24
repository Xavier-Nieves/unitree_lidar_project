#!/usr/bin/env python3
"""
rpi_health_node.py
──────────────────
Publishes Raspberry Pi system health metrics directly to foxglove-bridge
as a WebSocket client. No ROS installation required.

Metrics published every 2 seconds on topic: /rpi/health
  - cpu_percent      : overall CPU usage %
  - cpu_temp         : CPU temperature °C  (via /sys/class/thermal or vcgencmd)
  - mem_percent      : RAM usage %
  - mem_used_mb      : RAM used in MB
  - mem_total_mb     : RAM total in MB
  - disk_percent     : root partition disk usage %
  - disk_used_gb     : disk used in GB
  - disk_total_gb    : disk total in GB
  - cpu_freq_mhz     : current CPU frequency in MHz
  - cpu_count        : number of CPU cores
  - throttled        : True if Pi is currently throttled (over-temp / under-volt)
  - throttle_bits    : raw vcgencmd throttle hex value
  - uptime_s         : system uptime in seconds
  - load_avg_1m      : 1-minute load average
  - load_avg_5m      : 5-minute load average

Requirements:
  pip3 install psutil websocket-client

Usage:
  python3 rpi_health_node.py
  python3 rpi_health_node.py --url ws://localhost:8765 --rate 2.0

Run this alongside your foxglove-bridge. The bridge must already be running.
"""

import argparse
import json
import os
import subprocess
import time
import threading
import sys

try:
    import psutil
except ImportError:
    print("[rpi_health] ERROR: psutil not found. Run: pip3 install psutil")
    sys.exit(1)

try:
    import websocket
except ImportError:
    print("[rpi_health] ERROR: websocket-client not found. Run: pip3 install websocket-client")
    sys.exit(1)


# ─── Configuration ────────────────────────────────────────────────────────────
DEFAULT_URL  = "ws://localhost:8765"
DEFAULT_RATE = 2.0          # seconds between publishes
TOPIC        = "/rpi/health"
SCHEMA       = "std_msgs/String"   # foxglove-bridge compatible

start_time = time.time()


# ─── System metric readers ────────────────────────────────────────────────────

def read_cpu_temp() -> float:
    """Read CPU temperature. Tries thermal zone first, falls back to vcgencmd."""
    # Method 1: sysfs thermal zone (works on most Linux including Pi)
    thermal_path = "/sys/class/thermal/thermal_zone0/temp"
    if os.path.exists(thermal_path):
        try:
            with open(thermal_path) as f:
                return round(int(f.read().strip()) / 1000.0, 1)
        except Exception:
            pass

    # Method 2: vcgencmd (Raspberry Pi specific)
    try:
        result = subprocess.run(
            ["vcgencmd", "measure_temp"],
            capture_output=True, text=True, timeout=1
        )
        # Output: temp=47.7'C
        raw = result.stdout.strip()
        temp_str = raw.replace("temp=", "").replace("'C", "").replace("°C", "")
        return round(float(temp_str), 1)
    except Exception:
        pass

    return -1.0  # Unavailable


def read_throttle_status() -> tuple[bool, str]:
    """
    Read Pi throttle status via vcgencmd.
    Returns (is_throttled: bool, hex_value: str)

    Bit meanings (vcgencmd get_throttled):
      Bit 0  : Under-voltage detected
      Bit 1  : Arm frequency capped
      Bit 2  : Currently throttled
      Bit 3  : Soft temperature limit active
      Bit 16 : Under-voltage has occurred
      Bit 17 : Arm frequency capping has occurred
      Bit 18 : Throttling has occurred
      Bit 19 : Soft temperature limit has occurred
    """
    try:
        result = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True, text=True, timeout=1
        )
        raw = result.stdout.strip()  # e.g. "throttled=0x50000"
        hex_val = raw.replace("throttled=", "").strip()
        bits = int(hex_val, 16)
        # Bits 0-3 indicate current conditions
        currently_throttled = bool(bits & 0xF)
        return currently_throttled, hex_val
    except Exception:
        return False, "unavailable"


def read_cpu_freq_mhz() -> float:
    """Read current CPU frequency in MHz."""
    try:
        freq = psutil.cpu_freq()
        if freq:
            return round(freq.current, 1)
    except Exception:
        pass

    # Fallback: read from sysfs
    freq_path = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
    if os.path.exists(freq_path):
        try:
            with open(freq_path) as f:
                return round(int(f.read().strip()) / 1000.0, 1)  # kHz → MHz
        except Exception:
            pass
    return -1.0


def collect_metrics() -> dict:
    """Collect all system metrics and return as a dict."""
    # CPU — first call to cpu_percent with interval=None returns 0.0 on first
    # call. We call with interval=0.1 for a quick snapshot.
    cpu_pct = psutil.cpu_percent(interval=0.1)

    mem = psutil.virtual_memory()
    disk = psutil.disk_usage("/")
    load_avg = os.getloadavg()  # (1m, 5m, 15m)
    uptime_s = int(time.time() - psutil.boot_time())
    cpu_temp = read_cpu_temp()
    cpu_freq = read_cpu_freq_mhz()
    throttled, throttle_bits = read_throttle_status()

    return {
        "cpu_percent":    round(cpu_pct, 1),
        "cpu_temp":       cpu_temp,
        "cpu_freq_mhz":   cpu_freq,
        "cpu_count":      psutil.cpu_count(logical=True),
        "mem_percent":    round(mem.percent, 1),
        "mem_used_mb":    round(mem.used / 1024 / 1024, 1),
        "mem_total_mb":   round(mem.total / 1024 / 1024, 1),
        "disk_percent":   round(disk.percent, 1),
        "disk_used_gb":   round(disk.used / 1024 / 1024 / 1024, 2),
        "disk_total_gb":  round(disk.total / 1024 / 1024 / 1024, 2),
        "throttled":      throttled,
        "throttle_bits":  throttle_bits,
        "uptime_s":       uptime_s,
        "load_avg_1m":    round(load_avg[0], 2),
        "load_avg_5m":    round(load_avg[1], 2),
        "timestamp":      round(time.time(), 3),
    }


# ─── Foxglove-bridge WebSocket publisher ──────────────────────────────────────

class RpiHealthPublisher:
    def __init__(self, url: str, rate_s: float):
        self.url     = url
        self.rate_s  = rate_s
        self.ws      = None
        self.channel_id = 1
        self.running = False
        self._advertised = False

    def _advertise(self):
        """Tell foxglove-bridge about our topic/channel."""
        advert = {
            "op": "advertise",
            "channels": [
                {
                    "id":       self.channel_id,
                    "topic":    TOPIC,
                    "encoding": "json",
                    "schemaName": SCHEMA,
                    "schema": json.dumps({
                        "type": "object",
                        "title": "RPI Health",
                        "properties": {
                            "data": {"type": "string"}
                        }
                    })
                }
            ]
        }
        self.ws.send(json.dumps(advert))
        self._advertised = True
        print(f"[rpi_health] Advertised topic: {TOPIC}")

    def _publish(self, metrics: dict):
        """Publish one message to foxglove-bridge."""
        # std_msgs/String wraps payload in a .data field
        msg_payload = {"data": json.dumps(metrics)}
        msg = {
            "op":        "publish",
            "channelId": self.channel_id,
            "data":      msg_payload,
        }
        self.ws.send(json.dumps(msg))

    def _publish_loop(self):
        """Background thread: collect and publish metrics at set rate."""
        # Wait for connection + advertise to settle
        time.sleep(0.5)
        self._advertise()

        while self.running:
            try:
                metrics = collect_metrics()
                self._publish(metrics)
                print(
                    f"[rpi_health] CPU:{metrics['cpu_percent']}%  "
                    f"Temp:{metrics['cpu_temp']}°C  "
                    f"Mem:{metrics['mem_percent']}%  "
                    f"Disk:{metrics['disk_percent']}%  "
                    f"Throttled:{metrics['throttled']}"
                )
            except Exception as e:
                print(f"[rpi_health] Publish error: {e}")
            time.sleep(self.rate_s)

    def run(self):
        """Connect and start publishing. Reconnects on disconnect."""
        while True:
            print(f"[rpi_health] Connecting to foxglove-bridge at {self.url} ...")
            try:
                self.ws = websocket.WebSocketApp(
                    self.url,
                    on_open=self._on_open,
                    on_message=self._on_message,
                    on_error=self._on_error,
                    on_close=self._on_close,
                )
                self.ws.run_forever()
            except Exception as e:
                print(f"[rpi_health] Connection failed: {e}")

            print("[rpi_health] Disconnected. Retrying in 5s...")
            self.running = False
            self._advertised = False
            time.sleep(5)

    def _on_open(self, ws):
        print("[rpi_health] Connected to foxglove-bridge.")
        self.running = True
        t = threading.Thread(target=self._publish_loop, daemon=True)
        t.start()

    def _on_message(self, ws, message):
        # foxglove-bridge may send serverInfo on connect — we can ignore it
        pass

    def _on_error(self, ws, error):
        print(f"[rpi_health] WebSocket error: {error}")

    def _on_close(self, ws, code, msg):
        print(f"[rpi_health] Connection closed (code={code})")
        self.running = False


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Publish RPI health metrics to foxglove-bridge"
    )
    parser.add_argument(
        "--url",
        default=DEFAULT_URL,
        help=f"WebSocket URL of foxglove-bridge (default: {DEFAULT_URL})"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_RATE,
        help=f"Publish interval in seconds (default: {DEFAULT_RATE})"
    )
    args = parser.parse_args()

    print("=" * 60)
    print("  RPI Health Node")
    print(f"  Bridge URL : {args.url}")
    print(f"  Topic      : {TOPIC}")
    print(f"  Rate       : {args.rate}s")
    print("=" * 60)

    # Quick sanity check — report initial metrics
    print("[rpi_health] Initial metrics:")
    m = collect_metrics()
    for k, v in m.items():
        print(f"  {k:20s}: {v}")
    print()

    publisher = RpiHealthPublisher(url=args.url, rate_s=args.rate)
    publisher.run()


if __name__ == "__main__":
    main()
