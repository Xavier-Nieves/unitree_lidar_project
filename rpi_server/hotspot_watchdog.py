#!/usr/bin/env python3
"""
hotspot_watchdog.py
───────────────────
Keeps the dronepi-hotspot Wi-Fi connection alive on wlan0.
Runs as a persistent systemd service. Checks every 10 seconds
and brings the connection back up if it drops.

Location: /home/dronepi/unitree_lidar_project/rpi_server/hotspot_watchdog.py
"""

import subprocess
import time
import logging
import sys

# ── config ────────────────────────────────────────────────────────────────────
CONNECTION_NAME = "dronepi-hotspot"
INTERFACE       = "wlan0"
CHECK_INTERVAL  = 10   # seconds between checks
RETRY_DELAY     = 5    # seconds to wait after a failed bring-up

# ── logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  [hotspot] %(message)s",
    datefmt="%H:%M:%S",
    stream=sys.stdout,
)


class HotspotWatchdog:
    """Monitors wlan0 and keeps dronepi-hotspot connection active."""

    def __init__(self, connection: str, interface: str):
        self.connection = connection
        self.interface  = interface

    def _is_up(self) -> bool:
        """Return True if the interface has an IP address assigned."""
        try:
            result = subprocess.run(
                ["ip", "addr", "show", self.interface],
                capture_output=True, text=True, timeout=3
            )
            return "10.42.0.1" in result.stdout
        except Exception:
            return False

    def _bring_up(self) -> bool:
        """Attempt to bring up the connection. Returns True on success."""
        try:
            result = subprocess.run(
                ["nmcli", "connection", "up", self.connection],
                capture_output=True, text=True, timeout=15
            )
            return result.returncode == 0
        except Exception as e:
            logging.error(f"bring-up error: {e}")
            return False

    def run(self):
        """Main watchdog loop."""
        logging.info(f"Starting — monitoring {self.interface} ({self.connection})")

        # Initial bring-up on start
        if not self._is_up():
            logging.info("Hotspot not active on start — bringing up...")
            if self._bring_up():
                logging.info("Hotspot up successfully.")
            else:
                logging.warning("Initial bring-up failed — will retry in loop.")

        while True:
            time.sleep(CHECK_INTERVAL)
            if not self._is_up():
                logging.warning(f"{self.interface} lost IP — reconnecting...")
                time.sleep(RETRY_DELAY)
                if self._bring_up():
                    logging.info("Hotspot restored successfully.")
                else:
                    logging.error("Reconnect failed — will retry next cycle.")
            # else: silent — no log spam when healthy


def main():
    watchdog = HotspotWatchdog(
        connection=CONNECTION_NAME,
        interface=INTERFACE,
    )
    watchdog.run()


if __name__ == "__main__":
    main()
