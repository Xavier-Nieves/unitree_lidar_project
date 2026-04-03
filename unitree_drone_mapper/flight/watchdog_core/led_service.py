#!/usr/bin/env python3
"""
flight/led_service.py — DronePi LED Observer Service.

ARCHITECTURE
------------
This is the SOLE process that owns GPIO. It observes system state by reading
status files written by drone_watchdog.py and main.py, derives the correct
LED state using a priority table, and drives the hardware accordingly.

Neither drone_watchdog.py nor main.py tells this service what colour to show.
They report facts about the system (FCU connected, armed, lock mode, Hailo
health). This service decides what those facts mean visually.

ADDING NEW STATES
-----------------
To add a new LED pattern:
  1. Add a row to _PRIORITY_RULES in _derive_state() with your condition.
  2. Add the pattern to _PATTERNS dict (channel, blink_hz or None, dual).
  3. That is all — no changes needed in watchdog or main.py.

STATUS FILES (read by this service)
------------------------------------
  /tmp/dronepi_mission.lock     Written by watchdog/main. JSON with "mode" key.
                                Modes: manual_scan, autonomous, bench_scan, absent
  /tmp/watchdog_status.json     Written by drone_watchdog.py every poll cycle.
                                Schema: {
                                  "ts":        float,   # Unix timestamp (heartbeat)
                                  "fcu":       bool,    # FCU connected
                                  "armed":     bool,    # Vehicle armed
                                  "processing": bool,   # Post-flight processing active
                                }
  /tmp/main_status.json         Written by main.py every poll cycle.
                                Schema: {
                                  "ts":          float, # Unix timestamp (heartbeat)
                                  "hailo_active":  bool,
                                  "hailo_degraded":bool,
                                  "hailo_failed":  bool,
                                }

PHYSICAL PIN MAP
----------------
  GND    →  Pin 34  (common ground)
  Green  →  BCM 25
  Yellow →  BCM  8
  Red    →  BCM  7

  BCM numbers must match watchdog_core/led_controller.py and test_led_controller.py.

HEARTBEAT TIMEOUTS
------------------
  WATCHDOG_DEAD_S = 5.0   Watchdog heartbeat stale → red slow blink
  MAIN_DEAD_S     = 8.0   Main heartbeat stale during autonomous → red fast blink
                          (longer timeout because main.py has longer blocking calls)

USAGE
-----
  Managed by systemd led.service. See /etc/systemd/system/led.service.
  Manual run (stop led.service first):
    GPIOZERO_PIN_FACTORY=lgpio python3 flight/led_service.py

REQUIREMENTS
------------
  pip install gpiozero lgpio   (inside the dronepi conda env)
"""

import json
import logging
import os
import sys
import time
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# Backend must be forced before any gpiozero import.
# ---------------------------------------------------------------------------
os.environ.setdefault("GPIOZERO_PIN_FACTORY", "lgpio")
os.environ.setdefault("HOME", "/tmp")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [LED] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

WATCHDOG_STATUS_FILE = "/tmp/watchdog_status.json"
MAIN_STATUS_FILE     = "/tmp/main_status.json"
MISSION_LOCK_FILE    = "/tmp/dronepi_mission.lock"

POLL_HZ          = 10      # State derivation rate
WATCHDOG_DEAD_S  = 5.0     # Watchdog heartbeat timeout (seconds)
MAIN_DEAD_S      = 8.0     # Main heartbeat timeout during autonomous (seconds)

_BCM_PINS: dict[str, int] = {
    "green":  25,
    "yellow": 8,
    "red":    7,
}

# ---------------------------------------------------------------------------
# Pattern registry
#
# Each entry maps a state name to its hardware pattern:
#   channel  : "green" | "yellow" | "red" | None (all off)
#   blink_hz : float blink rate, or None for solid
#   dual     : True = drive both green AND yellow solid (HAILO_ACTIVE)
#
# To add a new visual pattern, add an entry here and a rule in _derive_state().
# ---------------------------------------------------------------------------

@dataclass
class _Pattern:
    channel:  Optional[str]
    blink_hz: Optional[float]
    dual:     bool = False

    @property
    def half_cycle(self) -> Optional[float]:
        """Blink half-cycle duration in seconds, or None if solid."""
        return (1.0 / self.blink_hz / 2.0) if self.blink_hz else None


_PATTERNS: dict[str, _Pattern] = {
    # ── System states ─────────────────────────────────────────────────────────
    "OFF":           _Pattern(None,     None   ),   # All LEDs off
    "IDLE":          _Pattern("green",  None   ),   # Green solid
    "SCANNING":      _Pattern("green",  1.5    ),   # Green 1.5 Hz blink
    "PROCESSING":    _Pattern("yellow", None   ),   # Yellow solid
    "WAITING_FCU":   _Pattern("yellow", 1.5    ),   # Yellow 1.5 Hz blink
    "ERROR":         _Pattern("red",    None   ),   # Red solid
    "WARNING":       _Pattern("red",    1.5    ),   # Red 1.5 Hz blink
    # ── Hailo states ──────────────────────────────────────────────────────────
    "HAILO_ACTIVE":  _Pattern("green",  None,  True),  # Green + Yellow solid
    "HAILO_DEGRADED":_Pattern("yellow", 3.0    ),   # Yellow 3 Hz fast blink
    "HAILO_FAILED":  _Pattern("red",    3.0    ),   # Red 3 Hz fast blink
    # ── Fault states ──────────────────────────────────────────────────────────
    "WATCHDOG_DEAD": _Pattern("red",    0.8    ),   # Red 0.8 Hz slow blink
    "MAIN_DEAD":     _Pattern("red",    3.0    ),   # Red 3 Hz fast blink
}


# ---------------------------------------------------------------------------
# System state snapshot — populated by reading status files each poll cycle
# ---------------------------------------------------------------------------

@dataclass
class _SystemSnapshot:
    """Decoded view of all system status files at one point in time."""
    watchdog_ts:       float = 0.0
    watchdog_fcu:      bool  = False
    watchdog_armed:    bool  = False
    watchdog_processing: bool = False

    main_ts:           float = 0.0
    hailo_active:      bool  = False
    hailo_degraded:    bool  = False
    hailo_failed:      bool  = False

    lock_mode:         str   = ""   # manual_scan | autonomous | bench_scan | ""


def _read_json(path: str) -> dict:
    """Read a JSON file safely. Returns empty dict on any error."""
    try:
        with open(path, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {}


def _read_snapshot() -> _SystemSnapshot:
    """
    Read all status files and return a unified system snapshot.

    All reads are non-blocking and fault-tolerant — a missing or malformed
    file is treated as zeros/defaults, not a crash.
    """
    s = _SystemSnapshot()

    wd = _read_json(WATCHDOG_STATUS_FILE)
    s.watchdog_ts          = float(wd.get("ts",         0.0))
    s.watchdog_fcu         = bool (wd.get("fcu",        False))
    s.watchdog_armed       = bool (wd.get("armed",      False))
    s.watchdog_processing  = bool (wd.get("processing", False))

    mn = _read_json(MAIN_STATUS_FILE)
    s.main_ts              = float(mn.get("ts",            0.0))
    s.hailo_active         = bool (mn.get("hailo_active",  False))
    s.hailo_degraded       = bool (mn.get("hailo_degraded",False))
    s.hailo_failed         = bool (mn.get("hailo_failed",  False))

    lock = _read_json(MISSION_LOCK_FILE)
    s.lock_mode            = str  (lock.get("mode",        ""))

    return s


# ---------------------------------------------------------------------------
# State derivation — priority table
# ---------------------------------------------------------------------------

def _derive_state(snap: _SystemSnapshot, now: float) -> str:
    """
    Derive the correct LED state from the current system snapshot.

    Rules are evaluated top-to-bottom. The first matching rule wins.
    This is the single place where all LED logic lives — to add a new
    condition, add a rule here and a pattern in _PATTERNS.

    Parameters
    ----------
    snap : _SystemSnapshot
        Decoded view of all status files at this poll cycle.
    now : float
        Current time.time() — passed in so all staleness checks use the
        same reference point.

    Returns
    -------
    str
        Key into _PATTERNS.
    """
    watchdog_age = now - snap.watchdog_ts if snap.watchdog_ts > 0 else float("inf")
    main_age     = now - snap.main_ts     if snap.main_ts     > 0 else float("inf")

    # ── Priority 1: Watchdog process dead ────────────────────────────────────
    # Only declared after at least one valid heartbeat has been seen (ts > 0)
    # so we do not false-alarm during the startup window.
    if snap.watchdog_ts > 0 and watchdog_age > WATCHDOG_DEAD_S:
        return "WATCHDOG_DEAD"

    # ── Priority 2: Main process dead during autonomous mission ───────────────
    # main.py owns the flight stack in autonomous mode. If it dies mid-mission
    # that is a critical fault — distinct from watchdog dead.
    if snap.lock_mode == "autonomous" and snap.main_ts > 0 and main_age > MAIN_DEAD_S:
        return "MAIN_DEAD"

    # ── Priority 3: Hailo hard failure ────────────────────────────────────────
    if snap.hailo_failed:
        return "HAILO_FAILED"

    # ── Priority 4: Hailo degraded ────────────────────────────────────────────
    if snap.hailo_degraded:
        return "HAILO_DEGRADED"

    # ── Priority 5: Hailo active during autonomous flight ─────────────────────
    if snap.lock_mode == "autonomous" and snap.hailo_active:
        return "HAILO_ACTIVE"

    # ── Priority 6: Flight stack running (any scan mode) ──────────────────────
    if snap.lock_mode in ("manual_scan", "autonomous"):
        return "SCANNING"

    # ── Priority 7: Post-flight processing ────────────────────────────────────
    if snap.watchdog_processing:
        return "PROCESSING"

    # ── Priority 8: FCU not yet connected ─────────────────────────────────────
    if not snap.watchdog_fcu:
        return "WAITING_FCU"

    # ── Priority 9: Default — system healthy, no active task ──────────────────
    return "IDLE"


# ---------------------------------------------------------------------------
# Blink thread — single instance, replaced on pattern change
# ---------------------------------------------------------------------------

class _BlinkThread(threading.Thread):
    """
    Daemon thread that toggles one LED channel at a fixed rate.

    Uses threading.Event.wait() instead of time.sleep() so stop() causes
    an immediate wakeup. Drives the pin LOW on exit so the pin state is
    always determinate after stop() + join().
    """

    def __init__(self, led, half_cycle: float) -> None:
        super().__init__(daemon=True)
        self._led        = led
        self._half_cycle = half_cycle
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
            self._led.on()
            if self._stop_event.wait(timeout=self._half_cycle):
                break
            self._led.off()
            if self._stop_event.wait(timeout=self._half_cycle):
                break
        try:
            self._led.off()
        except Exception:
            pass

    def stop(self) -> None:
        self._stop_event.set()


# ---------------------------------------------------------------------------
# LED hardware driver
# ---------------------------------------------------------------------------

class LEDDriver:
    """
    Hardware abstraction for the three-channel LED indicator.

    Instantiated once at service startup. apply() is the only public method —
    it accepts a state name string, looks up the pattern, and drives hardware.
    All GPIO operations are internal to this class.

    Extending:
        Add a new entry to _PATTERNS at module level, then call
        driver.apply("YOUR_NEW_STATE") from led_service main loop or
        _derive_state(). No changes to this class are required.
    """

    def __init__(self) -> None:
        from gpiozero import LED
        self._leds: dict[str, object] = {
            name: LED(bcm) for name, bcm in _BCM_PINS.items()
        }
        self._blink_thread: Optional[_BlinkThread] = None
        self._current: str = ""
        self._all_off()
        logger.info(
            "GPIO initialised — BCM G:%d Y:%d R:%d",
            _BCM_PINS["green"], _BCM_PINS["yellow"], _BCM_PINS["red"],
        )

    def apply(self, state_name: str) -> None:
        """
        Transition to the named LED state.

        No-op if already in that state — avoids unnecessary blink thread
        restarts on every poll cycle. Unknown state names are logged and
        treated as OFF so the service never crashes on a new string.

        Parameters
        ----------
        state_name : str
            Must be a key in _PATTERNS. Unknown keys degrade to OFF.
        """
        if state_name == self._current:
            return

        if state_name not in _PATTERNS:
            logger.warning("Unknown state '%s' — treating as OFF. Add to _PATTERNS.", state_name)
            state_name = "OFF"

        pattern = _PATTERNS[state_name]

        self._stop_blink()
        self._all_off()

        if pattern.channel is not None:
            if pattern.half_cycle is not None:
                self._start_blink(pattern.channel, pattern.half_cycle)
            else:
                self._leds[pattern.channel].on()
                if pattern.dual:
                    self._leds["yellow"].on()

        self._current = state_name
        logger.info("State → %s", state_name)

    def cleanup(self) -> None:
        """Drive all pins LOW, stop blink thread, release GPIO resources."""
        self._stop_blink()
        self._all_off()
        for led in self._leds.values():
            try:
                led.close()
            except Exception:
                pass
        self._leds.clear()
        logger.info("GPIO released.")

    # ── Internal ──────────────────────────────────────────────────────────────

    def _all_off(self) -> None:
        for led in self._leds.values():
            try:
                led.off()
            except Exception:
                pass

    def _start_blink(self, channel: str, half_cycle: float) -> None:
        self._blink_thread = _BlinkThread(self._leds[channel], half_cycle)
        self._blink_thread.start()

    def _stop_blink(self) -> None:
        if self._blink_thread is None:
            return
        self._blink_thread.stop()
        # Timeout = two half-cycles + 0.1s margin.
        # Worst-case wakeup is one half-cycle; the margin covers scheduling jitter.
        longest_half = max(p.half_cycle for p in _PATTERNS.values()
                           if p.half_cycle is not None)
        self._blink_thread.join(timeout=longest_half * 2 + 0.1)
        if self._blink_thread.is_alive():
            logger.warning("Blink thread did not exit within timeout.")
        self._blink_thread = None


# ---------------------------------------------------------------------------
# Main service loop
# ---------------------------------------------------------------------------

def main() -> None:
    logger.info("=" * 52)
    logger.info("  DronePi LED Observer Service")
    logger.info("  Watchdog timeout : %.1fs → WATCHDOG_DEAD", WATCHDOG_DEAD_S)
    logger.info("  Main timeout     : %.1fs → MAIN_DEAD (autonomous only)", MAIN_DEAD_S)
    logger.info("  Poll rate        : %d Hz", POLL_HZ)
    logger.info("=" * 52)

    try:
        driver = LEDDriver()
    except Exception as exc:
        logger.error("GPIO init failed: %s", exc)
        logger.error("Ensure lgpio is installed and user is in the gpio group.")
        sys.exit(1)

    poll_interval = 1.0 / POLL_HZ

    try:
        while True:
            snap  = _read_snapshot()
            state = _derive_state(snap, now=time.time())
            driver.apply(state)
            time.sleep(poll_interval)

    except KeyboardInterrupt:
        logger.info("Interrupted.")
    finally:
        driver.cleanup()


if __name__ == "__main__":
    main()
