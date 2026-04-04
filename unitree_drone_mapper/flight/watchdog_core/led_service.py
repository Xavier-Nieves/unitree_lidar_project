#!/usr/bin/env python3
"""
flight/led_service.py — matched LED state machine for DronePi.

Adds richer LED patterns so the LEDs line up with the buzzer semantics.
This service still derives most states from status files, but now also
understands transient LED override events written by the watchdog:
- led_state
- led_until
- warning
- error
- critical
- system_failure
- stack_running
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

os.environ.setdefault("GPIOZERO_PIN_FACTORY", "lgpio")
os.environ.setdefault("HOME", "/tmp")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [LED] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

WATCHDOG_STATUS_FILE = "/tmp/watchdog_status.json"
MAIN_STATUS_FILE     = "/tmp/main_status.json"
MISSION_LOCK_FILE    = "/tmp/dronepi_mission.lock"

POLL_HZ          = 10
WATCHDOG_DEAD_S  = 5.0
MAIN_DEAD_S      = 8.0

_BCM_PINS: dict[str, int] = {
    "green":  25,
    "yellow": 8,
    "red":    7,
}


@dataclass
class _Pattern:
    mode: str                          # off | solid | blink | sequence
    channels: tuple[str, ...] = ()
    blink_hz: Optional[float] = None
    frames: tuple[tuple[tuple[str, ...], float], ...] = ()

    @property
    def half_cycle(self) -> Optional[float]:
        return (1.0 / self.blink_hz / 2.0) if self.blink_hz else None


_PATTERNS: dict[str, _Pattern] = {
    "OFF":            _Pattern("off"),
    "SYSTEM_START":   _Pattern(
        "sequence",
        frames=(
            (("green",),  0.22),
            (tuple(),     0.06),
            (("yellow",), 0.22),
            (tuple(),     0.06),
            (("red",),    0.22),
            (tuple(),     0.06),
        ),
    ),
    "IDLE":           _Pattern("solid", channels=("green",)),
    "WAITING_FCU":    _Pattern("blink", channels=("yellow",), blink_hz=1.5),
    "SCAN_READY":     _Pattern("solid", channels=("green", "yellow")),
    "SCAN_START":     _Pattern("blink", channels=("green", "yellow"), blink_hz=2.5),
    "SCANNING":       _Pattern("blink", channels=("green",), blink_hz=1.5),
    "SCAN_FINISHED":  _Pattern("blink", channels=("green", "yellow"), blink_hz=2.0),
    "PROCESSING":     _Pattern("solid", channels=("yellow",)),
    "WARNING":        _Pattern("blink", channels=("yellow",), blink_hz=2.5),
    "ERROR":          _Pattern("solid", channels=("red",)),
    "CRITICAL":       _Pattern("blink", channels=("green", "yellow", "red"), blink_hz=2.5),
    "SYSTEM_FAILURE": _Pattern("blink", channels=("green", "yellow", "red"), blink_hz=5.0),
    "HAILO_ACTIVE":   _Pattern("solid", channels=("green", "yellow")),
    "HAILO_DEGRADED": _Pattern("blink", channels=("yellow",), blink_hz=3.0),
    "HAILO_FAILED":   _Pattern("blink", channels=("red",), blink_hz=3.0),
    "WATCHDOG_DEAD":  _Pattern("blink", channels=("red",), blink_hz=0.8),
    "MAIN_DEAD":      _Pattern("blink", channels=("red",), blink_hz=3.0),
}


@dataclass
class _SystemSnapshot:
    watchdog_ts: float = 0.0
    watchdog_fcu: bool = False
    watchdog_armed: bool = False
    watchdog_processing: bool = False
    watchdog_stack_running: bool = False

    led_state: str = ""
    led_until: float = 0.0

    warning: bool = False
    error: bool = False
    critical: bool = False
    system_failure: bool = False

    main_ts: float = 0.0
    hailo_active: bool = False
    hailo_degraded: bool = False
    hailo_failed: bool = False

    lock_mode: str = ""


def _read_json(path: str) -> dict:
    try:
        with open(path, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {}


def _read_snapshot() -> _SystemSnapshot:
    s = _SystemSnapshot()

    wd = _read_json(WATCHDOG_STATUS_FILE)
    s.watchdog_ts           = float(wd.get("ts", 0.0))
    s.watchdog_fcu          = bool(wd.get("fcu", False))
    s.watchdog_armed        = bool(wd.get("armed", False))
    s.watchdog_processing   = bool(wd.get("processing", False))
    s.watchdog_stack_running= bool(wd.get("stack_running", False))

    s.led_state             = str(wd.get("led_state", ""))
    s.led_until             = float(wd.get("led_until", 0.0))

    s.warning               = bool(wd.get("warning", False))
    s.error                 = bool(wd.get("error", False))
    s.critical              = bool(wd.get("critical", False))
    s.system_failure        = bool(wd.get("system_failure", False))

    mn = _read_json(MAIN_STATUS_FILE)
    s.main_ts               = float(mn.get("ts", 0.0))
    s.hailo_active          = bool(mn.get("hailo_active", False))
    s.hailo_degraded        = bool(mn.get("hailo_degraded", False))
    s.hailo_failed          = bool(mn.get("hailo_failed", False))

    lock = _read_json(MISSION_LOCK_FILE)
    s.lock_mode             = str(lock.get("mode", ""))

    return s


def _derive_state(snap: _SystemSnapshot, now: float) -> str:
    watchdog_age = now - snap.watchdog_ts if snap.watchdog_ts > 0 else float("inf")
    main_age     = now - snap.main_ts if snap.main_ts > 0 else float("inf")

    # Priority 1: process-health death states
    if snap.watchdog_ts > 0 and watchdog_age > WATCHDOG_DEAD_S:
        return "WATCHDOG_DEAD"
    if snap.lock_mode == "autonomous" and snap.main_ts > 0 and main_age > MAIN_DEAD_S:
        return "MAIN_DEAD"

    # Priority 2: transient watchdog-led events
    if snap.led_state and now < snap.led_until:
        return snap.led_state

    # Priority 3: fault ladder
    if snap.system_failure:
        return "SYSTEM_FAILURE"
    if snap.critical:
        return "CRITICAL"
    if snap.error:
        return "ERROR"
    if snap.warning:
        return "WARNING"

    # Priority 4: Hailo states
    if snap.hailo_failed:
        return "HAILO_FAILED"
    if snap.hailo_degraded:
        return "HAILO_DEGRADED"
    if snap.lock_mode == "autonomous" and snap.hailo_active:
        return "HAILO_ACTIVE"

    # Priority 5: core operating states
    if snap.watchdog_stack_running or snap.lock_mode == "autonomous":
        return "SCANNING"
    if snap.watchdog_processing:
        return "PROCESSING"
    if not snap.watchdog_fcu:
        return "WAITING_FCU"

    return "IDLE"


class _PatternThread(threading.Thread):
    def __init__(self, leds: dict[str, object], pattern: _Pattern) -> None:
        super().__init__(daemon=True)
        self._leds = leds
        self._pattern = pattern
        self._stop_event = threading.Event()

    def _all_off(self) -> None:
        for led in self._leds.values():
            try:
                led.off()
            except Exception:
                pass

    def _set_channels(self, channels: tuple[str, ...]) -> None:
        self._all_off()
        for ch in channels:
            self._leds[ch].on()

    def run(self) -> None:
        if self._pattern.mode == "blink":
            half_cycle = self._pattern.half_cycle or 0.2
            while not self._stop_event.is_set():
                self._set_channels(self._pattern.channels)
                if self._stop_event.wait(half_cycle):
                    break
                self._all_off()
                if self._stop_event.wait(half_cycle):
                    break
        elif self._pattern.mode == "sequence":
            while not self._stop_event.is_set():
                for channels, duration in self._pattern.frames:
                    self._set_channels(channels)
                    if self._stop_event.wait(duration):
                        self._all_off()
                        return
        self._all_off()

    def stop(self) -> None:
        self._stop_event.set()


class LEDDriver:
    def __init__(self) -> None:
        from gpiozero import LED
        self._leds: dict[str, object] = {name: LED(bcm) for name, bcm in _BCM_PINS.items()}
        self._thread: Optional[_PatternThread] = None
        self._current = ""
        self._all_off()
        logger.info(
            "GPIO initialised — BCM G:%d Y:%d R:%d",
            _BCM_PINS["green"], _BCM_PINS["yellow"], _BCM_PINS["red"],
        )

    def apply(self, state_name: str) -> None:
        if state_name == self._current:
            return
        if state_name not in _PATTERNS:
            logger.warning("Unknown state '%s' — treating as OFF", state_name)
            state_name = "OFF"

        pattern = _PATTERNS[state_name]
        self._stop_worker()
        self._all_off()

        if pattern.mode == "solid":
            for ch in pattern.channels:
                self._leds[ch].on()
        elif pattern.mode in ("blink", "sequence"):
            self._thread = _PatternThread(self._leds, pattern)
            self._thread.start()

        self._current = state_name
        logger.info("State → %s", state_name)

    def cleanup(self) -> None:
        self._stop_worker()
        self._all_off()
        for led in self._leds.values():
            try:
                led.close()
            except Exception:
                pass
        self._leds.clear()
        logger.info("GPIO released.")

    def _stop_worker(self) -> None:
        if self._thread is not None:
            self._thread.stop()
            self._thread.join(timeout=1.0)
            self._thread = None

    def _all_off(self) -> None:
        for led in self._leds.values():
            try:
                led.off()
            except Exception:
                pass


def main() -> int:
    driver = LEDDriver()
    try:
        while True:
            snap = _read_snapshot()
            state = _derive_state(snap, time.time())
            driver.apply(state)
            time.sleep(1.0 / POLL_HZ)
    except KeyboardInterrupt:
        logger.info("Interrupted.")
        return 0
    finally:
        driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
