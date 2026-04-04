#!/usr/bin/env python3
"""drone_watchdog.py — watchdog with matched buzzer + LED semantic states."""

from __future__ import annotations

import json
import signal
import threading
import os
import sys
import time
from pathlib import Path

from watchdog_core.buzzer import (
    FaultAlarmManager,
    ScanBeeper,
    TUNE_SCAN_FINISHED,
    TUNE_SCAN_READY,
    TUNE_SCAN_START,
    TUNE_SYSTEM_START,
)
from watchdog_core.flight_stack import FlightStack, ROSBAG_DIR
from watchdog_core.logging_utils import log
from watchdog_core.mavros_reader import ENABLE_BUZZER, RC_TOGGLE_CHANNEL, MavrosReader
from watchdog_core.postflight import PostflightMonitor
from watchdog_core.led_controller import LED_STATES  # noqa: F401

_shutdown = threading.Event()

def _handle_shutdown(signum, frame):
    log(f"Shutdown signal received: {signum}")
    _shutdown.set()

_WATCHDOG_STATUS_FILE = "/tmp/watchdog_status.json"
_WATCHDOG_STATUS_FILE_TMP = _WATCHDOG_STATUS_FILE + ".tmp"

ENABLE_RC_TOGGLE = True
POLL_HZ = 10
MONITOR_HZ = 10
MAVROS_WAIT_S = 15

MISSION_LOCK = Path("/tmp/dronepi_mission.lock")


def _write_status(
    fcu: bool = False,
    armed: bool = False,
    processing: bool = False,
    stack_running: bool = False,
    led_state: str = "",
    led_until: float = 0.0,
    warning: bool = False,
    error: bool = False,
    critical: bool = False,
    system_failure: bool = False,
) -> None:
    try:
        payload = json.dumps({
            "ts": time.time(),
            "fcu": fcu,
            "armed": armed,
            "processing": processing,
            "stack_running": stack_running,
            "led_state": led_state,
            "led_until": led_until,
            "warning": warning,
            "error": error,
            "critical": critical,
            "system_failure": system_failure,
        })
        with open(_WATCHDOG_STATUS_FILE_TMP, "w") as f:
            f.write(payload)
        os.replace(_WATCHDOG_STATUS_FILE_TMP, _WATCHDOG_STATUS_FILE)
    except Exception as exc:
        log(f"[STATUS] Watchdog status write failed: {exc}")


def _clear_status() -> None:
    try:
        Path(_WATCHDOG_STATUS_FILE).unlink(missing_ok=True)
        Path(_WATCHDOG_STATUS_FILE_TMP).unlink(missing_ok=True)
    except Exception:
        pass


def read_lock_mode() -> str:
    if not MISSION_LOCK.exists():
        return ""
    try:
        data = json.loads(MISSION_LOCK.read_text())
        return data.get("mode", "")
    except Exception:
        return ""


def _play_tune(reader: MavrosReader, tune: str, label: str = "") -> None:
    if not ENABLE_BUZZER:
        return
    try:
        reader.play_tune(tune)
        if label:
            log(f"[BUZZER] {label}")
    except Exception as exc:
        log(f"[BUZZER] Failed to play {label or 'tune'}: {exc}")


def _wait_for_mavros(reader: MavrosReader) -> bool:
    log(f"Waiting for MAVROS connection (up to {MAVROS_WAIT_S}s)...")
    deadline = time.time() + MAVROS_WAIT_S
    while time.time() < deadline:
        if reader.connected:
            log(f"FCU connected. Mode: {reader.mode} Armed: {reader.armed}")
            return True
        _write_status(fcu=False, armed=reader.armed, led_state="WAITING_FCU", led_until=time.time() + 0.2)
        time.sleep(1.0)
    log("[WARN] FCU not connected — continuing anyway")
    return False


def main() -> None:
    log("=" * 60)
    log("DronePi Flight Stack Watchdog (LED-synced)")
    log(f"  RC Toggle: CH{RC_TOGGLE_CHANNEL + 1}")
    log(f"  Buzzer:    {'ENABLED (QBASIC)' if ENABLE_BUZZER else 'DISABLED'}")
    log("=" * 60)

    ROSBAG_DIR.mkdir(parents=True, exist_ok=True)

    try:
        reader = MavrosReader()
    except Exception as exc:
        log(f"[FAIL] ROS 2 init failed: {exc}")
        _clear_status()
        sys.exit(1)

    scan_beeper = ScanBeeper(lambda tune: _play_tune(reader, tune, "scan_active"))
    alarms = FaultAlarmManager(
        play_tune_fn=lambda tune: _play_tune(reader, tune, "fault_alarm"),
        is_armed_fn=lambda: bool(reader.armed),
        startup_failure_repeats=True,
    )

    fault_warning = False
    fault_error = False
    fault_critical = False
    fault_system_failure = False
    led_state = "SYSTEM_START"
    led_until = time.time() + 2.2
    _write_status(fcu=False, armed=False, led_state=led_state, led_until=led_until)
    _play_tune(reader, TUNE_SYSTEM_START, "system_start")

    fcu_ok = _wait_for_mavros(reader)
    if fcu_ok:
        _play_tune(reader, TUNE_SCAN_READY, "scan_ready")
        led_state = "SCAN_READY"
        led_until = time.time() + 1.0
    else:
        fault_warning = True
        alarms.set_warning(True)

    postflight = PostflightMonitor(reader)
    stack = FlightStack(reader, postflight_fn=postflight.trigger)

    try:
        signal.signal(signal.SIGTERM, _handle_shutdown)
        signal.signal(signal.SIGINT, _handle_shutdown)
        while not _shutdown.is_set():
            lock_mode = read_lock_mode()

            fault_critical = bool(stack.is_running and reader.armed and not reader.connected)
            fault_error = False
            fault_system_failure = False

            alarms.set_error(fault_error)
            alarms.set_critical(fault_critical)
            alarms.set_system_failure(fault_system_failure)

            if lock_mode == "bench_scan":
                if stack.is_running:
                    scan_beeper.stop()
                    stack.stop()
                    _play_tune(reader, TUNE_SCAN_FINISHED, "scan_finished")
                    led_state = "SCAN_FINISHED"
                    led_until = time.time() + 1.5
                _write_status(
                    fcu=reader.connected,
                    armed=reader.armed,
                    stack_running=False,
                    processing=postflight.is_active,
                    led_state=led_state,
                    led_until=led_until,
                    warning=fault_warning,
                    error=fault_error,
                    critical=fault_critical,
                    system_failure=fault_system_failure,
                )
                time.sleep(1.0 / POLL_HZ)
                continue

            if lock_mode == "autonomous":
                if stack.is_running:
                    scan_beeper.stop()
                    stack.stop()
                    _play_tune(reader, TUNE_SCAN_FINISHED, "scan_finished")
                    led_state = "SCAN_FINISHED"
                    led_until = time.time() + 1.5
                _write_status(
                    fcu=reader.connected,
                    armed=reader.armed,
                    stack_running=False,
                    processing=postflight.is_active,
                    led_state=led_state,
                    led_until=led_until,
                    warning=fault_warning,
                    error=fault_error,
                    critical=fault_critical,
                    system_failure=fault_system_failure,
                )
                time.sleep(1.0 / POLL_HZ)
                continue

            if lock_mode == "manual_scan":
                if not stack.is_running:
                    log("[WATCHDOG] manual_scan lock detected — starting stack")
                    ok = stack.start("MANUAL_LOCK")
                    if not ok:
                        fault_error = True
                        alarms.set_error(True)
                        if MISSION_LOCK.exists():
                            MISSION_LOCK.unlink()
                        _write_status(
                            fcu=reader.connected,
                            armed=reader.armed,
                            processing=postflight.is_active,
                            stack_running=False,
                            warning=fault_warning,
                            error=True,
                            critical=fault_critical,
                            system_failure=fault_system_failure,
                        )
                        time.sleep(1.0 / POLL_HZ)
                        continue

                    _play_tune(reader, TUNE_SCAN_START, "scan_start")
                    led_state = "SCAN_START"
                    led_until = time.time() + 1.2
                    scan_beeper.start()

                stack.check_health()

                if not reader.armed:
                    log("[WATCHDOG] Disarmed — stopping stack")
                    scan_beeper.stop()
                    stack.stop()
                    _play_tune(reader, TUNE_SCAN_FINISHED, "scan_finished")
                    led_state = "SCAN_FINISHED"
                    led_until = time.time() + 1.5
                    if MISSION_LOCK.exists():
                        MISSION_LOCK.unlink()
                        log("Lock cleared")

                _write_status(
                    fcu=reader.connected,
                    armed=reader.armed,
                    processing=postflight.is_active,
                    stack_running=stack.is_running,
                    led_state=led_state,
                    led_until=led_until,
                    warning=fault_warning,
                    error=fault_error,
                    critical=fault_critical,
                    system_failure=fault_system_failure,
                )
                time.sleep(1.0 / MONITOR_HZ)
                continue

            if not stack.is_running:
                if ENABLE_RC_TOGGLE and reader.check_toggle_pressed():
                    log("RC button pressed — starting stack")
                    _play_tune(reader, TUNE_SCAN_READY, "button_ack")
                    led_state = "SCAN_READY"
                    led_until = time.time() + 0.8
                    ok = stack.start("MANUAL_RC")
                    if ok:
                        _play_tune(reader, TUNE_SCAN_START, "scan_start")
                        led_state = "SCAN_START"
                        led_until = time.time() + 1.2
                        scan_beeper.start()
                    else:
                        fault_error = True
                        alarms.set_error(True)
                else:
                    _write_status(
                        fcu=reader.connected,
                        armed=reader.armed,
                        processing=postflight.is_active,
                        stack_running=False,
                        led_state=led_state,
                        led_until=led_until,
                        warning=fault_warning,
                        error=fault_error,
                        critical=fault_critical,
                        system_failure=fault_system_failure,
                    )
                    time.sleep(1.0 / POLL_HZ)
            else:
                stack.check_health()
                if ENABLE_RC_TOGGLE and reader.check_toggle_pressed():
                    log("RC button pressed — stopping stack")
                    scan_beeper.stop()
                    stack.stop()
                    _play_tune(reader, TUNE_SCAN_FINISHED, "scan_finished")
                    led_state = "SCAN_FINISHED"
                    led_until = time.time() + 1.5

                _write_status(
                    fcu=reader.connected,
                    armed=reader.armed,
                    processing=postflight.is_active,
                    stack_running=stack.is_running,
                    led_state=led_state,
                    led_until=led_until,
                    warning=fault_warning,
                    error=fault_error,
                    critical=fault_critical,
                    system_failure=fault_system_failure,
                )
                time.sleep(1.0 / MONITOR_HZ)

    except Exception as exc:
        log(f"[FAIL] Watchdog crashed: {exc}")
        raise
    finally:
        scan_beeper.stop()
        alarms.shutdown()
        if stack.is_running:
            stack.stop()
        reader.shutdown()
        _clear_status()
        log("Watchdog stopped.")


if __name__ == "__main__":
    main()
