#!/usr/bin/env python3
"""drone_watchdog.py — Flight stack supervisor with RC toggle and buzzer feedback.

Runs as the drone-watchdog systemd service. Works in coordination with
main.py (dronepi-main service) via a shared lock file at
/tmp/dronepi_mission.lock.

Lock File Behaviour
-------------------
  absent        — watchdog operates normally (arm + OFFBOARD detection)
  manual_scan   — main.py has committed MODE 2; watchdog starts the flight
                  stack (Point-LIO + SLAM bridge + bag recorder) and handles
                  disarm + post-flight trigger
  autonomous    — main.py owns the full stack for MODE 3; watchdog skips
                  launch entirely but still handles disarm + post-flight
  bench_scan    — test script owns the stack; watchdog yields entirely

RC Toggle (Manual Scan Override)
---------------------------------
  Momentary button on CH6 (index 5) starts or stops the flight stack when
  no lock file is present. Has no effect once a mode is locked.

Buzzer Feedback (QBASIC Format = 1)
------------------------------------
  Rising tone          — stack starting
  Falling tone         — stack stopping
  Single beep          — button acknowledged / FCU connected
  Mid pulse (5s repeat)— post-processing running
  Two-note up          — post-processing finished
  Staccato x3          — error

Module Layout
-------------
  watchdog_core/
    buzzer.py         — tune definitions + PostflightBeeper periodic thread
    mavros_reader.py  — ROS 2 state / RC / buzzer interface
    flight_stack.py   — process helpers, command builders, FlightStack class
    postflight.py     — PostflightMonitor (subprocess, log pipe, beeper)
    logging_utils.py  — shared log() function

Deployment
----------
  sudo systemctl restart drone-watchdog
  sudo journalctl -u drone-watchdog -f
"""

import json
import os
import sys
import time
from pathlib import Path

from watchdog_core.logging_utils import log
from watchdog_core.mavros_reader  import MavrosReader
from watchdog_core.flight_stack   import FlightStack, ROSBAG_DIR
from watchdog_core.postflight     import PostflightMonitor

# LED IPC — watchdog writes system status; led_service.py observes and drives GPIO.
# Importing pin/state constants from led_controller for reference only.
from watchdog_core.led_controller import LED_STATES   # noqa: F401 — validation reference

_WATCHDOG_STATUS_FILE     = "/tmp/watchdog_status.json"
_WATCHDOG_STATUS_FILE_TMP = _WATCHDOG_STATUS_FILE + ".tmp"


def _write_status(
    fcu: bool = False,
    armed: bool = False,
    processing: bool = False,
) -> None:
    """
    Write the watchdog heartbeat and system status to /tmp/watchdog_status.json.

    Called on every poll cycle so led_service.py receives a fresh timestamp.
    If the timestamp goes stale beyond WATCHDOG_DEAD_S (5s), led_service.py
    transitions to WATCHDOG_DEAD — visually distinct from any flight state.

    Parameters
    ----------
    fcu        : FCU connected via MAVROS
    armed      : Vehicle armed state
    processing : Post-flight processing currently running
    """
    try:
        import json as _json
        payload = _json.dumps({
            "ts":         time.time(),
            "fcu":        fcu,
            "armed":      armed,
            "processing": processing,
        })
        with open(_WATCHDOG_STATUS_FILE_TMP, "w") as f:
            f.write(payload)
        os.replace(_WATCHDOG_STATUS_FILE_TMP, _WATCHDOG_STATUS_FILE)
    except Exception as exc:
        log(f"[STATUS] Watchdog status write failed: {exc}")


def _clear_status() -> None:
    """Remove status file on clean shutdown so led_service sees a clean exit."""
    try:
        Path(_WATCHDOG_STATUS_FILE).unlink(missing_ok=True)
        Path(_WATCHDOG_STATUS_FILE_TMP).unlink(missing_ok=True)
    except Exception:
        pass

# ── Configuration ─────────────────────────────────────────────────────────────

from watchdog_core.mavros_reader import RC_TOGGLE_CHANNEL, ENABLE_BUZZER

ENABLE_RC_TOGGLE = True
POLL_HZ          = 10
MONITOR_HZ       = 10
MAVROS_WAIT_S    = 15

MISSION_LOCK = Path("/tmp/dronepi_mission.lock")
HAILO_LOCK   = Path("/tmp/dronepi_hailo.lock")

# SLAM chain verification — mirrors main.py constants
# Watchdog uses these when it owns the stack (MODE 2 / manual_scan)
ROS_SETUP  = "/opt/ros/jazzy/setup.bash"
WS_SETUP   = str(Path.home() / "unitree_lidar_project/RPI5/ros2_ws/install/setup.bash")
SLAM_MIN_HZ           = 5.0
BRIDGE_MIN_HZ         = 8.0
SLAM_TOPIC_TIMEOUT_S  = 30.0
BRIDGE_TOPIC_TIMEOUT_S = 15.0


# ── Lock File ─────────────────────────────────────────────────────────────────

def read_lock_mode() -> str:
    """Return the lock mode string, or empty string if no lock is present."""
    if not MISSION_LOCK.exists():
        return ""
    try:
        data = json.loads(MISSION_LOCK.read_text())
        return data.get("mode", "")
    except Exception:
        return ""


# ── Startup ───────────────────────────────────────────────────────────────────

def _wait_for_mavros(reader: MavrosReader) -> None:
    log(f"Waiting for MAVROS connection (up to {MAVROS_WAIT_S}s)...")
    deadline = time.time() + MAVROS_WAIT_S
    while time.time() < deadline:
        if reader.connected:
            log(f"FCU connected.  Mode: {reader.mode}  Armed: {reader.armed}")
            time.sleep(0.5)
            reader.beep_ack()
            return
        time.sleep(1.0)
    log("[WARN] FCU not connected — continuing anyway")


# ── Main Loop ─────────────────────────────────────────────────────────────────

def main() -> None:
    log("=" * 60)
    log("DronePi Flight Stack Watchdog")
    log(f"  RC Toggle: CH{RC_TOGGLE_CHANNEL + 1}")
    log(f"  Buzzer:    {'ENABLED (QBASIC)' if ENABLE_BUZZER else 'DISABLED'}")
    log("=" * 60)

    ROSBAG_DIR.mkdir(parents=True, exist_ok=True)

    # ── LED controller ────────────────────────────────────────────────────────
    _write_status(fcu=False, armed=False, processing=False)

    # ── Initialise components ─────────────────────────────────────────────────
    try:
        reader = MavrosReader()
    except Exception as exc:
        log(f"[FAIL] ROS 2 init failed: {exc}")
        _clear_status()
        sys.exit(1)

    _wait_for_mavros(reader)

    # Status will be updated in the main loop

    postflight = PostflightMonitor(reader)
    stack      = FlightStack(reader, postflight_fn=postflight.trigger)

    # ── Main poll loop ────────────────────────────────────────────────────────
    try:
        while True:
            lock_mode = read_lock_mode()

            # ── BENCH TEST — test script owns the stack ───────────────────────
            if lock_mode == "bench_scan":
                if stack.is_running:
                    log("Lock mode switched to bench_scan — stopping watchdog stack")
                    stack.stop()
                log("[WATCHDOG] bench_scan lock active — yielding to test script")
                _write_status(fcu=reader.connected, armed=reader.armed)
                time.sleep(1.0 / POLL_HZ)
                continue

            # ── MODE 3 (autonomous) — main.py owns the stack ─────────────────
            if lock_mode == "autonomous":
                if stack.is_running:
                    log("Lock mode switched to autonomous — stopping watchdog stack")
                    stack.stop()
                log("[WATCHDOG] autonomous lock active — yielding to main.py")
                _write_status(fcu=reader.connected, armed=reader.armed)
                time.sleep(1.0 / POLL_HZ)
                continue

            # ── MODE 2 (manual_scan) — watchdog owns the stack ───────────────
            if lock_mode == "manual_scan":
                if not stack.is_running:
                    log("[WATCHDOG] manual_scan lock detected — starting stack")
                    ok = stack.start("MANUAL_LOCK")
                    if not ok:
                        log("[WATCHDOG] Stack startup failed — clearing lock, returning to IDLE")
                        if MISSION_LOCK.exists():
                            MISSION_LOCK.unlink()
                        time.sleep(1.0 / POLL_HZ)
                        continue
                stack.check_health()
                _write_status(fcu=reader.connected, armed=reader.armed)

                # Wait for disarm, then trigger post-flight and clear lock
                if not reader.armed:
                    log("[WATCHDOG] Disarmed — stopping stack")
                    stack.stop()
                    if MISSION_LOCK.exists():
                        MISSION_LOCK.unlink()
                        log("Lock cleared")

                time.sleep(1.0 / MONITOR_HZ)
                continue

            # ── No lock — standard RC toggle operation ────────────────────────
            if not stack.is_running:
                if ENABLE_RC_TOGGLE and reader.check_toggle_pressed():
                    log("RC button pressed — starting stack")
                    reader.beep_ack()
                    ok = stack.start("MANUAL_RC")

                else:
                    ch_val = reader.get_rc_channel(RC_TOGGLE_CHANNEL)
                    log(
                        f"[WAITING] armed={reader.armed}  "
                        f"mode={reader.mode or '?'}  "
                        f"CH{RC_TOGGLE_CHANNEL + 1}={ch_val}"
                    )
                    _write_status(fcu=reader.connected, armed=reader.armed)
                    time.sleep(1.0 / POLL_HZ)
            else:
                stack.check_health()
                _write_status(fcu=reader.connected, armed=reader.armed)
                if ENABLE_RC_TOGGLE and reader.check_toggle_pressed():
                    log("RC button pressed — stopping stack")
                    reader.beep_ack()
                    stack.stop()
                    _write_status(fcu=reader.connected, armed=reader.armed, processing=True)
                else:
                    time.sleep(1.0 / MONITOR_HZ)

    except KeyboardInterrupt:
        log("Interrupted — shutting down")
    finally:
        if stack.is_running:
            stack.stop()
        reader.shutdown()
        _clear_status()
        log("Watchdog stopped.")
        

if __name__ == "__main__":
    main()
