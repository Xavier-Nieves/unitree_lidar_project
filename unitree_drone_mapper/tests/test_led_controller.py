#!/usr/bin/env python3
"""
tests/test_led_controller.py — Standalone GPIO LED channel verification test.

PURPOSE
-------
Validates physical wiring of the three-channel LED indicator before enabling
it in production.  Run this directly on the Pi after GPIO wiring is complete
and before setting LED_ENABLED = True in led_controller.py.

This is a STANDALONE TEST SCRIPT — it reimplements a minimal version of the
GPIO control logic so it can be run without importing any project modules.
Isolation ensures a bug in led_controller.py does not mask a wiring fault.

WATCHDOG INTERACTION
--------------------
The drone-watchdog systemd service owns the GPIO pins while it is running.
lgpio does not allow two processes to claim the same pin simultaneously —
attempting to do so raises: lgpio.error: 'GPIO busy'

This script handles that automatically:
  1. At startup it detects whether drone-watchdog is active.
  2. If active, it stops the service (requires sudo — see USAGE below).
  3. It runs all requested tests.
  4. On exit (normal or KeyboardInterrupt) it restarts the service,
     returning the system to its pre-test state.

If the watchdog is not running when the script starts, it is left stopped
on exit — the script does not start a service that was not already running.

COMPATIBILITY NOTE
------------------
This script uses gpiozero with the lgpio backend.
RPi.GPIO does NOT support the Raspberry Pi 5 (RP1 GPIO southbridge).
lgpio communicates directly with /dev/gpiochip* and requires no daemon.
Reference: https://gpiozero.readthedocs.io/en/latest/api_pins.html

PHYSICAL PIN MAP (verify against your wiring before running)
------------------------------------------------------------
  GND    →  Pin 34  (common ground)
  Green  →  BCM 25
  Yellow →  BCM  8
  Red    →  BCM  7

BCM numbers are used exclusively by gpiozero.

USAGE
-----
  # Must be run with sudo so the script can stop/start the watchdog service.
  sudo python3 tests/test_led_controller.py

  # Single channel (useful for tracing one wire at a time)
  sudo python3 tests/test_led_controller.py --channel green
  sudo python3 tests/test_led_controller.py --channel yellow
  sudo python3 tests/test_led_controller.py --channel red

  # Step through every LEDState pattern
  sudo python3 tests/test_led_controller.py --states

  # Skip watchdog management (use only if watchdog is already stopped)
  python3 tests/test_led_controller.py --no-watchdog-mgmt

REQUIREMENTS
------------
  pip install gpiozero lgpio   (inside the dronepi conda env)
  GPIOZERO_PIN_FACTORY=lgpio   (set automatically by this script)

  No ROS 2 sourcing required.
"""

import argparse
import os
import subprocess
import sys
import time

# ---------------------------------------------------------------------------
# Force lgpio backend before any gpiozero import.
# This must be set before the first gpiozero import or the env var is ignored.
# ---------------------------------------------------------------------------
os.environ.setdefault("GPIOZERO_PIN_FACTORY", "lgpio")

# ---------------------------------------------------------------------------
# BCM pin map — must stay in sync with led_controller.py
# ---------------------------------------------------------------------------
_BCM_PINS: dict[str, int] = {
    "green":  25,
    "yellow": 8,
    "red":    7,
}

_BLINK_ON  = 0.33   # seconds LED HIGH per blink half-cycle
_BLINK_OFF = 0.33   # seconds LED LOW  per blink half-cycle

_WATCHDOG_SERVICE = "drone-watchdog"


# ---------------------------------------------------------------------------
# Watchdog service management
# ---------------------------------------------------------------------------

def _watchdog_is_active() -> bool:
    """
    Return True if the drone-watchdog systemd service is currently running.

    Uses 'systemctl is-active' which exits 0 for active, non-zero otherwise.
    Does not require sudo.
    """
    result = subprocess.run(
        ["systemctl", "is-active", "--quiet", _WATCHDOG_SERVICE],
        check=False,
    )
    return result.returncode == 0


def _stop_watchdog() -> bool:
    """
    Stop the drone-watchdog service and wait for it to fully release GPIO pins.

    Returns True if the stop succeeded, False if it failed (e.g. no sudo).

    Why poll instead of fixed sleep?
    lgpio holds the gpiochip file descriptor open until the process exits.
    systemd sends SIGTERM then waits up to TimeoutStopSec (15s in the unit
    file) before sending SIGKILL. Polling lets the test proceed as soon as
    the pins are actually free rather than waiting a fixed worst-case delay.
    """
    print(f"  [WATCHDOG] Stopping {_WATCHDOG_SERVICE}...")
    result = subprocess.run(
        ["sudo", "systemctl", "stop", _WATCHDOG_SERVICE],
        check=False,
    )
    if result.returncode != 0:
        print(f"  [WATCHDOG] ERROR: Could not stop service. Run this script with sudo.")
        return False

    # Poll until inactive — max 20 s (watchdog TimeoutStopSec is 15s)
    for _ in range(40):
        if not _watchdog_is_active():
            print(f"  [WATCHDOG] Service stopped. GPIO pins released.")
            return True
        time.sleep(0.5)

    print(f"  [WATCHDOG] WARNING: Service did not stop within 20s.")
    return False


def _start_watchdog() -> None:
    """
    Restart the drone-watchdog service after testing completes.

    Called unconditionally from the finally block in main() so the watchdog
    is always restored even if a test raises an unexpected exception.
    """
    print(f"\n  [WATCHDOG] Restarting {_WATCHDOG_SERVICE}...")
    result = subprocess.run(
        ["sudo", "systemctl", "start", _WATCHDOG_SERVICE],
        check=False,
    )
    if result.returncode == 0:
        print(f"  [WATCHDOG] Service restarted successfully.")
    else:
        print(f"  [WATCHDOG] WARNING: Could not restart service.")
        print(f"             Start manually: sudo systemctl start {_WATCHDOG_SERVICE}")


# ---------------------------------------------------------------------------
# GPIO helpers
# ---------------------------------------------------------------------------

def _require_gpiozero() -> None:
    """
    Confirm gpiozero and lgpio are importable; exit with a diagnostic if not.

    Both must be installed inside the active Python environment.
    Install with: pip install gpiozero lgpio
    """
    try:
        import gpiozero  # noqa: F401
    except ImportError:
        print("[ERROR] gpiozero not found. Install: pip install gpiozero lgpio")
        sys.exit(1)
    try:
        import lgpio  # noqa: F401
    except ImportError:
        print("[ERROR] lgpio not found. Install: pip install lgpio")
        sys.exit(1)


def _make_leds() -> dict:
    """
    Instantiate one gpiozero.LED object per channel.

    gpiozero.LED exposes:
      .on()    — drive HIGH
      .off()   — drive LOW
      .close() — release the pin (equivalent to GPIO.cleanup)

    A plain dict is returned to keep this standalone script free of class
    definitions while still providing named channel access.
    """
    from gpiozero import LED
    return {name: LED(bcm) for name, bcm in _BCM_PINS.items()}


def _all_off(leds: dict) -> None:
    """Drive all channels LOW."""
    for led in leds.values():
        led.off()


def _blink_blocking(led, cycles: int = 3) -> None:
    """
    Blocking blink implementation.

    gpiozero's built-in .blink() is non-blocking (spawns a background thread).
    For a sequential verification script, blocking behaviour is required so
    each step completes and prints its status line before the next begins.
    """
    for _ in range(cycles):
        led.on()
        time.sleep(_BLINK_ON)
        led.off()
        time.sleep(_BLINK_OFF)


def _solid(led, duration: float = 1.5) -> None:
    """Drive a single channel HIGH for `duration` seconds, then LOW."""
    led.on()
    time.sleep(duration)
    led.off()


def _close_leds(leds: dict) -> None:
    """
    Release all GPIO pins.

    LED.close() is equivalent to RPi.GPIO.cleanup(pin).
    Called explicitly because this script does not use context managers —
    test functions need independent lifecycles to remain standalone.
    """
    for led in leds.values():
        led.close()


# ---------------------------------------------------------------------------
# Test routines
# ---------------------------------------------------------------------------

def test_single_channel(channel: str) -> None:
    """
    Blink and hold a single channel for visual wire tracing.

    Activates only the requested channel so the tester can trace one wire
    without visual interference from the other channels.
    """
    if channel not in _BCM_PINS:
        print(f"[ERROR] Unknown channel '{channel}'. Choose: {list(_BCM_PINS)}")
        sys.exit(1)

    bcm = _BCM_PINS[channel]
    print(f"\n── Single channel test: {channel.upper()} (BCM {bcm}) ──")
    print(f"   Blink × 3, then solid 2 s, then off.")

    leds = _make_leds()
    _blink_blocking(leds[channel], cycles=3)
    time.sleep(0.2)
    _solid(leds[channel], duration=2.0)
    _all_off(leds)
    _close_leds(leds)
    print("   Done.")


def test_all_channels() -> None:
    """
    Step through each channel individually to verify wiring.

    Each channel is driven solid then blinked before moving to the next.
    If a channel does not light, the BCM pin assignment in _BCM_PINS or the
    physical wire at the corresponding header pin should be inspected.

    BCM values are read from _BCM_PINS rather than hardcoded here so this
    function stays in sync with the pin map automatically.
    """
    leds = _make_leds()

    tests = [
        ("green",  "Idle / scanning"),
        ("yellow", "Processing / FCU wait"),
        ("red",    "Error / warning"),
    ]

    print("\n── Individual channel test ──")
    for channel, meaning in tests:
        bcm = _BCM_PINS[channel]
        print(f"\n  [{channel.upper():6s}]  BCM {bcm}  —  {meaning}")
        print(f"           Solid 1 s...")
        _solid(leds[channel], duration=1.0)
        time.sleep(0.2)
        print(f"           Blink × 3...")
        _blink_blocking(leds[channel], cycles=3)
        time.sleep(0.3)

    _all_off(leds)
    _close_leds(leds)


def test_named_states() -> None:
    """
    Demonstrate every LEDState visual pattern without importing the module.

    The state table below must mirror the LEDState enum in led_controller.py.
    This function confirms the physical output matches each enum value's
    documented behaviour before that module is enabled in production.
    """
    leds = _make_leds()

    # (state_name, channel_or_None, do_blink, hold_seconds, description)
    states = [
        ("OFF",         None,      False, 0.8,  "All off"),
        ("IDLE",        "green",   False, 1.5,  "Green solid"),
        ("SCANNING",    "green",   True,  2.5,  "Green blink"),
        ("PROCESSING",  "yellow",  False, 1.5,  "Yellow solid"),
        ("WAITING_FCU", "yellow",  True,  2.5,  "Yellow blink"),
        ("ERROR",       "red",     False, 1.5,  "Red solid"),
        ("WARNING",     "red",     True,  2.5,  "Red blink"),
    ]

    print("\n── Named state sequence test ──")
    print("   Reproduces every state defined in LEDState enum.\n")

    for state_name, channel, do_blink, duration, desc in states:
        print(f"  {state_name:<14}  {desc}")
        _all_off(leds)

        if channel is None:
            time.sleep(duration)
            continue

        if do_blink:
            cycles = max(1, int(duration / (_BLINK_ON + _BLINK_OFF)))
            _blink_blocking(leds[channel], cycles=cycles)
        else:
            _solid(leds[channel], duration=duration)

        time.sleep(0.2)

    _all_off(leds)
    _close_leds(leds)


def test_sequence_all() -> None:
    """Run the full channel sweep followed by the full named-state sequence."""
    test_all_channels()
    time.sleep(0.5)
    test_named_states()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Standalone GPIO LED wiring verification for DronePi status LEDs.\n"
            "Automatically stops drone-watchdog before testing and restarts it\n"
            "after, so GPIO pins are not busy. Run with sudo."
        )
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--channel",
        choices=["green", "yellow", "red"],
        help="Test a single LED channel only.",
    )
    group.add_argument(
        "--states",
        action="store_true",
        help="Step through every LEDState pattern instead of the channel test.",
    )
    parser.add_argument(
        "--no-watchdog-mgmt",
        action="store_true",
        help="Skip automatic watchdog stop/start. Use only if watchdog is already stopped.",
    )
    args = parser.parse_args()

    _require_gpiozero()

    print("=" * 52)
    print("  DronePi LED Controller — Wiring Verification")
    print("  Backend : gpiozero + lgpio (Pi 5 / Ubuntu 24)")
    print(f"  Pins    : G=BCM{_BCM_PINS['green']}  "
          f"Y=BCM{_BCM_PINS['yellow']}  "
          f"R=BCM{_BCM_PINS['red']}")
    print("=" * 52)

    # ── Watchdog management ──────────────────────────────────────────────────
    watchdog_was_active = False
    if not args.no_watchdog_mgmt:
        watchdog_was_active = _watchdog_is_active()
        if watchdog_was_active:
            ok = _stop_watchdog()
            if not ok:
                print("\n[ERROR] Could not stop drone-watchdog.")
                print(f"        Re-run with sudo: sudo {' '.join(sys.argv)}")
                sys.exit(1)
        else:
            print("  [WATCHDOG] Service not running — no action needed.")
    else:
        print("  [WATCHDOG] Management skipped (--no-watchdog-mgmt).")

    # ── Run tests ────────────────────────────────────────────────────────────
    try:
        if args.channel:
            test_single_channel(args.channel)
        elif args.states:
            test_named_states()
            print("\n  All states complete.")
        else:
            test_sequence_all()
            print("\n  All tests complete. If all LEDs lit as described,")
            print("  set LED_ENABLED = True in watchdog_core/led_controller.py")

    except KeyboardInterrupt:
        print("\n  Interrupted — cleaning up pins.")
        try:
            leds = _make_leds()
            _all_off(leds)
            _close_leds(leds)
        except Exception:
            pass

    finally:
        # Runs on normal exit, KeyboardInterrupt, and unhandled exceptions —
        # guaranteeing the watchdog is always restored after testing.
        if not args.no_watchdog_mgmt and watchdog_was_active:
            _start_watchdog()

    sys.exit(0)


if __name__ == "__main__":
    main()
