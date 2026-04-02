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

COMPATIBILITY NOTE
------------------
This script uses gpiozero instead of RPi.GPIO.
RPi.GPIO does NOT support the Raspberry Pi 5 (RP1 GPIO southbridge).
gpiozero automatically selects the lgpio backend on Pi 5, making it the
correct choice for Pi 4 and Pi 5 without any code changes between boards.

Reference:
  https://gpiozero.readthedocs.io/en/latest/api_output.html#gpiozero.LED
  https://github.com/raspberrypi/RPi.GPIO/issues  (Pi 5 breakage history)

PHYSICAL PIN MAP (verify against your wiring before running)
------------------------------------------------------------
  Pin 34  GND    (common ground)
  Pin 36  Green  LED   →  BCM GPIO 16
  Pin 38  Yellow LED   →  BCM GPIO 20
  Pin 40  Red    LED   →  BCM GPIO 21

gpiozero uses BCM numbering exclusively. The BOARD→BCM mapping above is
sourced from https://pinout.xyz (physical header, Pi 40-pin standard).

USAGE
-----
  # Full sequence: individual channels then all states
  python3 tests/test_led_controller.py

  # Single channel (useful for tracing one wire at a time)
  python3 tests/test_led_controller.py --channel green
  python3 tests/test_led_controller.py --channel yellow
  python3 tests/test_led_controller.py --channel red

  # Step through every LEDState pattern
  python3 tests/test_led_controller.py --states

REQUIREMENTS
------------
  gpiozero  — sudo apt install python3-gpiozero
  lgpio     — sudo apt install python3-lgpio   (Pi 5 backend, auto-selected)

  No conda env or ROS 2 sourcing required.
"""

import argparse
import sys
import time

# ---------------------------------------------------------------------------
# BCM pin map — must stay in sync with led_controller.py
#
# Why BCM and not BOARD?
#   gpiozero only accepts BCM numbers. BOARD pin 36 = BCM 16, etc.
#   Source: https://pinout.xyz
# ---------------------------------------------------------------------------
_BCM_PINS: dict[str, int] = {
    "green":  16,   # BOARD 36
    "yellow": 20,   # BOARD 38
    "red":    21,   # BOARD 40
}

_BLINK_ON  = 0.33   # seconds LED HIGH per blink half-cycle
_BLINK_OFF = 0.33   # seconds LED LOW  per blink half-cycle


# ---------------------------------------------------------------------------
# GPIO backend — gpiozero replaces RPi.GPIO for Pi 5 compatibility
# ---------------------------------------------------------------------------

def _require_gpiozero() -> None:
    """
    Confirm gpiozero is importable; exit with a diagnostic if not.

    gpiozero is pre-installed on Raspberry Pi OS (Bookworm and later).
    If missing, install with:  sudo apt install python3-gpiozero python3-lgpio
    """
    try:
        import gpiozero  # noqa: F401  — import check only
    except ImportError:
        print("[ERROR] gpiozero not found.")
        print("        Install with: sudo apt install python3-gpiozero python3-lgpio")
        print("        This test must run on the Raspberry Pi.")
        sys.exit(1)


def _make_leds() -> dict:
    """
    Instantiate one gpiozero.LED object per channel.

    gpiozero.LED wraps a single output pin and exposes:
      .on()    — drive HIGH
      .off()   — drive LOW
      .blink() — background thread blink (non-blocking)
      .close() — release the pin (equivalent to GPIO.cleanup)

    Returning a plain dict keeps this test script free of class definitions
    while still giving named access to each LED object.
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
    For a verification script we want the main thread to wait so that each
    test step completes before printing the next status line.  A simple loop
    achieves this without threading primitives.
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

    gpiozero.LED.close() is the equivalent of RPi.GPIO.cleanup(pin).
    It must be called explicitly here because this script does not use a
    context manager (with LED(...) as led:) — keeping the test functions
    independent of each other requires owning the lifecycle manually.
    """
    for led in leds.values():
        led.close()


# ---------------------------------------------------------------------------
# Test routines
# ---------------------------------------------------------------------------

def test_single_channel(channel: str) -> None:
    """
    Blink and hold a single channel for visual wire tracing.

    Useful when only one wire is suspect; avoids activating the other channels
    so the tester can focus on one LED without visual noise.
    """
    if channel not in _BCM_PINS:
        print(f"[ERROR] Unknown channel '{channel}'. Choose: {list(_BCM_PINS)}")
        sys.exit(1)

    bcm = _BCM_PINS[channel]
    print(f"\n── Single channel test: {channel.upper()} (BCM {bcm} / BOARD {bcm}) ──")
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

    Each LED is driven solid then blinked before moving to the next channel.
    If a channel does not light, the corresponding BCM→BOARD mapping or the
    physical wire at that header pin should be checked.
    """
    leds = _make_leds()

    tests = [
        ("green",  "Idle / scanning",        16, 36),
        ("yellow", "Processing / FCU wait",  20, 38),
        ("red",    "Error / warning",        21, 40),
    ]

    print("\n── Individual channel test ──")
    for channel, meaning, bcm, board in tests:
        print(f"\n  [{channel.upper():6s}]  BCM {bcm}  /  BOARD {board}  —  {meaning}")
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
    This function exists to confirm the physical output matches each enum
    value's documented behaviour before that module is enabled.
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
    """Run the full channel sweep followed by the full state sequence."""
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
            "Uses gpiozero (Pi 5 compatible). RPi.GPIO is NOT used."
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
    args = parser.parse_args()

    _require_gpiozero()

    print("=" * 52)
    print("  DronePi LED Controller — Wiring Verification")
    print("  Backend : gpiozero (lgpio on Pi 5)")
    print("  Pins    : GND=34  G=BCM16  Y=BCM20  R=BCM21")
    print("=" * 52)

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
        print("\n  Interrupted — cleaning up.")
        # Re-instantiate to guarantee all pins go LOW on exit.
        # This handles the case where KeyboardInterrupt fires inside a
        # test function before its local _close_leds() call is reached.
        try:
            leds = _make_leds()
            _all_off(leds)
            _close_leds(leds)
        except Exception:
            pass
        sys.exit(0)


if __name__ == "__main__":
    main()
