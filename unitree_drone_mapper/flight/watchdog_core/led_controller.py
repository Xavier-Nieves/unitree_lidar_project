#!/usr/bin/env python3
"""
watchdog_core/led_controller.py — LED system reference and constants.

ROLE IN THE NEW ARCHITECTURE
-----------------------------
This file no longer instantiates GPIO objects or drives hardware directly.
It is the single authoritative source for:

  1. BCM pin assignments (_BCM_GREEN, _BCM_YELLOW, _BCM_RED)
  2. The canonical list of LED state names (LED_STATES)
  3. Documentation of what each state means visually

The LED state machine, GPIO driver, blink threads, and hardware patterns
all live in flight/led_service.py. The IPC writers (set_led calls) live in
drone_watchdog.py and main.py.

WHY THIS SEPARATION
--------------------
  led_service.py    — process that owns GPIO, observes system, drives LEDs
  led_controller.py — shared constants imported by any module that needs
                      to know pin numbers or valid state name strings

ADDING A NEW LED STATE
-----------------------
  1. Add the state name string to LED_STATES below (documentation + validation).
  2. Add a _Pattern entry to _PATTERNS in flight/led_service.py.
  3. Add a priority rule to _derive_state() in flight/led_service.py.
  4. No changes needed in drone_watchdog.py or main.py unless the new state
     requires a new status field in the watchdog/main status files.

PHYSICAL PIN MAP
----------------
  GND    →  Pin 34  (common ground — not software-controlled)
  Green  →  BCM 25  (BOARD pin 22)
  Yellow →  BCM  8  (BOARD pin 24)
  Red    →  BCM  7  (BOARD pin 26)

  All three values below must stay in sync with:
    flight/led_service.py     _BCM_PINS
    tests/test_led_controller.py  _BCM_PINS
"""

# ── BCM Pin Constants ─────────────────────────────────────────────────────────
# Import these in any module that needs pin numbers rather than hardcoding them.

BCM_GREEN:  int = 25   # BOARD pin 22
BCM_YELLOW: int = 8    # BOARD pin 24
BCM_RED:    int = 7    # BOARD pin 26

# ── Canonical State Names ─────────────────────────────────────────────────────
# This tuple is the single source of truth for valid LED state name strings.
# led_service.py _PATTERNS must have an entry for every name listed here.
# drone_watchdog.py and main.py write these strings to the status files.

LED_STATES: tuple[str, ...] = (
    # ── System states ─────────────────────────────────────────────────────────
    "OFF",           # All LEDs off — system initialising or powered down
    "IDLE",          # Green solid — healthy, no active task
    "SCANNING",      # Green blink 1.5 Hz — LiDAR recording in progress
    "PROCESSING",    # Yellow solid — post-flight processing running
    "WAITING_FCU",   # Yellow blink 1.5 Hz — waiting for MAVROS FCU connection
    "ERROR",         # Red solid — fatal error, watchdog entered error state
    "WARNING",       # Red blink 1.5 Hz — non-fatal warning (e.g. low disk)
    # ── Hailo states ──────────────────────────────────────────────────────────
    "HAILO_ACTIVE",  # Green + Yellow solid — Hailo augmenting EKF2
    "HAILO_DEGRADED",# Yellow blink 3 Hz — flow fallback, consecutive rejections
    "HAILO_FAILED",  # Red blink 3 Hz — Hailo hard-failed mid-flight
    # ── Fault states ──────────────────────────────────────────────────────────
    "WATCHDOG_DEAD", # Red blink 0.8 Hz — watchdog heartbeat stale
    "MAIN_DEAD",     # Red blink 3 Hz — main.py heartbeat stale during autonomous
)

# ── Visual Reference ──────────────────────────────────────────────────────────
# Maps each state to a human-readable description of the visual pattern.
# Used for documentation and test script output only.

LED_STATE_DESCRIPTIONS: dict[str, str] = {
    "OFF":            "All off",
    "IDLE":           "Green solid",
    "SCANNING":       "Green blink 1.5 Hz",
    "PROCESSING":     "Yellow solid",
    "WAITING_FCU":    "Yellow blink 1.5 Hz",
    "ERROR":          "Red solid",
    "WARNING":        "Red blink 1.5 Hz",
    "HAILO_ACTIVE":   "Green + Yellow solid",
    "HAILO_DEGRADED": "Yellow blink 3 Hz",
    "HAILO_FAILED":   "Red blink 3 Hz",
    "WATCHDOG_DEAD":  "Red blink 0.8 Hz",
    "MAIN_DEAD":      "Red blink 3 Hz",
}
