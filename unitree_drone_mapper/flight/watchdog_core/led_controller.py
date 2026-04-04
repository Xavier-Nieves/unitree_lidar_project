#!/usr/bin/env python3
"""
watchdog_core/led_controller.py — canonical LED states for the matched sound/LED UX.

This version expands the LED vocabulary so it aligns with the buzzer semantics:
- system start
- scan ready
- scan start
- scanning
- scan finished
- processing
- warning
- error
- critical
- system failure
"""

BCM_GREEN:  int = 25   # BOARD pin 22
BCM_YELLOW: int = 8    # BOARD pin 24
BCM_RED:    int = 7    # BOARD pin 26

LED_STATES: tuple[str, ...] = (
    # Core system
    "OFF",
    "SYSTEM_START",
    "IDLE",
    "WAITING_FCU",
    "SCAN_READY",
    "SCAN_START",
    "SCANNING",
    "SCAN_FINISHED",
    "PROCESSING",
    # Fault ladder
    "WARNING",
    "ERROR",
    "CRITICAL",
    "SYSTEM_FAILURE",
    # Hailo / process-health
    "HAILO_ACTIVE",
    "HAILO_DEGRADED",
    "HAILO_FAILED",
    "WATCHDOG_DEAD",
    "MAIN_DEAD",
)

LED_STATE_DESCRIPTIONS: dict[str, str] = {
    "OFF":            "All off",
    "SYSTEM_START":   "Boot sequence: green -> yellow -> red, semi-fast repeating",
    "IDLE":           "Green solid",
    "WAITING_FCU":    "Yellow blink 1.5 Hz",
    "SCAN_READY":     "Green + Yellow solid",
    "SCAN_START":     "Green + Yellow blink 2.5 Hz",
    "SCANNING":       "Green blink 1.5 Hz",
    "SCAN_FINISHED":  "Green + Yellow double-flash style blink 2 Hz",
    "PROCESSING":     "Yellow solid",
    "WARNING":        "Yellow blink 2.5 Hz",
    "ERROR":          "Red solid",
    "CRITICAL":       "All LEDs blink 2.5 Hz",
    "SYSTEM_FAILURE": "All LEDs blink 5 Hz",
    "HAILO_ACTIVE":   "Green + Yellow solid",
    "HAILO_DEGRADED": "Yellow blink 3 Hz",
    "HAILO_FAILED":   "Red blink 3 Hz",
    "WATCHDOG_DEAD":  "Red blink 0.8 Hz",
    "MAIN_DEAD":      "Red blink 3 Hz",
}
