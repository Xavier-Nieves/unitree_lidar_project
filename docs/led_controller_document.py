#!/usr/bin/env python3
"""
watchdog_core/led_controller.py — External RGB LED status indicator.

Overview
--------
Controls a 3-colour (Red / Yellow / Green) LED PCB wired directly to
Raspberry Pi GPIO pins. Provides visual system-state feedback that is
intentionally separate from the Pixhawk's onboard LED, which is owned
by PX4 Commander and reflects flight modes.

This module reflects *software and system state* only:
  - Is the watchdog ready?
  - Is a scan session active?
  - Is postprocessing running or done?
  - Has a critical error occurred?

Hardware
--------
  Component : 3-LED PCB (Red, Yellow, Green)
  Interface : Raspberry Pi GPIO (direct drive)
  Polarity  : TBD — confirm common anode vs common cathode before wiring
                Common anode  : HIGH = off, LOW = on
                Common cathode: HIGH = on,  LOW = off
  GPIO pins : TBD — assign and update PIN_RED, PIN_YELLOW, PIN_GREEN below
                Suggested defaults: Red=17, Yellow=27, Green=22 (BCM numbering)
  Resistors : Use 220–330Ω inline resistors on each LED leg if not already
              on the PCB to protect the Pi GPIO pins (3.3V, max 16mA per pin)

Wiring Steps (to complete before enabling)
-------------------------------------------
  1. Identify common anode vs cathode on your PCB (check PCB silkscreen or
     measure with a multimeter — common pin to 3.3V = anode, to GND = cathode)
  2. Connect each LED's signal pin to the chosen GPIO pin via resistor
  3. Connect common pin to 3.3V (anode) or GND (cathode)
  4. Update PIN_RED, PIN_YELLOW, PIN_GREEN constants below
  5. Update LED_ACTIVE_HIGH to match your polarity
  6. Set ENABLE_LED = True to activate

Dependencies
------------
  pip install RPi.GPIO
  or on Pi 5:
  pip install rpi-lgpio        # RPi.GPIO drop-in for Pi 5 kernel

State Machine
-------------
States are mutually exclusive and applied in priority order (highest first):

  CRITICAL_ERROR  — All three LEDs on solid
  ─────────────────────────────────────────────────────────────────────────
  Triggered by any of:
    • SSD unmounted or rosbag directory missing/inaccessible
    • Corrupted bag file detected (metadata.yaml missing after close wait)
    • Point-LIO process crashed mid-scan
    • Bag recorder exited unexpectedly during a session
    • SLAM bridge exited unexpectedly during a session
    • Postflight script exited with non-zero return code
    • MAVROS FCU connection lost after initial connect
    • Lock file unreadable or JSON-corrupted
  Clears: only on watchdog restart (requires human inspection)

  POSTPROCESSING_RUNNING — Yellow fast blink (0.3s on/off)
  ─────────────────────────────────────────────────────────────────────────
  Triggered by: PostflightMonitor.trigger() called
  Clears:       postflight subprocess exits (success or failure)
                On failure → transitions to CRITICAL_ERROR

  POSTPROCESSING_DONE — Yellow solid (holds for 5s then → READY)
  ─────────────────────────────────────────────────────────────────────────
  Triggered by: postflight subprocess exits with code 0
  Clears:       after DONE_HOLD_S seconds automatically

  STACK_ACTIVE — Green slow blink (1s on/off)
  ─────────────────────────────────────────────────────────────────────────
  Triggered by: FlightStack.start() called
  Clears:       FlightStack.stop() called

  READY — Green solid
  ─────────────────────────────────────────────────────────────────────────
  Triggered by: MAVROS connected, no active session, no postprocessing
  Clears:       any higher-priority state becomes active

  BOOTING — Red slow blink (1s on/off)
  ─────────────────────────────────────────────────────────────────────────
  Triggered by: watchdog startup before MAVROS connects
  Clears:       MAVROS FCU connection confirmed

State transition diagram
-------------------------

  [Power on]
       │
       ▼
  ┌─────────┐   MAVROS connects    ┌───────┐
  │ BOOTING │─────────────────────▶│ READY │◀──────────────────────┐
  └─────────┘                      └───────┘                       │
       │                               │                           │
       │                         stack │start()           done + 5s│
       │                               ▼                           │
       │                      ┌──────────────┐   stop()  ┌──────────────────┐
       │                      │ STACK_ACTIVE │──────────▶│ POSTPROCESSING   │
       │                      └──────────────┘           │ RUNNING          │
       │                               │                 └──────────────────┘
       │                               │ crash                     │
       │                               │                     rc=0  │
       │                               ▼                           ▼
       │                      ┌────────────────┐         ┌─────────────────┐
       └────────────────────▶ │ CRITICAL_ERROR │◀────────│ POSTPROCESSING  │
          any critical event  └────────────────┘  rc≠0   │ DONE            │
                                                          └─────────────────┘

Visual Summary
--------------

  State                  RED        YELLOW      GREEN
  ─────────────────────────────────────────────────────
  BOOTING                blink(1s)  off         off
  READY                  off        off         solid
  STACK_ACTIVE           off        off         blink(1s)
  POSTPROCESSING_RUNNING off        blink(0.3s) off
  POSTPROCESSING_DONE    off        solid       off
  CRITICAL_ERROR         solid      solid       solid
  ─────────────────────────────────────────────────────

  blink(Xs) = X second period, 50% duty cycle

Integration Points
------------------
All calls into this module are one-liners. Place them at these locations:

  drone_watchdog.py
    _wait_for_mavros()        → led.set_booting()   on entry
                              → led.set_ready()     on connect
    main loop                 → led.set_error()     on MAVROS disconnect

  watchdog_core/flight_stack.py
    FlightStack.start()       → led.set_stack_active()
    FlightStack.stop()        → (LED handed to postflight, no call needed)
    FlightStack.check_health()
      Point-LIO crash         → led.set_error()
      Bag recorder crash      → led.set_error()
      SLAM bridge crash       → led.set_error()

  watchdog_core/postflight.py
    PostflightMonitor.trigger()        → led.set_postprocessing()
    _monitor() rc == 0                 → led.set_postprocessing_done()
    _monitor() rc != 0                 → led.set_error()

  watchdog_core/mavros_reader.py
    _state_cb() connected → False      → led.set_error()

  (future) SSD health check            → led.set_error()
  (future) lock file corruption        → led.set_error()

Public API (to implement)
-------------------------
  led = LEDController()

  led.set_booting()            # Red slow blink
  led.set_ready()              # Green solid
  led.set_stack_active()       # Green slow blink
  led.set_postprocessing()     # Yellow fast blink
  led.set_postprocessing_done()# Yellow solid → auto-clears after DONE_HOLD_S
  led.set_error()              # All solid — latches until restart

  led.shutdown()               # Turn all LEDs off cleanly on exit

Implementation Notes (for when GPIO pins are confirmed)
-------------------------------------------------------
  - Use RPi.GPIO in BCM mode (GPIO.setmode(GPIO.BCM))
  - All blink patterns should run on a daemon thread (same pattern as
    PostflightBeeper in buzzer.py) so they never block the watchdog loop
  - set_error() should latch — once set, no other state can clear it
    short of a full watchdog restart. This ensures errors are not silently
    swallowed by a subsequent state transition.
  - DONE_HOLD_S = 5  (yellow solid duration before returning to READY)
  - Thread safety: use threading.Lock() to protect GPIO state, same
    pattern as MavrosReader._lock

Configuration Constants (fill in before enabling)
--------------------------------------------------
  ENABLE_LED     = False       # Set True once wired and tested
  LED_ACTIVE_HIGH = True       # True = cathode, False = anode (confirm on PCB)
  PIN_RED        = None        # BCM pin number — e.g. 17
  PIN_YELLOW     = None        # BCM pin number — e.g. 27
  PIN_GREEN      = None        # BCM pin number — e.g. 22
  DONE_HOLD_S    = 5           # Seconds to hold POSTPROCESSING_DONE before READY
  BLINK_SLOW_S   = 1.0         # Period for slow blink states
  BLINK_FAST_S   = 0.3         # Period for fast blink (postprocessing running)
"""

# ── Placeholder — implementation pending GPIO wiring ─────────────────────────
#
# Once PIN_RED, PIN_YELLOW, PIN_GREEN are confirmed and ENABLE_LED is set
# to True, implement LEDController here following the notes above.
#
# Skeleton to fill in:
#
# import threading
# import time
# import RPi.GPIO as GPIO   # or: import lgpio as GPIO for Pi 5
#
# from .logging_utils import log
#
# ENABLE_LED      = False
# LED_ACTIVE_HIGH = True
# PIN_RED         = None
# PIN_YELLOW      = None
# PIN_GREEN       = None
# DONE_HOLD_S     = 5
# BLINK_SLOW_S    = 1.0
# BLINK_FAST_S    = 0.3
#
#
# class LEDController:
#     def set_booting(self):            ...
#     def set_ready(self):              ...
#     def set_stack_active(self):       ...
#     def set_postprocessing(self):     ...
#     def set_postprocessing_done(self):...
#     def set_error(self):              ...
#     def shutdown(self):               ...
