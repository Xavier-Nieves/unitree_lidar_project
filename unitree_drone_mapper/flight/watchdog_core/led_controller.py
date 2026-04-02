#!/usr/bin/env python3
"""
watchdog_core/led_controller.py — RGB status LED controller.

HARDWARE CONFIGURATION
----------------------
  Physical pin 34  →  GND    (common ground for all LED channels)
  Physical pin 36  →  Green  LED  (BCM GPIO 16)
  Physical pin 38  →  Yellow LED  (BCM GPIO 20)
  Physical pin 40  →  Red    LED  (BCM GPIO 21)

STATUS MAPPING
--------------
  GREEN   solid       System idle / healthy, no active scan
  GREEN   blink       Scan session active — LiDAR recording in progress
  YELLOW  solid       Post-flight processing running
  YELLOW  blink       Waiting for MAVROS FCU connection
  RED     solid       Fatal error — watchdog entered error state
  RED     blink       Non-fatal warning (e.g. low disk space)
  ALL OFF             System initialising or powered down

  ── Hailo states ──────────────────────────────────────────────────────────
  GREEN+YELLOW  solid       Hailo flight node active and augmenting EKF2
  YELLOW        fast blink  Hailo degraded — flow fallback consecutive rejections
  RED           fast blink  Hailo hard-failed during flight (node crashed or
                            /dev/hailo0 lost) — mission continues without Hailo

  The Hailo states use a faster blink rate (3 Hz) than standard WARNING (1.5 Hz)
  so they are visually distinct from standard warning conditions.

ENABLE / DISABLE
----------------
Set LED_ENABLED = True once GPIO wiring is complete.
All public methods are no-ops when False — no conditional guards needed
at call sites.

EXTENDING LEDState
------------------
When adding a new LEDState value, you MUST add a matching branch in
LEDController.set_state(). Omitting it causes set_state() to log a warning
and return without updating _current_state — subsequent calls with the same
new state are then silently skipped due to the equality guard.
"""

import threading
import logging
from enum import Enum, auto

logger = logging.getLogger(__name__)

# ── Configuration ─────────────────────────────────────────────────────────────

LED_ENABLED: bool = False   # Set True once GPIO wiring is physically connected

_BCM_GREEN:  int = 16   # BOARD pin 36
_BCM_YELLOW: int = 20   # BOARD pin 38
_BCM_RED:    int = 21   # BOARD pin 40

_BLINK_HZ:       float = 1.5                           # Standard blink rate
_BLINK_ON:       float = (1.0 / _BLINK_HZ) * 0.5      # 50% duty cycle
_BLINK_HZ_FAST:  float = 3.0                           # Fast blink for Hailo states
_BLINK_ON_FAST:  float = (1.0 / _BLINK_HZ_FAST) * 0.5


# ── State Enum ────────────────────────────────────────────────────────────────

class LEDState(Enum):
    """
    Named LED states. Each value maps to a specific channel + pattern in
    LEDController.set_state(). Adding a value here without a matching branch
    in set_state() produces a logged warning and silent no-op.
    """
    OFF             = auto()  # All LEDs off
    IDLE            = auto()  # Green solid
    SCANNING        = auto()  # Green blink (standard rate)
    PROCESSING      = auto()  # Yellow solid
    WAITING_FCU     = auto()  # Yellow blink (standard rate)
    ERROR           = auto()  # Red solid
    WARNING         = auto()  # Red blink (standard rate)
    # ── Hailo-specific states ──────────────────────────────────────────────
    HAILO_ACTIVE    = auto()  # Green + Yellow solid (Hailo augmenting flight)
    HAILO_DEGRADED  = auto()  # Yellow fast blink (flow fallback — non-fatal)
    HAILO_FAILED    = auto()  # Red fast blink (Hailo hard-failed mid-flight)


# ── Blink Thread ──────────────────────────────────────────────────────────────

class _BlinkThread(threading.Thread):
    """Daemon thread that toggles a gpiozero LED until stopped.

    Drives the pin LOW before returning so LED state is always determinate
    after stop() + join().
    """

    def __init__(self, led, on_time: float) -> None:
        super().__init__(daemon=True)
        self._led        = led
        self._on_time    = on_time
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
            self._led.on()
            if self._stop_event.wait(timeout=self._on_time):
                break
            self._led.off()
            if self._stop_event.wait(timeout=self._on_time):
                break
        try:
            self._led.off()
        except Exception:
            pass

    def stop(self) -> None:
        self._stop_event.set()


# ── Controller ────────────────────────────────────────────────────────────────

class LEDController:
    """Three-channel GPIO LED controller for DronePi status indication.

    Instantiate once at watchdog or main.py startup. Call set_state() to
    transition states. Call cleanup() on shutdown.

    Thread safety: set_state() and cleanup() acquire an internal lock.

    Example:
        leds = LEDController()
        leds.set_state(LEDState.IDLE)
        # ... on shutdown ...
        leds.cleanup()
    """

    def __init__(self) -> None:
        self._leds:          dict                  = {}
        self._initialised:   bool                  = False
        self._current_state: LEDState              = LEDState.OFF
        self._blink_thread:  _BlinkThread | None   = None
        self._lock:          threading.Lock        = threading.Lock()

        if not LED_ENABLED:
            logger.info(
                "[LEDController] LED_ENABLED=False — GPIO suppressed. "
                "Set True in led_controller.py once wiring is complete."
            )
            return

        try:
            from gpiozero import LED  # type: ignore[import]
            self._leds = {
                "green":  LED(_BCM_GREEN),
                "yellow": LED(_BCM_YELLOW),
                "red":    LED(_BCM_RED),
            }
            self._initialised = True
            logger.info(
                "[LEDController] gpiozero ready. BCM G:%d Y:%d R:%d",
                _BCM_GREEN, _BCM_YELLOW, _BCM_RED,
            )
        except ImportError:
            logger.warning(
                "[LEDController] gpiozero not available. "
                "Install: sudo apt install python3-gpiozero python3-lgpio"
            )
        except Exception as exc:
            logger.error("[LEDController] GPIO setup failed: %s", exc)

    # ── Public API ────────────────────────────────────────────────────────────

    def is_available(self) -> bool:
        """True if GPIO is initialised and LED output is active."""
        return self._initialised

    def set_state(self, state: LEDState) -> None:
        """Transition to a named LED state. Thread-safe.

        Stops any active blink, turns all channels off, applies new pattern.
        No-op if already in the requested state.
        """
        if not self._initialised:
            return

        with self._lock:
            if state == self._current_state:
                return

            self._stop_blink_locked()
            self._all_off_locked()

            # ── Standard states ───────────────────────────────────────────
            if   state == LEDState.OFF:
                pass                                        # already off
            elif state == LEDState.IDLE:
                self._solid_locked("green")
            elif state == LEDState.SCANNING:
                self._blink_locked("green", _BLINK_ON)
            elif state == LEDState.PROCESSING:
                self._solid_locked("yellow")
            elif state == LEDState.WAITING_FCU:
                self._blink_locked("yellow", _BLINK_ON)
            elif state == LEDState.ERROR:
                self._solid_locked("red")
            elif state == LEDState.WARNING:
                self._blink_locked("red", _BLINK_ON)

            # ── Hailo states ──────────────────────────────────────────────
            elif state == LEDState.HAILO_ACTIVE:
                # Green + Yellow solid — visually distinct from IDLE (green only)
                # Signals: scanning + Hailo augmentation both running
                self._solid_locked("green")
                self._solid_locked("yellow")
            elif state == LEDState.HAILO_DEGRADED:
                # Yellow fast blink — Hailo flow rejected repeatedly, non-fatal
                # Flight continues on SLAM-only velocity
                self._blink_locked("yellow", _BLINK_ON_FAST)
            elif state == LEDState.HAILO_FAILED:
                # Red fast blink — Hailo node crashed or /dev/hailo0 lost
                # Fast rate distinguishes from WARNING (standard red blink)
                self._blink_locked("red", _BLINK_ON_FAST)

            else:
                logger.warning(
                    "[LEDController] Unhandled state: %s — add branch to set_state()",
                    state,
                )
                return  # Do NOT update _current_state

            self._current_state = state
            logger.debug("[LEDController] → %s", state.name)

    def current_state(self) -> LEDState:
        """Return the current LED state."""
        with self._lock:
            return self._current_state

    def cleanup(self) -> None:
        """Turn off all LEDs, stop blink thread, release GPIO. Safe to call twice."""
        if not self._initialised:
            return

        with self._lock:
            self._stop_blink_locked()
            self._all_off_locked()
            for led in self._leds.values():
                try:
                    led.close()
                except Exception as exc:
                    logger.warning("[LEDController] GPIO close warning: %s", exc)
            self._leds.clear()
            self._initialised = False
            logger.info("[LEDController] GPIO released.")

    # ── Internal helpers (must be called with self._lock held) ────────────────

    def _solid_locked(self, channel: str) -> None:
        self._leds[channel].on()

    def _blink_locked(self, channel: str, on_time: float) -> None:
        self._blink_thread = _BlinkThread(self._leds[channel], on_time)
        self._blink_thread.start()

    def _all_off_locked(self) -> None:
        for led in self._leds.values():
            try:
                led.off()
            except Exception:
                pass

    def _stop_blink_locked(self) -> None:
        """Stop the active blink thread and wait for it to exit."""
        if self._blink_thread is None:
            return
        self._blink_thread.stop()
        # Timeout = 2 full half-cycles + margin — guarantees thread wakes
        # from Event.wait() and executes led.off() before join() returns
        timeout = _BLINK_ON * 2 + 0.1
        self._blink_thread.join(timeout=timeout)
        if self._blink_thread.is_alive():
            logger.warning(
                "[LEDController] Blink thread did not exit within %.2fs", timeout
            )
        self._blink_thread = None
