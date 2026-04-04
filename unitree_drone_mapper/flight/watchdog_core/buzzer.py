"""watchdog_core/buzzer.py — semantic buzzer tunes, periodic beepers, and fault alarms.

QBASIC Format 1 tunes for Pixhawk / MAVROS.

Event tones
-----------
- SYSTEM_START: long happy boot
- SCAN_READY: short neutral ack
- SCAN_START: upward medium start
- SCAN_ACTIVE: short repetitive medium-tone tick
- SCAN_FINISHED: descending stop tone
- POSTPROCESS_ACTIVE: short repetitive high-tone tick
- POSTPROCESS_DONE: happy completion
- WARNING: one-shot caution

Latched alarms
--------------
- ERROR: repeating low unpleasant tone
- CRITICAL: repeating critical tone
- SYSTEM_FAILURE: repeating failure alarm

Rules
-----
- WARNING is one-shot only.
- ERROR / CRITICAL / SYSTEM_FAILURE repeat while active.
- Fault priority: SYSTEM_FAILURE > CRITICAL > ERROR.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

TUNE_FORMAT = 1  # QBASIC Format 1

# ── Event Tunes ───────────────────────────────────────────────────────────────

TUNE_SYSTEM_START       = "T180L8O4CEGCEG>C"   # start_v3_fanfare
TUNE_SCAN_READY         = "T250L32O5CE"        # ack_v4_chirp
TUNE_SCAN_START         = "T200L12O5CEG>C"     # start_v2_slower
TUNE_SCAN_ACTIVE        = "T200L64O4GE"        # proc_v4_twotick
TUNE_SCAN_FINISHED      = "T180L8O5C<GEG<C"    # stop_v3_twostep
TUNE_POSTPROCESS_ACTIVE = "T220L64O5C"         # proc_v5_hightick
TUNE_POSTPROCESS_DONE   = "T180L12O5CEGCE"     # done_v2_chord
TUNE_WARNING            = "T200L32O5CC"        # short warning

# ── Fault Tunes ───────────────────────────────────────────────────────────────

TUNE_ERROR          = "T140L4O4C<C"           # long low unpleasant tone
TUNE_CRITICAL       = "T180L8O6C<A<F"         # descending unsettling
TUNE_SYSTEM_FAILURE = "T220L8O6CECECE"        # repeating alarm

# ── Intervals ─────────────────────────────────────────────────────────────────

SCAN_BEEP_INTERVAL_S         = 1.0
POSTPROCESS_BEEP_INTERVAL_S  = 1.0
ERROR_REPEAT_INTERVAL_S      = 2.0
CRITICAL_REPEAT_INTERVAL_S   = 1.25
SYSTEM_FAILURE_INTERVAL_S    = 0.85


def _sleep_responsive(active_fn: Callable[[], bool], seconds: float) -> None:
    deadline = time.time() + seconds
    while time.time() < deadline:
        if not active_fn():
            return
        time.sleep(0.1)


class PeriodicBeeper:
    """Generic repeating beeper for active states."""

    def __init__(self, play_tune_fn: Callable[[str], None], tune: str, interval_s: float) -> None:
        self._play = play_tune_fn
        self._tune = tune
        self._interval_s = interval_s
        self._active = False
        self._thread: Optional[threading.Thread] = None

    @property
    def active(self) -> bool:
        return self._active

    def start(self) -> None:
        if self._active:
            return
        self._active = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._active = False
        if self._thread:
            self._thread.join(timeout=self._interval_s + 1.0)
            self._thread = None

    def _loop(self) -> None:
        while self._active:
            try:
                self._play(self._tune)
            except Exception:
                pass
            _sleep_responsive(lambda: self._active, self._interval_s)


class ScanBeeper(PeriodicBeeper):
    """Repeats the scan-active tone while scanning is in progress."""

    def __init__(self, play_tune_fn: Callable[[str], None]) -> None:
        super().__init__(play_tune_fn, TUNE_SCAN_ACTIVE, SCAN_BEEP_INTERVAL_S)


class PostflightBeeper(PeriodicBeeper):
    """Repeats the post-process tone while post-processing is in progress."""

    def __init__(self, play_tune_fn: Callable[[str], None]) -> None:
        super().__init__(play_tune_fn, TUNE_POSTPROCESS_ACTIVE, POSTPROCESS_BEEP_INTERVAL_S)


class FaultAlarmManager:
    """Manages one-shot warning plus repeating fault alarms.

    Repeating alarms only sound while their fault is active and the system is
    armed, except system failure may optionally repeat during startup even if
    unarmed.
    """

    def __init__(
        self,
        play_tune_fn: Callable[[str], None],
        is_armed_fn: Callable[[], bool],
        startup_failure_repeats: bool = True,
    ) -> None:
        self._play = play_tune_fn
        self._is_armed = is_armed_fn
        self._startup_failure_repeats = startup_failure_repeats

        self._warning_latched = False
        self._error_active = False
        self._critical_active = False
        self._failure_active = False

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)

    def set_warning(self, active: bool = True) -> None:
        if active and not self._warning_latched:
            self._warning_latched = True
            try:
                self._play(TUNE_WARNING)
            except Exception:
                pass
        elif not active:
            self._warning_latched = False

    def set_error(self, active: bool) -> None:
        self._error_active = bool(active)

    def set_critical(self, active: bool) -> None:
        self._critical_active = bool(active)

    def set_system_failure(self, active: bool) -> None:
        self._failure_active = bool(active)

    def clear_all(self) -> None:
        self._warning_latched = False
        self._error_active = False
        self._critical_active = False
        self._failure_active = False

    def _current_alarm(self) -> tuple[Optional[str], float]:
        if self._failure_active:
            if self._is_armed() or self._startup_failure_repeats:
                return TUNE_SYSTEM_FAILURE, SYSTEM_FAILURE_INTERVAL_S
        if not self._is_armed():
            return None, 0.1
        if self._critical_active:
            return TUNE_CRITICAL, CRITICAL_REPEAT_INTERVAL_S
        if self._error_active:
            return TUNE_ERROR, ERROR_REPEAT_INTERVAL_S
        return None, 0.1

    def _loop(self) -> None:
        while self._running:
            tune, interval_s = self._current_alarm()
            if tune is None:
                time.sleep(interval_s)
                continue
            try:
                self._play(tune)
            except Exception:
                pass
            _sleep_responsive(lambda: self._running, interval_s)
