"""watchdog_core/buzzer.py — Buzzer tune definitions and periodic beeper.

Tune strings use QBASIC Format 1 (confirmed working on this firmware).

Tunes
-----
  TUNE_START      Rising scale  — flight stack starting
  TUNE_STOP       Falling scale — flight stack stopping
  TUNE_ACK        Single beep   — button acknowledged / FCU connected
  TUNE_ERROR      Staccato x3   — error condition
  TUNE_PROCESSING Single mid    — post-processing heartbeat pulse
  TUNE_DONE       Two-note up   — post-processing finished

PostflightBeeper
----------------
  Background thread that plays TUNE_PROCESSING every POSTFLIGHT_BEEP_INTERVAL_S
  seconds while post-processing is active. Call start() when postflight begins
  and stop() when it ends.
"""

import threading
import time

# ── Tune Format ───────────────────────────────────────────────────────────────

TUNE_FORMAT = 1  # QBASIC1_1

# ── Tune Strings ──────────────────────────────────────────────────────────────

TUNE_START      = "T240L16O5CEGC"      # Rising scale:    stack starting
TUNE_STOP       = "T240L16O5CG<EGC"    # Falling scale:   stack stopping
TUNE_ACK        = "T200L32O6C"         # Single beep:     button ack / FCU ready
TUNE_ERROR      = "T200L16O5CPCPC"     # Staccato x3:     error
TUNE_PROCESSING = "T200L32O4G"         # Mid single pulse: postflight heartbeat
TUNE_DONE       = "T240L16O5CE"        # Two-note up:     postflight finished

# ── Interval ──────────────────────────────────────────────────────────────────

POSTFLIGHT_BEEP_INTERVAL_S = 5.0


# ── Periodic Beeper ───────────────────────────────────────────────────────────

class PostflightBeeper:
    """Plays TUNE_PROCESSING every POSTFLIGHT_BEEP_INTERVAL_S on a daemon thread.

    Usage:
        beeper = PostflightBeeper(play_tune_fn)
        beeper.start()   # called when postflight subprocess launches
        beeper.stop()    # called when subprocess exits
    """

    def __init__(self, play_tune_fn):
        """
        Args:
            play_tune_fn: callable that accepts a tune string and publishes it
                          to the Pixhawk (i.e. MavrosReader.play_tune).
        """
        self._play   = play_tune_fn
        self._active = False
        self._thread = None

    def start(self) -> None:
        if self._active:
            return
        self._active = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._active = False
        if self._thread:
            self._thread.join(timeout=POSTFLIGHT_BEEP_INTERVAL_S + 1)
            self._thread = None

    def _loop(self) -> None:
        while self._active:
            self._play(TUNE_PROCESSING)
            # Sleep in small increments so stop() is responsive
            for _ in range(int(POSTFLIGHT_BEEP_INTERVAL_S * 10)):
                if not self._active:
                    break
                time.sleep(0.1)
