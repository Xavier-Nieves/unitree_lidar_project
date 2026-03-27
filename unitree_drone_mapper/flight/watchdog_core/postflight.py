"""watchdog_core/postflight.py — Post-flight processing monitor.

Launches run_postflight.py as a subprocess and:
  - Pipes its stdout line-by-line to the watchdog log (prefixed [POSTFLIGHT])
  - Plays TUNE_PROCESSING every 5 seconds while it runs (via PostflightBeeper)
  - Plays TUNE_DONE when it finishes successfully
  - Plays TUNE_ERROR if it exits with a non-zero return code

Everything runs on daemon threads so the watchdog main loop is never blocked.

Public API
----------
  PostflightMonitor.trigger()   — call after each flight session ends
  PostflightMonitor.is_active   — True while a postflight job is in progress
"""

import subprocess
import sys
import threading
from pathlib import Path

from .buzzer import PostflightBeeper
from .logging_utils import log

# ── Path ──────────────────────────────────────────────────────────────────────

SCRIPT_DIR        = Path(__file__).parent.parent          # project root
POSTFLIGHT_SCRIPT = SCRIPT_DIR.parent / "utils" / "run_postflight.py"


# ── PostflightMonitor ─────────────────────────────────────────────────────────

class PostflightMonitor:
    """Manages a single postflight subprocess with beeper feedback and log piping.

    Args:
        reader:  MavrosReader — used to play buzzer tunes.
    """

    def __init__(self, reader):
        self._reader  = reader
        self._beeper  = PostflightBeeper(reader.play_tune)
        self._lock    = threading.Lock()
        self._active  = False

    @property
    def is_active(self) -> bool:
        with self._lock:
            return self._active

    def trigger(self) -> None:
        """Launch the postflight script. No-op if one is already running."""
        if not POSTFLIGHT_SCRIPT.exists():
            log(f"[WARN] Post-flight script not found: {POSTFLIGHT_SCRIPT}")
            return

        with self._lock:
            if self._active:
                log("[POSTFLIGHT] Already running — skipping duplicate trigger")
                return
            self._active = True

        log("[POSTFLIGHT] Launching post-flight processing...")
        self._reader.beep_processing()
        self._beeper.start()

        try:
            proc = subprocess.Popen(
                [sys.executable, str(POSTFLIGHT_SCRIPT), "--auto", "--skip-wait"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,   # merge stderr into stdout
                text=True,
                bufsize=1,                  # line-buffered
            )
        except Exception as exc:
            log(f"[POSTFLIGHT] Launch failed: {exc}")
            self._reader.beep_error()
            self._beeper.stop()
            with self._lock:
                self._active = False
            return

        # Monitor in background — never blocks the watchdog loop
        monitor_thread = threading.Thread(
            target=self._monitor,
            args=(proc,),
            daemon=True,
        )
        monitor_thread.start()

    # ── Internal ──────────────────────────────────────────────────────────────

    def _monitor(self, proc: subprocess.Popen) -> None:
        """Pipe subprocess output to watchdog log, then play finish beep."""
        try:
            for line in proc.stdout:
                stripped = line.rstrip()
                if stripped:
                    log(f"[POSTFLIGHT] {stripped}")
            proc.wait()
        except Exception as exc:
            log(f"[POSTFLIGHT] Log pipe error: {exc}")
        finally:
            self._beeper.stop()

            if proc.returncode == 0:
                log("[POSTFLIGHT] Processing complete ✓")
                self._reader.beep_done()
            else:
                log(f"[POSTFLIGHT] Processing failed (exit {proc.returncode})")
                self._reader.beep_error()

            with self._lock:
                self._active = False
