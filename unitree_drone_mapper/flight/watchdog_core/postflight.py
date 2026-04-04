"""watchdog_core/postflight.py — Post-flight processing monitor.

Launches run_postflight.py as a subprocess and:
  - Pipes stdout line-by-line to the watchdog log
  - Plays a repeating post-process tone while it runs (via PostflightBeeper)
  - Plays TUNE_POSTPROCESS_DONE when it finishes successfully
  - Plays TUNE_ERROR once if it exits non-zero

Everything runs on daemon threads so the watchdog main loop is never blocked.
"""

from __future__ import annotations

import subprocess
import sys
import threading
from pathlib import Path

from .buzzer import PostflightBeeper, TUNE_ERROR, TUNE_POSTPROCESS_DONE
from .logging_utils import log

SCRIPT_DIR = Path(__file__).parent.parent
POSTFLIGHT_SCRIPT = SCRIPT_DIR.parent / "utils" / "run_postflight.py"


class PostflightMonitor:
    """Manages a single postflight subprocess with beeper feedback and log piping."""

    def __init__(self, reader) -> None:
        self._reader = reader
        self._beeper = PostflightBeeper(reader.play_tune)
        self._lock = threading.Lock()
        self._active = False

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
        self._beeper.start()

        try:
            proc = subprocess.Popen(
                [sys.executable, str(POSTFLIGHT_SCRIPT), "--auto", "--skip-wait"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except Exception as exc:
            log(f"[POSTFLIGHT] Launch failed: {exc}")
            self._reader.play_tune(TUNE_ERROR)
            self._beeper.stop()
            with self._lock:
                self._active = False
            return

        monitor_thread = threading.Thread(target=self._monitor, args=(proc,), daemon=True)
        monitor_thread.start()

    def _monitor(self, proc: subprocess.Popen) -> None:
        """Pipe subprocess output to watchdog log, then play finish tone."""
        try:
            if proc.stdout is not None:
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
                self._reader.play_tune(TUNE_POSTPROCESS_DONE)
            else:
                log(f"[POSTFLIGHT] Processing failed (exit {proc.returncode})")
                self._reader.play_tune(TUNE_ERROR)

            with self._lock:
                self._active = False
