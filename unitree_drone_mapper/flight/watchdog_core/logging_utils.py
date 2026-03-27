"""watchdog_core/logging_utils.py — Shared timestamped logger.

All watchdog_core modules import log() from here so output format is consistent
and can be changed in one place.
"""

from datetime import datetime


def log(msg: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)
