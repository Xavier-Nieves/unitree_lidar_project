"""
status.py — Pipeline status tracking and failure reporting.

Provides stage tracking so errors report exactly where the pipeline
crashed, and writes metadata.json on failure so the web viewer can
display meaningful error messages instead of showing stale data.

Usage:
    from debugger_tools import PipelineStatus, PipelineStage, write_failure_status

    current_stage = PipelineStage.INIT

    try:
        current_stage = PipelineStage.BAG_EXTRACT
        pts = reader.extract()

        current_stage = PipelineStage.MLS
        pts = smoother.smooth(pts)

    except Exception as exc:
        write_failure_status(bag_path, current_stage.value, exc)
        sys.exit(1)
"""

import json
import sys
import traceback
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Optional


class PipelineStage(Enum):
    """Pipeline execution stages for error tracking."""
    INIT            = "init"
    BAG_EXTRACT     = "bag_extract"
    CAP             = "cap"
    MLS             = "mls"
    GROUND_CLASSIFY = "ground_classify"
    DTM             = "dtm"
    DSM             = "dsm"
    MERGE           = "merge"
    TEXTURE         = "texture"        # Stage 6 — IMX477 projection onto mesh
    PUBLISH         = "publish"
    COMPLETE        = "complete"


class PipelineStatus:
    """
    Tracks pipeline execution status with automatic failure handling.

    Can be used as a context manager or manually.

    Usage (context manager):
        with PipelineStatus(bag_path) as status:
            status.stage = PipelineStage.BAG_EXTRACT
            pts = reader.extract()

    Usage (manual):
        status = PipelineStatus(bag_path)
        try:
            status.stage = PipelineStage.BAG_EXTRACT
            pts = reader.extract()
        except Exception as e:
            status.fail(e)
            sys.exit(1)
    """

    def __init__(self, bag_path: Path, session_id: str = None):
        self.bag_path   = Path(bag_path)
        self.session_id = session_id or self.bag_path.name
        self.stage      = PipelineStage.INIT
        self.start_time = datetime.now()
        self.error: Optional[Exception] = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_val is not None:
            self.fail(exc_val)
        return False

    def fail(self, exc: Exception) -> None:
        """Handle pipeline failure — log, print traceback, write metadata."""
        self.error = exc
        print(f"\n{'='*60}", file=sys.stderr)
        print(f"[FAIL] Pipeline crashed at stage '{self.stage.value}'",
              file=sys.stderr)
        print(f"       Error: {type(exc).__name__}: {exc}", file=sys.stderr)
        print(f"{'='*60}", file=sys.stderr)
        traceback.print_exc()
        write_failure_status(
            bag_path   = self.bag_path,
            stage      = self.stage.value,
            exc        = exc,
            session_id = self.session_id,
        )

    def complete(self) -> None:
        self.stage = PipelineStage.COMPLETE


def write_failure_status(
    bag_path:    Path,
    stage:       str,
    exc:         Exception,
    session_id:  str   = None,
    duration_s:  float = None,
    point_count: int   = None,
) -> None:
    """
    Write metadata.json marking the pipeline as failed.

    Called from exception handlers so the web viewer can display an error
    message instead of stale or missing results. Also logs the failure to
    flight_history.log when flight_logger is available.

    Parameters
    ----------
    bag_path    : Directory where metadata.json is written.
    stage       : Pipeline stage name where failure occurred.
    exc         : The exception that caused the failure.
    session_id  : Session name (defaults to bag_path.name).
    duration_s  : Time elapsed before failure (optional, for flight log).
    point_count : Points processed before failure (optional, for flight log).
    """
    bag_path   = Path(bag_path)
    session_id = session_id or bag_path.name
    error_msg  = f"{type(exc).__name__}: {exc}"

    # ── Write metadata.json for web viewer ───────────────────────────────────
    payload = {
        "id":           session_id,
        "status":       "failed",
        "failed_stage": stage,
        "error":        error_msg,
        "processed_at": datetime.now().strftime("%Y-%m-%d %H:%M"),
    }
    try:
        out = bag_path / "metadata.json"
        with open(out, "w") as f:
            json.dump(payload, f, indent=2)
        print(f"  [Pipeline] Failure status written → {out}")
    except Exception as write_err:
        print(f"  [Pipeline] Could not write failure status: {write_err}",
              file=sys.stderr)

    # ── Log to flight_history.log via flight_logger ───────────────────────────
    # Resolved by file path so this works regardless of working directory.
    # Silently skipped if flight_logger.py is not found.
    try:
        import importlib.util
        _logger_path = Path(__file__).resolve().parents[1] / "flight_logger.py"
        if _logger_path.exists():
            spec = importlib.util.spec_from_file_location(
                "flight_logger", _logger_path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            mod.log_failure(
                bag_name    = session_id,
                stage       = stage,
                error       = error_msg,
                duration_s  = duration_s,
                point_count = point_count,
            )
    except Exception as log_err:
        print(f"  [Pipeline] Flight logger unavailable: {log_err}",
              file=sys.stderr)
