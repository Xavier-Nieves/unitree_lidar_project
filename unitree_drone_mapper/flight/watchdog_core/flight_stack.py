"""watchdog_core/flight_stack.py — manage scan-session processes and health checks.

This updated version removes old reader.beep_* helper usage so sound ownership
stays in drone_watchdog.py and postflight.py.

Responsibilities
----------------
- Start the scan stack processes
- Stop the scan stack processes
- Trigger postflight after a completed scan session
- Expose health checks used by the watchdog

Sound ownership
---------------
- Start / stop / ready / active / finished tones are handled by the watchdog
- Postflight tones are handled by postflight.py
- This file should not directly decide buzzer meaning
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Callable, Optional

from .logging_utils import log

# These paths/commands may need to match your project exactly.
# They are kept generic here to preserve the structure of the original file.
PROJECT_ROOT = Path.home() / "unitree_lidar_project"
ROSBAG_DIR = PROJECT_ROOT / "bags"

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = str(PROJECT_ROOT / "RPI5/ros2_ws/install/setup.bash")

SLAM_MIN_HZ = 5.0
BRIDGE_MIN_HZ = 8.0
SLAM_TOPIC_TIMEOUT_S = 30.0
BRIDGE_TOPIC_TIMEOUT_S = 15.0


class FlightStack:
    """Owns the runtime scan stack subprocesses and basic health supervision."""

    def __init__(self, reader, postflight_fn: Optional[Callable[[], None]] = None) -> None:
        self._reader = reader
        self._postflight_fn = postflight_fn

        self._running = False
        self._session_reason = ""
        self._started_at = 0.0

        self._bag_proc: Optional[subprocess.Popen] = None
        self._slam_proc: Optional[subprocess.Popen] = None
        self._bridge_proc: Optional[subprocess.Popen] = None

    # ── Public state ─────────────────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return self._running

    # ── Start / Stop ─────────────────────────────────────────────────────────

    def start(self, reason: str = "UNKNOWN") -> bool:
        """Start the scan stack.

        Returns True on success, False on startup failure.
        """
        if self._running:
            log("[STACK] Start requested, but stack is already running")
            return True

        self._session_reason = reason
        self._started_at = time.time()

        try:
            ROSBAG_DIR.mkdir(parents=True, exist_ok=True)

            # NOTE:
            # Replace these placeholder commands with your exact original ones if needed.
            # They are intentionally simple so the generated file is easy to merge.
            self._bag_proc = self._spawn(
                f"source {ROS_SETUP} && source {WS_SETUP} && ros2 bag record -a -o {ROSBAG_DIR / 'latest_scan'}"
            )
            self._slam_proc = self._spawn(
                f"source {ROS_SETUP} && source {WS_SETUP} && ros2 launch point_lio mapping.launch.py"
            )
            self._bridge_proc = self._spawn(
                f"source {ROS_SETUP} && source {WS_SETUP} && ros2 run unitree_lidar_ros2 bridge_node"
            )

            self._running = True
            log(f"[STACK] Started (reason={reason})")
            return True

        except Exception as exc:
            log(f"[STACK] Startup failed: {exc}")
            self._cleanup_partial_start()
            self._running = False
            return False

    def stop(self) -> None:
        """Stop the scan stack and optionally trigger postflight."""
        if not self._running:
            log("[STACK] Stop requested, but stack is not running")
            return

        log("[STACK] Stopping stack...")
        self._terminate_proc(self._bridge_proc, "bridge")
        self._terminate_proc(self._slam_proc, "slam")
        self._terminate_proc(self._bag_proc, "rosbag")

        self._bridge_proc = None
        self._slam_proc = None
        self._bag_proc = None
        self._running = False

        duration_s = time.time() - self._started_at if self._started_at else 0.0
        log(f"[STACK] Stopped after {duration_s:.1f}s")

        if self._postflight_fn is not None:
            try:
                self._postflight_fn()
            except Exception as exc:
                log(f"[STACK] Postflight trigger failed: {exc}")

    # ── Health ───────────────────────────────────────────────────────────────

    def check_health(self) -> None:
        """Log process-health issues.

        This version does not directly raise alarms. The watchdog can inspect
        process state or extend this class later to return structured faults.
        """
        if not self._running:
            return

        bad = False

        if self._bag_proc is not None and self._bag_proc.poll() is not None:
            log(f"[STACK][HEALTH] rosbag exited unexpectedly with code {self._bag_proc.returncode}")
            bad = True

        if self._slam_proc is not None and self._slam_proc.poll() is not None:
            log(f"[STACK][HEALTH] slam exited unexpectedly with code {self._slam_proc.returncode}")
            bad = True

        if self._bridge_proc is not None and self._bridge_proc.poll() is not None:
            log(f"[STACK][HEALTH] bridge exited unexpectedly with code {self._bridge_proc.returncode}")
            bad = True

        if bad:
            log("[STACK][HEALTH] One or more stack processes died")

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _spawn(self, command: str) -> subprocess.Popen:
        log(f"[STACK] Launching: {command}")
        return subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )

    def _terminate_proc(self, proc: Optional[subprocess.Popen], label: str) -> None:
        if proc is None:
            return

        if proc.poll() is not None:
            log(f"[STACK] {label} already exited with code {proc.returncode}")
            return

        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            try:
                proc.wait(timeout=5.0)
                log(f"[STACK] {label} terminated cleanly")
            except subprocess.TimeoutExpired:
                log(f"[STACK] {label} did not terminate in time; killing")
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=2.0)
        except Exception as exc:
            log(f"[STACK] Failed to stop {label}: {exc}")

    def _cleanup_partial_start(self) -> None:
        self._terminate_proc(self._bridge_proc, "bridge")
        self._terminate_proc(self._slam_proc, "slam")
        self._terminate_proc(self._bag_proc, "rosbag")
        self._bridge_proc = None
        self._slam_proc = None
        self._bag_proc = None
