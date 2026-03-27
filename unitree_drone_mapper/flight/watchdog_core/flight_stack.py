"""watchdog_core/flight_stack.py — Flight stack process management.

Manages three subprocesses for each scanning session:
  1. Point-LIO   — LiDAR odometry and mapping
  2. SLAM bridge — vision-pose fusion relay to MAVROS
  3. Bag recorder — ROS 2 bag capture of key topics

Public API
----------
  FlightStack.start(mode)   — launches all three processes
  FlightStack.stop()        — stops them in reverse order, then triggers postflight
  FlightStack.check_health() — warns if any process exited unexpectedly
  FlightStack.is_running    — bool property
"""

import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

from .logging_utils import log

# ── Paths ─────────────────────────────────────────────────────────────────────

ROS_SETUP  = "/opt/ros/jazzy/setup.bash"
WS_SETUP   = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)

SCRIPT_DIR         = Path(__file__).parent.parent   # project root (one above watchdog_core/)
SLAM_BRIDGE_SCRIPT = SCRIPT_DIR / "_slam_bridge.py"
LAUNCH_FILE        = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)

ROSBAG_DIR = Path("/mnt/ssd/rosbags")

BAG_TOPICS = [
    "/cloud_registered",
    "/aft_mapped_to_init",
    "/unilidar/imu",
    "/mavros/state",
    "/mavros/local_position/pose",
    "/mavros/global_position/global",
    "/mavros/vision_pose/pose",
]

# ── Tuning ────────────────────────────────────────────────────────────────────

GRACEFUL_KILL_S = 5


# ── Low-level Process Helpers ─────────────────────────────────────────────────

def start_process(name: str, cmd: str) -> subprocess.Popen:
    """Launch a bash command in its own process group. Returns the Popen handle."""
    log(f"Starting {name}...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    log(f"{name} started (PID {proc.pid})")
    return proc


def stop_process(name: str, proc: subprocess.Popen) -> None:
    """Send SIGINT to the process group, escalating to SIGKILL if needed."""
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return

    log(f"Stopping {name}...")
    try:
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return

    try:
        proc.wait(timeout=GRACEFUL_KILL_S)
        log(f"{name} stopped cleanly.")
    except subprocess.TimeoutExpired:
        log(f"{name} — sending SIGKILL.")
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()


# ── Command Builders ──────────────────────────────────────────────────────────

def _pointlio_cmd() -> str:
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"ros2 launch {LAUNCH_FILE} rviz:=false port:=/dev/ttyUSB0"
    )


def _slam_bridge_cmd() -> str:
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"python3 {SLAM_BRIDGE_SCRIPT}"
    )


def _bag_record_cmd() -> str:
    ts     = datetime.now().strftime("%Y%m%d_%H%M%S")
    out    = ROSBAG_DIR / f"scan_{ts}"
    topics = " ".join(BAG_TOPICS)
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"ros2 bag record -o {out} {topics}"
    )


# ── FlightStack ───────────────────────────────────────────────────────────────

class FlightStack:
    """Manages Point-LIO, SLAM bridge, and bag recorder subprocesses.

    Args:
        reader:           MavrosReader — used to play buzzer tunes.
        postflight_fn:    callable() — called after stop() to trigger postflight.
                          Decoupled here so flight_stack has no import dependency
                          on postflight.py.
    """

    def __init__(self, reader, postflight_fn):
        self._reader         = reader
        self._postflight_fn  = postflight_fn
        self._pointlio_proc  = None
        self._bridge_proc    = None
        self._bag_proc       = None
        self._is_running     = False
        self.session_count   = 0

    @property
    def is_running(self) -> bool:
        return self._is_running

    def start(self, mode: str) -> None:
        if self._is_running:
            log("[WARN] Stack already running — ignoring start request")
            return

        self.session_count += 1
        log(f"[ACTIVATING] Session #{self.session_count} ({mode})")
        self._reader.beep_start()

        self._pointlio_proc = start_process("Point-LIO", _pointlio_cmd())
        time.sleep(3.0)

        if SLAM_BRIDGE_SCRIPT.exists():
            self._bridge_proc = start_process("SLAM bridge", _slam_bridge_cmd())
            time.sleep(1.0)
        else:
            log("[WARN] SLAM bridge script not found — vision fusion disabled")

        self._bag_proc   = start_process("Bag recorder", _bag_record_cmd())
        self._is_running = True
        log(f"[ACTIVE] Session #{self.session_count} — scanning")

    def stop(self) -> None:
        if not self._is_running:
            return

        log(f"[DEACTIVATING] Session #{self.session_count}")
        self._reader.beep_stop()

        # Stop in reverse launch order to ensure clean bag finalisation
        stop_process("Bag recorder", self._bag_proc)
        self._bag_proc = None

        stop_process("SLAM bridge", self._bridge_proc)
        self._bridge_proc = None

        stop_process("Point-LIO", self._pointlio_proc)
        self._pointlio_proc = None

        self._is_running = False
        log(f"[WAITING] Session #{self.session_count} complete")

        self._postflight_fn()

    def check_health(self) -> None:
        """Warn if any managed process has exited unexpectedly."""
        if self._pointlio_proc and self._pointlio_proc.poll() is not None:
            log("[WARN] Point-LIO exited unexpectedly")
            self._pointlio_proc = None
        if self._bridge_proc and self._bridge_proc.poll() is not None:
            log("[WARN] SLAM bridge exited unexpectedly")
            self._bridge_proc = None
        if self._bag_proc and self._bag_proc.poll() is not None:
            log("[WARN] Bag recorder exited unexpectedly")
            self._bag_proc = None
