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
    "/mavros/vision_pose/pose",   # SLAM bridge output — EKF2 fusion input
    "/hailo/optical_flow",        # Hailo velocity output — logged when active
    "/hailo/ground_class",        # Hailo ground classifier — logged when active
]

# ── Tuning ────────────────────────────────────────────────────────────────────

GRACEFUL_KILL_S = 5

# SLAM chain verification — topic rate thresholds replace fixed sleeps.
# These must match the constants in main.py to ensure consistent behaviour
# across MODE 2 (watchdog-owned) and MODE 3 (main.py-owned) launches.
SLAM_MIN_HZ            = 5.0    # Minimum Hz for /aft_mapped_to_init
BRIDGE_MIN_HZ          = 8.0    # Minimum Hz for /mavros/vision_pose/pose
SLAM_TOPIC_TIMEOUT_S   = 30.0   # Max seconds to wait for Point-LIO
BRIDGE_TOPIC_TIMEOUT_S = 15.0   # Max seconds to wait for SLAM bridge


def _wait_for_topic_hz(topic: str, min_hz: float, timeout_s: float) -> bool:
    """Verify a ROS 2 topic is publishing above min_hz within timeout_s.

    Shells out to `ros2 topic hz` with a short window sample. Returns True
    when the measured rate meets or exceeds min_hz, False on timeout.

    This replaces all fixed time.sleep() calls after process launch — a sleep
    makes a timing assumption, this function verifies actual health.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            result = subprocess.run(
                ["bash", "-c",
                 f"source {ROS_SETUP} && source {WS_SETUP} && "
                 f"timeout 3 ros2 topic hz {topic} --window 5 2>/dev/null "
                 f"| grep -oP 'average rate: \\K[0-9.]+'"],
                capture_output=True, text=True, timeout=5,
            )
            hz_str = result.stdout.strip()
            if hz_str:
                hz = float(hz_str)
                if hz >= min_hz:
                    return True
        except Exception:
            pass
        time.sleep(2.0)
    return False


def _pointlio_already_running() -> bool:
    """Return True if Point-LIO is already publishing /aft_mapped_to_init.

    Called at the start of FlightStack.start() to detect whether the
    pointlio-standby.service has already brought Point-LIO up before arm.
    Uses a short 2-second window — if it is running it will be obvious
    immediately; if not we fall through to the normal launch path.

    This check means the pilot can benefit from pre-arm SLAM initialisation
    (map already built, EKF2 already fusing) without any change to the
    arm/disarm workflow.
    """
    return _wait_for_topic_hz("/aft_mapped_to_init", SLAM_MIN_HZ, timeout_s=2.0)


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

    start() now verifies that each process is actively publishing on its
    expected ROS 2 topic before proceeding to the next step. This replaces
    the previous fixed time.sleep() approach which made a timing assumption
    rather than a health check.

    If Point-LIO or the SLAM bridge fail to publish within their timeout,
    start() aborts cleanly, stops all launched processes, and returns False.
    The caller (drone_watchdog main loop) should then return to IDLE and
    wait for the next arm event — not retry immediately.

    Args:
        reader:           MavrosReader — used to play buzzer tunes.
        postflight_fn:    callable() — called after stop() to trigger postflight.
    """

    def __init__(self, reader, postflight_fn):
        self._reader             = reader
        self._postflight_fn      = postflight_fn
        self._pointlio_proc      = None
        self._pointlio_external  = False   # True when Point-LIO was pre-running
        self._bridge_proc        = None
        self._bag_proc           = None
        self._is_running         = False
        self.session_count       = 0

    @property
    def is_running(self) -> bool:
        return self._is_running

    def _abort(self, reason: str) -> bool:
        """Stop any launched processes and log the abort reason.

        Does not stop Point-LIO if it was externally managed
        (started by pointlio-standby.service before arm).
        Returns False so callers can use: return self._abort("reason").
        """
        log(f"[ABORT] Stack startup failed: {reason}")
        log("[ABORT] Stopping any launched processes — returning to IDLE")
        stop_process("Bag recorder", self._bag_proc)
        self._bag_proc = None
        stop_process("SLAM bridge",  self._bridge_proc)
        self._bridge_proc = None
        if not self._pointlio_external:
            stop_process("Point-LIO", self._pointlio_proc)
            self._pointlio_proc = None
        else:
            log("Point-LIO was pre-running — leaving it active for next attempt")
            self._pointlio_external = False
        self._is_running = False
        return False

    def start(self, mode: str) -> bool:
        """Launch Point-LIO → SLAM bridge → bag recorder with topic verification.

        Each process is verified to be publishing on its expected topic before
        the next process is launched. Returns True on success, False on failure.

        The fixed time.sleep(3.0) and time.sleep(1.0) that previously followed
        each launch call have been removed. The topic rate check is the gate.

        Returns:
            True  — all three processes running and verified
            False — one process failed to publish; all stopped, stack is clean
        """
        if self._is_running:
            log("[WARN] Stack already running — ignoring start request")
            return True

        self.session_count += 1
        log(f"[ACTIVATING] Session #{self.session_count} ({mode})")
        self._reader.beep_start()

        # ── Step 1: Point-LIO ─────────────────────────────────────────────────
        # Check if pointlio-standby.service already has Point-LIO running.
        # Fast 2-second check — if it is up it will be obvious immediately.
        # If pre-running: adopt it without launching a new process and without
        # owning it at stop() time. The standby service keeps it alive between
        # sessions so the next arm gets an already-warm SLAM map.
        # If not pre-running: launch it ourselves and own the lifecycle.
        if _pointlio_already_running():
            log("Point-LIO pre-running (standby service) — adopting, skipping launch")
            self._pointlio_external = True
        else:
            self._pointlio_external = False
            self._pointlio_proc = start_process("Point-LIO", _pointlio_cmd())
            log(f"Waiting for Point-LIO /aft_mapped_to_init "
                f"(timeout={SLAM_TOPIC_TIMEOUT_S:.0f}s, min={SLAM_MIN_HZ}Hz)...")
            if not _wait_for_topic_hz(
                "/aft_mapped_to_init", SLAM_MIN_HZ, SLAM_TOPIC_TIMEOUT_S
            ):
                return self._abort(
                    f"Point-LIO did not publish /aft_mapped_to_init within "
                    f"{SLAM_TOPIC_TIMEOUT_S:.0f}s — check LiDAR connection"
                )
        log("Point-LIO verified")

        # ── Step 2: SLAM bridge ───────────────────────────────────────────────
        if SLAM_BRIDGE_SCRIPT.exists():
            self._bridge_proc = start_process("SLAM bridge", _slam_bridge_cmd())
            log(f"Waiting for SLAM bridge /mavros/vision_pose/pose "
                f"(timeout={BRIDGE_TOPIC_TIMEOUT_S:.0f}s, min={BRIDGE_MIN_HZ}Hz)...")
            if not _wait_for_topic_hz(
                "/mavros/vision_pose/pose", BRIDGE_MIN_HZ, BRIDGE_TOPIC_TIMEOUT_S
            ):
                return self._abort(
                    f"SLAM bridge did not publish /mavros/vision_pose/pose within "
                    f"{BRIDGE_TOPIC_TIMEOUT_S:.0f}s — EKF2 has no position source"
                )
            log("SLAM bridge verified — EKF2 position source confirmed")
        else:
            log("[WARN] SLAM bridge script not found — vision fusion disabled")
            log(f"       Expected: {SLAM_BRIDGE_SCRIPT}")

        # ── Step 3: Bag recorder ──────────────────────────────────────────────
        # Bag starts after SLAM is verified so the first recorded frames
        # contain valid vision pose data rather than startup noise.
        self._bag_proc   = start_process("Bag recorder", _bag_record_cmd())
        self._is_running = True
        log(f"[ACTIVE] Session #{self.session_count} — scanning")
        return True

    def stop(self) -> None:
        if not self._is_running:
            return

        log(f"[DEACTIVATING] Session #{self.session_count}")
        self._reader.beep_stop()

        # Stop in reverse launch order — bag must close before Point-LIO
        # stops publishing so the final frames are written cleanly.
        stop_process("Bag recorder", self._bag_proc)
        self._bag_proc = None

        stop_process("SLAM bridge",  self._bridge_proc)
        self._bridge_proc = None

        # Only stop Point-LIO if we launched it ourselves.
        # If pointlio-standby.service owns it, leave it running so the
        # next session arm gets an already-warm map with zero wait.
        if not self._pointlio_external:
            stop_process("Point-LIO", self._pointlio_proc)
            self._pointlio_proc = None
        else:
            log("Point-LIO owned by standby service — leaving running for next session")
            self._pointlio_external = False

        self._is_running = False
        log(f"[WAITING] Session #{self.session_count} complete")

        self._postflight_fn()

    def check_health(self) -> None:
        """Warn if any managed process has exited unexpectedly.

        Point-LIO is only checked if we own it — if it is externally managed
        by pointlio-standby.service, systemd is responsible for monitoring it.
        """
        if not self._pointlio_external:
            if self._pointlio_proc and self._pointlio_proc.poll() is not None:
                log(f"[WARN] Point-LIO exited unexpectedly "
                    f"(exit code: {self._pointlio_proc.poll()})")
                self._pointlio_proc = None

        checks = [
            (self._bridge_proc, "SLAM bridge"),
            (self._bag_proc,    "Bag recorder"),
        ]
        for proc, name in checks:
            if proc is not None and proc.poll() is not None:
                log(f"[WARN] {name} exited unexpectedly (exit code: {proc.poll()})")
                if name == "SLAM bridge":  self._bridge_proc = None
                elif name == "Bag recorder": self._bag_proc  = None
