#!/usr/bin/env python3
"""
drone_watchdog.py — Flight stack supervisor for DronePi.

Monitors /mavros/state and manages the Point-LIO SLAM node and ROS 2 bag
recorder based on the drone's armed and flight mode state.

STATE MACHINE
-------------

  WAITING
      Polls /mavros/state every second.
      Condition to advance: armed=True AND mode=OFFBOARD.
      When condition met -> launches Point-LIO + bag recorder -> ACTIVE.

  ACTIVE
      Monitors /mavros/state continuously.
      Condition to revert: armed=False OR mode != OFFBOARD.
      When condition lost -> stops Point-LIO + bag recorder -> WAITING.
      A new session starts automatically if the pilot re-arms in OFFBOARD.

SAFETY DESIGN
-------------
  - This script NEVER arms the drone or switches flight modes.
    Arming and mode selection are exclusively the pilot's responsibility
    via the RC transmitter. The watchdog is observe-only on those topics.
  - Point-LIO is killed with SIGINT (clean shutdown) first, then SIGKILL
    after a 5-second grace period if it does not exit.
  - The bag recorder is stopped with SIGINT so the MCAP file is closed
    correctly. An unclean bag close corrupts the recording.
  - All state transitions are logged to stdout (captured by journalctl).

DEPENDENCIES
------------
  rclpy, mavros_msgs  (ROS 2 Jazzy)
  Standard library only for process management.

DEPLOYMENT
----------
  Managed by drone-watchdog.service (systemd).
  See setup_flight_services.sh for installation.

  Manual run (for testing):
      source /opt/ros/jazzy/setup.bash
      source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
      python3 ~/unitree_lidar_project/unitree_drone_mapper/flight/drone_watchdog.py

  View logs:
      sudo journalctl -u drone-watchdog -f
"""

import os
import re
import signal
import subprocess
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

# ── config ────────────────────────────────────────────────────────────────────

ROS_SETUP       = "/opt/ros/jazzy/setup.bash"
WS_SETUP        = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
ROSBAG_DIR      = "/mnt/ssd/rosbags"
# Path to run_postflight.py -- auto-triggered after each session.
# Must live in the same directory as this script.
POSTFLIGHT_SCRIPT = Path(__file__).parent / "run_postflight.py"

# Activation mode.
# REQUIRE_OFFBOARD = True  -- field use: armed AND mode=OFFBOARD required.
# REQUIRE_OFFBOARD = False -- bench use: armed only, no setpoint stream needed.
# Set to False for bench testing, True before any real flight.
REQUIRE_OFFBOARD = True

POLL_HZ         = 2          # state poll rate while WAITING (Hz)
MONITOR_HZ      = 5          # state monitor rate while ACTIVE (Hz)
GRACEFUL_KILL_S = 5          # seconds before SIGKILL after SIGINT
MAVROS_WAIT_S   = 15         # seconds to wait for MAVROS to come up after boot

# Topics recorded in every bag session.
# /cloud_registered  — Point-LIO accumulated map (primary scan data)
# /aft_mapped_to_init — SLAM pose output
# /unilidar/imu      — raw IMU for post-processing
# /mavros/state      — flight mode and arm state for session metadata
BAG_TOPICS = [
    "/cloud_registered",
    "/aft_mapped_to_init",
    "/unilidar/imu",
    "/mavros/state",
    "/mavros/local_position/pose",
    "/mavros/global_position/global",
]

# Point-LIO launch command.
# Sources both the base ROS 2 Jazzy setup and the project workspace overlay.
# rviz:=false -- no display available on headless Pi.
# port matches the Unitree 4D L1 USB connection.
LAUNCH_FILE = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)
POINTLIO_CMD = (
    f"source {ROS_SETUP} && "
    f"source {WS_SETUP} && "
    f"ros2 launch {LAUNCH_FILE} "
    "rviz:=false port:=/dev/ttyUSB0"
)

# ── logging ───────────────────────────────────────────────────────────────────

def log(msg: str):
    """Timestamped log line -- captured by journalctl."""
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def log_state(state: str, detail: str = ""):
    sep = f"  {detail}" if detail else ""
    log(f"[{state}]{sep}")

# ── process management ────────────────────────────────────────────────────────

def start_process(name: str, cmd: str) -> subprocess.Popen:
    """
    Launch a shell command in a new process group.

    Using a process group (setsid) ensures that when we send SIGINT or SIGKILL
    to stop the process, all child processes spawned by the shell (e.g. the
    actual ros2 node binaries launched by the launch file) are also terminated.
    Without this, orphan ROS 2 nodes can persist and block the next launch.
    """
    log(f"Starting {name}...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    log(f"{name} started (PID {proc.pid})")
    return proc


def stop_process(name: str, proc: subprocess.Popen):
    """
    Stop a process cleanly.

    Sends SIGINT first to allow the process to flush buffers and close files
    (critical for the bag recorder -- an unclean exit corrupts the MCAP file).
    Waits GRACEFUL_KILL_S seconds, then sends SIGKILL if still running.
    """
    if proc is None or proc.poll() is not None:
        return  # already dead

    pgid = None
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return

    log(f"Stopping {name} (PID {proc.pid}, PGID {pgid})...")

    try:
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return

    try:
        proc.wait(timeout=GRACEFUL_KILL_S)
        log(f"{name} stopped cleanly.")
    except subprocess.TimeoutExpired:
        log(f"{name} did not exit in {GRACEFUL_KILL_S}s -- sending SIGKILL.")
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()
        log(f"{name} killed.")

# ── bag recording ─────────────────────────────────────────────────────────────

def bag_record_cmd() -> str:
    """
    Build the ros2 bag record command for the current session.

    Directory name follows the scan_YYYYMMDD_HHMMSS convention expected
    by /api/flights in serve.py and postprocess_mesh.py.
    """
    ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = f"{ROSBAG_DIR}/scan_{ts}"
    topics = " ".join(BAG_TOPICS)
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"ros2 bag record -o {out} {topics}"
    )

# ── MAVROS state reader ───────────────────────────────────────────────────────

class MavrosStateReader:
    """
    Lightweight ROS 2 subscriber that reads /mavros/state.

    Runs rclpy.spin in a background thread so the main watchdog loop
    can poll state without blocking. Uses BEST_EFFORT QoS to match
    MAVROS's publisher profile -- RELIABLE would silently receive nothing.
    """

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from mavros_msgs.msg import State

        self._rclpy  = rclpy
        self._armed  = False
        self._mode   = ""
        self._connected = False
        self._lock   = threading.Lock()

        rclpy.init()
        self._node = Node("drone_watchdog")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._node.create_subscription(
            State, "/mavros/state", self._cb, qos)


        self._thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._thread.start()
        log("MAVROS state subscriber started.")

    def _cb(self, msg):
        with self._lock:
            self._armed     = msg.armed
            self._mode      = msg.mode
            self._connected = msg.connected


    @property
    def armed(self) -> bool:
        with self._lock:
            return self._armed

    @property
    def mode(self) -> str:
        with self._lock:
            return self._mode

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    def flight_conditions_met(self) -> bool:
        """
        Return True when activation conditions are met.

        FIELD mode (REQUIRE_OFFBOARD=True, default):
            armed=True AND mode=OFFBOARD
            Use for actual flights -- OFFBOARD requires a live setpoint
            stream which is only present during a real flight operation.

        BENCH mode (REQUIRE_OFFBOARD=False):
            armed=True only
            Use for bench testing the watchdog activation without needing
            a setpoint stream. Set REQUIRE_OFFBOARD=False below to enable.
        """
        with self._lock:
            if REQUIRE_OFFBOARD:
                return self._armed and self._mode == "OFFBOARD"
            else:
                return self._armed

    def shutdown(self):
        self._rclpy.shutdown()

# ── post-flight trigger ──────────────────────────────────────────────────────

def trigger_postflight() -> subprocess.Popen | None:
    """
    Launch run_postflight.py in a background subprocess after a session ends.

    Returns the Popen handle so the watchdog can monitor completion.
    While this process is running, flight_conditions_met() will block
    new session activation to prevent nodes from piling up.

    Returns None if the script is missing or failed to launch.
    """
    if not POSTFLIGHT_SCRIPT.exists():
        log(f"[WARN] run_postflight.py not found at {POSTFLIGHT_SCRIPT}")
        log("       Post-flight processing skipped -- run manually.")
        return None

    log("Launching post-flight pipeline in background...")
    log("New session will be blocked until processing completes.")
    try:
        proc = subprocess.Popen(
            [sys.executable, str(POSTFLIGHT_SCRIPT), "--auto", "--skip-wait"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        log(f"Post-flight pipeline started (PID {proc.pid}). "
            "Viewer will auto-load when complete.")
        return proc
    except Exception as e:
        log(f"[WARN] Could not launch post-flight pipeline: {e}")
        log("       Run manually: python3 run_postflight.py")
        return None


# ── main watchdog loop ────────────────────────────────────────────────────────

def wait_for_mavros(reader: MavrosStateReader):
    """
    Block until MAVROS reports a connected FCU.

    Called once at startup. MAVROS may take 10-20 seconds to initialize
    after the service starts. The watchdog waits patiently rather than
    entering the polling loop with stale disconnected state.
    """
    log(f"Waiting for MAVROS FCU connection (up to {MAVROS_WAIT_S}s)...")
    deadline = time.time() + MAVROS_WAIT_S
    while time.time() < deadline:
        if reader.connected:
            log(f"FCU connected. Mode: {reader.mode}  Armed: {reader.armed}")
            return
        time.sleep(1.0)
    log("[WARN] FCU not connected after wait -- continuing anyway.")
    log("       Check Pixhawk USB cable and /dev/ttyACM0 permissions.")


def main():
    log("=" * 50)
    log("DronePi Flight Stack Watchdog")
    if REQUIRE_OFFBOARD:
        log("Condition to activate: armed=True AND mode=OFFBOARD  (field mode)")
    else:
        log("Condition to activate: armed=True only  (bench mode -- REQUIRE_OFFBOARD=False)")
    log("This script never arms the drone or changes flight mode.")
    log("=" * 50)

    # Ensure rosbag directory exists
    Path(ROSBAG_DIR).mkdir(parents=True, exist_ok=True)

    # Import ROS 2 -- will fail if setup.bash was not sourced
    try:
        reader = MavrosStateReader()
    except Exception as e:
        log(f"[FAIL] Could not start ROS 2 node: {e}")
        log("       Ensure ROS 2 Jazzy is sourced and mavros_msgs is installed.")
        sys.exit(1)

    wait_for_mavros(reader)

    pointlio_proc   = None
    bag_proc        = None
    postflight_proc = None   # tracked to block new sessions during processing
    session_count   = 0

    try:
        while True:

            # ── WAITING state ─────────────────────────────────────────────────
            # Block new session if post-flight processing is still running.
            # This prevents Point-LIO and bag nodes from piling up when a
            # new arm+OFFBOARD cycle starts before the previous mesh is done.
            if postflight_proc is not None:
                rc = postflight_proc.poll()
                if rc is None:
                    # Still running -- block activation regardless of arm state
                    log_state("WAITING",
                              f"armed={reader.armed}  mode={reader.mode or '?'}  "
                              f"[POST-PROCESSING -- new session blocked]")
                    time.sleep(1.0 / POLL_HZ)
                    continue
                else:
                    # Processing finished
                    if rc == 0:
                        log("Post-flight processing complete. New session allowed.")
                    else:
                        log(f"[WARN] Post-flight processing exited with code {rc}.")
                        log("       Check: python3 run_postflight.py --latest")
                    postflight_proc = None

            if not reader.flight_conditions_met():
                log_state("WAITING",
                          f"armed={reader.armed}  mode={reader.mode or '?'}")
                time.sleep(1.0 / POLL_HZ)
                continue

            # ── Conditions met -- launch flight stack ─────────────────────────
            session_count += 1
            log_state("ACTIVATING",
                      f"Session #{session_count}  "
                      f"armed={reader.armed}  mode={reader.mode}")

            pointlio_proc = start_process("Point-LIO", POINTLIO_CMD)
            # Give Point-LIO 3 seconds to initialize before starting the bag
            # so the first frames are not lost during node startup.
            time.sleep(3.0)
            bag_proc = start_process("Bag recorder", bag_record_cmd())

            log_state("ACTIVE",
                      f"Session #{session_count} running -- "
                      f"monitoring for disarm or mode change")

            # ── ACTIVE state -- monitor continuously ──────────────────────────
            while True:
                time.sleep(1.0 / MONITOR_HZ)

                # Check if either subprocess died unexpectedly
                if pointlio_proc and pointlio_proc.poll() is not None:
                    log("[WARN] Point-LIO exited unexpectedly.")
                    pointlio_proc = None

                if bag_proc and bag_proc.poll() is not None:
                    log("[WARN] Bag recorder exited unexpectedly.")
                    bag_proc = None

                # Check flight conditions
                if not reader.flight_conditions_met():
                    log_state("DEACTIVATING",
                              f"armed={reader.armed}  mode={reader.mode}")
                    break

            # ── Conditions lost -- stop flight stack ──────────────────────────
            stop_process("Bag recorder", bag_proc)
            bag_proc = None
            stop_process("Point-LIO", pointlio_proc)
            pointlio_proc = None

            # Trigger post-flight processing in background.
            # Store the handle -- watchdog blocks new sessions until done.
            postflight_proc = trigger_postflight()

            log_state("WAITING",
                      f"Session #{session_count} complete. "
                      f"Re-arm in OFFBOARD to start new session.")

    except KeyboardInterrupt:
        log("Watchdog interrupted -- shutting down.")
    finally:
        stop_process("Bag recorder", bag_proc)
        stop_process("Point-LIO", pointlio_proc)
        reader.shutdown()
        log("Watchdog stopped.")


if __name__ == "__main__":
    main()
