#!/usr/bin/env python3
"""test_ekf2_latency.py — Measure Pi→PX4 vision latency and recommend EKF2_EV_DELAY.

Manages the full stack internally: writes a bench_scan watchdog lock, launches
Point-LIO and slam_bridge, collects latency samples from /mavros/vision_pose/pose,
prints a report, and optionally applies EKF2_EV_DELAY via MAVLink.

MAVROS is NOT launched — it is provided by the drone-watchdog systemd service.
Launching a second instance would conflict on /dev/ttyAMA0.

Measurement Chain
-----------------
  LiDAR scan → Point-LIO SLAM → header.stamp   ← measured from here
      → slam_bridge publishes /mavros/vision_pose/pose
      → this script receives the message         ← to here
      → MAVROS serialises → serial 57600 baud → PX4 EKF2  ← +SERIAL_OFFSET_MS

Lock File Protocol
------------------
  Writes bench_scan lock before any subprocess is started.
  Watchdog detects this and yields entirely, preventing /dev/ttyUSB0
  port conflicts. Lock is cleared in all exit paths (normal + Ctrl+C + error).

Usage
-----
  source /opt/ros/jazzy/setup.bash
  source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash

  python3 test_ekf2_latency.py                  # 200 samples, prompt before applying
  python3 test_ekf2_latency.py --samples 500    # higher confidence
  python3 test_ekf2_latency.py --auto-apply     # apply param without prompting
  python3 test_ekf2_latency.py --no-apply       # report only, skip param set
  python3 test_ekf2_latency.py --skip-pointlio  # Point-LIO already running
  python3 test_ekf2_latency.py --skip-bridge    # slam_bridge already running

Safety
------
  Ctrl+C at any time stops all subprocesses cleanly and clears the lock.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped

# ── Constants ─────────────────────────────────────────────────────────────────

# Fixed offset added on top of measured ROS-side latency to account for:
#   - MAVROS serialisation + MAVLink framing         ~3 ms
#   - Serial TX at 57600 baud (PoseStamped ≈ 72 B)  ~10 ms
#   - PX4 EKF scheduling jitter                      ~2 ms
SERIAL_OFFSET_MS = 15
SAFETY_MARGIN_MS = 5

POINTLIO_TIMEOUT_S = 45   # Max seconds to wait for /cloud_registered
BRIDGE_STABILISE_S  = 5   # Fixed wait after slam_bridge starts before sampling
GRACEFUL_KILL_S    = 5

# ── Paths ─────────────────────────────────────────────────────────────────────

PROJECT_ROOT = Path(__file__).parent.parent

ROS_SETUP    = "/opt/ros/jazzy/setup.bash"
WS_SETUP     = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
LAUNCH_FILE  = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)
SLAM_BRIDGE  = PROJECT_ROOT / "flight" / "_slam_bridge.py"
LIDAR_PORT   = Path("/dev/ttyUSB0")
MISSION_LOCK = Path("/tmp/dronepi_mission.lock")

# ── QoS ───────────────────────────────────────────────────────────────────────

VISION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ── Logging ───────────────────────────────────────────────────────────────────

def log(msg: str):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


# ── Watchdog Lock ─────────────────────────────────────────────────────────────

def write_lock():
    MISSION_LOCK.write_text(json.dumps({
        "mode":       "bench_scan",
        "started_at": datetime.now().isoformat(),
        "owner":      "test_ekf2_latency",
    }))
    log("Watchdog lock written (bench_scan) — watchdog yielding")


def clear_lock():
    if MISSION_LOCK.exists():
        try:
            data = json.loads(MISSION_LOCK.read_text())
            if data.get("owner") == "test_ekf2_latency":
                MISSION_LOCK.unlink()
                log("Watchdog lock cleared — watchdog resuming normal operation")
        except Exception:
            MISSION_LOCK.unlink()


# ── Subprocess Helpers ────────────────────────────────────────────────────────

def start_proc(name: str, cmd: str) -> subprocess.Popen:
    log(f"Starting {name}...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    log(f"{name} started (PID {proc.pid})")
    return proc


def stop_proc(name: str, proc: subprocess.Popen):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=GRACEFUL_KILL_S)
        log(f"{name} stopped cleanly")
    except subprocess.TimeoutExpired:
        log(f"{name} did not exit — sending SIGKILL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    except ProcessLookupError:
        pass


# ── Readiness: wait for /cloud_registered ─────────────────────────────────────

def wait_for_cloud_registered(node: Node, timeout: float = POINTLIO_TIMEOUT_S) -> bool:
    """Block until /cloud_registered publishes at least one message.

    Reuses the caller's node to stay within the single rclpy context.
    Returns True on success, False on timeout (non-fatal — proceeds anyway).
    """
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import ReliabilityPolicy, HistoryPolicy

    received = threading.Event()
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    node.create_subscription(PointCloud2, "/cloud_registered",
                             lambda msg: received.set(), qos)

    t0       = time.time()
    last_log = -1
    while not received.is_set():
        elapsed = time.time() - t0
        if elapsed >= timeout:
            log(f"[WARN] /cloud_registered not seen after {timeout:.0f}s — proceeding anyway")
            return False
        tick = int(elapsed) // 5
        if tick != last_log and int(elapsed) > 0:
            log(f"Waiting for /cloud_registered... ({elapsed:.0f}s / {timeout:.0f}s)")
            last_log = tick
        rclpy.spin_once(node, timeout_sec=0.2)

    log(f"/cloud_registered confirmed ({time.time() - t0:.1f}s after launch)")
    return True


# ── Latency Probe ─────────────────────────────────────────────────────────────

def attach_latency_probe(node: Node, n_samples: int, delays_ms: list) -> None:
    """Attach a /mavros/vision_pose/pose subscription to an existing node.

    Samples are appended to delays_ms in the callback. Using the caller's
    node (rather than a separate Node subclass) ensures all subscriptions
    in this script are driven by the same spin_once call, which is the only
    safe pattern when multiple subscriptions share a single rclpy context.
    """
    def _cb(msg: PoseStamped):
        now_ns   = node.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        delay_ms = (now_ns - stamp_ns) / 1e6
        if 0.0 < delay_ms < 2_000.0:
            delays_ms.append(delay_ms)
            n = len(delays_ms)
            if n % 50 == 0 or n == n_samples:
                log(f"Collected {n}/{n_samples} samples (latest={delay_ms:.1f} ms)")

    node.create_subscription(PoseStamped, "/mavros/vision_pose/pose", _cb, VISION_QOS)
    log(f"Subscribed to /mavros/vision_pose/pose — collecting {n_samples} samples")


# ── Statistics + Report ───────────────────────────────────────────────────────

def print_report(delays_ms: list) -> int:
    """Print latency statistics. Returns the recommended EKF2_EV_DELAY in ms."""
    arr = np.array(delays_ms)

    mean_ms = float(np.mean(arr))
    med_ms  = float(np.median(arr))
    std_ms  = float(np.std(arr))
    p75_ms  = float(np.percentile(arr, 75))
    p90_ms  = float(np.percentile(arr, 90))
    p95_ms  = float(np.percentile(arr, 95))
    p99_ms  = float(np.percentile(arr, 99))
    max_ms  = float(np.max(arr))

    raw_rec     = p95_ms + SERIAL_OFFSET_MS + SAFETY_MARGIN_MS
    recommended = int(math.ceil(raw_rec / 5.0) * 5)

    print()
    print("=" * 54)
    print("  EKF2_EV_DELAY Latency Report")
    print(f"  Samples : {len(arr)}")
    print("=" * 54)
    print(f"  Mean    : {mean_ms:6.1f} ms")
    print(f"  Median  : {med_ms:6.1f} ms")
    print(f"  Std dev : {std_ms:6.1f} ms")
    print(f"  p75     : {p75_ms:6.1f} ms")
    print(f"  p90     : {p90_ms:6.1f} ms")
    print(f"  p95     : {p95_ms:6.1f} ms   ← basis for recommendation")
    print(f"  p99     : {p99_ms:6.1f} ms")
    print(f"  Max     : {max_ms:6.1f} ms")
    print("-" * 54)
    print(f"  p95 ({p95_ms:.1f}) + serial ({SERIAL_OFFSET_MS} ms)"
          f" + safety ({SAFETY_MARGIN_MS} ms) = {raw_rec:.1f} ms")
    print(f"  Rounded up to nearest 5 ms = {recommended} ms")
    print("=" * 54)
    print()
    print("  ╔══════════════════════════════════════════════╗")
    print(f"  ║  Set  EKF2_EV_DELAY = {recommended} ms  in QGC         ║")
    print("  ╚══════════════════════════════════════════════╝")
    print()

    if p95_ms > 100:
        log("[WARN] p95 > 100 ms — high latency. Check Point-LIO CPU load and ROS DDS settings.")
    elif p95_ms < 5:
        log("[WARN] p95 < 5 ms — suspiciously low. Verify slam_bridge is publishing.")
    if std_ms > 20:
        log("[WARN] High jitter (std > 20 ms) — DDS scheduling may be non-deterministic under load.")

    return recommended


# ── EKF2 Param Set via MAVROS ─────────────────────────────────────────────────

def apply_ekf2_ev_delay(node: Node, value_ms: int) -> bool:
    """Call /mavros/param/set to write EKF2_EV_DELAY to the FCU.

    Returns True on success. On failure, prints QGC fallback instructions.
    """
    try:
        from mavros_msgs.srv import ParamSet
        from mavros_msgs.msg import ParamValue
    except ImportError:
        log("[FAIL] mavros_msgs not importable — set EKF2_EV_DELAY manually in QGC.")
        return False

    client = node.create_client(ParamSet, "/mavros/param/set")
    log("Waiting for /mavros/param/set service...")
    if not client.wait_for_service(timeout_sec=10.0):
        log("[FAIL] /mavros/param/set not available. Is MAVROS running?")
        return False

    req = ParamSet.Request()
    req.param_id = "EKF2_EV_DELAY"
    req.value    = ParamValue(real=float(value_ms), integer=0)

    log(f"Setting EKF2_EV_DELAY = {value_ms} ms ...")
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if not future.done() or future.result() is None:
        log("[FAIL] param_set timed out or returned no result.")
        return False
    if not future.result().success:
        log("[FAIL] FCU rejected the parameter set.")
        return False

    log(f"[OK] EKF2_EV_DELAY confirmed = {value_ms} ms")
    log("     Power-cycle the Pixhawk for the change to take effect.")
    return True


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Measure SLAM→MAVROS latency and recommend EKF2_EV_DELAY.")
    parser.add_argument("--samples",      type=int,   default=200,
                        help="Number of pose samples to collect (default: 200)")
    parser.add_argument("--timeout",      type=float, default=120.0,
                        help="Max seconds to wait for samples once stack is up (default: 120)")
    parser.add_argument("--auto-apply",   action="store_true",
                        help="Apply EKF2_EV_DELAY via MAVLink without prompting")
    parser.add_argument("--no-apply",     action="store_true",
                        help="Skip param set entirely — report only")
    parser.add_argument("--skip-pointlio", action="store_true",
                        help="Skip Point-LIO launch (use if already running)")
    parser.add_argument("--skip-bridge",  action="store_true",
                        help="Skip slam_bridge launch (use if already running)")
    args = parser.parse_args()

    print()
    print("=" * 55)
    print("  EKF2_EV_DELAY Latency Test")
    print("=" * 55)
    print()

    # Validate prerequisites before writing the lock
    if not args.skip_pointlio and not Path(LAUNCH_FILE).exists():
        log(f"[FAIL] Point-LIO launch file not found: {LAUNCH_FILE}")
        sys.exit(1)
    if not args.skip_pointlio and not LIDAR_PORT.exists():
        log(f"[FAIL] LiDAR not detected at {LIDAR_PORT} — power on and plug in USB")
        sys.exit(1)
    if not args.skip_bridge and not SLAM_BRIDGE.exists():
        log(f"[FAIL] slam_bridge not found: {SLAM_BRIDGE}")
        sys.exit(1)

    write_lock()

    pointlio_proc  = None
    bridge_proc    = None

    def _cleanup():
        stop_proc("slam_bridge", bridge_proc)
        stop_proc("Point-LIO",   pointlio_proc)
        clear_lock()

    def _sighandler(signum, frame):
        print()
        log("Interrupted — stopping stack cleanly...")
        _cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _sighandler)
    signal.signal(signal.SIGTERM, _sighandler)

    try:
        # ── Launch Point-LIO ──────────────────────────────────────────────────
        if not args.skip_pointlio:
            pointlio_cmd = (
                f"source {ROS_SETUP} && source {WS_SETUP} && "
                f"ros2 launch {LAUNCH_FILE} rviz:=false port:={LIDAR_PORT}"
            )
            pointlio_proc = start_proc("Point-LIO", pointlio_cmd)

        # ── Init single ROS2 context ──────────────────────────────────────────
        # One rclpy.init() per process. All subscriptions and service calls
        # below reuse this same node to stay within that constraint.
        rclpy.init()
        node = Node("ekf2_latency_test")

        # ── Wait for Point-LIO output ─────────────────────────────────────────
        if not args.skip_pointlio:
            wait_for_cloud_registered(node)

        # ── Launch slam_bridge ────────────────────────────────────────────────
        if not args.skip_bridge:
            bridge_cmd = (
                f"source {ROS_SETUP} && source {WS_SETUP} && "
                f"python3 {SLAM_BRIDGE}"
            )
            bridge_proc = start_proc("slam_bridge", bridge_cmd)
            log(f"Allowing slam_bridge {BRIDGE_STABILISE_S}s to stabilise...")
            time.sleep(BRIDGE_STABILISE_S)

        # ── Collect latency samples ───────────────────────────────────────────
        # All subscriptions attach to the same node and are driven by the same
        # spin_once call below. This is the only correct pattern — spinning a
        # different node object would starve the /cloud_registered subscription
        # created above and cause a freeze.
        delays_ms = []
        attach_latency_probe(node, args.samples, delays_ms)

        t_start = time.time()
        try:
            while rclpy.ok() and len(delays_ms) < args.samples:
                rclpy.spin_once(node, timeout_sec=0.1)
                if time.time() - t_start > args.timeout:
                    log(f"[WARN] Timeout — only {len(delays_ms)} samples collected.")
                    break
        except KeyboardInterrupt:
            log("Interrupted during measurement.")

        # ── Report ────────────────────────────────────────────────────────────
        if len(delays_ms) < 20:
            log(f"[FAIL] Not enough samples ({len(delays_ms)}).")
            log("       Is slam_bridge publishing?")
            log("       Check: ros2 topic hz /mavros/vision_pose/pose")
            sys.exit(1)

        recommended = print_report(delays_ms)

        # ── Apply EKF2_EV_DELAY ───────────────────────────────────────────────
        if not args.no_apply:
            if args.auto_apply:
                apply_ekf2_ev_delay(node, recommended)
            else:
                print()
                try:
                    ans = input(
                        f"  Apply EKF2_EV_DELAY = {recommended} ms via MAVLink now? [y/N]: "
                    ).strip().lower()
                except (EOFError, KeyboardInterrupt):
                    ans = ""
                if ans in ("y", "yes"):
                    apply_ekf2_ev_delay(node, recommended)
                else:
                    log("Param set skipped. Apply manually in QGC.")
                    log(f"  EKF2_EV_DELAY = {recommended}")

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    finally:
        _cleanup()


if __name__ == "__main__":
    main()
