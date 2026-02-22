#!/usr/bin/env python3
"""
Autonomous Drone Mapping System — Single Entry Point

Manages the full lifecycle:
  - python3 main.py           — Full end-to-end (fly + process)
  - python3 main.py fly       — Flight + data collection only
  - python3 main.py process   — Post-processing only (--session or --latest)
  - python3 main.py check     — System health check only
  - python3 main.py scan      — Handheld LiDAR scan (no flight)

Examples:
  python3 main.py fly
  python3 main.py process --latest
  python3 main.py process --session 20260222_143000
  python3 main.py check
"""

import sys
import os
import time
import signal
import argparse
import yaml
from pathlib import Path

# Add project root to path
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)

from utils.logger import setup_logger, get_session_id
from utils.file_io import create_session_dirs

logger = setup_logger("main")


def load_config(config_path: str = None) -> dict:
    """Load system configuration from YAML."""
    if config_path is None:
        config_path = os.path.join(PROJECT_ROOT, "config.yaml")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    return config


# ──────────────────────────────────────────────
# PHASE: System Health Check
# ──────────────────────────────────────────────

def run_check(config: dict) -> bool:
    """Run system health check — verify sensors, PX4, storage.

    Returns:
        True if all checks pass.
    """
    logger.info("=" * 60)
    logger.info("SYSTEM HEALTH CHECK")
    logger.info("=" * 60)

    all_ok = True

    # Check LiDAR device
    lidar_port = config.get("lidar", {}).get("device", "/dev/ttyUSB0")
    if os.path.exists(lidar_port):
        logger.info(f"[OK]  LiDAR device: {lidar_port}")
    else:
        logger.error(f"[FAIL] LiDAR device not found: {lidar_port}")
        all_ok = False

    # Check PX4 device
    px4_conn = config.get("px4", {}).get("connection", "/dev/ttyACM0:921600")
    px4_port = px4_conn.split(":")[0]
    if os.path.exists(px4_port):
        logger.info(f"[OK]  PX4 device: {px4_port}")
    else:
        logger.warning(f"[WARN] PX4 device not found: {px4_port} (OK for scan-only mode)")

    # Check camera
    video_dev = config.get("camera", {}).get("video_device", "/dev/video0")
    if os.path.exists(video_dev):
        logger.info(f"[OK]  Camera device: {video_dev}")
    else:
        logger.warning(f"[WARN] Camera device not found: {video_dev}")

    # Check storage
    data_dir = os.path.expanduser(
        config.get("system", {}).get("data_dir", "~/unitree_drone_mapper/data/flights")
    )
    Path(data_dir).mkdir(parents=True, exist_ok=True)
    # Check available space
    stat = os.statvfs(data_dir)
    free_gb = (stat.f_bavail * stat.f_frsize) / (1024 ** 3)
    if free_gb > 1.0:
        logger.info(f"[OK]  Storage: {free_gb:.1f} GB free")
    else:
        logger.error(f"[FAIL] Low storage: {free_gb:.1f} GB free")
        all_ok = False

    # Check ROS 2
    import shutil
    if shutil.which("ros2"):
        logger.info("[OK]  ROS 2 available")
    else:
        logger.error("[FAIL] ROS 2 not found in PATH")
        all_ok = False

    # Check Open3D
    try:
        import open3d
        logger.info(f"[OK]  Open3D {open3d.__version__}")
    except ImportError:
        logger.warning("[WARN] Open3D not installed (needed for post-processing)")

    logger.info("=" * 60)
    if all_ok:
        logger.info("All critical checks passed!")
    else:
        logger.error("Some checks failed — review above")
    logger.info("=" * 60)

    return all_ok


# ──────────────────────────────────────────────
# PHASE: Handheld LiDAR Scan (no flight)
# ──────────────────────────────────────────────

def run_scan(config: dict, session_id: str):
    """Run a handheld LiDAR scan session — no flight controller needed.

    Starts LiDAR driver + Point-LIO, accumulates point cloud,
    and processes on Ctrl+C.
    """
    import rclpy

    data_dir = os.path.expanduser(
        config.get("system", {}).get("data_dir", "~/unitree_drone_mapper/data/flights")
    )
    dirs = create_session_dirs(data_dir, session_id)

    logger.info("=" * 60)
    logger.info(f"HANDHELD SCAN — Session: {session_id}")
    logger.info("=" * 60)
    logger.info("Walk around with the LiDAR to scan your area.")
    logger.info("Press Ctrl+C when done to process and save.")
    logger.info("")

    # Start LiDAR driver
    from drivers.lidar_driver import LidarDriver
    lidar = LidarDriver(config.get("lidar", {}))
    if not lidar.start():
        logger.error("Failed to start LiDAR driver")
        return

    # Start Point-LIO
    from slam.point_lio_runner import PointLIORunner
    slam = PointLIORunner(config.get("slam", {}))
    time.sleep(2)  # Wait for LiDAR to initialize
    if not slam.start():
        logger.error("Failed to start Point-LIO")
        lidar.stop()
        return

    # Start cloud accumulator
    rclpy.init()
    from slam.cloud_accumulator import CloudAccumulator
    from slam.trajectory_logger import TrajectoryLogger

    accumulator = CloudAccumulator(dirs["processed"], config.get("slam", {}))
    traj_logger = TrajectoryLogger(dirs["metadata"], config.get("slam", {}))

    # Spin until Ctrl+C
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(accumulator)
    executor.add_node(traj_logger)

    shutdown = False

    def signal_handler(sig, frame):
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while not shutdown:
            executor.spin_once(timeout_sec=0.1)
    finally:
        logger.info("")
        logger.info("Stopping scan...")

        # Save data
        accumulator.save("cloud_raw.pcd")
        traj_logger.save()

        # Cleanup
        executor.shutdown()
        accumulator.destroy_node()
        traj_logger.destroy_node()
        rclpy.shutdown()

        slam.stop()
        lidar.stop()

        logger.info(f"Scan data saved to: {dirs['session']}")
        logger.info(f"Points collected: {accumulator.get_point_count():,}")
        logger.info(f"Poses recorded: {traj_logger.pose_count}")

        # Offer to process
        logger.info("")
        logger.info("To process this scan:")
        logger.info(f"  python3 main.py process --session {session_id}")


# ──────────────────────────────────────────────
# PHASE: Flight
# ──────────────────────────────────────────────

def run_flight(config: dict, session_id: str, mission_file: str = None):
    """Run flight mission with data collection.

    Starts all sensors, runs SLAM, executes mission waypoints,
    triggers camera captures, and saves everything.
    """
    import rclpy

    data_dir = os.path.expanduser(
        config.get("system", {}).get("data_dir", "~/unitree_drone_mapper/data/flights")
    )
    dirs = create_session_dirs(data_dir, session_id)

    logger.info("=" * 60)
    logger.info(f"FLIGHT MODE — Session: {session_id}")
    logger.info("=" * 60)

    # Start LiDAR
    from drivers.lidar_driver import LidarDriver
    lidar = LidarDriver(config.get("lidar", {}))
    lidar.start()

    # Start Point-LIO
    from slam.point_lio_runner import PointLIORunner
    slam = PointLIORunner(config.get("slam", {}))
    time.sleep(2)
    slam.start()

    # Initialize ROS 2
    rclpy.init()

    # Start SLAM logging
    from slam.cloud_accumulator import CloudAccumulator
    from slam.trajectory_logger import TrajectoryLogger
    accumulator = CloudAccumulator(dirs["processed"], config.get("slam", {}))
    traj_logger = TrajectoryLogger(dirs["metadata"], config.get("slam", {}))

    # Initialize PX4 interface
    from flight.px4_interface import PX4Interface
    px4 = PX4Interface(config)

    # Initialize collision avoidance
    from flight.collision_avoidance import CollisionAvoidance
    collision = CollisionAvoidance(config)

    # Initialize camera trigger
    from drivers.insta360_driver import Insta360Driver
    from flight.camera_trigger import CameraTrigger
    camera = Insta360Driver(config.get("camera", {}))
    trigger = CameraTrigger(camera, config)
    trigger.setup(dirs["images"], dirs["metadata"])

    # Load mission
    from flight.mission_executor import MissionExecutor
    mission = MissionExecutor(px4, config)
    if mission_file:
        mission.load_mission(mission_file)
    else:
        # Default grid survey
        default_mission = os.path.join(
            PROJECT_ROOT, "config", "mission_templates", "grid_survey.yaml"
        )
        if os.path.exists(default_mission):
            mission.load_mission(default_mission)

    # Trigger camera at each waypoint
    mission.set_waypoint_callback(
        lambda idx, wp: trigger.trigger_at_waypoint(idx, wp)
    )

    # Setup executor
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    for node in [accumulator, traj_logger, px4, collision]:
        executor.add_node(node)

    shutdown = False

    def signal_handler(sig, frame):
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Start mission
        mission.start()

        while not shutdown:
            executor.spin_once(timeout_sec=0.05)
            mission.update()

            # Safety checks
            if px4.check_battery_rtl():
                logger.warning("Low battery — returning to launch!")
                mission.abort()
                break

    finally:
        logger.info("Shutting down flight...")

        # Save all data
        accumulator.save("cloud_raw.pcd")
        traj_logger.save()
        trigger.save_metadata()

        # Cleanup
        executor.shutdown()
        for node in [accumulator, traj_logger, px4, collision]:
            node.destroy_node()
        rclpy.shutdown()

        slam.stop()
        lidar.stop()

        logger.info(f"Flight data saved to: {dirs['session']}")


# ──────────────────────────────────────────────
# PHASE: Post-Processing
# ──────────────────────────────────────────────

def run_process(config: dict, session_dir: str):
    """Run the post-processing mapping pipeline on a session."""
    from mapping.pipeline import run_pipeline

    logger.info("=" * 60)
    logger.info(f"POST-PROCESSING: {session_dir}")
    logger.info("=" * 60)

    success = run_pipeline(session_dir, config)

    if success:
        logger.info("Processing complete!")
    else:
        logger.error("Processing failed")


def find_latest_session(config: dict) -> str:
    """Find the most recent flight session directory."""
    data_dir = os.path.expanduser(
        config.get("system", {}).get("data_dir", "~/unitree_drone_mapper/data/flights")
    )

    if not os.path.exists(data_dir):
        raise FileNotFoundError(f"No data directory: {data_dir}")

    sessions = sorted(Path(data_dir).iterdir(), reverse=True)
    if not sessions:
        raise FileNotFoundError("No sessions found")

    return str(sessions[0])


# ──────────────────────────────────────────────
# CLI Entry Point
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Autonomous Drone Mapping System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Commands:
  (default)  Full end-to-end: fly + process
  fly        Flight + data collection only
  scan       Handheld LiDAR scan (no flight controller)
  process    Post-processing only
  check      System health check

Examples:
  python3 main.py check
  python3 main.py scan
  python3 main.py fly
  python3 main.py fly --mission config/mission_templates/grid_survey.yaml
  python3 main.py process --latest
  python3 main.py process --session 20260222_143000
        """,
    )

    parser.add_argument(
        "command",
        nargs="?",
        default="full",
        choices=["fly", "scan", "process", "check", "full"],
        help="Command to run",
    )
    parser.add_argument("--config", default=None, help="Path to config.yaml")
    parser.add_argument("--session", default=None, help="Session ID for processing")
    parser.add_argument("--latest", action="store_true", help="Process most recent session")
    parser.add_argument("--mission", default=None, help="Mission YAML file for flight")

    args = parser.parse_args()

    # Load config
    config = load_config(args.config)
    log_level = config.get("system", {}).get("log_level", "INFO")
    logger.setLevel(log_level)

    session_id = get_session_id()

    logger.info("Autonomous Drone Mapping System")
    logger.info(f"Command: {args.command}")
    logger.info("")

    if args.command == "check":
        run_check(config)

    elif args.command == "scan":
        run_scan(config, session_id)

    elif args.command == "fly":
        run_flight(config, session_id, args.mission)

    elif args.command == "process":
        if args.latest:
            session_dir = find_latest_session(config)
        elif args.session:
            data_dir = os.path.expanduser(
                config.get("system", {}).get("data_dir", "~/unitree_drone_mapper/data/flights")
            )
            session_dir = os.path.join(data_dir, args.session)
        else:
            logger.error("Specify --session <ID> or --latest")
            sys.exit(1)

        if not os.path.exists(session_dir):
            logger.error(f"Session not found: {session_dir}")
            sys.exit(1)

        run_process(config, session_dir)

    elif args.command == "full":
        # Full end-to-end
        if not run_check(config):
            logger.error("Health check failed — fix issues before flying")
            sys.exit(1)

        run_flight(config, session_id, args.mission)

        data_dir = os.path.expanduser(
            config.get("system", {}).get("data_dir", "~/unitree_drone_mapper/data/flights")
        )
        session_dir = os.path.join(data_dir, session_id)
        run_process(config, session_dir)


if __name__ == "__main__":
    main()
