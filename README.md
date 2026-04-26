# DronePi — Autonomous LiDAR Mapping Hexacopter

**UPRM Capstone Project — May 2026 Demonstration**
**Repository:** `Xavier-Nieves/unitree_lidar_project`

---

## Overview

DronePi is an autonomous hexacopter that performs GPS-surveyed flight paths,
captures LiDAR point clouds in real time, and produces a textured 3D mesh
viewable in a browser within seconds of landing — with zero manual steps
between disarm and mesh display.

**Demonstration goal:** Fully autonomous GPS survey flight → textured 3D
mesh auto-loads in browser at `http://10.42.0.1:8080/meshview.html`.

---

## Hardware

| Component | Part |
|---|---|
| Frame | Tarot 810 hexacopter |
| Companion computer | Raspberry Pi 5 (16 GB) |
| Flight controller | Pixhawk 6X |
| LiDAR | Unitree L1 |
| Camera | IMX477 ArduCam |
| AI accelerator | Hailo-8L (PCIe HAT+) |
| Storage | SanDisk USB 3.0 SSD, mounted at `/mnt/ssd` |

---

## Software Stack

| Layer | Technology |
|---|---|
| OS | Ubuntu 24.04 LTS (arm64) |
| Robotics middleware | ROS 2 Jazzy |
| Flight controller bridge | MAVROS |
| SLAM | Point-LIO (point_lio_ros2) |
| Flight controller firmware | PX4 |
| AI inference | HailoRT 4.23.0 (`hailo_inference_env` venv) |
| Python environment | Miniforge Conda (`dronepi` env) |

---

## Repository Structure

```
unitree_lidar_project/
|
|-- main.py                        Top-level orchestrator (sole entry point)
|-- preflight_check.sh             Arming gate — 17 sections, 63 checks
|-- logs/flight_history.log        Persistent flight log
|
|-- flight/
|   |-- drone_watchdog.py          Flight stack supervisor
|   |-- _slam_bridge.py            Point-LIO -> MAVROS EKF2 bridge
|   |-- _flow_bridge.py            Hailo optical flow -> MAVROS velocity bridge
|   |-- collision_monitor.py       3-zone obstacle + AGL estimator
|   |-- gap_detector.py            LiDAR coverage gap detector
|   |-- gps_reader.py              GPS secondary layer (MAVROS-backed)
|   |-- camera_capture.py          IMX477 capture node (Picamera2)
|   |-- flight_controller.py       Thread-safe MAVROS primitives
|   +-- watchdog_core/             Modular watchdog subpackage
|       |-- buzzer.py              Audio feedback tunes
|       |-- mavros_reader.py       ROS 2 MAVROS interface
|       |-- flight_stack.py        Subprocess manager
|       |-- postflight.py          Monitored post-flight trigger
|       +-- led_controller.py      RGB LED state machine
|
|-- hailo/
|   |-- hailo_device.py            VDevice lifecycle wrapper
|   |-- hailo_optical_flow.py      SCDepthV3 velocity estimator
|   |-- hailo_ground_class.py      YOLOv8m ground surface classifier
|   +-- hailo_flight_node.py       ROS 2 node (hailo_inference_env)
|
|-- core/
|   |-- camera_model.py            Intrinsics + extrinsics wrapper
|   +-- texture_projector.py       Per-vertex color accumulator
|
|-- tests/
|   |-- test_offboard_flight.py    OFFBOARD hover test
|   |-- test_offboard_gps.py       GPS waypoint mission
|   |-- test_manual_scan.py        Manual RC + LiDAR scan
|   |-- test_slam_bridge_flight.py SLAM bridge validation
|   |-- collision_zone_test.py     Zone characterization
|   +-- test_mesh_algorithms.py    Algorithm comparison
|
|-- utils/
|   |-- postprocess_mesh.py        7-stage mesh pipeline orchestrator
|   |-- run_postflight.py          Post-flight trigger
|   |-- flight_logger.py           Flight history log
|   |-- bench_test.py              MAVROS 8-point bench test
|   |-- foxglove_events.py         Foxglove Events API client
|   |-- foxglove_cloud_rotation.py Foxglove cloud recording rotation
|   +-- mesh_tools/
|       |-- bag_reader.py
|       |-- mls_smoother.py
|       |-- ground_classifier.py
|       |-- dtm_builder.py
|       |-- dsm_builder.py
|       |-- mesh_merger.py
|       +-- publisher.py
|
|-- config/
|   |-- px4_config.yaml
|   |-- px4_pluginlists.yaml
|   +-- camera_calibration.yaml
|
|-- rpi_server/
|   |-- serve.py                   HTTP server (port 8080)
|   |-- meshview.html              Pi-hosted 3D browser viewer
|   +-- local_test.html            Offline laptop viewer
|
|-- RPI5/ros2_ws/src/
|   |-- point_lio_ros2/
|   |-- unitree_lidar_ros2/        (symlink — required for colcon discovery)
|   +-- unilidar_sdk/
|
|-- setup_scripts/
|   |-- setup.sh                   Master setup script (run once as root)
|   |-- preflight_check.sh         Preflight gate (not an install step)
|   |-- setup_hotspot_service.sh
|   |-- setup_mesh_server_service.sh
|   |-- setup_foxglove_service.sh
|   |-- setup_mavros_service.sh
|   |-- setup_watchdog_service.sh
|   +-- setup_main_service.sh
|
+-- docs/
    |-- architecture.md            System architecture and design principles
    |-- hardware_setup.md          Hardware assembly and known issues
    |-- pipeline_overview.md       Data flow and layer reference
    |-- ekf_slam_fusion.md         EKF2 and sensor fusion reference
    |-- post_processing.md         Mesh pipeline reference
    |-- flight_procedure.md        Flight operations manual
    |-- calibration.md             Camera, LiDAR-camera, and PX4 calibration
    |-- test_procedure.md          9-phase bench-to-field gate sequence
    +-- texture_pipeline.md        Texture mapping architecture and status
```

---

## Quick Start — Fresh Pi Setup

Run once as root. The master script handles all dependencies, environments,
udev rules, ROS 2 workspace build, and systemd services.

```bash
git clone https://github.com/Xavier-Nieves/unitree_lidar_project.git \
  ~/unitree_lidar_project
cd ~/unitree_lidar_project/setup_scripts
sudo bash setup.sh
```

After the script completes:

```bash
# 1. Log out and back in (group membership — dialout, hailo)
# 2. Mount and verify SSD
lsblk && df -h /mnt/ssd

# 3. Verify udev symlinks
ls -l /dev/ttyPixhawk

# 4. Reboot for services
sudo reboot

# 5. After reboot — verify all services
systemctl status mavros drone-watchdog drone-mesh-server foxglove-bridge

# 6. Run bench test
python3 utils/bench_test.py

# 7. Open viewer
# Connect to dronepi-ap Wi-Fi, then open:
# http://10.42.0.1:8080/meshview.html
```

---

## Pre-Flight Gate

The preflight check must pass with zero FAIL before every flight.

```bash
sudo bash setup_scripts/preflight_check.sh
```

17 sections, 63 automated checks covering: power, SSD, Pixhawk serial,
Hailo-8 PCIe, LiDAR, GPS, ROS 2 workspace, MAVROS topics, systemd services,
Python environments, and output directories.

Pass criteria: `STATUS: ALL SYSTEMS GO`

---

## Network Reference

| Interface | Address | Purpose |
|---|---|---|
| Hotspot (field) | `10.42.0.1` | Primary field access — no router needed |
| Lab router | `192.168.2.2` | Lab development access |
| Mesh viewer | `http://10.42.0.1:8080/meshview.html` | 3D mesh browser |
| Flight API | `http://10.42.0.1:8080/api/flights` | Flight index JSON |
| Foxglove | `ws://10.42.0.1:8765` | Live ROS 2 telemetry |
| SSH | `ssh dronepi@10.42.0.1` | Remote terminal |

---

## Systemd Services

All five services start automatically on boot.

| Service | Purpose | Port |
|---|---|---|
| `mavros` | FCU bridge | — |
| `drone-watchdog` | Flight stack supervisor | — |
| `dronepi-hotspot` | Wi-Fi AP | — |
| `drone-mesh-server` | HTTP server | 8080 |
| `foxglove-bridge` | WebSocket telemetry | 8765 |

```bash
# Check all services
systemctl status mavros drone-watchdog drone-mesh-server \
  foxglove-bridge dronepi-hotspot

# View live logs
journalctl -u drone-watchdog -f
```

---

## Python Environments

Two isolated environments are required. They do not share packages and
must not be mixed.

**`dronepi` (Conda)** — flight stack, SLAM, mesh pipeline, all ROS 2 nodes
except Hailo inference.

```bash
conda activate dronepi
python3 main.py
```

**`hailo_inference_env` (venv)** — Hailo-8 inference only.
Launched automatically by `flight_stack.py` via the explicit interpreter path
`~/hailo_inference_env/bin/python3`. Never activated manually during flight.

```bash
# Verify Hailo environment
~/hailo_inference_env/bin/python3 -c "import hailo_platform; print('ok')"
```

---

## Running a Mission

```bash
conda activate dronepi
source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
python3 main.py
```

`main.py` is the sole entry point. It starts all subsystems (bag recorder,
SLAM, bridge, Hailo node, GPS reader, collision monitor), runs the preflight
gate, presents the arm prompt, executes the survey mission, and triggers
post-processing on disarm. Mesh appears in the browser within 10 seconds
of landing.

---

## Post-Processing Pipeline (Standalone)

To reprocess an existing rosbag without flying:

```bash
conda activate dronepi
python3 utils/postprocess_mesh.py --bag /mnt/ssd/rosbags/<flight_id>.mcap
```

Seven stages: bag read → MLS smooth → ground classify → DTM → DSM →
texture project → publish. Each stage is a standalone importable class.
The texture stage is skipped automatically if camera data is absent
(`--no-texture` also suppresses it).

---

## Configuration

All tunable parameters are in `unitree_drone_mapper/config.yaml`.
Environment variable overrides are supported for every parameter — see
`config_loader.py` for the full list.

Key PX4 parameters (set in QGroundControl):

| Parameter | Value | Purpose |
|---|---|---|
| `EKF2_EV_CTRL` | `15` | Enable all external vision inputs |
| `EKF2_GPS_CTRL` | `0` | Disable GPS as EKF2 source (SLAM is sole source) |
| `EKF2_EV_DELAY` | `25` | ms — accounts for SLAM bridge + Hailo latency |
| `TRIG_MODE` | `4` | PWM camera trigger for IMX477 sync |

---

## Documentation

| Document | Contents |
|---|---|
| `docs/architecture.md` | System architecture, design principles, service dependency graph |
| `docs/hardware_setup.md` | Assembly, wiring, known hardware issues and resolutions |
| `docs/pipeline_overview.md` | Seven-layer data flow, lock file state machine |
| `docs/ekf_slam_fusion.md` | EKF2 configuration, SLAM bridge covariance, GPS augmentation |
| `docs/post_processing.md` | Mesh pipeline stage reference |
| `docs/flight_procedure.md` | Pre-flight checklist, RC assignments, emergency procedures |
| `docs/calibration.md` | Camera intrinsic, LiDAR-camera extrinsic, PX4 sensor calibration |
| `docs/test_procedure.md` | 9-phase gate sequence from bench to full autonomous mission |
| `docs/texture_pipeline.md` | Texture projection architecture, occlusion culling, PX4 trigger |

---

## Flight Safety Invariants

Every flight script in this repository enforces the following. Any script
that violates these invariants must not be merged.

1. `SafeFlightMixin` is inherited before any arming call.
2. `rosbag` recorder is the first subprocess launched, before any other init.
3. Every setpoint publisher has a heartbeat watchdog with a 2-second maximum
   timeout; loss of heartbeat triggers immediate disarm or land.
4. RC Channel 7 monitoring is present and capable of interrupting autonomous
   operation at any point.

---

## Known Open Items

| Item | Status |
|---|---|
| OFFBOARD flight consistency | Inconsistent; under investigation |
| Mid-flight live view dropout (WiFi range) | Options identified, not implemented |
| Texture pipeline real-bag validation | First flight pending camera remount |
| IMX477 camera remount | Awaiting Arducam CSI-to-HDMI kit |
| Post-processing 9-phase gate completion | Phases 1-7 verified; 8-9 pending flight |
| `EKF2_RNG_POS_X/Y/Z` entry in QGC | Measured, not yet set |

---

## License

UPRM Capstone Project — 2025-2026. All rights reserved.
