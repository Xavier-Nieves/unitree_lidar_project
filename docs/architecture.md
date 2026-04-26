# System Architecture

**Document:** 6 of 9
**Repo path:** `docs/architecture.md`
**Last updated:** April 2026
**Project:** DronePi — Autonomous LiDAR Mapping Drone — UPRM Capstone

---

## Table of Contents

1. [Architecture Diagram](#1-architecture-diagram)
2. [Layer Descriptions](#2-layer-descriptions)
3. [Systemd Service Dependency Graph](#3-systemd-service-dependency-graph)
4. [Repository Quick Reference](#4-repository-quick-reference)
5. [Network Reference](#5-network-reference)
6. [Key Design Principles](#6-key-design-principles)
7. [Two-Environment Isolation](#7-two-environment-isolation)

---

## 1. Architecture Diagram

The diagram below shows the full DronePi system architecture across seven
layers, from raw hardware inputs to browser-rendered 3D mesh output.

The source SVG file is at `docs/dronepi_architecture.svg`. Embed it in any
document or render it in a browser directly.

![DronePi System Architecture](dronepi_architecture.svg)

---

## 2. Layer Descriptions

### Layer 0 — Hardware

Four physical sensors feed the system. The Pixhawk 6X handles all
stabilization, arming, and mode management via PX4 firmware. The Unitree L1
LiDAR provides 360-degree point cloud data at 9.8 Hz over USB serial at
2,000,000 baud. The M9N GPS provides global position via the Pixhawk internal
port. The IMX477 camera (Arducam CSI-to-HDMI extension) provides imagery for
post-flight texture projection. The Hailo-8 AI accelerator (PCIe HAT+) runs
SCDepthV3 optical flow and YOLOv8m ground classification in a dedicated
inference environment.

### Layer 1 — ROS 2 Bridge

Five ROS 2 nodes translate hardware data into topics consumed by the flight
stack.

MAVROS bridges MAVLink from the Pixhawk to ROS 2 topics, including
`/mavros/state`, `/mavros/local_position/pose`, and
`/mavros/odometry/out`.

Point-LIO SLAM processes the LiDAR stream and produces a registered point
cloud on `/cloud_registered` and a pose estimate on `/aft_mapped_to_init`.

The SLAM bridge (`_slam_bridge.py`) converts the Point-LIO ENU pose to the
format EKF2 expects and publishes it on `/mavros/odometry/out` at
approximately 90 Hz with explicit diagonal covariance on both pose and twist.
SLAM is the sole EKF2 position source (`EKF2_GPS_CTRL = 0`).

The collision monitor (`collision_monitor.py`) processes the point cloud to
produce a three-zone obstacle distance LaserScan and an AGL height estimate,
both fed back to PX4 via MAVROS rangefinder topics.

The GPS reader (`gps_reader.py`) provides a secondary, non-authoritative
position layer using MAVROS topics as its backend
(`/mavros/global_position/raw/fix`, `/mavros/gpsstatus/gps1/raw`,
`/mavros/global_position/global`). It applies a three-criterion quality gate
(HDOP, satellite count, fix type) matching PX4 EKF2 defaults. GPS failure is
always non-fatal. SLAM remains sole EKF2 position source.

The Hailo flight node (`hailo/hailo_flight_node.py`) runs inside
`hailo_inference_env` and publishes optical flow velocity on
`/hailo/optical_flow` and surface classification on `/hailo/ground_class`.
It subscribes to `/arducam/image_raw` published by `camera_capture.py`.
Hailo augmentation is additive to Point-LIO SLAM; failures never block core
flight or SLAM.

### Layer 2 — Flight Stack

Three components coordinate via a lock file at `/tmp/dronepi_mission.lock`.

`main.py` is the top-level state machine. It is the sole entry point.
It starts all subsystems in the correct order (rosbag recorder first, then
SLAM, bridge, Hailo node, GPS reader, collision monitor), presents the arm
prompt, manages mission mode transitions, and triggers post-processing on
disarm.

`drone_watchdog.py` supervises the scan session subprocesses — Point-LIO,
SLAM bridge, and bag recorder — starting them on arm with OFFBOARD and
stopping them in the correct order (bag then Hailo then bridge then SLAM)
on disarm. The watchdog monitors RC Channel 7 for emergency interrupt at
all times.

`preflight_check.sh` is the single arming gate — 17 sections, 63
automated checks. No arming-related health checks are scattered in any
other file. Zero FAIL is required before any arm.

The flow bridge (`_flow_bridge.py`) gates Hailo optical flow estimates by
confidence threshold, message age, and velocity magnitude before forwarding
valid estimates to `/mavros/odometry/out` for EKF2 velocity fusion
(`EKF2_EV_CTRL` bit 2).

### Layer 3 — Post-Processing Trigger

Triggered automatically on disarm. `run_postflight.py` locates the most
recent bag on the SSD and calls `postprocess_mesh.py`.
`flight_logger.py` appends a one-line record to the persistent flight
history log at `logs/flight_history.log`. Foxglove cloud rotation
(`foxglove_cloud_rotation.py`) runs if `FOXGLOVE_API_KEY` is set; cloud
failures never propagate to the local pipeline.

### Layer 4 — mesh_tools Pipeline

Seven independent, importable classes form a staged pipeline. Each class has
a single public method with typed inputs and outputs. No class imports another
from the same pipeline.

`BagReader` reads MCAP rosbags and returns a NumPy point array.
`MLSSmoother` applies Moving Least Squares noise reduction via Open3D.
`GroundClassifier` splits terrain from structure using PDAL SMRF.
`DTMBuilder` constructs a Delaunay 2.5D terrain mesh.
`DSMBuilder` constructs a Ball Pivoting surface mesh.
`TextureProjector` (Stage 6, `core/texture_projector.py`) projects IMX477
camera frames onto mesh vertices using the pinhole model with Z-buffer
occlusion culling. Stage is skipped if camera data is absent or
`--no-texture` is passed.
`Publisher` writes PLY files and JSON manifests to `/mnt/ssd/maps/`.

### Layer 5 — Outputs and HTTP Serving

`Publisher` writes `combined_cloud.ply`, `mesh_final.ply`,
`textured_mesh.ply` (when camera data present), and `metadata.json` to
`/mnt/ssd/maps/`. `serve.py` serves them over HTTP on port 8080 with a
`/api/flights` endpoint and a camera MJPEG stream proxy at `/stream`.
The Foxglove bridge exposes all ROS 2 topics as a WebSocket on port 8765.

### Layer 6 — Viewers

`meshview.html` is the Pi-hosted Three.js browser viewer. It polls
`latest.json` every 10 seconds and auto-loads new meshes after landing.
It requires no internet connection — Three.js and PLYLoader are served from
the SSD. It includes measurement tools, height color ramp, flight database
sidebar, ortho-map tab (Leaflet), debug file toggle, and camera mosaic view.

`local_test.html` is the offline laptop viewer with drag-and-drop flight
folder support. Confirmed working in Edge with trackpad support.

Foxglove Studio connects to `ws://10.42.0.1:8765` for live telemetry,
EKF2 innovations, and 3D point cloud visualization during flight.

---

## 3. Systemd Service Dependency Graph

All services start automatically on power-on.

```
network.target
  |
  +-- mavros.service
  |     FCU bridge — always running
  |
  +-- drone-watchdog.service
  |     Flight stack supervisor
  |     Requires: mavros.service
  |
  +-- dronepi-main.service
        Top-level orchestrator
        Requires: mavros.service + drone-watchdog.service
        Inactive until first arm

mnt-ssd.mount
  |
  +-- dronepi-hotspot.service
  |     Wi-Fi AP — oneshot, exited state is normal
  |
  +-- drone-mesh-server.service
        HTTP server, port 8080
        Requires: mnt-ssd.mount + dronepi-hotspot.service
        ExecStartPre: /bin/mountpoint -q /mnt/ssd

foxglove-bridge.service
  WebSocket, port 8765
  No hard dependencies

rpi-health.service
  Health monitor — publishes /rpi/health every 2s
  No hard dependencies
```

Approximate boot timing from power-on:

```
~15s   dronepi-hotspot       Wi-Fi AP on wlan0 — SSID dronepi-ap
~16s   drone-mesh-server     HTTP server on port 8080
~18s   foxglove-bridge       WebSocket on port 8765
~20s   rpi-health            Publishes /rpi/health every 2s
~25s   mavros                FCU bridge via /dev/ttyPixhawk
~26s   drone-watchdog        Polls /mavros/state, manages flight stack
```

`ExecStartPre` guards are used for runtime checks that systemd dependency
declarations cannot express — specifically, verifying that `/mnt/ssd` is a
live mountpoint before starting the HTTP server.

### Lock file coordination

`main.py` and `drone_watchdog.py` share state through a JSON lock file at
`/tmp/dronepi_mission.lock`. A lock file is readable by any process, survives
service restarts, and can be inspected with `cat`. It does not require both
processes to be alive simultaneously to communicate intent.

| Lock value | Written by | Watchdog behavior | main.py behavior |
|---|---|---|---|
| absent | — | RC toggle on CH6 starts and stops stack | Monitors only |
| `manual_scan` | main.py MODE 2 | Owns the stack, monitors disarm | Delegates to watchdog |
| `autonomous` | main.py MODE 3 | Yields — does not touch stack | Owns the stack |
| `bench_scan` | Test scripts | Yields — test script owns stack | Inactive |

Stale lock recovery:

```bash
rm /tmp/dronepi_mission.lock
sudo systemctl restart drone-watchdog
```

---

## 4. Repository Quick Reference

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
|   |-- camera_capture.py          IMX477 capture node (Picamera2, sole owner)
|   |-- flight_controller.py       Thread-safe MAVROS state and control
|   |-- led_service.py             LED state derivation from system snapshot
|   +-- watchdog_core/             Modular watchdog subpackage
|       |-- buzzer.py              Audio feedback tunes
|       |-- mavros_reader.py       ROS 2 MAVROS interface
|       |-- flight_stack.py        Subprocess manager (Hailo, SLAM, bag)
|       |-- postflight.py          Monitored post-flight trigger
|       +-- led_controller.py      RGB LED state machine
|
|-- hailo/
|   |-- hailo_device.py            VDevice lifecycle wrapper
|   |-- hailo_optical_flow.py      SCDepthV3 depth-difference velocity estimator
|   |-- hailo_ground_class.py      YOLOv8m nadir ground surface classifier
|   +-- hailo_flight_node.py       ROS 2 node — runs in hailo_inference_env
|
|-- core/
|   |-- camera_model.py            Intrinsics + extrinsics, pinhole projection
|   +-- texture_projector.py       Per-vertex color accumulator with Z-buffer
|
|-- tests/
|   |-- test_offboard_flight.py    OFFBOARD hover test (standalone, has main())
|   |-- test_offboard_gps.py       GPS waypoint mission
|   |-- test_manual_scan.py        Manual RC + LiDAR scan
|   |-- test_slam_bridge_flight.py SLAM bridge validation
|   |-- collision_zone_test.py     Zone characterization
|   +-- test_mesh_algorithms.py    Algorithm comparison
|
|-- utils/
|   |-- postprocess_mesh.py        7-stage pipeline orchestrator
|   |-- run_postflight.py          Post-flight trigger
|   |-- flight_logger.py           Flight history log
|   |-- bench_test.py              MAVROS 8-point bench test
|   |-- foxglove_events.py         Foxglove Events API client
|   |-- foxglove_cloud_rotation.py Foxglove cloud recording rotation manager
|   +-- mesh_tools/
|       |-- bag_reader.py
|       |-- mls_smoother.py
|       |-- ground_classifier.py
|       |-- dtm_builder.py
|       |-- dsm_builder.py
|       |-- mesh_merger.py
|       +-- publisher.py
|
|-- unitree_drone_mapper/
|   |-- config.yaml                All tunable parameters
|   |-- config_loader.py           Two-pass YAML loader with env var overrides
|   +-- ...                        Package modules mirrored from flight/
|
|-- config/
|   |-- px4_config.yaml
|   |-- px4_pluginlists.yaml
|   +-- camera_calibration.yaml    Intrinsics (RMS 1.99), extrinsics, trigger mode
|
|-- rpi_server/
|   |-- serve.py                   HTTP + camera stream proxy (port 8080)
|   |-- meshview.html              Pi-hosted Three.js viewer
|   +-- local_test.html            Offline laptop viewer
|
|-- RPI5/ros2_ws/src/
|   |-- point_lio_ros2/
|   |-- unitree_lidar_ros2/        (symlink at correct colcon discovery depth)
|   +-- unilidar_sdk/
|
|-- setup_scripts/
|   |-- setup.sh                   Master setup (run once as root)
|   |-- preflight_check.sh         63-check arming gate
|   +-- setup_*.sh                 Per-service systemd installers
|
+-- docs/
    |-- architecture.md            This file
    |-- hardware_setup.md          Hardware assembly and known issues
    |-- pipeline_overview.md       Data flow and layer reference
    |-- ekf_slam_fusion.md         EKF2 and sensor fusion reference
    |-- post_processing.md         Mesh pipeline reference
    |-- flight_procedure.md        Flight operations manual
    |-- calibration.md             Camera intrinsic, extrinsic, PX4 calibration
    |-- test_procedure.md          9-phase bench-to-field gate sequence
    +-- texture_pipeline.md        Texture projection architecture and status
```

---

## 5. Network Reference

| Interface | Address | Purpose |
|---|---|---|
| Hotspot (field) | `10.42.0.1` | Primary field access — no router needed |
| Lab router | `192.168.2.2` | Lab development access |
| Mesh viewer | `http://10.42.0.1:8080/meshview.html` | 3D mesh browser |
| Flight API | `http://10.42.0.1:8080/api/flights` | Flight index JSON |
| Camera stream | `http://10.42.0.1:8080/stream` | Live MJPEG proxy |
| Foxglove | `ws://10.42.0.1:8765` | Live ROS 2 telemetry |
| SSH | `ssh dronepi@10.42.0.1` | Remote terminal |

---

## 6. Key Design Principles

### Modular classes with a single orchestrator

Every processing stage is an independent importable class. A single
orchestrator script calls the classes in sequence. No production class has
a `main()`. Test scripts are standalone with their own `main()` and are
never imported by production code.

Any stage can be tested in isolation, replaced, or skipped without modifying
any other component.

### Hardware-optional modules use runtime detection

Modules that depend on optional hardware — Hailo-8, RGB LED, IMX477 camera —
implement an `is_available()` classmethod that checks for the hardware or
dependency at runtime. If the hardware is absent, the module logs a warning
and the pipeline continues with a fallback or skips the stage. The pipeline
never fails hard on missing optional hardware.

### preflight_check.sh is the single arming gate

All health checks live in `preflight_check.sh` as numbered sections. No
arming-related checks are scattered in startup scripts or service files. When
a new check is needed, it is added as a new section to this script only.

### Systemd services declare explicit dependencies

Every service that needs a resource declares it explicitly with `Requires=`
and `After=`. `ExecStartPre` guards handle runtime checks that systemd
dependency declarations cannot express, such as verifying `/mnt/ssd` is a
live mountpoint.

### Lock file coordination instead of IPC

`main.py` and `drone_watchdog.py` share state through a JSON lock file.
This is intentional: a lock file is readable by any process, survives
restarts, and can be inspected manually. It does not require both processes
to be alive simultaneously.

### External service failures are never fatal

All external integrations — Foxglove Events API, Foxglove cloud rotation —
are wrapped in try/except. Cloud or network failures never propagate into
local pipeline outputs. The mesh pipeline always produces a local result
regardless of network state.

### GPS is secondary and non-authoritative

The GPS reader provides a supplementary ENU position layer. SLAM remains the
sole EKF2 position source (`EKF2_GPS_CTRL = 0`). GPS quality-gating matches
PX4 EKF2 acceptance thresholds (HDOP, satellite count, fix type). GPS failure
is always non-fatal and logged only, never surfaced to the pilot as a blocking
condition.

### Hailo augmentation is additive

The Hailo optical flow and ground classification pipeline is additive to
Point-LIO SLAM. Failures in the inference chain — missed frames, low
confidence, environment absence — must never block core flight or SLAM
operations. The flow bridge applies confidence gating and consecutive
fallback counters before any estimate reaches EKF2.

---

## 7. Two-Environment Isolation

Two Python environments run on the system and must not be mixed.

**`dronepi` (Conda)** — all flight stack, SLAM, mesh pipeline, and ROS 2
nodes except Hailo inference. Activated by default in `.bashrc`. Used for
`main.py`, all `utils/`, all `mesh_tools/`, and all test scripts.

**`hailo_inference_env` (venv)** — Hailo-8 inference only (HailoRT 4.23.0,
`hailo_platform`). Never activated manually during flight. The
`flight_stack.py` subprocess manager launches `hailo_flight_node.py` using
the explicit interpreter path `~/hailo_inference_env/bin/python3`. If the
interpreter path or script is absent, the Hailo node is skipped gracefully.

Communication between environments is exclusively via ROS 2 topics. There
are no shared imports and no shared venv.
