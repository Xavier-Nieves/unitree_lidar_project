# Test Procedure

**Document:** 7 of 7
**Repo path:** `docs/test_procedure.md`
**Companion script:** `tests/test_order.sh`
**Last updated:** March 2026
**Project:** DronePi — Autonomous LiDAR Mapping Drone — UPRM Capstone

---

## Overview

This document defines the complete test sequence from initial bench validation
to full autonomous survey flight. It is a strict gate sequence — no phase may
be skipped unless the prerequisite conditions are already confirmed from a
previous session.

Run `preflight_check.sh` at the start of every session regardless of which
phase you are resuming. Log every session result.

To run tests interactively from the command line:

```bash
# Interactive — prompts before each command
bash tests/test_order.sh

# Jump directly to a phase
bash tests/test_order.sh --phase 5
```

---

## Phase Summary

| Phase | Name | Location | Props | GPS |
|---|---|---|---|---|
| 0 | System health check | Bench | No | No |
| 1 | Service verification | Bench | No | No |
| 2 | LiDAR and SLAM validation | Bench | No | No |
| 3 | Post-flight pipeline validation | Bench | No | No |
| 4 | Watchdog arm detection | Bench | No | No |
| 5 | OFFBOARD flight tests | Outdoor | Yes | Required |
| 6 | SLAM bridge flight test | Outdoor | Yes | Required |
| 7 | Full mission dry run | Outdoor | No | Required |
| 8 | Full autonomous survey mission | Outdoor | Yes | Required |
| 9 | Post-camera integration | Outdoor | Yes | Required |

---

## Phase 0 — System Health Check

Run at the start of every test session without exception.

```bash
cd ~/unitree_lidar_project/unitree_drone_mapper
sudo bash preflight_check.sh
```

Review the generated log:

```bash
ls ~/unitree_lidar_project/logs/
cat ~/unitree_lidar_project/logs/log_N.txt
```

Pass criteria: zero FAIL. All WARN items reviewed and accepted before
proceeding.

---

## Phase 1 — Service Verification

**Location:** Bench. Props off.

### 1.1 Confirm all services running

```bash
sudo systemctl status mavros drone-watchdog dronepi-main \
  drone-mesh-server foxglove-bridge
```

### 1.2 Confirm watchdog is polling

```bash
sudo journalctl -u drone-watchdog -f
# Expected: [WAITING] armed=False mode=ALTCTL  (every 0.5s)
```

### 1.3 Confirm main.py is running

```bash
sudo journalctl -u dronepi-main -f
# Expected: [IDLE] armed=False mode=ALTCTL lidar=no/yes
```

### 1.4 Confirm MAVROS FCU connection

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /mavros/state --once
# Expected: connected: true  armed: false  mode: ALTCTL
```

### 1.5 Confirm mesh server and browser viewer

```bash
curl http://10.42.0.1:8080/api/flights
# Open browser: http://10.42.0.1:8080/meshview.html
```

### 1.6 Confirm Foxglove connection

Open Foxglove Studio, connect to `ws://10.42.0.1:8765`. Topic list should
appear within 5 seconds.

Pass criteria: all services active, FCU connected, browser loads, Foxglove
topic list appears.

---

## Phase 2 — LiDAR and SLAM Validation

**Location:** Bench. Props off.

### 2.1 Start Point-LIO

```bash
source /opt/ros/jazzy/setup.bash
source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
ros2 launch ~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/\
combined_lidar_mapping.launch.py rviz:=false port:=/dev/ttyUSB0
```

### 2.2 Verify topics publishing

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
ros2 topic hz /unilidar/cloud --window 5
ros2 topic hz /aft_mapped_to_init --window 5
ros2 topic hz /cloud_registered --window 5
# Expected: ~10 Hz, ~250 Hz, ~10 Hz
```

### 2.3 Verify Z axis correction in Foxglove

Open the 3D panel in Foxglove Studio, subscribe to `/cloud_registered`.
Lift the LiDAR upward — the cloud should move up in the viewer.
Plot `/aft_mapped_to_init.pose.pose.position.z` — value should increase
when the LiDAR is lifted.

### 2.4 Run live SLAM test

```bash
python3 tests/test_slam_live.py --no-rviz
# Walk around with the LiDAR for 60 seconds
# Expected: all three topics at expected rates in topic status output
```

### 2.5 Test SLAM bridge

With Point-LIO running in Terminal 1:

```bash
# Terminal 2
python3 flight/_slam_bridge.py
# Expected: "SLAM bridge ready: /aft_mapped_to_init -> /mavros/vision_pose/pose"
# Then every 250 frames: "Bridge: 250 frames  ENU=(x, y, z)"

# Terminal 3
ros2 topic hz /mavros/vision_pose/pose --window 5
# Expected: ~250 Hz (matching /aft_mapped_to_init output rate)
```

Pass criteria: all topics at expected rates, Z increases on lift, SLAM bridge
publishes vision pose at ~250 Hz.

---

## Phase 3 — Post-Flight Pipeline Validation

**Location:** Bench. Props off.

### 3.1 Process an existing bag

```bash
python3 utils/run_postflight.py \
  --bag /mnt/ssd/rosbags/scan_20260315_191900
# Expected: cloud + mesh PLY generated, browser auto-loads within 10s
```

### 3.2 Process latest bag automatically

```bash
python3 utils/run_postflight.py
# Expected: finds latest bag, runs pipeline, updates latest.json
```

### 3.3 Verify browser result

Open `http://10.42.0.1:8080/meshview.html`. Toggle Cloud and Mesh buttons.
Check the flight database sidebar shows the session.

Pass criteria: PLY files generated, viewer loads, cloud and mesh toggle works.

---

## Phase 4 — Watchdog Arm Detection

**Location:** Bench. Props off.

### 4.1 Watch watchdog respond to arm/disarm

```bash
sudo journalctl -u drone-watchdog -f
```

On the RC transmitter, arm the drone (no props). Expected output:

```
[WAITING] armed=True mode=ALTCTL
```

Switch to OFFBOARD on the transmitter while setpoints are streaming. Expected:

```
[ACTIVATING] Session #1 armed=True mode=OFFBOARD
```

Disarm. Expected:

```
[DEACTIVATING] then [POSTFLIGHT] lines appearing
```

### 4.2 Test manual scan mode detection in main.py

```bash
sudo journalctl -u dronepi-main -f
```

Arm without switching to OFFBOARD. Wait 10 seconds. Expected:

```
[MANUAL_SCAN] committed
```

Disarm. Expected: back to IDLE.

### 4.3 Verify lock file behavior

While armed in manual scan mode:

```bash
cat /tmp/dronepi_mission.lock
# Expected: {"mode": "manual_scan", "started_at": "..."}
```

After disarm:

```bash
ls /tmp/dronepi_mission.lock
# Expected: file gone (cleared after postflight completes)
```

Pass criteria: state machine transitions correct, lock file written and cleared.

---

## Phase 5 — OFFBOARD Flight Tests

**Location:** Outdoor, open field. Props on. GPS required.

**Safety:** RC transmitter in hand at all times. Kill switch confirmed before
each test. Minimum 10 satellites and HDOP below 1.2 confirmed in QGroundControl
before arming. Area clear of personnel and obstacles.

### 5.1 Preflight check outdoors

```bash
sudo bash preflight_check.sh
# Must show: GPS home position received, FCU connected, zero FAIL
```

### 5.2 GPS hover dry run — no motors

```bash
python3 tests/test_offboard_gps.py --dry-run
# Expected: GPS lock confirmed, home position captured
# No motors activate during dry run
```

### 5.3 OFFBOARD hover test — 3m AGL, 15s hold

```bash
python3 tests/test_offboard_flight.py --alt 3.0 --hold 15
# Script arms and switches to OFFBOARD automatically
# Drone climbs to 3m, holds for 15s, lands via AUTO.LAND
# RC override available at any time by moving sticks
```

### 5.4 GPS waypoint hover test — 5m AGL, 10s hold

```bash
python3 tests/test_offboard_gps.py --alt 5.0 --hold 10
```

Pass criteria: drone takes off, holds position without excessive drift,
lands cleanly. Pixhawk LED shows purple during OFFBOARD.

---

## Phase 6 — SLAM Bridge Flight Test

**Location:** Outdoor, open field. Props on. GPS required.

**Safety:** Same as Phase 5.

### 6.1 Dry run — no motors

```bash
python3 tests/test_slam_bridge_flight.py --dry-run --no-bridge --no-bag
# Expected: EKF stable, home position captured, waypoints previewed
# No motors activate
```

### 6.2 Out-and-back — GPS only, no recording

Establishes a baseline for position hold accuracy without SLAM fusion.

```bash
python3 tests/test_slam_bridge_flight.py \
  --distance 5 --alt 3.0 --hold 5 --no-bridge --no-bag
# 5m forward, 5s hold, return, land
```

### 6.3 Out-and-back — SLAM bridge active, no recording

Compare position hold accuracy against Phase 6.2. Watch the EKF Innovations
panel in Foxglove Studio — vision residuals should appear and remain small.

```bash
python3 tests/test_slam_bridge_flight.py \
  --distance 5 --alt 3.0 --hold 5 --no-bag
```

### 6.4 Out-and-back — full recording with post-flight

```bash
python3 tests/test_slam_bridge_flight.py \
  --distance 8 --alt 4.0 --hold 10
# Bag saved, post-flight runs automatically on landing
# Mesh appears in browser within 10s of disarm
```

Pass criteria: drone follows path accurately, SLAM bridge active and
publishing vision pose, mesh appears in browser after landing.

---

## Phase 7 — Full Mission Dry Run

**Location:** Outdoor. No motors. GPS required.

Upload the survey mission in QGroundControl before running these steps:
Plan → Survey → draw bounding box over test area → set altitude 15m AGL,
overlap 75% → Upload to Vehicle.

### 7.1 Mission executor dry run

```bash
python3 flight/mission_executor.py --dry-run
# Expected: waypoints received, converted to local ENU, list printed
# No motors activate
```

### 7.2 Full flight mission dry run

```bash
python3 flight_mission.py --dry-run
# Runs all 6 pre-flight checks:
#   [1] FCU connected
#   [2] GPS lock + home
#   [3] EKF stable
#   [4] SSD writable
#   [5] Launch file exists
#   [6] Mission waypoints present
# Prints mission waypoints in ENU
# No motors activate
```

Pass criteria: all 6 pre-flight checks pass, waypoints displayed correctly
in ENU frame.

---

## Phase 8 — Full Autonomous Survey Mission

**Location:** Outdoor, open field. Props on. GPS required.

**Safety:** Only proceed after all previous phases pass in the same field
session or a recent confirmed session. Second person as safety observer
strongly recommended. Area fully clear. RC transmitter in hand throughout.

### 8.1 Final preflight check

```bash
sudo bash preflight_check.sh
# Zero FAIL required — no exceptions
```

### 8.2 Confirm mission uploaded in QGroundControl

Plan view: waypoints visible. Fly view: home position set. No active
failsafes or warnings in the status bar.

### 8.3 Execute full autonomous mission

```bash
python3 flight_mission.py --trigger-mode stop --fov-deg 70.0
```

Expected sequence:
- All 6 pre-flight checks pass
- Watchdog yields to mission controller
- Point-LIO, SLAM bridge, and bag recorder start
- Arm prompt appears — arm on RC transmitter, switch to OFFBOARD
- Drone climbs, executes survey waypoints, fills coverage gaps
- AUTO.RTL → land → disarm
- Watchdog restores normal operation
- Post-flight pipeline runs automatically
- Mesh appears in browser within 10s of disarm

### 8.4 Verify mesh in browser

```
http://10.42.0.1:8080/meshview.html
Toggle cloud and mesh views
Check flight database sidebar for the new session
```

Pass criteria: mission completes without manual intervention, RTL and
landing clean, textured mesh (or point cloud) appears in browser.

---

## Phase 9 — Post-Camera Integration

**Prerequisite:** IMX477 camera reinstalled, calibration verified.
See `docs/calibration.md` before running this phase.

### 9.1 Camera calibration verification

```bash
python3 tests/test_camera_preview.py
# Verify image stream active
# Verify calibration YAML matches actual FOV
# Check for barrel distortion in preview
```

### 9.2 Full mission with camera capture

```bash
python3 flight_mission.py --trigger-mode stop
# Camera capture stubs replaced by real IMX477 captures
# Verify images saved to /mnt/ssd/ alongside bag
# Verify each image has a matching SLAM pose within 5ms
```

### 9.3 Textured mesh generation

```bash
python3 utils/run_postflight.py
# Verify textured PLY loads in browser with real RGB color
# Compare edge alignment between texture and geometry
```

Pass criteria: images captured and timestamped, texture projected onto mesh
without visible misalignment, colored mesh loads in browser.

---

## Quick Reference

| Test | Command |
|---|---|
| System check | `sudo bash preflight_check.sh` |
| Watchdog logs | `sudo journalctl -u drone-watchdog -f` |
| main.py logs | `sudo journalctl -u dronepi-main -f` |
| MAVROS state | `ros2 topic echo /mavros/state --once` |
| SLAM live test | `python3 tests/test_slam_live.py --no-rviz` |
| Process bag | `python3 utils/run_postflight.py` |
| OFFBOARD hover | `python3 tests/test_offboard_flight.py --alt 3.0` |
| GPS hover dry run | `python3 tests/test_offboard_gps.py --dry-run` |
| Out-and-back dry run | `python3 tests/test_slam_bridge_flight.py --dry-run --no-bridge --no-bag` |
| Out-and-back live | `python3 tests/test_slam_bridge_flight.py --distance 5 --alt 3.0 --no-bag` |
| Mission dry run | `python3 flight_mission.py --dry-run` |
| Full mission | `python3 flight_mission.py` |
| Viewer | `http://10.42.0.1:8080/meshview.html` |
| Lock file check | `cat /tmp/dronepi_mission.lock` |
| Clear lock | `rm /tmp/dronepi_mission.lock` |
| View session logs | `ls ~/unitree_lidar_project/logs/` |
