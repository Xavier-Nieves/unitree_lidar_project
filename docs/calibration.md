# Calibration Guide

**Document:** 8 of 7 (supplementary)
**Repo path:** `docs/calibration.md`
**Last updated:** March 2026
**Project:** DronePi — Autonomous LiDAR Mapping Drone — UPRM Capstone

---

## Table of Contents

1. [Overview](#1-overview)
2. [Camera Intrinsic Calibration](#2-camera-intrinsic-calibration)
3. [LiDAR-Camera Extrinsic Calibration](#3-lidar-camera-extrinsic-calibration)
4. [EKF2 Offset Parameters](#4-ekf2-offset-parameters)
5. [PX4 Sensor Calibration](#5-px4-sensor-calibration)
6. [Calibration File Reference](#6-calibration-file-reference)
7. [Verification Procedure](#7-verification-procedure)

---

## 1. Overview

DronePi requires four categories of calibration. Each has its own method,
tool, and output file. Calibration must be repeated any time a sensor is
physically moved or replaced.

| Calibration | Method | Output | Status |
|---|---|---|---|
| Camera intrinsic | Checkerboard, OpenCV | `config/camera_calibration.yaml` | Complete — RMS 1.99 |
| LiDAR-camera extrinsic | Physical measurement + future refinement | `config/camera_calibration.yaml` | Measured, code unverified |
| EKF2 LiDAR offset | Physical measurement, enter in QGC | QGC parameter `EKF2_RNG_POS_X/Y/Z` | Measured, not yet set in QGC |
| PX4 sensors | QGroundControl calibration wizard | Stored on Pixhawk | Complete |

The camera is currently dismounted pending the Arducam CSI-to-HDMI kit
arrival. Camera intrinsic values are confirmed from the previous mount.
Extrinsic values are measured but must be verified in code before the
first texture capture flight.

---

## 2. Camera Intrinsic Calibration

Intrinsic calibration determines the focal length, principal point, and
distortion coefficients of the IMX477 camera at its current lens setting.
These values are required for `cv2.undistort()` in the texture pipeline.

### Current confirmed values

Calibration was performed with a 10x7 inner-corner checkerboard pattern.
RMS reprojection error: 1.99 pixels (acceptable for this application).

```yaml
# config/camera_calibration.yaml
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx,
         0, fy, cy,
         0,  0,  1]

distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]

image_width: 4056
image_height: 3040
```

Exact numeric values are stored in `config/camera_calibration.yaml` in
the repository. Do not edit them manually — re-run calibration if values
need to change.

### When to recalibrate intrinsics

Recalibrate camera intrinsics if any of the following change:

- Camera is physically removed and remounted
- Lens focal length setting is adjusted (Tamron 4–12mm C-mount)
- C-CS adapter is added or removed
- Camera PCB or sensor is replaced

### Recalibration procedure

Requires: printed checkerboard (10x7 inner corners, 30mm square size),
good lighting, camera reinstalled and confirmed publishing.

```bash
# Confirm camera is publishing
libcamera-hello --list-cameras
ros2 topic hz /camera/image_raw --window 5

# Collect calibration images (minimum 20, aim for 30)
# Move checkerboard to different positions and angles — cover corners and edges
ros2 run camera_calibration cameracalibrator \
  --size 10x7 --square 0.030 \
  image:=/camera/image_raw \
  camera:=/camera

# Save result to config/camera_calibration.yaml when RMS < 2.0
```

If RMS exceeds 2.0, collect more images covering a wider range of board
positions and angles, particularly tilted views and corner positions.

---

## 3. LiDAR-Camera Extrinsic Calibration

The extrinsic calibration defines the rigid transform between the LiDAR
coordinate frame and the camera coordinate frame. This transform is applied
in `test_texture_live.py` to project camera images onto the LiDAR mesh.

### Current measured values

Physical measurement from the drone sensor plate:

```yaml
# config/camera_calibration.yaml
extrinsic:
  translation: [-0.070, 0.000, 0.030]  # [x, y, z] in metres
  rotation_rpy: [0.0, 0.0, 0.0]        # [roll, pitch, yaw] in radians
```

Interpretation:
- Camera is 70mm behind the LiDAR center (negative X)
- Camera is centered laterally (Y = 0)
- Camera is 30mm above the LiDAR center (positive Z)
- No rotation — both sensors are mounted aligned with the drone frame

### Critical audit required before first texture flight

Before any capture flight, verify that `test_texture_live.py` uses these
values and not an identity matrix. An identity extrinsic means no offset
is applied, causing texture misalignment proportional to the physical
sensor separation.

```bash
grep -n "extrinsic\|translation\|rotation\|identity\|eye(4)" \
  tests/test_texture_live.py
```

The output should reference the values from `config/camera_calibration.yaml`,
not `np.eye(4)` or any hardcoded zero matrix.

### Refinement method (after camera is online)

Physical measurement gives approximately 5–10mm accuracy. For better
texture alignment, refine using a calibration target visible to both sensors.

**Method — mutual information:**

1. Place a calibration target (flat board with known geometry) at 2–4m range
2. Record a short bag with both `/cloud_registered` and `/camera/image_raw`
3. Extract one LiDAR frame and one synchronized camera frame from the bag
4. Use an iterative optimizer to find the transform that maximizes mutual
   information between the projected LiDAR intensity and image gradients
5. Update `extrinsic.translation` and `extrinsic.rotation_rpy` in
   `config/camera_calibration.yaml`

**Method — corner correspondence:**

1. Place a checkerboard visible to both sensors
2. Detect checkerboard corners in the camera image (OpenCV)
3. Segment the checkerboard plane in the LiDAR cloud (planar RANSAC)
4. Solve for the transform that maps the LiDAR plane to the camera-detected
   corners using point-to-point ICP or direct linear transform
5. Update the calibration file with the solved values

After refinement, run the verification procedure in section 7.

### Rotation conventions

`rotation_rpy` uses radians in the order [roll, pitch, yaw], applied as
intrinsic rotations (body frame). If the camera is rotated 90 degrees
clockwise when viewed from the front, set `yaw = -1.5708` (negative pi/2).

The transform is applied as: `P_camera = R * P_lidar + t`

where `R` is the rotation matrix derived from `rotation_rpy` and `t` is
the translation vector.

---

## 4. EKF2 Offset Parameters

EKF2 needs to know the physical position of the LiDAR relative to the
drone center of gravity to correctly fuse the AGL rangefinder measurement
during banked turns.

### Current measured values

Measured from drone sensor plate to estimated center of gravity:

| Parameter | Value | Description |
|---|---|---|
| `EKF2_RNG_POS_X` | -0.070 m | LiDAR is 70mm behind CG (negative forward) |
| `EKF2_RNG_POS_Y` | 0.000 m | LiDAR is centered laterally |
| `EKF2_RNG_POS_Z` | 0.030 m | LiDAR is 30mm above CG |

### Setting in QGroundControl

These values must be entered in QGroundControl under Parameters before
the first flight where AGL rangefinder fusion is used.

```
QGroundControl → Parameters → search "EKF2_RNG_POS"
EKF2_RNG_POS_X = -0.070
EKF2_RNG_POS_Y = 0.000
EKF2_RNG_POS_Z = 0.030
```

Save parameters and reboot the flight controller.

### How to re-measure

If the sensor plate layout changes:

1. Find the drone center of gravity by balancing on a knife edge along
   each axis. Mark the CG position on the frame.
2. Measure the straight-line distance from the CG mark to the center of
   the LiDAR scanning plane in X (forward), Y (right), and Z (up).
3. Sign convention: X positive = forward, Y positive = right, Z positive = up.
   Distances behind CG are negative X. Distances below CG are negative Z.

---

## 5. PX4 Sensor Calibration

All PX4 sensor calibrations are performed through QGroundControl and stored
on the Pixhawk flight controller. These do not produce files in the repository.

| Calibration | Status | Tool | When to redo |
|---|---|---|---|
| Accelerometer | Complete | QGC calibration wizard | After crash or hard landing |
| Magnetometer | Complete | QGC calibration wizard | When flying in a new location or after frame changes |
| RC transmitter | Complete | QGC radio setup | When RC transmitter or receiver is replaced |
| ESC | Complete | QGC ESC calibration | After motor or ESC replacement |
| Level horizon | Complete | QGC calibration wizard | After sensor plate is remounted |

### Performing accelerometer calibration

Required any time the flight controller is moved or if EKF2 shows
persistent pitch or roll offset at rest.

```
QGroundControl → Vehicle Setup → Sensors → Accelerometer
Follow the on-screen rotation sequence (6 orientations)
Takes approximately 2 minutes
```

### Performing magnetometer calibration

Perform outdoors, away from metal structures, vehicles, or electronics.
Rotate the drone slowly through a figure-eight pattern in all three axes.

```
QGroundControl → Vehicle Setup → Sensors → Compass
Follow the rotation sequence
Takes approximately 60 seconds of rotation
```

---

## 6. Calibration File Reference

All software calibration values are stored in `config/camera_calibration.yaml`.

```yaml
# config/camera_calibration.yaml
# Full calibration reference for DronePi IMX477 + Unitree L1

# Camera intrinsic calibration
# Method: 10x7 checkerboard, 30mm squares, 28 images
# RMS reprojection error: 1.99 pixels
# Resolution: 4056 x 3040 (full IMX477)
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0.0, cx,
         0.0, fy, cy,
         0.0, 0.0, 1.0]

distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]

image_width: 4056
image_height: 3040

# LiDAR-camera extrinsic calibration
# Method: physical measurement from sensor plate
# Status: measured, code verification required before first texture flight
extrinsic:
  translation: [-0.070, 0.000, 0.030]   # [x, y, z] metres, LiDAR frame to camera frame
  rotation_rpy: [0.0, 0.0, 0.0]         # [roll, pitch, yaw] radians

# Camera trigger
# Method: event-driven via /mavros/cam_imu_sync (planned)
# Current fallback: time-based capture in test_texture_live.py
trigger_mode: time_based   # change to cam_imu_sync after PX4 trigger config

# PX4 camera trigger parameters (to configure in QGC)
# TRIG_MODE = 4      PWM trigger on GPIO
# TRIG_INTERFACE = 3 MAVLink trigger messages
# Verify /mavros/cam_imu_sync publishes after configuration
```

---

## 7. Verification Procedure

Run this sequence after any calibration change to confirm the values are
being applied correctly before a texture capture flight.

### Step 1 — Verify extrinsic is not identity

```bash
python3 -c "
import yaml
with open('config/camera_calibration.yaml') as f:
    cal = yaml.safe_load(f)
t = cal['extrinsic']['translation']
r = cal['extrinsic']['rotation_rpy']
print('Translation:', t)
print('Rotation RPY:', r)
assert t != [0.0, 0.0, 0.0], 'Translation is zero — check calibration file'
print('Extrinsic OK')
"
```

### Step 2 — Verify test_texture_live.py reads the calibration file

```bash
grep -n "camera_calibration\|extrinsic\|translation" \
  tests/test_texture_live.py
# Must show a reference to camera_calibration.yaml — not hardcoded values
```

### Step 3 — Verify camera intrinsics load correctly

```bash
python3 -c "
import yaml, numpy as np
with open('config/camera_calibration.yaml') as f:
    cal = yaml.safe_load(f)
K = np.array(cal['camera_matrix']['data']).reshape(3, 3)
D = np.array(cal['distortion_coefficients']['data'])
print('Camera matrix K:')
print(K)
print('Distortion D:', D)
assert K[0,0] > 0, 'fx is zero — calibration file is corrupt'
print('Intrinsics OK')
"
```

### Step 4 — Visual check after first capture

After the first scan with the camera online, process the bag and inspect
the textured mesh:

1. Open the mesh in `meshview.html`
2. Find a straight edge in the scene (building corner, door frame)
3. Toggle between cloud and textured mesh views
4. The texture edge should align with the geometric edge within approximately
   one pixel at 2m range
5. If edges are shifted, adjust `extrinsic.translation` by the measured offset
   and reprocess the same bag

A consistent shift in one direction indicates a translation error.
A rotational misalignment (edges fan out from a central point) indicates
a rotation error in `extrinsic.rotation_rpy`.
