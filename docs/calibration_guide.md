# LiDAR-Camera Extrinsic Calibration Guide

## Overview

The extrinsic calibration defines the rigid transform between the
LiDAR coordinate frame and the camera coordinate frame. This is
needed to project 360-degree photos onto the LiDAR mesh.

## Calibration File

Edit `config/camera_calibration.yaml`:

```yaml
extrinsic:
  translation: [0.0, 0.0, 0.05]  # [x, y, z] meters
  rotation_rpy: [0.0, 0.0, 0.0]  # [roll, pitch, yaw] radians
```

## Measurement Method (Quick)

1. Measure the physical offset between LiDAR center and camera center
2. Enter as [x, y, z] in meters
3. If camera is directly above LiDAR: [0, 0, 0.05] for 5cm offset
4. Rotation is usually [0, 0, 0] if mounted aligned

## Refinement Method (Accurate)

1. Place a calibration target (checkerboard) visible to both sensors
2. Record a bag with LiDAR and camera data
3. Use mutual information or feature matching to refine the transform
4. Update camera_calibration.yaml with refined values

## Verification

After calibration, run a test scan and process it:
```bash
python3 main.py scan
python3 main.py process --latest
```

Check if textures align with geometry in the output model.
If edges appear shifted, adjust the translation values.
