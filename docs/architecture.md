# System Architecture

## Overview

The Autonomous Drone Mapping System runs natively on a Raspberry Pi 5
with ROS 2 Jazzy. It uses PX4 DDS (not MAVROS) for flight control.

## Component Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                   main.py (orchestrator)                     │
├─────────────┬────────────┬──────────────┬───────────────────┤
│   flight/   │   slam/    │   mapping/   │    drivers/       │
│             │            │              │                   │
│ px4_iface   │ point_lio  │ pipeline     │ lidar_driver      │
│ mission_exe │ traj_log   │ cloud_clean  │ insta360_driver   │
│ collision   │ cloud_acc  │ mesh_gen     │ imu_interface     │
│ prec_land   │            │ traj_extract │                   │
│ cam_trigger │            │ cam_poses    │                   │
│             │            │ sph_project  │                   │
│             │            │ mesh_refine  │                   │
├─────────────┴────────────┴──────────────┴───────────────────┤
│                    utils/ + config/                           │
│ ros_helpers | transforms | bag_tools | file_io | logger      │
└─────────────────────────────────────────────────────────────┘
```

## Data Flow

### Phase 1: Pre-Flight
1. System health check (sensors, PX4, storage)
2. Create session folder (data/flights/TIMESTAMP/)
3. Load mission from config or CLI
4. Initialize all nodes

### Phase 2: Flight / Scan
1. LiDAR driver -> /unilidar/cloud + /unilidar/imu
2. Point-LIO SLAM -> /cloud_registered + /Odometry + /path
3. CloudAccumulator saves registered clouds
4. TrajectoryLogger saves SLAM poses
5. CollisionAvoidance -> ObstacleDistance to PX4
6. MissionExecutor drives waypoints via PX4Interface
7. CameraTrigger captures Insta360 photos at waypoints

### Phase 3: Post-Processing
1. Load raw point cloud from session
2. Clean: outlier removal + voxel downsample
3. Generate mesh: normals + Poisson reconstruction
4. Extract SLAM trajectory from saved poses
5. Build camera poses: SLAM pose + extrinsic calibration
6. Project textures: equirectangular images onto mesh
7. Refine: seam smoothing + color correction
8. Save final textured model (OBJ + MTL + texture)

## Session Directory Structure

```
data/flights/YYYYMMDD_HHMMSS/
├── raw/
│   ├── bags/            # ROS 2 bag files
│   ├── images/          # Insta360 equirectangular captures
│   └── metadata/        # trajectory.json, capture_log.json
├── processed/
│   ├── cloud_raw.pcd
│   ├── cloud_clean.pcd
│   ├── mesh.obj
│   ├── camera_poses.json
│   └── model_textured/
│       ├── model.obj
│       ├── model.mtl
│       └── texture.png
└── logs/
    ├── flight.log
    ├── slam.log
    └── processing.log
```
