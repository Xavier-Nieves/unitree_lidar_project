# Project Restructure Proposal
## Autonomous Drone Mapping System — Pi 5 Native

---

## Proposed Directory Structure

```
unitree_drone_mapper/
│
├── main.py                          # Single entry point — orchestrates everything
├── config.yaml                      # All system configuration in one place
├── setup.sh                         # One-shot full system setup
├── README.md                        # Project overview
├── CLAUDE.md                        # Updated project context
│
├── flight/                          # Everything that runs DURING flight
│   ├── __init__.py
│   ├── px4_interface.py             # PX4 DDS offboard control, arm/disarm/mode
│   ├── mission_executor.py          # Waypoint execution, mission state machine
│   ├── collision_avoidance.py       # LiDAR → ObstacleDistance for PX4
│   ├── precision_landing.py         # ArUco detection + landing controller
│   └── camera_trigger.py            # Insta360 X3 HTTP API capture + metadata log
│
├── slam/                            # Real-time SLAM (runs during flight)
│   ├── __init__.py
│   ├── point_lio_runner.py          # Launch/manage Point-LIO process
│   ├── trajectory_logger.py         # Subscribe /odometry, /path → save poses
│   └── cloud_accumulator.py         # Subscribe /cloud_registered → save .pcd
│
├── mapping/                         # Post-flight processing pipeline
│   ├── __init__.py
│   ├── pipeline.py                  # Master post-processing orchestrator
│   ├── cloud_cleaner.py             # Statistical outlier removal, voxel downsample
│   ├── mesh_generator.py            # Normal estimation + Poisson reconstruction
│   ├── trajectory_extractor.py      # Extract SLAM poses from bag / saved trajectory
│   ├── camera_pose_builder.py       # SLAM pose + extrinsic → camera poses
│   ├── spherical_projector.py       # Direct equirect texture projection (no virtual cams)
│   └── mesh_refiner.py              # Seam reduction, color correction, cleanup
│
├── drivers/                         # Hardware interfaces
│   ├── __init__.py
│   ├── lidar_driver.py              # Unitree L1 launch/config wrapper
│   ├── insta360_driver.py           # Insta360 X3 WiFi HTTP API client
│   └── imu_interface.py             # IMU data handling if separate from LiDAR
│
├── utils/                           # Shared utilities
│   ├── __init__.py
│   ├── ros_helpers.py               # Common ROS 2 QoS profiles, node utilities
│   ├── transforms.py                # Coordinate frame math, quaternion ops
│   ├── bag_tools.py                 # Bag reading, topic extraction
│   ├── file_io.py                   # PCD/PLY/OBJ read/write helpers
│   └── logger.py                    # Unified logging config
│
├── config/                          # Configuration files
│   ├── point_lio_params.yaml        # Point-LIO tuning parameters
│   ├── px4_params.yaml              # PX4 flight parameters
│   ├── camera_calibration.yaml      # Insta360 intrinsics + LiDAR↔camera extrinsic
│   └── mission_templates/           # Pre-defined flight patterns
│       ├── grid_survey.yaml
│       └── perimeter_scan.yaml
│
├── data/                            # All flight data lives here
│   ├── .gitkeep
│   └── flights/                     # Auto-created per flight session
│       └── YYYYMMDD_HHMMSS/         # Example session folder
│           ├── raw/
│           │   ├── bags/            # ROS 2 bag files
│           │   ├── images/          # Insta360 equirectangular captures
│           │   └── metadata/        # GPS stamps, pose log, flight log
│           ├── processed/
│           │   ├── cloud_raw.pcd    # Raw registered cloud from SLAM
│           │   ├── cloud_clean.pcd  # After outlier removal
│           │   ├── mesh.obj         # LiDAR geometry mesh
│           │   ├── camera_poses.json # Derived camera poses
│           │   └── model_textured/  # Final output
│           │       ├── model.obj
│           │       ├── model.mtl
│           │       └── texture.png
│           └── logs/                # Session-specific logs
│               ├── flight.log
│               ├── slam.log
│               └── processing.log
│
├── launch/                          # ROS 2 launch files
│   ├── flight.launch.py             # Full flight stack
│   ├── slam_only.launch.py          # SLAM without flight control
│   └── sensors_only.launch.py       # Just drivers for testing
│
├── tests/                           # Test scripts
│   ├── test_takeoff.py              # Basic flight test
│   ├── test_collision.py            # Collision avoidance test
│   ├── test_obstacle_sim.py         # Simulated obstacles
│   └── test_processing.py           # Post-processing on sample data
│
├── archive/                         # Old Docker files for reference
│   ├── docker/
│   │   ├── docker-compose.yml
│   │   ├── lidar/Dockerfile
│   │   └── px4_dds/Dockerfile
│   └── README.md                    # "These are archived Docker dev configs"
│
└── docs/
    ├── architecture.md              # Updated system design
    ├── hardware_setup.md            # Physical assembly guide
    ├── calibration_guide.md         # LiDAR↔camera extrinsic calibration
    ├── flight_procedure.md          # Operational procedures
    └── pipeline_overview.md         # How the mapping pipeline works
```

---

## How main.py Works

`main.py` is the single entry point. It manages the full lifecycle:

```
┌─────────────────────────────────────────────────────────┐
│                      main.py                             │
│                                                          │
│  PHASE 1: PRE-FLIGHT                                    │
│  ├─ System health check (sensors, PX4, storage)         │
│  ├─ Create session folder (data/flights/TIMESTAMP/)     │
│  ├─ Load mission from config or CLI                     │
│  └─ Initialize all nodes                                │
│                                                          │
│  PHASE 2: FLIGHT                                        │
│  ├─ Start SLAM (Point-LIO + trajectory logger)          │
│  ├─ Start collision avoidance                           │
│  ├─ Execute mission waypoints                           │
│  ├─ Trigger Insta360 captures at waypoints              │
│  ├─ Log all metadata (GPS, pose, timestamps)            │
│  └─ Precision landing on return                         │
│                                                          │
│  PHASE 3: POST-FLIGHT PROCESSING                        │
│  ├─ Export registered cloud + trajectory                 │
│  ├─ Clean point cloud (outlier removal)                 │
│  ├─ Generate mesh (Poisson reconstruction)              │
│  ├─ Build camera poses from SLAM trajectory             │
│  ├─ Texture projection (spherical → mesh)               │
│  ├─ Mesh refinement                                     │
│  └─ Save final textured model                           │
│                                                          │
│  OUTPUT: data/flights/SESSION/processed/model_textured/  │
└─────────────────────────────────────────────────────────┘
```

It can also be run in partial modes:
- `python3 main.py fly` — flight + data collection only
- `python3 main.py process --session 20260219_143000` — post-processing only
- `python3 main.py process --latest` — process most recent flight
- `python3 main.py check` — system health check only
- `python3 main.py` — full end-to-end (default)

---

## How setup.sh Works

Single script that takes a fresh Ubuntu 24.04 Pi 5 to fully operational:

```
setup.sh
├── [1/7] System update + base tools (git, cmake, pip, etc.)
├── [2/7] Install ROS 2 Jazzy + colcon
├── [3/7] Install LiDAR stack (Livox SDK2, Livox ROS driver, Unitree SDK)
├── [4/7] Install Point-LIO ROS 2
├── [5/7] Install PX4 dependencies (px4_msgs, XRCE-DDS agent)
├── [6/7] Install mapping dependencies (Open3D, OpenCV, numpy, scipy)
├── [7/7] Configure system
│   ├── udev rules for USB devices
│   ├── User group permissions
│   ├── CPU governor → performance
│   ├── Source workspace in .bashrc
│   ├── Build entire colcon workspace
│   └── Create systemd service for auto-start
│
└── Done — reboot and fly
```

---

## File Migration Map

### Files that move (with refactoring):

| Old Location | New Location | Changes |
|-------------|-------------|---------|
| `docker/px4_dds/lidar_collision_node.py` | `flight/collision_avoidance.py` | Clean up, add config params |
| `docker/px4_dds/aruco_landing_node.py` | `flight/precision_landing.py` | Merge with landing_controller |
| `docker/px4_dds/landing_controller.py` | `flight/precision_landing.py` | Merge into single module |
| `docker/px4_dds/test_takeoff.py` | `tests/test_takeoff.py` | Minor cleanup |
| `docker/px4_dds/test_collision_flight.py` | `tests/test_collision.py` | Minor cleanup |
| `docker/px4_dds/test_obstacle_sim.py` | `tests/test_obstacle_sim.py` | Minor cleanup |
| `src/tools/bag_tools/bag_to_pcd.py` | `utils/bag_tools.py` | Generalize, add trajectory extraction |
| `src/tools/cloudcompare_utils/auto_mesh_generator.py` | Split → `mapping/cloud_cleaner.py` + `mapping/mesh_generator.py` | Decouple from live ROS, make file-based |
| `src/tools/cloudcompare_utils/optimized_mesh_generator.py` | Superseded by above split | Merge best parts |
| `scripts/common/record_bag.sh` | Absorbed into `slam/trajectory_logger.py` | Programmatic recording |
| `scripts/pi/install_dependencies.sh` | `setup.sh` | Updated for Ubuntu 24.04 + Jazzy |
| `scripts/pi/build_workspace.sh` | `setup.sh` | Merged into unified setup |
| `scripts/pi/setup_services.sh` | `setup.sh` | Merged into unified setup |

### Files that archive:

| File | Reason |
|------|--------|
| `docker/` (all) | Not used on Pi 5 |
| `docker/px4_dds/run.sh` | Docker-only helper |
| `docker/px4_dds/entrypoint.sh` | Docker-only |
| `docker/px4_dds/drone_test_world.sdf` | Can go to tests/ or archive |
| `scripts/lidar/start_lidar.sh` | Docker-only menu |
| `scripts/lidar/save_pointcloud.sh` | Docker-only copy |
| `scripts/mavros/start_mavros.sh` | MAVROS replaced by DDS |
| `scripts/mavros/px4_sitl.sh` | Dev-only, keep in archive |

### Files that are new:

| File | Purpose |
|------|---------|
| `main.py` | Entry point orchestrator |
| `config.yaml` | Unified configuration |
| `setup.sh` | One-shot system setup |
| `flight/px4_interface.py` | Clean PX4 DDS abstraction |
| `flight/mission_executor.py` | Waypoint state machine |
| `flight/camera_trigger.py` | Insta360 HTTP capture |
| `slam/point_lio_runner.py` | Point-LIO process manager |
| `slam/trajectory_logger.py` | Real-time pose saving |
| `slam/cloud_accumulator.py` | Real-time cloud saving |
| `mapping/pipeline.py` | Post-processing orchestrator |
| `mapping/trajectory_extractor.py` | SLAM pose extraction |
| `mapping/camera_pose_builder.py` | Extrinsic transform application |
| `mapping/spherical_projector.py` | Equirect texture projection |
| `mapping/mesh_refiner.py` | Final cleanup |
| `drivers/insta360_driver.py` | Camera WiFi API client |
| `utils/transforms.py` | Shared coordinate math |
| `config/camera_calibration.yaml` | Sensor calibration data |
| `launch/*.launch.py` | ROS 2 launch files |

---

## config.yaml Structure

```yaml
# System
system:
  ros_distro: jazzy
  data_dir: ~/unitree_drone_mapper/data/flights
  log_level: INFO

# Hardware
lidar:
  device: /dev/ttyUSB0
  model: unitree_l1

imu:
  source: lidar_internal  # L1 has built-in IMU

camera:
  model: insta360_x3
  wifi_ssid: Insta360_X3_XXXX
  capture_interval_m: 2.0  # meters between captures
  resolution: [5760, 2880]  # equirectangular

px4:
  connection: /dev/ttyACM0:921600
  dds_port: 8888

# Flight
flight:
  takeoff_altitude: 5.0
  cruise_speed: 2.0
  collision_min_distance_m: 2.0
  rtl_battery_percent: 25

# SLAM
slam:
  point_lio_config: config/point_lio_params.yaml
  save_trajectory: true
  save_cloud: true

# Mapping Pipeline
mapping:
  cloud_cleaning:
    outlier_neighbors: 20
    outlier_std_ratio: 2.0
    voxel_size: 0.02  # meters, 0 = no downsampling
  meshing:
    method: poisson
    poisson_depth: 9
  texturing:
    projection: spherical  # direct equirectangular projection
    blend_overlapping: true
    max_images: 500
  output_formats: [obj]

# Calibration
calibration:
  lidar_to_camera:
    translation: [0.0, 0.0, 0.05]  # [x, y, z] meters
    rotation: [0.0, 0.0, 0.0]      # [roll, pitch, yaw] radians
```

---

## Key Design Decisions

### Why Python main.py instead of bash?
- Can import and orchestrate ROS 2 nodes directly
- Proper error handling and state management
- Unified logging
- Can run post-processing without ROS if needed
- Cleaner than bash calling Python calling ROS

### Why merge landing nodes?
`aruco_landing_node.py` and `landing_controller.py` are tightly coupled — one detects the target, the other acts on it. Single module is cleaner.

### Why split auto_mesh_generator into 3 files?
The original does cloud cleaning + meshing + saving in one monolithic ROS node. Splitting lets you:
- Run cleaning independently (iterate on parameters)
- Run meshing independently (try different depths)
- Use the cleaner/mesher without ROS (post-processing mode)

### Why data/flights/SESSION/ structure?
Each flight is self-contained. You can:
- Process any flight independently
- Compare results across flights
- Archive or delete sessions easily
- Never overwrite previous data
