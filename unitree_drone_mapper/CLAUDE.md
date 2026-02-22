# Autonomous Drone Mapping System — Project Context

## Overview
Pi 5-native autonomous drone mapping system. Flies a mission, captures
LiDAR + 360-degree photos, builds a textured 3D model.

## Architecture
- **main.py** — Single entry point. Modes: `fly`, `scan`, `process`, `check`, `full`
- **flight/** — PX4 DDS control, mission execution, collision avoidance, precision landing
- **slam/** — Point-LIO process management, trajectory logging, cloud accumulation
- **mapping/** — Post-flight pipeline: cloud cleaning, mesh gen, texture projection
- **drivers/** — Hardware wrappers: Unitree L1 LiDAR, Insta360 X3, IMU
- **utils/** — Shared: ROS helpers, transforms, bag tools, file I/O, logging
- **config/** — All YAML parameters (Point-LIO, PX4, camera calibration, missions)

## Key Design Decisions
- Python-first (except Point-LIO C++ node and LiDAR driver C++ node)
- PX4 DDS (not MAVROS) for flight control
- ROS 2 Jazzy on Ubuntu 24.04
- Post-processing pipeline works without ROS (Open3D only)
- Each flight session is self-contained in data/flights/YYYYMMDD_HHMMSS/

## Hardware
- Raspberry Pi 5 (8GB)
- Unitree L1 LiDAR (USB serial /dev/ttyUSB0)
- Insta360 X3 (WiFi HTTP API)
- PX4 flight controller (USB serial /dev/ttyACM0, DDS over XRCE-DDS)

## ROS 2 Topics
- `/unilidar/cloud` — Raw LiDAR point cloud
- `/unilidar/imu` — LiDAR internal IMU
- `/cloud_registered` — SLAM-registered point cloud
- `/Odometry` — SLAM odometry
- `/path` — SLAM trajectory path
- `/fmu/in/*` — PX4 DDS input topics
- `/fmu/out/*` — PX4 DDS output topics

## Dependencies (installed by setup.sh)
- ROS 2 Jazzy + colcon
- Livox SDK2 + livox_ros_driver2
- Unitree LiDAR SDK + unitree_lidar_ros2
- Point-LIO ROS 2
- px4_msgs + Micro-XRCE-DDS Agent
- Open3D, OpenCV, NumPy, SciPy, PyYAML, requests
