# RPI5 - Unitree LiDAR L1 with ROS2 Jazzy

This workspace contains the ROS2 Jazzy setup for Unitree LiDAR L1 integration with point_lio on Raspberry Pi 5.

## Directory Structure

```
RPI5/
├── unitree_lidar_L1      # Main executable ⭐
├── ros2_ws/              # ROS2 workspace
│   └── src/              # Source packages
│       ├── unilidar_sdk/ (ROS2: unitree_lidar_ros2)
│       └── point_lio_ros2/ (Package: point_lio)
├── scripts/              # Helper scripts
└── config/               # Configuration files
```

## Quick Start

### ✅ Build is complete! See [SETUP_COMPLETE.md](SETUP_COMPLETE.md) for usage.

1. **Check status:**
   ```bash
   ./unitree_lidar_L1 status
   ```

2. **Run Unitree LiDAR L1:**
   ```bash
   ./unitree_lidar_L1 run
   ```

3. **Run with Point-LIO SLAM:**
   ```bash
   ./unitree_lidar_L1 pointlio
   ```

4. **Open RViz for visualization:**
   ```bash
   ./unitree_lidar_L1 rviz
   ```

## Requirements

- ROS2 Jazzy
- Raspberry Pi 5
- Unitree LiDAR L1
- Dependencies: see setup_workspace.sh

## Configuration

Edit `config/lidar_params.yaml` for LiDAR settings.
Edit `config/pointlio_params.yaml` for point_lio settings.
