# 🚁 Drone Sensor System - Complete Guide

**Platform:** Raspberry Pi 5
**OS:** Ubuntu 24.04 with ROS2 Jazzy
**Hardware:** Unitree LiDAR L1 + Insta360 X3 Camera
**Status:** ✅ Operational

---

## 📦 System Overview

### What This System Does

This is a complete drone sensor suite combining:
- **360° LiDAR scanning** (Unitree L1) - For navigation, mapping, and obstacle detection
- **360° Camera** (Insta360 X3) - For visual documentation and photogrammetry
- **Real-time SLAM** (Point-LIO) - For position tracking and map building

### Capabilities

- ✅ Real-time 3D environment mapping
- ✅ Autonomous navigation (position tracking)
- ✅ 360° obstacle detection
- ✅ Automated photo capture for 3D reconstruction
- ✅ Synchronized sensor recording
- ✅ ROS2 integration for development

---

## 🚀 Quick Start

### Check System
```bash
./verify_system.sh
```

### Run LiDAR Mapping
```bash
./unitree_lidar_L1 pointlio
```

### Run Camera Photogrammetry
```bash
./run_photogrammetry.sh
```

### View Live Camera Feed
```bash
python3 view_insta360_live.py
```

---

## 📂 System Structure

### Core Files (What You Need)

```
RPI5/
├── unitree_lidar_L1           ⭐ Main LiDAR control
├── test_insta360.sh           📷 Test camera
├── view_insta360_live.py      📹 Live camera viewer
├── run_photogrammetry.sh      📸 Auto-capture images
├── verify_system.sh           ✅ System check
├── README.md                  📖 Quick reference
└── SYSTEM_GUIDE.md            📚 This file

ros2_ws/                       # ROS2 workspace
├── src/
│   ├── unilidar_sdk/          # LiDAR driver
│   ├── point_lio_ros2/        # SLAM algorithm
│   └── insta360_photogrammetry/  # Camera node
└── install/                   # Built packages

config/                        # Configuration files
scripts/                       # Helper scripts
_archive/                      # Old files (can be deleted)

Output directories (created automatically):
├── insta360_captures/         # Quick photos/videos
└── photogrammetry_captures/   # Photogrammetry images
```

---

## 🎮 Commands Reference

### Main LiDAR Executable

```bash
./unitree_lidar_L1 [command]
```

**Commands:**
- `status` - Check system status
- `run` - Launch LiDAR only
- `pointlio` - Launch LiDAR + SLAM mapping
- `rviz` - Launch visualization
- `build` - Rebuild workspace
- `clean` - Clean build files
- `help` - Show help

### Camera Scripts

```bash
# Test camera detection
./test_insta360.sh

# Live viewer (interactive)
python3 view_insta360_live.py
# Press 1: Both lenses (front+back)
# Press 2: Front lens only
# Press 3: Back lens only
# Press S: Save snapshot
# Press Q: Quit

# Photogrammetry (auto-capture)
./run_photogrammetry.sh                    # Every 2 seconds
./run_photogrammetry.sh 1.0                # Every 1 second
./run_photogrammetry.sh 5.0 ~/my_scan     # Custom interval & folder
```

### System Verification

```bash
./verify_system.sh                         # Check all components
```

---

## 💡 Common Use Cases

### 1. Indoor Mapping

**Goal:** Create 3D map of indoor space

```bash
# Start LiDAR + SLAM
./unitree_lidar_L1 pointlio

# Walk slowly through space
# Map builds in real-time
# Ctrl+C when done
```

**Output:** Real-time map in RViz, trajectory saved

---

### 2. Object Scanning (Photogrammetry)

**Goal:** Capture images for 3D model reconstruction

```bash
# Start auto-capture (every 3 seconds)
./run_photogrammetry.sh 3.0

# Rotate object slowly on turntable
# Or walk around object
# Ctrl+C when complete
```

**Output:** Images in `photogrammetry_captures/`

**Process:**
1. Copy images to computer
2. Open in Meshroom (free software)
3. Export 3D model

---

### 3. Drone Flight Mission

**Goal:** Collect both LiDAR and camera data during flight

**Terminal 1 - LiDAR:**
```bash
./unitree_lidar_L1 pointlio
```

**Terminal 2 - Camera:**
```bash
./run_photogrammetry.sh 1.5
```

**Terminal 3 - Record Everything:**
```bash
source ros2_ws/install/setup.bash
ros2 bag record -a -o flight_$(date +%Y%m%d_%H%M%S)
```

**Output:**
- LiDAR point clouds → ROS bag
- Camera images → `photogrammetry_captures/`
- Position tracking → ROS bag

---

### 4. Real-time Monitoring

**Goal:** See both LiDAR and camera feeds live

**Terminal 1 - LiDAR Visualization:**
```bash
./unitree_lidar_L1 pointlio
# RViz opens automatically
```

**Terminal 2 - Camera Feed:**
```bash
python3 view_insta360_live.py
```

---

## 🔧 Configuration

### LiDAR Settings

**Connection:**
- Device: `/dev/ttyUSB0` (auto-detected)
- Frame rate: ~10 Hz
- Range: 0-50 meters

**SLAM Parameters:**
```bash
# Edit Point-LIO configuration
nano ros2_ws/src/point_lio_ros2/config/unilidar_l1.yaml
```

**Common adjustments:**
- `filter_size_surf` - Point cloud density (0.1-0.5)
- `cube_side_length` - Map size (1000 default)

---

### Camera Settings

**Connection:**
- Device: `/dev/video0` (auto-detected)
- Resolution: 1920x1080 @ 30fps
- Format: Dual fisheye (360° coverage)

**Photogrammetry:**
```bash
# Edit capture interval and other settings in:
run_photogrammetry.sh
```

**Manual configuration:**
```bash
source ros2_ws/install/setup.bash
ros2 launch insta360_photogrammetry photogrammetry.launch.py \
    capture_interval:=2.0 \
    output_dir:=./my_captures \
    video_device:=/dev/video0
```

---

## 📊 Understanding the Data

### LiDAR Data (Point Clouds)

**What it is:** Millions of 3D points representing the environment

**ROS Topics:**
- `/unilidar/cloud` - Raw LiDAR data
- `/cloud_registered` - SLAM-processed data
- `/Odometry` - Robot position
- `/path` - Movement trajectory

**View in real-time:**
```bash
# LiDAR publishes automatically when running
ros2 topic hz /unilidar/cloud        # Check rate (~10 Hz)
ros2 topic echo /Odometry            # See position
```

---

### Camera Data (Images)

**What it is:** 1920x1080 images, dual fisheye format

**Format:** Two ~180° fisheye lenses side-by-side
- Left half = Front camera
- Right half = Back camera
- Together = Full 360° coverage

**Why split?** This is webcam mode output. For 360° panoramas, use Insta360 Studio software to stitch.

**ROS Topics:**
- `/insta360_x3/image_raw` - Uncompressed images
- `/insta360_x3/image_raw/compressed` - Compressed (smaller)

---

## 🗺️ Data Processing Workflows

### Workflow 1: LiDAR Map → Point Cloud File

```bash
# 1. Run mapping
./unitree_lidar_L1 pointlio

# 2. In another terminal, save point cloud
source ros2_ws/install/setup.bash
ros2 run pcl_ros pointcloud_to_pcd \
    input:=/cloud_registered _prefix:=my_map

# 3. Open in CloudCompare (on your computer)
```

---

### Workflow 2: Images → 3D Model

```bash
# 1. Capture images
./run_photogrammetry.sh 2.0

# 2. Copy to your computer
# From your computer:
scp -r dronepi@dronepi-desktop:~/unitree_lidar_project/RPI5/photogrammetry_captures ./

# 3. Process in Meshroom
# - Download Meshroom (free): https://alicevision.org
# - Drag images into Meshroom
# - Click "Start"
# - Export 3D model (OBJ/FBX)
```

---

### Workflow 3: Combined LiDAR + Camera → Textured 3D Map

```bash
# 1. Record synchronized data
# Terminal 1: LiDAR
./unitree_lidar_L1 pointlio

# Terminal 2: Camera
./run_photogrammetry.sh 1.5

# Terminal 3: Record
source ros2_ws/install/setup.bash
ros2 bag record -a -o combined_$(date +%Y%m%d)

# 2. Process LiDAR point cloud (ROS bag → PCD file)
# 3. Process images (Meshroom → 3D model)
# 4. Merge in CloudCompare or Blender
```

---

## 🛠️ Troubleshooting

### LiDAR Not Working

**Symptom:** No point cloud in RViz, or not mapping

**Checks:**
```bash
# 1. Verify connection
ls /dev/ttyUSB*           # Should see /dev/ttyUSB0

# 2. Check status
./unitree_lidar_L1 status

# 3. Test LiDAR only
./unitree_lidar_L1 run
```

**Common fixes:**
- Unplug and replug LiDAR USB cable
- Check LiDAR power indicator light
- Ensure clear 360° view (no obstructions)
- **Move around** - SLAM needs motion to initialize

---

### Camera Not Detected

**Symptom:** Camera not found by `test_insta360.sh`

**Checks:**
```bash
# 1. Verify camera connected
lsusb | grep -i insta

# 2. Check USB mode
# Camera must be in "USB Webcam" mode, NOT "Mass Storage"
# On camera: Settings → USB Mode → USB Webcam

# 3. Test after reconnecting
./test_insta360.sh
```

**Common fixes:**
- Switch camera to "USB Webcam" mode
- Unplug and replug USB cable
- Try different USB port
- Check camera battery is charged

---

### Point-LIO Not Initializing

**Symptom:** LiDAR running but no map building

**Solution:**
- **Move the sensor!** Point-LIO needs motion to initialize
- Point at feature-rich environment (corners, objects)
- Avoid blank walls or empty rooms
- Give it 5-10 seconds while moving

---

### ROS2 Workspace Issues

**Symptom:** Commands not found or package errors

**Fix:**
```bash
# Rebuild workspace
source /opt/ros/jazzy/setup.bash
cd ~/unitree_lidar_project/RPI5/ros2_ws
colcon build

# Source environment
source ~/.bashrc
```

---

### Performance Issues

**Symptom:** System running slow, dropping frames

**Solutions:**
- **Close RViz** if running on RPI5 (CPU intensive)
- Run RViz on another computer via ROS2 network
- Reduce LiDAR point density in config
- Lower camera capture interval
- Monitor with `htop`

---

## 📡 ROS2 Topics Quick Reference

### LiDAR Topics
```
/unilidar/cloud          # Raw point cloud (sensor_msgs/PointCloud2)
/unilidar/imu            # IMU data (sensor_msgs/Imu)
/cloud_registered        # Processed cloud (sensor_msgs/PointCloud2)
/Odometry                # Position estimate (nav_msgs/Odometry)
/path                    # Trajectory (nav_msgs/Path)
```

### Camera Topics
```
/insta360_x3/image_raw              # Raw images (sensor_msgs/Image)
/insta360_x3/camera_info            # Camera parameters
/insta360_x3/image_raw/compressed   # Compressed (sensor_msgs/CompressedImage)
```

### Useful Commands
```bash
ros2 topic list                        # List all topics
ros2 topic hz /unilidar/cloud          # Check publish rate
ros2 topic echo /Odometry              # View odometry data
ros2 node list                         # List running nodes
ros2 bag record -a                     # Record everything
```

---

## 🔄 System Maintenance

### Update ROS2 Packages
```bash
sudo apt update
sudo apt upgrade ros-jazzy-*
```

### Rebuild Workspace
```bash
cd ~/unitree_lidar_project/RPI5
./unitree_lidar_L1 clean
./unitree_lidar_L1 build
```

### Clean Old Data
```bash
# Clean captures
rm -rf insta360_captures/*
rm -rf photogrammetry_captures/*

# Clean ROS bags
rm -rf *.db3 *.yaml
```

### Backup Configuration
```bash
# Backup configs
tar -czf configs_backup.tar.gz config/ ros2_ws/src/*/config/
```

---

## 🎓 Learning Resources

### ROS2 Basics
- Official docs: https://docs.ros.org/en/jazzy/
- Tutorials: Start with ROS2 humble tutorials

### LiDAR & SLAM
- Point-LIO paper: https://github.com/hku-mars/Point-LIO
- Point cloud processing: CloudCompare, PCL

### Photogrammetry
- Meshroom (free): https://alicevision.org
- OpenDroneMap (drone mapping): https://opendronemap.org
- Photogrammetry guide: Search "photogrammetry best practices"

### Tools
- **CloudCompare** - Point cloud viewer
- **Meshroom** - Free photogrammetry
- **RViz2** - ROS visualization
- **Blender** - 3D modeling

---

## 📋 Specifications

### Hardware
- **Raspberry Pi 5** - 8GB RAM recommended
- **Unitree LiDAR L1** - 360° TOF laser scanner
- **Insta360 X3** - Dual fisheye 360° camera

### Performance
- **LiDAR rate**: ~10 Hz
- **Camera rate**: 30 fps
- **SLAM update**: Real-time
- **Photogrammetry**: 0.5-10 second intervals

### Data Output
- **Point clouds**: PCD, PLY formats
- **Images**: JPEG (95% quality)
- **ROS bags**: Compressed format
- **Maps**: Real-time in RViz

---

## 🆘 Getting Help

### Quick Diagnostics
```bash
./verify_system.sh           # Run full system check
./unitree_lidar_L1 status    # Check LiDAR system
./test_insta360.sh           # Test camera
```

### View Logs
```bash
# ROS2 logs
ros2 run rqt_console rqt_console

# Or check terminal output
```

### Check Archived Docs
If you need detailed documentation:
```bash
ls _archive/                 # Old detailed guides
cat _archive/TROUBLESHOOTING.md     # Detailed troubleshooting
cat _archive/INSTA360_GUIDE.md      # Full camera guide
```

---

## ✨ Summary

### System Status: ✅ OPERATIONAL

**What works:**
- ✅ Real-time LiDAR scanning
- ✅ SLAM mapping and localization
- ✅ 360° camera capture
- ✅ Automated photogrammetry
- ✅ Synchronized sensor recording
- ✅ ROS2 integration

**Main commands:**
```bash
./verify_system.sh              # Check system
./unitree_lidar_L1 pointlio     # Start mapping
./run_photogrammetry.sh         # Capture images
python3 view_insta360_live.py   # View camera
```

**Documentation:**
- This file (SYSTEM_GUIDE.md) - Complete guide
- README.md - Quick reference
- _archive/ - Detailed guides (if needed)

---

**Your drone sensor system is ready to use!** 🚁📸🗺️

For questions, run `./verify_system.sh` to check system status.

---

*Last updated: 2026-02-15*
*System version: 1.0*
*All components verified and operational*
