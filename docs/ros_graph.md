# ROS 2 Graph

## Node and Topic Structure

### Drivers Layer

#### LiDAR Driver
- **Node**: `/lidar_driver`
- **Published Topics**:
  - `/lidar/points` (sensor_msgs/PointCloud2) - Raw LiDAR points
  - `/lidar/imu` (sensor_msgs/Imu) - LiDAR IMU data

#### IMU Driver
- **Node**: `/imu_driver`
- **Published Topics**:
  - `/imu/data` (sensor_msgs/Imu) - IMU measurements

#### Camera Driver
- **Node**: `/camera_driver`
- **Published Topics**:
  - `/camera/image_raw` (sensor_msgs/Image) - Raw camera images
  - `/camera/camera_info` (sensor_msgs/CameraInfo) - Camera calibration

#### MAVLink Bridge
- **Node**: `/mavlink_bridge`
- **Published Topics**:
  - `/mavlink/state` (custom_msgs/VehicleState)
  - `/mavlink/battery` (sensor_msgs/BatteryState)
- **Subscribed Topics**:
  - `/mavlink/cmd_vel` (geometry_msgs/Twist)

---

### Perception Layer

#### Point-LIO SLAM
- **Node**: `/point_lio`
- **Published Topics**:
  - `/cloud_registered` (sensor_msgs/PointCloud2) - Registered point cloud
  - `/odometry` (nav_msgs/Odometry) - SLAM odometry
  - `/path` (nav_msgs/Path) - Trajectory
- **Subscribed Topics**:
  - `/lidar/points`
  - `/imu/data`

#### Hailo Inference
- **Node**: `/hailo_inference`
- **Published Topics**:
  - `/detections` (vision_msgs/Detection2DArray)
- **Subscribed Topics**:
  - `/camera/image_raw`

#### Semantic Fusion
- **Node**: `/semantic_fusion`
- **Published Topics**:
  - `/semantic_cloud` (sensor_msgs/PointCloud2) - Semantically labeled point cloud
- **Subscribed Topics**:
  - `/cloud_registered`
  - `/detections`

---

### Control Layer

#### Mission Control
- **Node**: `/mission_control`
- **Published Topics**:
  - `/mission/status` (custom_msgs/MissionStatus)
  - `/mavlink/cmd_vel`
- **Subscribed Topics**:
  - `/odometry`
  - `/mission/waypoints`

#### Health Monitor
- **Node**: `/health_monitor`
- **Published Topics**:
  - `/health/status` (custom_msgs/HealthStatus)
- **Subscribed Topics**:
  - All node heartbeats

#### Failsafe Manager
- **Node**: `/failsafe_manager`
- **Published Topics**:
  - `/emergency/stop` (std_msgs/Bool)
- **Subscribed Topics**:
  - `/health/status`
  - `/mavlink/battery`

---

## TF Tree

```
map
 └─ odom
     └─ base_link
         ├─ imu_link
         ├─ lidar_link
         └─ camera_link
```

## TODO
- [ ] Add message frequency requirements
- [ ] Document QoS settings
- [ ] Add service interfaces
- [ ] Create visualization of full graph