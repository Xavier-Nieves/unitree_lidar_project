# Flight Procedure

## Pre-Flight Checklist

### Hardware
- [ ] Battery fully charged
- [ ] All sensors powered on
- [ ] LiDAR spinning/active
- [ ] Camera feed visible
- [ ] MAVLink telemetry active
- [ ] Hailo AI accelerator detected
- [ ] GPS lock (if used)
- [ ] Propellers secure

### Software
- [ ] ROS 2 workspace sourced
- [ ] All nodes launching successfully
- [ ] TF tree valid (no missing transforms)
- [ ] Health monitor shows green
- [ ] Recording bag file (optional)

---

## Launch Sequence

### 1. Start Core System
```bash
# Source workspace
source ws/install/setup.bash

# Launch full system
ros2 launch system/launch full_system.launch.py
```

### 2. Verify Node Health
```bash
# Check all nodes are running
ros2 node list

# Check topics are publishing
ros2 topic list
ros2 topic hz /cloud_registered
ros2 topic hz /odometry
```

### 3. Start Mission
```bash
# Upload mission waypoints
ros2 service call /mission/upload custom_msgs/srv/UploadMission "..."

# Arm and takeoff
ros2 topic pub /mission/command std_msgs/String "data: 'arm'"
ros2 topic pub /mission/command std_msgs/String "data: 'takeoff'"
```

---

## In-Flight Operations

### Monitoring
- Watch health monitor status
- Check Point-LIO odometry drift
- Monitor battery level
- Verify detections (if using AI)

### Data Collection
```bash
# Start recording
ros2 bag record -a -o my_flight_$(date +%Y%m%d_%H%M%S)
```

### Emergency Procedures
- **Lost GPS**: Switch to Point-LIO odometry
- **Low Battery**: Automatic RTL (Return to Launch)
- **Sensor Failure**: Failsafe manager triggers safe landing

---

## Post-Flight

### 1. Land and Disarm
```bash
ros2 topic pub /mission/command std_msgs/String "data: 'land'"
ros2 topic pub /mission/command std_msgs/String "data: 'disarm'"
```

### 2. Stop Recording
```bash
# Ctrl+C to stop bag recording
# Verify bag file size and duration
ros2 bag info my_flight_*.db3
```

### 3. Process Data
```bash
# Extract point cloud
python3 ws/src/tools/bag_tools/bag_to_pcd.py my_flight.db3

# Generate mesh
python3 ws/src/tools/cloudcompare_utils/auto_mesh_generator.py
```

### 4. Shutdown System
```bash
# Graceful shutdown
ros2 lifecycle set /all shutdown

# Or kill all
pkill -9 -f ros
```

---

## Safety Considerations

### Environmental
- Avoid flying in high winds (>15 mph)
- Check for GPS interference
- Ensure clear line of sight
- Respect airspace regulations

### Technical
- Always have manual control override
- Test failsafe modes on ground
- Keep backup battery charged
- Monitor compute temperature

---

## TODO
- [ ] Add mission planning GUI instructions
- [ ] Document specific flight modes
- [ ] Create video tutorials
- [ ] Add failure recovery procedures