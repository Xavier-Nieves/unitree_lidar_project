# Hardware Setup

## Bill of Materials

### Drone Platform
- [ ] Drone frame/platform
- [ ] Flight controller (PX4/ArduPilot compatible)
- [ ] Battery and power management

### Sensors
- [ ] **Unitree LiDAR** (L1/L2)
  - Interface: USB/Ethernet
  - Range: 30m-150m
  - Rate: 10Hz
- [ ] **IMU** (if not integrated)
  - Type: 6-axis or 9-axis
  - Interface: I2C/SPI
- [ ] **Camera** (for Hailo AI)
  - Resolution: 1080p minimum
  - Frame rate: 30fps
  - Interface: USB/CSI

### Computing
- [ ] **Companion Computer**
  - Recommended: Jetson Orin Nano / Raspberry Pi 5
  - RAM: 8GB minimum
  - Storage: 128GB SSD

### AI Accelerator
- [ ] **Hailo-8 AI Accelerator** (optional)
  - Interface: M.2/PCIe
  - TOPS: 26

---

## Physical Connections

```
┌─────────────────────────────────────────────┐
│           Companion Computer                 │
│         (Running ROS 2 Stack)                │
├─────────────────────────────────────────────┤
│                                               │
│  USB/Eth ─┬─ Unitree LiDAR                   │
│           ├─ Camera                           │
│           └─ Hailo-8 (M.2)                    │
│                                               │
│  UART/MAVLink ─── Flight Controller          │
│                                               │
└─────────────────────────────────────────────┘
```

---

## Setup Steps

### 1. Hardware Assembly
1. Mount companion computer on drone frame
2. Connect LiDAR via USB/Ethernet
3. Connect camera to USB/CSI port
4. Install Hailo accelerator in M.2 slot
5. Connect flight controller via UART (MAVLink)

### 2. Power Considerations
- Total power budget: Calculate based on components
- Voltage regulation: Ensure stable 5V/12V supply
- Battery capacity: Account for compute + flight time

### 3. Physical Mounting
- LiDAR: Center of drone, minimal obstruction
- Camera: Forward-facing, aligned with LiDAR
- Companion computer: Vibration-isolated mount
- Heat management: Ensure airflow for compute

---

## Configuration Files

### LiDAR Driver Config
Location: `/ws/src/drivers/lidar_driver/config/lidar.yaml`

### Camera Calibration
Location: `/ws/src/drivers/camera_driver/config/calibration.yaml`

### TF Tree
Location: `/ws/src/system/tf_setup/config/transforms.yaml`

---

## TODO
- [ ] Add wiring diagrams
- [ ] Document pinouts for specific hardware
- [ ] Add photos of reference builds
- [ ] Create troubleshooting guide