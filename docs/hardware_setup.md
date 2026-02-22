# Hardware Setup

## Bill of Materials

| Component | Model | Interface |
|-----------|-------|-----------|
| Companion Computer | Raspberry Pi 5 (8GB) | — |
| LiDAR | Unitree L1 | USB serial (/dev/ttyUSB0) |
| Camera | Insta360 X3 | WiFi HTTP API |
| Flight Controller | PX4-compatible (Pixhawk) | USB serial (/dev/ttyACM0) |
| IMU | Built into Unitree L1 | Via LiDAR driver |

## Physical Connections

```
┌─────────────────────────────────────┐
│         Raspberry Pi 5              │
├─────────────────────────────────────┤
│ USB0 ─── Unitree L1 LiDAR          │
│ USB1 ─── PX4 Flight Controller     │
│ WiFi ─── Insta360 X3 (hotspot)     │
└─────────────────────────────────────┘
```

## Setup Steps

1. Mount Pi 5 on drone frame (vibration-isolated)
2. Connect Unitree L1 via USB — appears as /dev/ttyUSB0
3. Connect Pixhawk via USB — appears as /dev/ttyACM0
4. Power on Insta360 X3 — connect to its WiFi hotspot
5. Run `python3 main.py check` to verify all devices

## USB Device Rules

The setup.sh script creates udev rules automatically:
- `/dev/ttyUSB*` — LiDAR (mode 0666)
- `/dev/ttyACM*` — PX4 (mode 0666)

## Mounting Notes

- LiDAR: Center of drone, unobstructed 360-degree FOV
- Camera: Near LiDAR for minimal extrinsic offset
- Pi 5: Vibration-isolated, good airflow for cooling
