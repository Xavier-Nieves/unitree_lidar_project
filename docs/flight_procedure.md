# Flight Procedure

## Pre-Flight Checklist

### Hardware
- [ ] Battery fully charged
- [ ] LiDAR connected and spinning
- [ ] PX4 connected and armed-ready
- [ ] Insta360 X3 powered on, WiFi connected
- [ ] Propellers secure
- [ ] GPS lock (if used)

### Software
```bash
# Run health check
python3 main.py check

# Expected output: all [OK]
```

## Flight Modes

### Full Autonomous
```bash
python3 main.py fly --mission config/mission_templates/grid_survey.yaml
```

### Handheld Scan (No Flight)
```bash
python3 main.py scan
# Walk around, Ctrl+C when done
```

### Process Only
```bash
python3 main.py process --latest
# or
python3 main.py process --session 20260222_143000
```

## During Flight

- Monitor terminal output for warnings
- Battery auto-RTL at configured threshold (default 25%)
- Collision avoidance is active automatically
- Camera triggers at each waypoint

## Emergency Procedures

- **Lost link**: PX4 failsafe triggers RTL automatically
- **Low battery**: System triggers RTL at threshold
- **Obstacle detected**: Collision avoidance holds position
- **Manual override**: Always keep RC transmitter ready

## Post-Flight

1. Wait for landing and disarm
2. Check data was saved:
   ```bash
   ls data/flights/
   ```
3. Process the flight:
   ```bash
   python3 main.py process --latest
   ```
4. Output model is in `data/flights/SESSION/processed/model_textured/`
