# DronePi Watchdog — Refactor & Feedback Systems Report

**Project:** DronePi Capstone — Autonomous LiDAR Mapping Drone  
**Component:** Flight Stack Watchdog (`drone_watchdog.py`)  
**Author:** DronePi Capstone Team  
**Version:** 2.0.0  

---

## 1. Overview

This report documents the refactoring of the DronePi flight stack watchdog from a single monolithic script into a modular architecture, along with the design and partial implementation of two operator feedback systems: Pixhawk buzzer feedback and an external RGB LED indicator.

The watchdog runs as a `systemd` service (`drone-watchdog`) on the Raspberry Pi 5 companion computer. It supervises three flight stack subprocesses (Point-LIO, SLAM bridge, ROS 2 bag recorder), responds to RC toggle input via MAVROS, and triggers post-flight mesh processing after each scan session.

---

## 2. Original Architecture

The original `drone_watchdog.py` was a single 516-line file containing:

- ROS 2 node setup and MAVROS subscriptions
- RC channel edge detection and debounce logic
- Pixhawk buzzer publishing via `PlayTuneV2`
- Subprocess launch and teardown for three processes
- ROS 2 bag record command construction
- Post-flight script trigger (fire-and-forget)
- Main polling loop with lock file state machine
- A single shared `log()` function

**Problems with the monolithic structure:**
- Any change to one concern (e.g. buzzer tunes) required navigating the entire file
- Post-flight was launched as a detached subprocess with `stdout=DEVNULL` — failures were invisible in `journalctl`
- No mechanism to know when post-flight finished, making end-of-processing feedback impossible
- All configuration constants were scattered inline with logic
- Testing individual components in isolation was not possible

---

## 3. Refactored Architecture

### 3.1 Directory Structure

```
drone_watchdog.py              # Thin orchestrator — main loop only
watchdog_core/
  __init__.py                  # Package init with public API re-exports
  logging_utils.py             # Shared timestamped log() function
  buzzer.py                    # Tune constants + PostflightBeeper thread
  mavros_reader.py             # ROS 2 node: state, RC input, buzzer output
  flight_stack.py              # Process helpers, command builders, FlightStack
  postflight.py                # PostflightMonitor: subprocess, log pipe, beeper
  led_controller.py            # RGB LED state machine (documented, pending wiring)
```

### 3.2 Module Responsibilities

#### `logging_utils.py`
Single `log(msg)` function with `HH:MM:SS` timestamp, flushed to stdout (captured by `journalctl`). All modules import from here so output format is consistent and changeable in one place.

#### `buzzer.py`
All QBASIC Format 1 tune strings and the `PostflightBeeper` class.

| Constant | Tune | Trigger |
|---|---|---|
| `TUNE_START` | Rising scale (`T240L16O5CEGC`) | Flight stack starting |
| `TUNE_STOP` | Falling scale (`T240L16O5CG<EGC`) | Flight stack stopping |
| `TUNE_ACK` | Single beep (`T200L32O6C`) | Button press / FCU connected |
| `TUNE_ERROR` | Staccato ×3 (`T200L16O5CPCPC`) | Error condition |
| `TUNE_PROCESSING` | Mid pulse (`T200L32O4G`) | Post-flight heartbeat (every 5s) |
| `TUNE_DONE` | Two-note up (`T240L16O5CE`) | Post-flight complete |

`PostflightBeeper` runs on a daemon thread and plays `TUNE_PROCESSING` every 5 seconds while post-flight is active. It sleeps in 100ms increments so `stop()` is responsive without blocking.

#### `mavros_reader.py`
Encapsulates the entire ROS 2 interface:
- Subscribes to `/mavros/state` (RELIABLE QoS) for armed flag, flight mode, FCU connection
- Subscribes to `/mavros/rc/in` (BEST_EFFORT QoS) for RC channel values
- Publishes to `/mavros/play_tune` for Pixhawk buzzer output
- Implements rising-edge RC toggle detection with 200ms debounce on CH6 (index 5)
- Runs `rclpy.spin()` on a daemon thread — never blocks the main loop
- Exposes `beep_start()`, `beep_stop()`, `beep_ack()`, `beep_error()`, `beep_processing()`, `beep_done()` convenience wrappers

#### `flight_stack.py`
Manages the three scan session subprocesses:

| Process | Launch delay | Stop order |
|---|---|---|
| Point-LIO | First, 3s wait | Last (stopped third) |
| SLAM bridge | Second, 1s wait | Middle (stopped second) |
| Bag recorder | Third, no wait | First (stopped first — ensures clean MCAP finalisation) |

Each process is launched in its own process group (`os.setsid`) so SIGINT propagates to the full subprocess tree. Stop sequence: SIGINT → 5s grace → SIGKILL.

`FlightStack` receives a `postflight_fn` callable at construction — it does not import `postflight.py` directly, keeping the two modules decoupled and independently testable.

`check_health()` polls all three process handles each monitor cycle and logs warnings if any exited unexpectedly. This is the hook point for future LED critical error transitions.

#### `postflight.py`
`PostflightMonitor` replaced the original fire-and-forget `_trigger_postflight()` function with a fully monitored subprocess:

- Launches `run_postflight.py --auto --skip-wait` via `subprocess.Popen`
- Merges `stderr` into `stdout` (`stderr=STDOUT`) so crash tracebacks are captured
- Reads stdout line-by-line on a daemon thread and re-logs each line prefixed `[POSTFLIGHT]` — visible in `journalctl -u drone-watchdog`
- Plays `TUNE_PROCESSING` heartbeat every 5s via `PostflightBeeper` while running
- On exit code 0: plays `TUNE_DONE`, logs success
- On non-zero exit: plays `TUNE_ERROR`, logs failure with exit code
- Guards against duplicate triggers with a lock — if one job is already running, subsequent calls are no-ops with a log warning

#### `drone_watchdog.py` (orchestrator)
Reduced to under 120 lines of logic. Imports components from `watchdog_core/` and runs the main polling loop. The lock file state machine is unchanged in behaviour:

| Lock mode | Watchdog behaviour |
|---|---|
| absent | RC toggle operation (CH6 starts/stops stack) |
| `manual_scan` | Watchdog owns stack; stops on disarm |
| `autonomous` | main.py owns stack; watchdog yields |
| `bench_scan` | Test script owns stack; watchdog yields |

#### `__init__.py`
Re-exports the three main classes for cleaner imports:

```python
from watchdog_core import MavrosReader, FlightStack, PostflightMonitor
```

---

## 4. Post-Flight Logging Pipeline

A key improvement over the original implementation is full log capture from the post-flight mesh pipeline into the watchdog's `journalctl` stream.

**Original behaviour:**
```
[14:32:01] Launching post-flight processing...
# silence — no visibility into what happened
```

**New behaviour:**
```
[14:32:01] [POSTFLIGHT] Launching post-flight processing...
[14:32:02] [POSTFLIGHT] [1/6] Extracting point cloud from bag...
[14:32:08] [POSTFLIGHT] [2/6] MLS smoothing...
[14:32:21] [POSTFLIGHT] [3/6] Ground classification...
[14:32:24] [POSTFLIGHT] [4/6] Building DTM (Delaunay 2.5D)...
[14:32:31] [POSTFLIGHT] [5/6] Building DSM (Ball Pivoting)...
[14:32:44] [POSTFLIGHT] [6/6] Publishing outputs...
[14:32:45] [POSTFLIGHT] Processing complete ✓
```

All output is available via:
```bash
sudo journalctl -u drone-watchdog -f
```

---

## 5. Pixhawk Buzzer Feedback

### 5.1 Implementation
Buzzer output uses the MAVROS `PlayTuneV2` message on `/mavros/play_tune` with QBASIC Format 1 (`TUNE_FORMAT = 1`), confirmed working on this firmware.

### 5.2 Operator Audio Cues

| Sound | Meaning | When |
|---|---|---|
| Rising scale (C-E-G-C) | Stack starting | RC button pressed or lock detected |
| Falling scale (C-G-E-C) | Stack stopping | Disarm or RC button stop |
| Single beep | Acknowledged | RC button press / FCU connect |
| Staccato ×3 | Error | Post-flight failure |
| Mid pulse (every 5s) | Processing | Post-flight mesh pipeline running |
| Two-note up (C-E) | Done | Post-flight complete |

---

## 6. External RGB LED Indicator (Designed, Pending Wiring)

### 6.1 Rationale
The Pixhawk 6X onboard LED is controlled exclusively by PX4 Commander and reflects flight modes. Investigation confirmed that:

- `MAV_CMD_DO_LED_CONTROL` (command 528) is supported on firmware v1.16 (`success=True`) but PX4 Commander reasserts its own LED state on every cycle, overriding any external command within milliseconds
- NuttX shell `led_control` commands via `SERIAL_CONTROL` (MAVLink msg 126) were investigated extensively. The correct MAVROS topic on this system is `/uas1/mavlink_sink` (not `/mavros/mavlink/send` which does not exist), and QoS must be `BEST_EFFORT` to match the topic profile. Commands reach the NuttX shell but are overridden by Commander at the firmware level
- PX4 firmware v1.16 does not expose a parameter to disable Commander's LED ownership

An external 3-colour LED PCB (Red / Yellow / Green) wired directly to Raspberry Pi GPIO pins is the correct solution, as it is entirely independent of PX4.

### 6.2 Design — State Machine

The LED reflects **software and system state only**, not flight modes (which PX4 already handles).

| State | RED | YELLOW | GREEN | Trigger |
|---|---|---|---|---|
| `BOOTING` | Slow blink (1s) | Off | Off | Watchdog start, MAVROS not yet connected |
| `READY` | Off | Off | Solid | MAVROS connected, idle |
| `STACK_ACTIVE` | Off | Off | Slow blink (1s) | Flight stack running (scanning) |
| `POSTPROCESSING_RUNNING` | Off | Fast blink (0.3s) | Off | Mesh pipeline running |
| `POSTPROCESSING_DONE` | Off | Solid (5s) | Off | Mesh complete, auto-clears to READY |
| `CRITICAL_ERROR` | Solid | Solid | Solid | Any critical fault — latches until restart |

States are mutually exclusive. `CRITICAL_ERROR` is highest priority and cannot be cleared by any other state transition — it requires a watchdog restart, ensuring faults are never silently swallowed.

### 6.3 Critical Error Triggers

The following conditions transition immediately to `CRITICAL_ERROR`:

- SSD unmounted or rosbag directory inaccessible (voltage drop / filesystem fault)
- Corrupted bag file (metadata.yaml missing after bag close wait)
- Point-LIO process crashed mid-scan
- Bag recorder exited unexpectedly during a session
- SLAM bridge exited unexpectedly during a session
- Post-flight script exited with non-zero return code
- MAVROS FCU connection lost after initial connect
- Lock file unreadable or JSON-corrupted

### 6.4 Integration Points

Once wired, each call is a single line added to the existing module at these locations:

| File | Location | Call |
|---|---|---|
| `drone_watchdog.py` | `_wait_for_mavros()` entry | `led.set_booting()` |
| `drone_watchdog.py` | `_wait_for_mavros()` on connect | `led.set_ready()` |
| `drone_watchdog.py` | main loop MAVROS disconnect | `led.set_error()` |
| `flight_stack.py` | `FlightStack.start()` | `led.set_stack_active()` |
| `flight_stack.py` | `check_health()` on any crash | `led.set_error()` |
| `postflight.py` | `PostflightMonitor.trigger()` | `led.set_postprocessing()` |
| `postflight.py` | `_monitor()` rc == 0 | `led.set_postprocessing_done()` |
| `postflight.py` | `_monitor()` rc != 0 | `led.set_error()` |

### 6.5 Hardware Requirements (To Complete)

- Confirm common anode vs common cathode on PCB (multimeter or PCB silkscreen)
- Connect each LED signal pin to a Pi GPIO pin via 220–330Ω resistor
- Connect common pin to 3.3V (anode) or GND (cathode)
- Assign and record `PIN_RED`, `PIN_YELLOW`, `PIN_GREEN` (BCM numbering)
- Install GPIO library: `pip install RPi.GPIO` (Pi 4) or `pip install rpi-lgpio` (Pi 5)
- Set `ENABLE_LED = True` and `LED_ACTIVE_HIGH` to match polarity in `led_controller.py`

---

## 7. Key Design Decisions

### Decoupled postflight trigger
`FlightStack` receives a `postflight_fn` callable rather than importing `PostflightMonitor` directly. This means either module can be tested in isolation by passing a mock function, and neither creates a circular dependency.

### No blocking in the main loop
All long-running operations (ROS spin, postflight monitoring, buzzer beeping, future LED blinking) run on daemon threads. The main watchdog loop remains a simple poll at `POLL_HZ` / `MONITOR_HZ` (10 Hz).

### Error latching
`CRITICAL_ERROR` on the LED latches permanently. The watchdog already logs warnings on process crashes but previously had no persistent visual indicator. The latch ensures a crash noticed 10 minutes after it happened is still visible on the airframe.

### QoS matching for MAVROS topics
MAVROS on this system uses the `/uas1/` namespace (not `/mavros/`). The correct send topic is `/uas1/mavlink_sink`. Its QoS profile is `BEST_EFFORT` — a publisher with `RELIABLE` QoS will fail to connect silently. Both are documented and applied in the LED test scripts.

---

## 8. Files Produced

| File | Status | Description |
|---|---|---|
| `drone_watchdog.py` | Complete | Refactored thin orchestrator |
| `watchdog_core/__init__.py` | Complete | Package with public API re-exports |
| `watchdog_core/logging_utils.py` | Complete | Shared log() function |
| `watchdog_core/buzzer.py` | Complete | Tune constants + PostflightBeeper |
| `watchdog_core/mavros_reader.py` | Complete | ROS 2 MAVROS interface |
| `watchdog_core/flight_stack.py` | Complete | FlightStack process manager |
| `watchdog_core/postflight.py` | Complete | PostflightMonitor with log pipe |
| `watchdog_core/led_controller.py` | Documented | Pending GPIO wiring |
| `led_NuttX_shell_controller.py` | Archived | Superseded by external LED approach |
| `led_test_altctl.py` | Archived | Used to confirm SERIAL_CONTROL pipeline |

---

## 9. Deployment

```bash
# Restart the watchdog service after any change
sudo systemctl restart drone-watchdog

# Monitor all output including postflight pipeline
sudo journalctl -u drone-watchdog -f

# Check service status
sudo systemctl status drone-watchdog
```

---

## 10. Remaining Work

- [ ] Physically wire RGB LED PCB to Raspberry Pi GPIO pins
- [ ] Confirm common anode vs cathode, record pin assignments
- [ ] Implement `LEDController` class in `led_controller.py`
- [ ] Add `led.set_*()` calls at integration points listed in Section 6.4
- [ ] Add SSD mount health check to main loop (triggers `led.set_error()`)
- [ ] Add lock file corruption detection to `read_lock_mode()`
- [ ] Update `watchdog_core/__init__.py` to export `LEDController`
- [ ] Field test full feedback pipeline (buzzer + LED) on a complete scan session
