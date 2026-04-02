#!/bin/bash
# ============================================================
#  DRONE PRE-FLIGHT SYSTEM CHECK
#  Platform : Raspberry Pi 5 — Ubuntu 24.04
#  Project  : Autonomous Texture-Mapping Drone (UPRM Capstone)
#  Author   : Xavier
#  Usage    : bash preflight_check.sh   (no sudo needed)
#             sudo bash preflight_check.sh  (enables PMIC voltage read)
# ============================================================

# --- Colour codes ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

PASS=0
FAIL=0
WARN=0

pass() { echo -e "  ${GREEN}[PASS]${NC} $1"; ((PASS++)); }
fail() { echo -e "  ${RED}[FAIL]${NC} $1"; ((FAIL++)); }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $1"; ((WARN++)); }
section() { echo -e "\n${CYAN}${BOLD}━━━  $1  ━━━${NC}"; }

# ============================================================
# Resolve the actual user's home directory even when run via sudo
# ============================================================
if [ -n "$SUDO_USER" ]; then
    REAL_USER="$SUDO_USER"
    REAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    REAL_USER="$USER"
    REAL_HOME="$HOME"
fi

# Source ROS2 if not already sourced
if ! command -v ros2 > /dev/null 2>&1; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null
fi
if [ -f "$REAL_HOME/ros2_ws/install/setup.bash" ]; then
    source "$REAL_HOME/ros2_ws/install/setup.bash" 2>/dev/null
fi

# ============================================================
section "1. POWER RAIL & USB CURRENT OVERRIDE"
# ============================================================

if grep -q "usb_max_current_enable=1" /boot/firmware/config.txt 2>/dev/null; then
    pass "usb_max_current_enable=1 found in /boot/firmware/config.txt"
else
    fail "usb_max_current_enable=1 NOT found — USB ports may be limited to 600mA"
fi

THROTTLE=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
if [ "$THROTTLE" = "0x0" ]; then
    pass "No throttling or under-voltage events detected (throttled=0x0)"
else
    warn "Throttle flags active: $THROTTLE"
    echo "       Bit ref: 0x1=under-voltage now  0x4=throttled now  0x10000=under-voltage occurred"
fi

V5_RAW=$(vcgencmd pmic_read_adc EXT5V_V 2>/dev/null)
V5=$(echo "$V5_RAW" | grep -oP '[0-9]+\.[0-9]+' | head -1)
if [ -n "$V5" ]; then
    VCHECK=$(awk -v v="$V5" 'BEGIN{ print (v+0 >= 4.8) ? "ok" : "low" }')
    if [ "$VCHECK" = "ok" ]; then
        pass "5V rail voltage: ${V5}V (above 4.8V threshold)"
    else
        fail "5V rail voltage: ${V5}V — BELOW 4.8V threshold, risk of instability"
    fi
else
    warn "Could not read EXT5V_V — run as root for PMIC access (sudo bash preflight_check.sh)"
fi

# ============================================================
section "2. STORAGE — SanDisk Extreme SSD (/mnt/ssd)"
# ============================================================

SSD_MOUNT="/mnt/ssd"

if mountpoint -q "$SSD_MOUNT"; then
    pass "SSD mounted at $SSD_MOUNT"

    LABEL=$(lsblk -o LABEL,MOUNTPOINT | grep "$SSD_MOUNT" | awk '{print $1}')
    if [ "$LABEL" = "flight_data" ]; then
        pass "SSD filesystem label: $LABEL"
    else
        warn "SSD label is '${LABEL:-none}' (expected 'flight_data')"
    fi

    FREE_GB=$(df -BG "$SSD_MOUNT" | tail -1 | awk '{gsub("G",""); print $4}')
    if [ "${FREE_GB:-0}" -ge 20 ] 2>/dev/null; then
        pass "SSD free space: ${FREE_GB}GB"
    else
        warn "SSD free space LOW: ${FREE_GB}GB — clear old rosbags before flight"
    fi

    echo "  Running write speed test (256MB) — please wait..."
    WRITE_OUT=$(dd if=/dev/zero of="$SSD_MOUNT/.speed_test.tmp" \
        bs=1M count=256 oflag=direct 2>&1)
    rm -f "$SSD_MOUNT/.speed_test.tmp"
    WRITE_SPEED=$(echo "$WRITE_OUT" | grep -oP '[0-9.]+ [MG]B/s' | tail -1)
    if [ -n "$WRITE_SPEED" ]; then
        SPEED_VAL=$(echo "$WRITE_SPEED" | grep -oP '[0-9.]+')
        SPEED_UNIT=$(echo "$WRITE_SPEED" | grep -oP '[MG]B/s')
        SPEED_OK=$(awk -v v="$SPEED_VAL" -v u="$SPEED_UNIT" \
            'BEGIN{ print (u=="GB/s" || (u=="MB/s" && v+0>=150)) ? "ok" : "low" }')
        if [ "$SPEED_OK" = "ok" ]; then
            pass "SSD write speed: $WRITE_SPEED (sufficient for rosbag recording)"
        else
            warn "SSD write speed: $WRITE_SPEED — may be insufficient for concurrent rosbag streams"
        fi
    else
        warn "Could not measure SSD write speed"
    fi

    if lsusb | grep -qi "sandisk\|western digital"; then
        pass "SanDisk SSD detected on USB bus"
    else
        warn "SanDisk not positively identified on USB bus (may enumerate under WD ID)"
    fi

else
    fail "SSD NOT mounted at $SSD_MOUNT"
    echo "       If SSD is connected, check: sudo mount -a"
    echo "       If not connected, this is expected — connect before flight"
fi

# ============================================================
section "3. PIXHAWK 6X — MAVROS SERIAL"
# ============================================================

if [ -L /dev/ttyPixhawk ]; then
    DEVICE="/dev/ttyPixhawk"
    pass "Pixhawk udev symlink: /dev/ttyPixhawk → $(readlink /dev/ttyPixhawk)"
elif [ -e /dev/ttyACM0 ]; then
    DEVICE="/dev/ttyACM0"
    pass "Pixhawk serial device: /dev/ttyACM0"
    warn "Consider adding a udev symlink for stable device naming across reboots"
    echo "       See README for udev rule instructions"
else
    DEVICE=""
    fail "Pixhawk serial device NOT found at /dev/ttyACM0 or /dev/ttyPixhawk"
fi

if [ -n "$DEVICE" ]; then
    if [ -r "$DEVICE" ]; then
        pass "$DEVICE is readable"
    else
        fail "$DEVICE not readable — add user to dialout group"
        echo "       Fix: sudo usermod -aG dialout $REAL_USER"
    fi
fi

if id -nG "$REAL_USER" 2>/dev/null | grep -q "dialout"; then
    pass "User '$REAL_USER' is in dialout group"
else
    warn "User '$REAL_USER' NOT in dialout group"
    echo "       Fix: sudo usermod -aG dialout $REAL_USER  (then log out and back in)"
fi

if pgrep -f "mavros" > /dev/null 2>&1; then
    pass "MAVROS process is running"
else
    warn "MAVROS not currently running — launch before arming"
    echo "       Launch: ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:921600"
fi

# ============================================================
section "4. HAILO-8 AI ACCELERATOR HAT"
# ============================================================

if lspci 2>/dev/null | grep -qi "hailo"; then
    HAILO_INFO=$(lspci | grep -i hailo | head -1)
    pass "Hailo-8 on PCIe bus: $HAILO_INFO"
elif [ -e /dev/hailo0 ]; then
    pass "Hailo device node /dev/hailo0 present (PCIe enumeration may need lspci update)"
else
    fail "Hailo-8 NOT detected — check HAT seating on PCIe FPC connector"
fi

if [ -e /dev/hailo0 ]; then
    pass "/dev/hailo0 device node exists"
    if [ -r /dev/hailo0 ]; then
        pass "/dev/hailo0 is accessible by current user"
    else
        warn "/dev/hailo0 not accessible — check hailo group: sudo usermod -aG hailo $REAL_USER"
    fi
else
    warn "/dev/hailo0 not found — HailoRT driver may not be loaded"
    echo "       Check: lsmod | grep hailo"
fi

HAILO_VENV="$REAL_HOME/hailo_inference_env"
if [ -d "$HAILO_VENV" ]; then
    pass "hailo_inference_env venv found at $HAILO_VENV"
    HAILO_IMPORT=$("$HAILO_VENV/bin/python3" -c \
        "import hailo_platform; print('ok')" 2>/dev/null)
    if [ "$HAILO_IMPORT" = "ok" ]; then
        pass "hailo_platform imports successfully"
    else
        fail "hailo_platform import FAILED in hailo_inference_env"
        echo "       Check HailoRT installation: $HAILO_VENV/bin/pip list | grep hailo"
    fi
else
    fail "hailo_inference_env NOT found at $HAILO_VENV"
    echo "       Expected location: $HAILO_VENV"
fi

# ============================================================
section "5. CAMERA — Raspberry Pi HQ IMX477"
# ============================================================
#
# Detection strategy (in priority order):
#   A. libcamera / rpicam-hello  — primary tool, reports sensor model directly
#   B. media-ctl                 — kernel media graph; confirms CSI pipeline
#   C. v4l2-ctl                  — lists V4L2 nodes; confirms unicam/rp1 driver
#   D. /sys filesystem           — reads i2c device name without any binary
#
# The IMX477 enumerates via the CSI2 / i2c bus as i2c@88000/imx477@1a on RPi5.
# The rpicam-hello output you observed confirms the full path:
#   /base/axi/pcie@120000/rp1/i2c@88000/imx477@1a
# All checks below anchor to this known path or sensor name.
# ============================================================

CAM_PASS=0    # counts passing sub-checks in this section
CAM_FAIL=0    # counts failing sub-checks in this section

# ── A. libcamera / rpicam-hello ───────────────────────────────────────────────
# rpicam-hello --list-cameras exits 0 and prints sensor names when the ISP
# pipeline is healthy.  We search for "imx477" in the output.
# Try both /usr/local/bin (from the build you installed) and system path.

CAM_DETECTED=false
RPICAM_BIN=""

for BIN in \
    /usr/local/bin/rpicam-hello \
    /usr/bin/rpicam-hello \
    /usr/local/bin/libcamera-hello \
    /usr/bin/libcamera-hello; do
    if [ -x "$BIN" ]; then
        RPICAM_BIN="$BIN"
        break
    fi
done

if [ -n "$RPICAM_BIN" ]; then
    # --list-cameras: enumerates available cameras without opening a preview
    # --timeout 0: exit immediately after enumeration (no preview window)
    CAM_OUTPUT=$("$RPICAM_BIN" --list-cameras 2>&1)
    CAM_EXIT=$?

    if echo "$CAM_OUTPUT" | grep -qi "imx477"; then
        CAM_LINE=$(echo "$CAM_OUTPUT" | grep -i "imx477" | head -1 | xargs)
        pass "IMX477 confirmed by $( basename "$RPICAM_BIN" ): $CAM_LINE"
        CAM_DETECTED=true
        ((CAM_PASS++))
    elif echo "$CAM_OUTPUT" | grep -qiE "available cameras|camera [0-9]"; then
        warn "$( basename "$RPICAM_BIN" ) found a camera but could not confirm IMX477 model"
        echo "       Output: $(echo "$CAM_OUTPUT" | head -3 | xargs)"
        CAM_DETECTED=true
        ((CAM_PASS++))
    else
        fail "IMX477 NOT detected by $( basename "$RPICAM_BIN" ) (exit=$CAM_EXIT)"
        echo "       Output: $(echo "$CAM_OUTPUT" | head -3 | xargs)"
        echo "       Check CSI ribbon cable seating and lock tabs at both ends"
        ((CAM_FAIL++))
    fi
else
    warn "rpicam-hello / libcamera-hello not found in PATH or /usr/local/bin"
    echo "       Install: build rpicam-apps and run the find-based copy command"
    echo "       sudo find ~/rpicam-apps/build/apps -type f -executable -name 'rpicam-*'"
    echo "            -exec cp {} /usr/local/bin/ \\;"
fi

# ── B. media-ctl — kernel media graph ────────────────────────────────────────
# The RPi5 CSI pipeline registers a media device.  media-ctl -p lists all
# entities; imx477 appears as a subdev entity when the driver is loaded and
# the sensor is reachable.

if command -v media-ctl > /dev/null 2>&1; then
    MEDIA_OUT=$(media-ctl -p 2>/dev/null)
    if echo "$MEDIA_OUT" | grep -qi "imx477"; then
        MEDIA_ENTITY=$(echo "$MEDIA_OUT" | grep -i "imx477" | head -1 | xargs)
        pass "IMX477 entity in kernel media graph: $MEDIA_ENTITY"
        ((CAM_PASS++))
    else
        # media-ctl is present but IMX477 not in graph — could be a driver issue
        warn "media-ctl available but IMX477 not found in media graph"
        echo "       Check: media-ctl -p | grep -i imx477"
        ((CAM_FAIL++))
    fi
else
    warn "media-ctl not installed — skipping media graph check"
    echo "       Install: sudo apt install v4l-utils"
fi

# ── C. v4l2-ctl — V4L2 device enumeration ────────────────────────────────────
# unicam (RPi CSI receiver) and rp1-cfe register V4L2 nodes for the sensor.
# The device name contains "unicam", "rp1", or the sensor model.

if command -v v4l2-ctl > /dev/null 2>&1; then
    V4L2_OUT=$(v4l2-ctl --list-devices 2>/dev/null)
    if echo "$V4L2_OUT" | grep -qiE "unicam|imx477|rp1|cfe"; then
        V4L2_LINE=$(echo "$V4L2_OUT" | grep -iE "unicam|imx477|rp1|cfe" | head -1 | xargs)
        pass "CSI capture device via v4l2: $V4L2_LINE"
        ((CAM_PASS++))
    else
        warn "v4l2-ctl found no CSI capture device (unicam/rp1/imx477)"
        echo "       Full list: v4l2-ctl --list-devices"
        ((CAM_FAIL++))
    fi
else
    warn "v4l2-ctl not installed — skipping V4L2 device check"
    echo "       Install: sudo apt install v4l-utils"
fi

# ── D. /sys filesystem — i2c device node ─────────────────────────────────────
# The IMX477 driver registers under /sys/bus/i2c/devices/ as:
#   /sys/bus/i2c/devices/<bus>-001a/   (i2c address 0x1a)
# The driver name file at <device>/name contains "imx477".
# This check requires no binary and works even if libcamera is absent.

SYS_IMX477=$(find /sys/bus/i2c/devices -maxdepth 2 -name "name" \
    -exec grep -l "imx477" {} \; 2>/dev/null | head -1)

if [ -n "$SYS_IMX477" ]; then
    SYS_PATH=$(dirname "$SYS_IMX477")
    pass "IMX477 i2c device node: $SYS_PATH"
    ((CAM_PASS++))
else
    # Try the known RPi5 path directly
    KNOWN_SYS="/sys/bus/i2c/devices/10-001a"
    if [ -d "$KNOWN_SYS" ]; then
        pass "IMX477 i2c node at known RPi5 path: $KNOWN_SYS"
        ((CAM_PASS++))
    else
        warn "IMX477 i2c device node not found under /sys/bus/i2c/devices/"
        echo "       Expected pattern: /sys/bus/i2c/devices/*-001a/name = imx477"
        echo "       Check: find /sys/bus/i2c/devices -name name | xargs grep -l imx477"
    fi
fi

# ── E. Tuning file — ISP configuration ───────────────────────────────────────
# The rpicam-hello output confirmed libcamera uses:
#   /usr/local/share/libcamera/ipa/rpi/pisp/imx477.json
# If this file is missing the ISP cannot configure the sensor correctly.

TUNING_FILE="/usr/local/share/libcamera/ipa/rpi/pisp/imx477.json"
ALT_TUNING="/usr/share/libcamera/ipa/rpi/pisp/imx477.json"

if [ -f "$TUNING_FILE" ]; then
    pass "IMX477 tuning file: $TUNING_FILE"
    ((CAM_PASS++))
elif [ -f "$ALT_TUNING" ]; then
    pass "IMX477 tuning file (system): $ALT_TUNING"
    ((CAM_PASS++))
else
    warn "IMX477 tuning file not found at expected paths"
    echo "       Expected: $TUNING_FILE"
    echo "       Rebuild rpicam-apps or reinstall libcamera-ipa"
    ((CAM_FAIL++))
fi

# ── F. Picamera2 Python library ───────────────────────────────────────────────
# CameraCapture (camera_capture.py) requires picamera2.
# Check in the active conda/venv environment used by the drone stack.

DRONEPI_PYTHON=""
for PY in \
    "$REAL_HOME/miniforge3/envs/dronepi/bin/python3" \
    "$REAL_HOME/.venv/bin/python3" \
    "$(command -v python3 2>/dev/null)"; do
    if [ -x "$PY" ]; then
        DRONEPI_PYTHON="$PY"
        break
    fi
done

if [ -n "$DRONEPI_PYTHON" ]; then
    PY2_CHECK=$("$DRONEPI_PYTHON" -c "import picamera2; print('ok')" 2>/dev/null)
    if [ "$PY2_CHECK" = "ok" ]; then
        PY2_VER=$("$DRONEPI_PYTHON" -c \
            "import picamera2; print(picamera2.__version__)" 2>/dev/null)
        pass "picamera2 ${PY2_VER} available in $( basename "$( dirname "$DRONEPI_PYTHON" )" )"
        ((CAM_PASS++))
    else
        warn "picamera2 NOT importable from $DRONEPI_PYTHON"
        echo "       Fix: pip install picamera2   (in the dronepi conda env)"
        ((CAM_FAIL++))
    fi
else
    warn "Could not locate a Python 3 interpreter to check picamera2"
fi

# ── G. Calibration file ───────────────────────────────────────────────────────
CAL_FOUND=false
for CAL_PATH in \
    "$(pwd)/config/camera_calibration.yaml" \
    "$REAL_HOME/config/camera_calibration.yaml" \
    "$REAL_HOME/unitree_lidar_project/config/camera_calibration.yaml" \
    "/mnt/ssd/config/camera_calibration.yaml"; do
    if [ -f "$CAL_PATH" ]; then
        pass "Calibration file: $CAL_PATH"
        CAL_FOUND=true
        break
    fi
done
[ "$CAL_FOUND" = false ] && warn "camera_calibration.yaml not found — photogrammetry reconstruction will use default intrinsics"

# ── Camera section summary ────────────────────────────────────────────────────
echo ""
if [ "$CAM_DETECTED" = true ]; then
    if [ "$CAM_FAIL" -eq 0 ]; then
        echo -e "  ${GREEN}Camera section: IMX477 confirmed — all sub-checks passed${NC}"
    else
        echo -e "  ${YELLOW}Camera section: IMX477 detected but $CAM_FAIL sub-check(s) failed — review above${NC}"
    fi
else
    echo -e "  ${RED}Camera section: IMX477 NOT detected — flight camera unavailable${NC}"
    echo "  Verify: CSI ribbon cable seated, lock tabs closed at both ends"
    echo "          Tamron C-mount lens: C-CS spacer ring must be REMOVED for infinity focus"
fi

# ============================================================
section "6. LIDAR — Unitree 4D L1 (CP210x)"
# ============================================================

if lsusb | grep -qi "cp210\|10c4:ea60"; then
    LIDAR_USB=$(lsusb | grep -i "cp210\|10c4:ea60" | head -1)
    pass "LiDAR CP210x USB adapter detected: $LIDAR_USB"
elif lsusb | grep -qi "ch340\|ch341\|ftdi\|silabs"; then
    LIDAR_USB=$(lsusb | grep -i "ch340\|ch341\|ftdi\|silabs" | head -1)
    warn "USB serial adapter found but not CP210x (expected for L1): $LIDAR_USB"
elif ls /dev/ttyUSB* > /dev/null 2>&1; then
    warn "ttyUSB devices present but vendor unconfirmed: $(ls /dev/ttyUSB*)"
else
    fail "No LiDAR USB serial device detected — check USB connection and LiDAR power"
fi

if id -nG "$REAL_USER" 2>/dev/null | grep -q "dialout"; then
    pass "User has dialout access for LiDAR serial port"
else
    warn "dialout group needed for /dev/ttyUSB* access"
fi

if ros2 pkg list 2>/dev/null | grep -qi "point_lio\|pointlio"; then
    pass "Point-LIO ROS2 package available"
else
    warn "Point-LIO not found in current ROS2 environment"
    echo "       Fix: source $REAL_HOME/ros2_ws/install/setup.bash"
fi

# ============================================================
section "7. WI-FI DONGLE — TP-Link AC600 (RTL8821AU)"
# ============================================================

if lsusb | grep -qiE "2357:011e|2357:0115|tp-link.*rtl88"; then
    TP_INFO=$(lsusb | grep -iE "2357:011e|2357:0115|tp-link" | head -1)
    pass "TP-Link AC600 RTL8821AU detected: $TP_INFO"
else
    warn "TP-Link AC600 not confirmed on USB bus (check: lsusb | grep 2357)"
fi

WLAN_IF=$(ip link show | grep -oP '(?<=\d: )wlan[^:@\s]+' | head -1 | tr -d ':')
if [ -n "$WLAN_IF" ]; then
    pass "Wireless interface active: $WLAN_IF"
    SSID=$(iwgetid "$WLAN_IF" -r 2>/dev/null)
    if [ -n "$SSID" ]; then
        pass "Connected to Wi-Fi SSID: $SSID"
        GW=$(ip route | grep default | awk '{print $3}' | head -1)
        if ping -c 1 -W 2 "$GW" > /dev/null 2>&1; then
            pass "Gateway reachable: $GW"
        else
            warn "Gateway $GW not responding — check AP and signal strength"
        fi
    else
        warn "$WLAN_IF present but not connected — expected if flying without GCS link"
    fi
else
    fail "No wireless interface found — check 8821au driver: lsmod | grep 8821au"
fi

# ============================================================
section "8. ROS 2 JAZZY ENVIRONMENT"
# ============================================================

if command -v ros2 > /dev/null 2>&1; then
    ROS_VER=$(ros2 --version 2>/dev/null | head -1)
    pass "ROS 2 available: $ROS_VER"

    for PKG in "mavros" "rosbridge_server" "sensor_msgs" "geometry_msgs"; do
        if ros2 pkg list 2>/dev/null | grep -q "^${PKG}$"; then
            pass "ROS2 package: $PKG"
        else
            warn "ROS2 package missing: $PKG"
        fi
    done
else
    fail "ROS 2 not in PATH — sourcing failed"
    echo "       Manual fix: source /opt/ros/jazzy/setup.bash"
    echo "       Permanent fix (for this script): add to /root/.bashrc AND $REAL_HOME/.bashrc"
fi

# ============================================================
section "9. SYSTEM HEALTH"
# ============================================================

CPU_TEMP=$(vcgencmd measure_temp 2>/dev/null | grep -oP '[0-9.]+')
if [ -n "$CPU_TEMP" ]; then
    TEMP_STATUS=$(awk -v t="$CPU_TEMP" 'BEGIN{
        if (t+0 < 70) print "ok"
        else if (t+0 < 80) print "warn"
        else print "crit"
    }')
    if   [ "$TEMP_STATUS" = "ok"   ]; then pass "CPU temperature: ${CPU_TEMP}°C (nominal)"
    elif [ "$TEMP_STATUS" = "warn" ]; then warn "CPU temperature: ${CPU_TEMP}°C (elevated — verify active cooling)"
    else                                   fail "CPU temperature: ${CPU_TEMP}°C — CRITICAL, do not fly"
    fi
fi

FREE_RAM=$(free -m | awk '/^Mem:/{print $7}')
RAM_STATUS=$(awk -v r="${FREE_RAM:-0}" 'BEGIN{
    if (r+0 >= 1024) print "ok"
    else if (r+0 >= 512) print "warn"
    else print "crit"
}')
if   [ "$RAM_STATUS" = "ok"   ]; then pass "Available RAM: ${FREE_RAM}MB"
elif [ "$RAM_STATUS" = "warn" ]; then warn "Available RAM: ${FREE_RAM}MB — close unused processes before flight"
else                                   fail "Available RAM critically low: ${FREE_RAM}MB"
fi

CPU_HZ=$(vcgencmd measure_clock arm 2>/dev/null | grep -oP '[0-9]+$')
if [ -n "$CPU_HZ" ]; then
    CPU_GHZ=$(awk -v h="$CPU_HZ" 'BEGIN{ printf "%.2f", h/1000000000 }')
    pass "CPU clock: ${CPU_GHZ} GHz"
fi

ROOT_DEV=$(findmnt -n -o SOURCE / | sed 's/p\?[0-9]*$//')
if echo "$ROOT_DEV" | grep -q "mmcblk"; then
    warn "Root filesystem is on SD card — confirm all rosbag writes target /mnt/ssd"
else
    pass "Root filesystem on: $ROOT_DEV (not SD card)"
fi

# ============================================================
section "10. OUTPUT DIRECTORIES"
# ============================================================

for DIR in "/mnt/ssd/flights" "/mnt/ssd/rosbags" "/mnt/ssd/maps"; do
    if [ -d "$DIR" ]; then
        pass "Directory exists: $DIR"
    elif mountpoint -q "/mnt/ssd"; then
        warn "Creating missing directory: $DIR"
        mkdir -p "$DIR" 2>/dev/null && pass "Created: $DIR" || \
            fail "Could not create $DIR"
    else
        warn "Skipping $DIR — SSD not mounted"
    fi
done

# ============================================================
echo -e "\n${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BOLD}  PRE-FLIGHT SYSTEM CHECK — RESULTS${NC}"
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "  ${GREEN}PASS${NC} : $PASS"
echo -e "  ${YELLOW}WARN${NC} : $WARN"
echo -e "  ${RED}FAIL${NC} : $FAIL"
echo ""
if   [ "$FAIL" -gt 0 ]; then
    echo -e "  ${RED}${BOLD}▶ STATUS: NOT READY FOR FLIGHT — resolve all FAIL items above${NC}"
elif [ "$WARN" -gt 0 ]; then
    echo -e "  ${YELLOW}${BOLD}▶ STATUS: READY WITH WARNINGS — review WARN items before flight${NC}"
else
    echo -e "  ${GREEN}${BOLD}▶ STATUS: ALL SYSTEMS GO ✓${NC}"
fi
echo ""
