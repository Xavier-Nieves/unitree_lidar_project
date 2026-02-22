#!/bin/bash
# Insta360 X3 Camera Test Script
# Tests USB connection and camera capabilities

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "========================================"
echo "  Insta360 X3 Camera Detection Test"
echo "========================================"
echo ""

# Function to print colored output
print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check if running as root or in video group
if ! groups | grep -q video; then
    print_warning "You're not in the 'video' group. You may need sudo for some operations."
    print_info "Add yourself to video group: sudo usermod -a -G video \$USER"
fi

echo ""
print_info "Step 1: Checking USB devices..."
echo "----------------------------------------"

# Check for Insta360 camera (shows as "A9 Platform" in webcam mode)
if lsusb | grep -qi "insta\|Arashi\|A9 Platform"; then
    print_success "Insta360 camera detected!"
    lsusb | grep -i "insta\|Arashi\|A9 Platform"

    # Check if it's in the right mode
    if lsusb -v 2>/dev/null | grep -A 5 "A9 Platform" | grep -q "Video"; then
        print_success "Camera is in Video/Webcam mode!"
    elif lsusb -v 2>/dev/null | grep -A 5 "A9 Platform" | grep -q "Mass Storage"; then
        print_warning "Camera is in Mass Storage mode (file transfer)"
        print_info "Change mode on camera: Settings → USB Mode → 'USB Webcam'"
    fi
else
    print_warning "Insta360 X3 not found in USB devices"
    print_info "All USB devices:"
    lsusb
    echo ""
    print_info "Make sure the camera is:"
    echo "  1. Connected via USB cable"
    echo "  2. Powered ON"
    echo "  3. In the correct USB mode (Settings → USB Mode)"
fi

echo ""
print_info "Step 2: Checking video devices..."
echo "----------------------------------------"

# List all video devices
if ls /dev/video* >/dev/null 2>&1; then
    VIDEO_DEVICES=$(ls /dev/video* 2>/dev/null | sort -V)
    print_success "Found video devices:"
    for dev in $VIDEO_DEVICES; do
        echo "  - $dev"
    done
else
    print_error "No video devices found!"
    exit 1
fi

echo ""
print_info "Step 3: Installing required tools (if needed)..."
echo "----------------------------------------"

# Check for v4l-utils
if ! command -v v4l2-ctl &> /dev/null; then
    print_warning "v4l-utils not installed. Installing..."
    sudo apt-get update
    sudo apt-get install -y v4l-utils
else
    print_success "v4l-utils already installed"
fi

# Check for ffmpeg
if ! command -v ffmpeg &> /dev/null; then
    print_warning "ffmpeg not installed. Installing..."
    sudo apt-get install -y ffmpeg
else
    print_success "ffmpeg already installed"
fi

echo ""
print_info "Step 4: Scanning video device capabilities..."
echo "----------------------------------------"

for device in /dev/video{0..10}; do
    if [ -e "$device" ] 2>/dev/null; then
        echo ""
        echo "Device: $device"
        echo "----------------"

        # Get device info
        if v4l2-ctl -d "$device" --info 2>/dev/null | head -5; then
            echo ""

            # Check if it's a capture device
            if v4l2-ctl -d "$device" --all 2>/dev/null | grep -q "Video Capture"; then
                print_success "This is a capture device!"

                # Show supported formats
                echo "Supported formats:"
                v4l2-ctl -d "$device" --list-formats-ext 2>/dev/null | head -20

                # Check if it might be the Insta360
                if v4l2-ctl -d "$device" --info 2>/dev/null | grep -qi "insta\|x3\|360"; then
                    print_success ">>> This might be your Insta360 X3! <<<"
                    INSTA_DEVICE="$device"
                fi
            fi
        fi
    fi
done

echo ""
echo "========================================"
echo "  Test Results Summary"
echo "========================================"

if [ -n "$INSTA_DEVICE" ]; then
    print_success "Insta360 X3 detected at: $INSTA_DEVICE"
    echo ""
    print_info "Quick test commands:"
    echo ""
    echo "1. View camera info:"
    echo "   v4l2-ctl -d $INSTA_DEVICE --all"
    echo ""
    echo "2. List supported resolutions:"
    echo "   v4l2-ctl -d $INSTA_DEVICE --list-formats-ext"
    echo ""
    echo "3. Capture a test image:"
    echo "   ffmpeg -f v4l2 -i $INSTA_DEVICE -frames 1 test_capture.jpg"
    echo ""
    echo "4. Record 10 seconds of video:"
    echo "   ffmpeg -f v4l2 -i $INSTA_DEVICE -t 10 test_video.mp4"
    echo ""
else
    print_warning "Insta360 X3 not automatically detected"
    echo ""
    print_info "Try these troubleshooting steps:"
    echo ""
    echo "1. Check camera USB mode:"
    echo "   - On camera: Settings → USB Mode → Select 'USB Webcam' or 'USB Video'"
    echo ""
    echo "2. Reconnect the camera:"
    echo "   - Unplug and replug the USB cable"
    echo "   - Watch for new devices: watch -n 1 'ls -l /dev/video*'"
    echo ""
    echo "3. Check dmesg for USB events:"
    echo "   dmesg | tail -30"
    echo ""
    echo "4. Manual testing of video devices:"
    for device in /dev/video{0..5}; do
        if [ -e "$device" ]; then
            echo "   ffmpeg -f v4l2 -list_formats all -i $device"
        fi
    done
fi

echo ""
print_info "For ROS2 integration, see: create_camera_node.sh"
echo "========================================"
