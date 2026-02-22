#!/bin/bash
# Simple photogrammetry capture using OpenCV directly
# No ROS2 needed - just captures images at intervals

INTERVAL="${1:-2.0}"
OUTPUT_DIR="${2:-./photogrammetry_captures}"
DEVICE="${3:-/dev/video0}"

echo "================================================"
echo "  Simple Photogrammetry Capture"
echo "================================================"
echo "Device: $DEVICE"
echo "Capture interval: $INTERVAL seconds"
echo "Output directory: $OUTPUT_DIR"
echo ""
echo "Press Ctrl+C to stop"
echo "================================================"
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Run Python capture script
python3 << PYEOF
import cv2
import time
import os
from datetime import datetime

DEVICE = "$DEVICE"
INTERVAL = float("$INTERVAL")
OUTPUT_DIR = "$OUTPUT_DIR"

print(f"Opening camera: {DEVICE}")
cap = cv2.VideoCapture(DEVICE)

if not cap.isOpened():
    print(f"Error: Could not open camera at {DEVICE}")
    exit(1)

# Set resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(f"Camera opened: {width}x{height}")
print(f"Capturing every {INTERVAL} seconds...")
print("")

count = 0
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame")
            time.sleep(0.1)
            continue

        # Save image
        count += 1
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"img_{count:06d}_{timestamp}.jpg"
        filepath = os.path.join(OUTPUT_DIR, filename)

        cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        print(f"[{count}] Saved: {filename}")

        # Wait for interval
        time.sleep(INTERVAL)

except KeyboardInterrupt:
    print(f"\nStopping... Total images captured: {count}")
finally:
    cap.release()

PYEOF
