#!/bin/bash
# Script to install custom launch files into the Unitree LiDAR SDK package

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
SDK_DIR="$PROJECT_DIR/ros2_ws/src/unilidar_sdk/unitree_lidar_ros2"

echo "Installing custom launch files..."

# Check if SDK exists
if [ ! -d "$SDK_DIR" ]; then
    echo "Error: Unitree LiDAR SDK not found at $SDK_DIR"
    echo "Please run './unitree_lidar_L1 setup' first"
    exit 1
fi

# Create launch directory if it doesn't exist
mkdir -p "$SDK_DIR/launch"

# Copy launch file
if [ -f "$PROJECT_DIR/config/lidar_with_pointlio.launch.py" ]; then
    cp "$PROJECT_DIR/config/lidar_with_pointlio.launch.py" "$SDK_DIR/launch/"
    chmod +x "$SDK_DIR/launch/lidar_with_pointlio.launch.py"
    echo "✓ Installed lidar_with_pointlio.launch.py"
else
    echo "✗ Launch file not found"
    exit 1
fi

# Create RViz config directory
mkdir -p "$SDK_DIR/rviz"

echo "Custom launch files installed successfully!"
echo "Rebuild the workspace with: ./unitree_lidar_L1 build"
