#!/bin/bash
# ============================================================
# Autonomous Drone Mapping System — One-Shot Full Setup
# Takes a fresh Ubuntu 24.04 Pi 5 to fully operational
# ============================================================
set -e

echo "============================================================"
echo "  Autonomous Drone Mapping System — Setup"
echo "  Target: Ubuntu 24.04 (Noble) on Raspberry Pi 5"
echo "============================================================"
echo ""

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo "Detected: $PRETTY_NAME"
else
    echo "WARNING: Cannot detect OS version"
fi

echo ""
read -p "Continue with installation? (y/N) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 0
fi

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS2_WS="$PROJECT_DIR/ros2_ws"

# ─────────────────────────────────────────────
# [1/7] System update + base tools
# ─────────────────────────────────────────────
echo ""
echo "[1/7] System update + base tools..."
echo "────────────────────────────────────"
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    git curl wget \
    build-essential cmake \
    python3-pip python3-venv \
    software-properties-common \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libpcl-dev

# ─────────────────────────────────────────────
# [2/7] Install ROS 2 Jazzy
# ─────────────────────────────────────────────
echo ""
echo "[2/7] Installing ROS 2 Jazzy..."
echo "────────────────────────────────────"
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
    ros-jazzy-ros-base \
    ros-jazzy-ros-dev-tools \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-visualization-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-rviz2 \
    python3-colcon-common-extensions

# ─────────────────────────────────────────────
# [3/7] Install LiDAR stack
# ─────────────────────────────────────────────
echo ""
echo "[3/7] Installing LiDAR stack..."
echo "────────────────────────────────────"
mkdir -p "$ROS2_WS/src"
cd "$ROS2_WS/src"

# Livox SDK2
if [ ! -d "Livox-SDK2" ]; then
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2 && mkdir -p build && cd build
    cmake .. && make -j$(nproc) && sudo make install
    cd "$ROS2_WS/src"
fi

# Livox ROS Driver 2
if [ ! -d "livox_ros_driver2" ]; then
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git
    cd livox_ros_driver2
    source /opt/ros/jazzy/setup.bash
    ./build.sh jazzy
    cd "$ROS2_WS/src"
fi

# Unitree LiDAR SDK
if [ ! -d "unilidar_sdk" ]; then
    git clone https://github.com/unitreerobotics/unilidar_sdk.git
    cd unilidar_sdk/unitree_lidar_sdk
    mkdir -p build && cd build
    cmake .. && make -j$(nproc)
    cd "$ROS2_WS/src"
fi

# ─────────────────────────────────────────────
# [4/7] Install Point-LIO ROS 2
# ─────────────────────────────────────────────
echo ""
echo "[4/7] Installing Point-LIO..."
echo "────────────────────────────────────"
cd "$ROS2_WS/src"
if [ ! -d "point_lio_ros2" ]; then
    git clone https://github.com/dfloreaa/point_lio_ros2.git
fi

# ─────────────────────────────────────────────
# [5/7] Install PX4 dependencies
# ─────────────────────────────────────────────
echo ""
echo "[5/7] Installing PX4 DDS dependencies..."
echo "────────────────────────────────────"

# px4_msgs for DDS communication
cd "$ROS2_WS/src"
if [ ! -d "px4_msgs" ]; then
    git clone https://github.com/PX4/px4_msgs.git
fi

# Micro XRCE-DDS Agent
if ! command -v MicroXRCEAgent &> /dev/null; then
    cd /tmp
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir -p build && cd build
    cmake .. && make -j$(nproc) && sudo make install
    sudo ldconfig
fi

# ─────────────────────────────────────────────
# [6/7] Install mapping dependencies
# ─────────────────────────────────────────────
echo ""
echo "[6/7] Installing mapping dependencies..."
echo "────────────────────────────────────"
pip3 install --user \
    open3d \
    opencv-python-headless \
    numpy \
    scipy \
    pyyaml \
    requests

# ─────────────────────────────────────────────
# [7/7] Configure system + build workspace
# ─────────────────────────────────────────────
echo ""
echo "[7/7] Configuring system..."
echo "────────────────────────────────────"

# USB device permissions
sudo tee /etc/udev/rules.d/99-drone-devices.rules > /dev/null <<EOF
# Unitree L1 LiDAR
KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"
# Pixhawk/PX4
KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger

# User groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER

# CPU governor
if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
    echo "Setting CPU governor to performance..."
    echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null
fi

# Build workspace
echo "Building ROS 2 workspace..."
cd "$ROS2_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
    --packages-select unitree_lidar_ros2 point_lio px4_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup bashrc
SETUP_LINES=(
    "source /opt/ros/jazzy/setup.bash"
    "source $ROS2_WS/install/setup.bash"
)
for line in "${SETUP_LINES[@]}"; do
    if ! grep -qF "$line" ~/.bashrc; then
        echo "$line" >> ~/.bashrc
    fi
done

# Create systemd service for auto-start
sudo tee /etc/systemd/system/drone-mapper.service > /dev/null <<EOF
[Unit]
Description=Drone Mapper SLAM Service
After=network.target

[Service]
Type=simple
User=$USER
Environment="ROS_DOMAIN_ID=0"
WorkingDirectory=$PROJECT_DIR
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && python3 $PROJECT_DIR/main.py scan"
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload

echo ""
echo "============================================================"
echo "  SETUP COMPLETE!"
echo "============================================================"
echo ""
echo "Next steps:"
echo "  1. Log out and log back in (for group permissions)"
echo "  2. Source environment: source ~/.bashrc"
echo "  3. Run health check: python3 main.py check"
echo "  4. Start scanning: python3 main.py scan"
echo ""
echo "Optional: Enable auto-start on boot:"
echo "  sudo systemctl enable drone-mapper.service"
echo ""
