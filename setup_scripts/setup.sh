#!/bin/bash
# =============================================================================
# DronePi — Master Setup Script
# Fresh Ubuntu 24.04 (arm64) on Raspberry Pi 5 → fully operational
#
# Usage:
#   sudo bash scripts/setup.sh
#
# What this does (in order):
#   [1]  System update + base build tools
#   [2]  ROS 2 Jazzy (native)
#   [3]  MAVROS
#   [4]  Unitree L1 LiDAR driver + colcon symlink fix
#   [5]  Point-LIO SLAM
#   [6]  Miniforge (Conda) + dronepi environment + all Python deps
#   [7]  udev rules (ttyPixhawk, ttyUSB0)
#   [8]  Build ROS 2 workspace
#   [9]  CPU governor → performance
#   [10] Systemd services (hotspot, mesh-server, foxglove, mavros,
#         drone-watchdog, dronepi-main)
#
# Run as root (sudo). The script will not proceed without it.
# Re-running is safe — each step checks before acting.
# =============================================================================

set -euo pipefail

# ── Guards ────────────────────────────────────────────────────────────────────
if [[ $EUID -ne 0 ]]; then
    echo "[ERROR] Run with sudo: sudo bash scripts/setup.sh"
    exit 1
fi

UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
if [[ "$UBUNTU_VERSION" != "24.04" ]]; then
    echo "[WARN] Expected Ubuntu 24.04, detected $UBUNTU_VERSION. Proceeding anyway."
fi

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_WS="$PROJECT_DIR/RPI5/ros2_ws"
DRONEPI_USER="dronepi"
DRONEPI_HOME="/home/$DRONEPI_USER"
CONDA_PREFIX="$DRONEPI_HOME/miniforge3"
CONDA_ENV="dronepi"

# ── Colour helpers ────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

step()  { echo -e "\n${CYAN}${BOLD}[$1] $2${NC}"; }
ok()    { echo -e "  ${GREEN}OK${NC}  $1"; }
warn()  { echo -e "  ${YELLOW}WARN${NC} $1"; }
fail()  { echo -e "  ${RED}FAIL${NC} $1"; exit 1; }
skip()  { echo -e "  ${YELLOW}SKIP${NC} $1 (already done)"; }

# ── Banner ────────────────────────────────────────────────────────────────────
echo -e "${BOLD}"
echo "  ╔══════════════════════════════════════════════════════╗"
echo "  ║   DronePi — Master Setup                            ║"
echo "  ║   Ubuntu 24.04 · Raspberry Pi 5 · ROS 2 Jazzy      ║"
echo "  ╚══════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo "  Project : $PROJECT_DIR"
echo "  User    : $DRONEPI_USER"
echo "  ROS2 WS : $ROS2_WS"
echo ""
read -rp "  Continue? (y/N) " REPLY
[[ "$REPLY" =~ ^[Yy]$ ]] || { echo "Aborted."; exit 0; }

# =============================================================================
# [1] System update + base tools
# =============================================================================
step "1/10" "System update + base tools"

apt-get update -qq
apt-get upgrade -y -qq
apt-get install -y -qq \
    git curl wget lsb-release gnupg2 \
    build-essential cmake ninja-build \
    python3-pip python3-venv \
    software-properties-common \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libpcl-dev \
    can-utils \
    net-tools \
    htop \
    rsync

ok "Base tools installed"

# =============================================================================
# [2] ROS 2 Jazzy
# =============================================================================
step "2/10" "ROS 2 Jazzy"

if dpkg -l ros-jazzy-ros-base &>/dev/null; then
    skip "ros-jazzy-ros-base already installed"
else
    add-apt-repository universe -y
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | tee /etc/apt/sources.list.d/ros2.list > /dev/null

    apt-get update -qq
    apt-get install -y -qq \
        ros-jazzy-ros-base \
        ros-jazzy-ros-dev-tools \
        ros-jazzy-pcl-ros \
        ros-jazzy-pcl-conversions \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-visualization-msgs \
        python3-colcon-common-extensions \
        python3-rosdep

    rosdep init 2>/dev/null || true
    sudo -u "$DRONEPI_USER" rosdep update

    ok "ROS 2 Jazzy installed"
fi

# =============================================================================
# [3] MAVROS
# =============================================================================
step "3/10" "MAVROS"

if dpkg -l ros-jazzy-mavros &>/dev/null; then
    skip "ros-jazzy-mavros already installed"
else
    apt-get install -y -qq \
        ros-jazzy-mavros \
        ros-jazzy-mavros-msgs \
        ros-jazzy-mavros-extras

    # Install GeographicLib datasets (required by MAVROS)
    /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh || \
        warn "GeographicLib install failed — run manually if MAVROS fails to start"

    ok "MAVROS installed"
fi

# foxglove bridge
if dpkg -l ros-jazzy-foxglove-bridge &>/dev/null; then
    skip "foxglove-bridge already installed"
else
    apt-get install -y -qq ros-jazzy-foxglove-bridge
    ok "foxglove-bridge installed"
fi

# =============================================================================
# [4] Unitree L1 LiDAR driver + colcon symlink fix
# =============================================================================
step "4/10" "Unitree L1 LiDAR driver"

mkdir -p "$ROS2_WS/src"
UNILIDAR_DEST="$ROS2_WS/src/unilidar_sdk"
UNILIDAR_LINK="$ROS2_WS/src/unitree_lidar_ros2"

if [[ ! -d "$UNILIDAR_DEST" ]]; then
    sudo -u "$DRONEPI_USER" git clone \
        https://github.com/unitreerobotics/unilidar_sdk.git \
        "$UNILIDAR_DEST"
    ok "unilidar_sdk cloned"
else
    skip "unilidar_sdk already present"
fi

# Colcon symlink fix — package.xml is nested two levels deep inside the SDK.
# Without this symlink colcon never discovers the package and the binary
# never gets built. This was the root cause of the historical serial timeout.
PACKAGE_DEEP="$UNILIDAR_DEST/unitree_lidar_ros2/src/unitree_lidar_ros2"
if [[ ! -L "$UNILIDAR_LINK" ]]; then
    sudo -u "$DRONEPI_USER" ln -s "$PACKAGE_DEEP" "$UNILIDAR_LINK"
    ok "colcon symlink created at correct discovery depth"
else
    skip "colcon symlink already exists"
fi

# =============================================================================
# [5] Point-LIO SLAM
# =============================================================================
step "5/10" "Point-LIO SLAM"

POINTLIO_DIR="$ROS2_WS/src/point_lio_ros2"
if [[ ! -d "$POINTLIO_DIR" ]]; then
    sudo -u "$DRONEPI_USER" git clone \
        https://github.com/dfloreaa/point_lio_ros2.git \
        "$POINTLIO_DIR"
    ok "point_lio_ros2 cloned"
else
    skip "point_lio_ros2 already present"
fi

# =============================================================================
# [6] Miniforge (Conda) + dronepi environment + Python deps
# =============================================================================
step "6/10" "Miniforge + dronepi Conda environment"

# Install Miniforge if not present
if [[ ! -f "$CONDA_PREFIX/bin/conda" ]]; then
    MINIFORGE_INSTALLER="/tmp/Miniforge3-Linux-aarch64.sh"
    curl -fsSL \
        https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh \
        -o "$MINIFORGE_INSTALLER"
    sudo -u "$DRONEPI_USER" bash "$MINIFORGE_INSTALLER" -b -p "$CONDA_PREFIX"
    rm -f "$MINIFORGE_INSTALLER"
    ok "Miniforge installed at $CONDA_PREFIX"
else
    skip "Miniforge already installed"
fi

CONDA_BIN="$CONDA_PREFIX/bin/conda"
CONDA_RUN="sudo -u $DRONEPI_USER $CONDA_PREFIX/bin/conda run -n $CONDA_ENV"

# Create dronepi environment if not present
if ! sudo -u "$DRONEPI_USER" "$CONDA_BIN" env list | grep -q "^$CONDA_ENV "; then
    sudo -u "$DRONEPI_USER" "$CONDA_BIN" create -n "$CONDA_ENV" \
        python=3.12 -y -q
    ok "dronepi Conda environment created"
else
    skip "dronepi Conda environment already exists"
fi

# Install conda-forge packages
echo "  Installing conda-forge packages (open3d, pdal)..."
sudo -u "$DRONEPI_USER" "$CONDA_BIN" install -n "$CONDA_ENV" -y -q \
    -c conda-forge \
    "open3d>=0.19" \
    "pdal>=3.5" \
    numpy scipy pyyaml psutil

ok "conda-forge packages installed"

# Install pip packages inside the Conda env
# empy must be pinned to 3.3.4 — ROS 2 Jazzy requires the legacy em module
# API that was removed in empy 4.x. Installing any newer version breaks colcon.
echo "  Installing pip packages inside dronepi env..."
$CONDA_RUN pip install -q \
    "empy==3.3.4" \
    catkin_pkg \
    lark \
    colcon-common-extensions \
    colcon-ros \
    rosdep \
    osrf-pycommon \
    trimesh \
    rosbags \
    pymeshlab \
    websocket-client \
    rpi-lgpio

ok "pip packages installed inside dronepi env"

# Add conda activate to .bashrc if not already there
BASHRC="$DRONEPI_HOME/.bashrc"
CONDA_INIT_LINE="source $CONDA_PREFIX/etc/profile.d/conda.sh"
CONDA_ACTIVATE_LINE="conda activate $CONDA_ENV"

if ! grep -qF "$CONDA_INIT_LINE" "$BASHRC" 2>/dev/null; then
    echo "" >> "$BASHRC"
    echo "# Conda — added by DronePi setup" >> "$BASHRC"
    echo "$CONDA_INIT_LINE" >> "$BASHRC"
    echo "$CONDA_ACTIVATE_LINE" >> "$BASHRC"
    chown "$DRONEPI_USER:$DRONEPI_USER" "$BASHRC"
    ok "Conda activate added to .bashrc"
else
    skip "Conda activate already in .bashrc"
fi

# Add ROS 2 source lines to .bashrc
ROS_SOURCE="source /opt/ros/jazzy/setup.bash"
WS_SOURCE="source $ROS2_WS/install/setup.bash"

for line in "$ROS_SOURCE" "$WS_SOURCE"; do
    if ! grep -qF "$line" "$BASHRC" 2>/dev/null; then
        echo "$line" >> "$BASHRC"
    fi
done

# =============================================================================
# [7] udev rules
# =============================================================================
step "7/10" "udev rules"

UDEV_FILE="/etc/udev/rules.d/99-dronepi.rules"

# Write rules whether or not the file exists to ensure they are current
tee "$UDEV_FILE" > /dev/null <<'EOF'
# Pixhawk 6X — persistent symlink at /dev/ttyPixhawk
# Matches any USB CDC ACM device. Refine ATTRS{idVendor} if needed.
SUBSYSTEM=="tty", KERNEL=="ttyACM*", MODE="0666", GROUP="dialout", SYMLINK+="ttyPixhawk"

# Unitree L1 LiDAR — CP210x USB-to-UART
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", MODE="0666", GROUP="dialout"
EOF

udevadm control --reload-rules
udevadm trigger

# Add user to dialout
usermod -aG dialout "$DRONEPI_USER"

ok "udev rules written and reloaded"

# =============================================================================
# [8] Build ROS 2 workspace
# =============================================================================
step "8/10" "Build ROS 2 workspace"

mkdir -p "$ROS2_WS"
chown -R "$DRONEPI_USER:$DRONEPI_USER" "$ROS2_WS"

# Build inside the dronepi Conda env so colcon uses the correct Python
# with catkin_pkg, empy==3.3.4, and colcon-ros already installed.
echo "  Building packages (this takes a few minutes on Pi 5)..."
sudo -u "$DRONEPI_USER" bash -c "
    source $CONDA_PREFIX/etc/profile.d/conda.sh
    conda activate $CONDA_ENV
    source /opt/ros/jazzy/setup.bash
    cd $ROS2_WS
    colcon build \
        --symlink-install \
        --packages-select unitree_lidar_ros2 point_lio \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_direct+
"

ok "ROS 2 workspace built"

# =============================================================================
# [9] CPU governor
# =============================================================================
step "9/10" "CPU governor → performance"

if [[ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]]; then
    echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null

    # Persist across reboots via rc.local
    RC_LOCAL="/etc/rc.local"
    GOV_LINE='echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null'
    if [[ ! -f "$RC_LOCAL" ]]; then
        echo '#!/bin/bash' > "$RC_LOCAL"
        echo "$GOV_LINE" >> "$RC_LOCAL"
        echo "exit 0" >> "$RC_LOCAL"
        chmod +x "$RC_LOCAL"
    elif ! grep -qF "$GOV_LINE" "$RC_LOCAL"; then
        sed -i "/^exit 0/i $GOV_LINE" "$RC_LOCAL"
    fi
    ok "CPU governor set to performance (persistent)"
else
    warn "cpufreq not available — skipping governor"
fi

# =============================================================================
# [10] Systemd services
# =============================================================================
step "10/10" "Systemd services"

SCRIPTS="$SCRIPT_DIR"

run_sub() {
    local sub="$1"
    local label="$2"
    if [[ -f "$SCRIPTS/$sub" ]]; then
        echo "  Running $sub..."
        bash "$SCRIPTS/$sub"
        ok "$label"
    else
        warn "$sub not found at $SCRIPTS/$sub — skipping"
    fi
}

run_sub "setup_hotspot_service.sh"      "dronepi-hotspot service"
run_sub "setup_mesh_server_service.sh"  "drone-mesh-server service"
run_sub "setup_foxglove_service.sh"     "foxglove-bridge service"
run_sub "setup_mavros_service.sh"       "mavros service"
run_sub "setup_watchdog_service.sh"     "drone-watchdog service"
run_sub "setup_main_service.sh"         "dronepi-main service"

# Reload daemon and enable all
systemctl daemon-reload
for svc in \
    dronepi-hotspot \
    drone-mesh-server \
    foxglove-bridge \
    mavros \
    drone-watchdog \
    dronepi-main; do
    if systemctl list-unit-files "${svc}.service" &>/dev/null; then
        systemctl enable "${svc}.service" 2>/dev/null || true
    fi
done

ok "All services enabled"

# =============================================================================
# Summary
# =============================================================================
echo ""
echo -e "${GREEN}${BOLD}"
echo "  ╔══════════════════════════════════════════════════════╗"
echo "  ║   Setup complete                                     ║"
echo "  ╚══════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo "  Next steps (do these in order):"
echo ""
echo "  1. Log out and back in (group membership takes effect)"
echo "  2. Connect SSD and confirm:  lsblk && df -h /mnt/ssd"
echo "  3. Verify udev symlinks:     ls -l /dev/ttyPixhawk"
echo "  4. Source environment:       source ~/.bashrc"
echo "  5. Run preflight check:      bash preflight_check.sh"
echo "  6. Reboot for services:      sudo reboot"
echo ""
echo "  After reboot:"
echo "    Check all services:  systemctl status mavros drone-watchdog \\"
echo "                           drone-mesh-server foxglove-bridge"
echo "    Run bench test:      python3 utils/bench_test.py"
echo "    Open viewer:         http://10.42.0.1:8080/meshview.html"
echo ""
