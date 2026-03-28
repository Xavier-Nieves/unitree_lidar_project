#!/bin/bash
# =============================================================================
# scripts/setup_main_service.sh
# Registers dronepi-main as a systemd service.
# Called by setup.sh or run standalone: sudo bash scripts/setup_main_service.sh
# =============================================================================
set -euo pipefail

SERVICE="dronepi-main"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
DRONEPI_USER="dronepi"
DRONEPI_HOME="/home/$DRONEPI_USER"
CONDA_PREFIX="$DRONEPI_HOME/miniforge3"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_WS="$PROJECT_DIR/RPI5/ros2_ws"
MAIN_PY="$PROJECT_DIR/main.py"

[[ $EUID -ne 0 ]] && { echo "[ERROR] Run with sudo."; exit 1; }

echo "[main] Writing service file..."
tee /etc/systemd/system/${SERVICE}.service > /dev/null <<UNIT
[Unit]
Description=DronePi Main — Top-level flight orchestrator
After=network.target mavros.service drone-watchdog.service
Requires=mavros.service drone-watchdog.service

[Service]
Type=simple
User=${DRONEPI_USER}
Environment="HOME=${DRONEPI_HOME}"
WorkingDirectory=${PROJECT_DIR}
ExecStart=/bin/bash -c "\
    source ${CONDA_PREFIX}/etc/profile.d/conda.sh && \
    conda activate dronepi && \
    source ${ROS_SETUP} && \
    source ${ROS2_WS}/install/setup.bash && \
    python3 ${MAIN_PY}"
ExecStopPost=/bin/rm -f /tmp/dronepi_mission.lock
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal
SyslogIdentifier=dronepi-main

[Install]
WantedBy=multi-user.target
UNIT

systemctl daemon-reload
systemctl enable ${SERVICE}.service

# main.py is inactive until first arm — do not start it now
echo "[main] Service installed and enabled."
echo "       It will activate automatically after first arm event."
