#!/bin/bash
# =============================================================================
# scripts/setup_watchdog_service.sh
# Registers drone-watchdog as a systemd service.
# Called by setup.sh or run standalone: sudo bash scripts/setup_watchdog_service.sh
# =============================================================================
set -euo pipefail

SERVICE="drone-watchdog"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
DRONEPI_USER="dronepi"
DRONEPI_HOME="/home/$DRONEPI_USER"
CONDA_PREFIX="$DRONEPI_HOME/miniforge3"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_WS="$PROJECT_DIR/RPI5/ros2_ws"
WATCHDOG="$PROJECT_DIR/flight/drone_watchdog.py"

[[ $EUID -ne 0 ]] && { echo "[ERROR] Run with sudo."; exit 1; }

echo "[watchdog] Writing service file..."
tee /etc/systemd/system/${SERVICE}.service > /dev/null <<UNIT
[Unit]
Description=DronePi Flight Stack Watchdog
After=network.target mavros.service
Requires=mavros.service

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
    python3 ${WATCHDOG}"
ExecStopPost=/bin/rm -f /tmp/dronepi_mission.lock
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal
SyslogIdentifier=drone-watchdog

[Install]
WantedBy=multi-user.target
UNIT

systemctl daemon-reload
systemctl enable ${SERVICE}.service
systemctl restart ${SERVICE}.service

sleep 3
STATUS=$(systemctl is-active ${SERVICE}.service)
if [[ "$STATUS" == "active" ]]; then
    echo "[watchdog] Service active"
else
    echo "[watchdog] WARN: Service status is $STATUS"
    echo "           MAVROS must be connected for watchdog to reach READY state."
    echo "           Check: sudo journalctl -u ${SERVICE} -n 30"
fi
