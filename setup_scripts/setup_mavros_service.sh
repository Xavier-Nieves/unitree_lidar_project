#!/bin/bash
# =============================================================================
# scripts/setup_mavros_service.sh
# Registers MAVROS as a systemd service using project config copies.
# Called by setup.sh or run standalone: sudo bash scripts/setup_mavros_service.sh
# =============================================================================
set -euo pipefail

SERVICE="mavros"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
DRONEPI_USER="dronepi"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
CONFIG_DIR="$PROJECT_DIR/config"

[[ $EUID -ne 0 ]] && { echo "[ERROR] Run with sudo."; exit 1; }

echo "[mavros] Writing service file..."
tee /etc/systemd/system/${SERVICE}.service > /dev/null <<UNIT
[Unit]
Description=MAVROS — MAVLink to ROS 2 bridge
After=network.target
Wants=network.target

[Service]
Type=simple
User=${DRONEPI_USER}
Environment="HOME=/home/${DRONEPI_USER}"
ExecStart=/bin/bash -c "source ${ROS_SETUP} && \
    ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:=/dev/ttyPixhawk:57600 \
    -p gcs_url:=udp://@10.42.0.255:14550 \
    --params-file ${CONFIG_DIR}/px4_config.yaml"
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=mavros

[Install]
WantedBy=multi-user.target
UNIT

systemctl daemon-reload
systemctl enable ${SERVICE}.service
systemctl restart ${SERVICE}.service

sleep 3
STATUS=$(systemctl is-active ${SERVICE}.service)
if [[ "$STATUS" == "active" ]]; then
    echo "[mavros] Service active — /dev/ttyPixhawk:57600"
else
    echo "[mavros] WARN: Service status is $STATUS"
    echo "         Pixhawk may not be connected yet — service will retry automatically."
    echo "         Check: sudo journalctl -u ${SERVICE} -n 30"
fi
