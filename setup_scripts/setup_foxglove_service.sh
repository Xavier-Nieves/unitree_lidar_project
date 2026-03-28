#!/bin/bash
# =============================================================================
# scripts/setup_foxglove_service.sh
# Registers foxglove-bridge as a systemd service.
# Called by setup.sh or run standalone: sudo bash scripts/setup_foxglove_service.sh
# =============================================================================
set -euo pipefail

SERVICE="foxglove-bridge"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
DRONEPI_USER="dronepi"

[[ $EUID -ne 0 ]] && { echo "[ERROR] Run with sudo."; exit 1; }

echo "[foxglove] Checking foxglove_bridge installation..."
if ! bash -c "source $ROS_SETUP && ros2 pkg list 2>/dev/null | grep -q foxglove_bridge"; then
    echo "[foxglove] Installing ros-jazzy-foxglove-bridge..."
    apt-get install -y -qq ros-jazzy-foxglove-bridge
fi

echo "[foxglove] Writing service file..."
tee /etc/systemd/system/${SERVICE}.service > /dev/null <<UNIT
[Unit]
Description=Foxglove Bridge — ROS 2 WebSocket on port 8765
After=dronepi-hotspot.service network.target
Wants=dronepi-hotspot.service

[Service]
Type=simple
User=${DRONEPI_USER}
Environment="HOME=/home/${DRONEPI_USER}"
ExecStart=/bin/bash -c "source ${ROS_SETUP} && \
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:=8765 \
    address:=0.0.0.0 \
    send_buffer_limit:=100000000 \
    max_qos_depth:=10"
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=foxglove-bridge

[Install]
WantedBy=multi-user.target
UNIT

systemctl daemon-reload
systemctl enable ${SERVICE}.service
systemctl restart ${SERVICE}.service

sleep 2
STATUS=$(systemctl is-active ${SERVICE}.service)
if [[ "$STATUS" == "active" ]]; then
    echo "[foxglove] Service active — ws://10.42.0.1:8765"
else
    echo "[foxglove] WARN: Service status is $STATUS"
    echo "           Check: sudo journalctl -u ${SERVICE} -n 30"
fi
