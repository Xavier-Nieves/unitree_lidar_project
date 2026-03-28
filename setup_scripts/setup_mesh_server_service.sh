#!/bin/bash
# =============================================================================
# scripts/setup_mesh_server_service.sh
# Installs serve.py and meshview.html to /mnt/ssd/maps/ and registers
# the drone-mesh-server systemd service.
# Called by setup.sh or run standalone: sudo bash scripts/setup_mesh_server_service.sh
# =============================================================================
set -euo pipefail

SERVICE="drone-mesh-server"
MAPS_DIR="/mnt/ssd/maps"
DRONEPI_USER="dronepi"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
SERVER_SRC="$PROJECT_DIR/rpi_server"

[[ $EUID -ne 0 ]] && { echo "[ERROR] Run with sudo."; exit 1; }

echo "[mesh-server] Checking SSD mount..."
if ! mountpoint -q /mnt/ssd; then
    echo "[mesh-server] WARN: /mnt/ssd is not mounted. Service will be installed"
    echo "              but will fail to start until SSD is mounted and fstab is set."
fi

echo "[mesh-server] Setting up $MAPS_DIR..."
mkdir -p "$MAPS_DIR"

for f in serve.py meshview.html local_test.html; do
    if [[ -f "$SERVER_SRC/$f" ]]; then
        cp "$SERVER_SRC/$f" "$MAPS_DIR/$f"
        echo "[mesh-server] Copied $f"
    else
        echo "[mesh-server] WARN: $f not found at $SERVER_SRC/$f — skipping"
    fi
done

chown -R "$DRONEPI_USER:$DRONEPI_USER" "$MAPS_DIR"

echo "[mesh-server] Writing service file..."
tee /etc/systemd/system/${SERVICE}.service > /dev/null <<UNIT
[Unit]
Description=Drone Mesh HTTP Server — port 8080
After=mnt-ssd.mount dronepi-hotspot.service
Wants=mnt-ssd.mount dronepi-hotspot.service
RequiresMountsFor=/mnt/ssd

[Service]
Type=simple
User=${DRONEPI_USER}
WorkingDirectory=/mnt/ssd/maps
ExecStartPre=/bin/mountpoint -q /mnt/ssd
ExecStart=/usr/bin/python3 /mnt/ssd/maps/serve.py
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=drone-mesh-server

[Install]
WantedBy=multi-user.target
UNIT

systemctl daemon-reload
systemctl enable ${SERVICE}.service

# Only start if SSD is mounted
if mountpoint -q /mnt/ssd; then
    systemctl restart ${SERVICE}.service
    sleep 2
    STATUS=$(systemctl is-active ${SERVICE}.service)
    if [[ "$STATUS" == "active" ]]; then
        echo "[mesh-server] Service active — http://10.42.0.1:8080/meshview.html"
    else
        echo "[mesh-server] WARN: Service status is $STATUS"
        echo "              Check: sudo journalctl -u ${SERVICE} -n 30"
    fi
else
    echo "[mesh-server] Service installed but not started (SSD not mounted)."
    echo "              It will start automatically after reboot with SSD connected."
fi
