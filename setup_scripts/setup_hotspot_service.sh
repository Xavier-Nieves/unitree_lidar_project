#!/bin/bash
# =============================================================================
# scripts/setup_hotspot_service.sh
# Configures the Pi as a Wi-Fi access point (SSID: dronepi-ap)
# and registers dronepi-hotspot as a systemd service.
# Called by setup.sh or run standalone: sudo bash scripts/setup_hotspot_service.sh
# =============================================================================
set -euo pipefail

SERVICE="dronepi-hotspot"
DRONEPI_USER="dronepi"
SSID="dronepi-ap"
PASSWORD="dronepi123"
INTERFACE="wlan0"
IP="10.42.0.1"
DHCP_RANGE="10.42.0.10,10.42.0.50,24h"

[[ $EUID -ne 0 ]] && { echo "[ERROR] Run with sudo."; exit 1; }

echo "[hotspot] Installing hostapd and dnsmasq..."
apt-get install -y -qq hostapd dnsmasq

systemctl stop hostapd dnsmasq 2>/dev/null || true
systemctl unmask hostapd

echo "[hotspot] Configuring hostapd..."
tee /etc/hostapd/hostapd.conf > /dev/null <<EOF
interface=${INTERFACE}
driver=nl80211
ssid=${SSID}
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=${PASSWORD}
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
EOF

echo 'DAEMON_CONF="/etc/hostapd/hostapd.conf"' > /etc/default/hostapd

echo "[hotspot] Configuring dnsmasq..."
# Back up original if not already backed up
[[ ! -f /etc/dnsmasq.conf.orig ]] && cp /etc/dnsmasq.conf /etc/dnsmasq.conf.orig

tee /etc/dnsmasq.conf > /dev/null <<EOF
interface=${INTERFACE}
dhcp-range=${DHCP_RANGE}
domain=local
address=/dronepi.local/${IP}
EOF

echo "[hotspot] Setting static IP on ${INTERFACE}..."
# Use NetworkManager nmcli if available, otherwise write dhcpcd config
if command -v nmcli &>/dev/null; then
    nmcli con delete "dronepi-hotspot" 2>/dev/null || true
    nmcli con add type wifi ifname "$INTERFACE" con-name "dronepi-hotspot" \
        autoconnect yes ssid "$SSID" \
        mode ap \
        ipv4.method shared \
        ipv4.addresses "${IP}/24" \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$PASSWORD" 2>/dev/null || true
    nmcli con up "dronepi-hotspot" 2>/dev/null || true
    echo "[hotspot] Configured via NetworkManager"
else
    # Fallback: static IP via dhcpcd
    DHCPCD_CONF="/etc/dhcpcd.conf"
    if ! grep -q "interface ${INTERFACE}" "$DHCPCD_CONF" 2>/dev/null; then
        echo "" >> "$DHCPCD_CONF"
        echo "interface ${INTERFACE}" >> "$DHCPCD_CONF"
        echo "    static ip_address=${IP}/24" >> "$DHCPCD_CONF"
        echo "    nohook wpa_supplicant" >> "$DHCPCD_CONF"
    fi
fi

echo "[hotspot] Writing service file..."
tee /etc/systemd/system/${SERVICE}.service > /dev/null <<UNIT
[Unit]
Description=DronePi Wi-Fi Hotspot (${SSID})
After=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c "\
    ip link set ${INTERFACE} up; \
    ip addr add ${IP}/24 dev ${INTERFACE} 2>/dev/null || true; \
    systemctl start hostapd; \
    systemctl start dnsmasq"
ExecStop=/bin/bash -c "\
    systemctl stop dnsmasq; \
    systemctl stop hostapd"
StandardOutput=journal
StandardError=journal
SyslogIdentifier=dronepi-hotspot

[Install]
WantedBy=multi-user.target
UNIT

systemctl daemon-reload
systemctl enable ${SERVICE}.service
systemctl enable hostapd
systemctl enable dnsmasq
systemctl start ${SERVICE}.service

sleep 3
STATUS=$(systemctl is-active ${SERVICE}.service)
if [[ "$STATUS" == "active" ]]; then
    echo "[hotspot] Service active — SSID: $SSID / $IP"
else
    echo "[hotspot] WARN: Service status is $STATUS"
    echo "          Check: sudo journalctl -u ${SERVICE} -n 30"
    echo "          The TP-Link RTL8821AU adapter must be present for hostapd to start."
fi
