#!/usr/bin/env bash
# setup_flight_services.sh — Install MAVROS and drone watchdog as systemd services.
#
# Run once as root from the directory containing the service files:
#   sudo bash setup_flight_services.sh
#
# What this script does:
#   1. Copies drone_watchdog.py to the project directory
#   2. Installs mavros.service and drone-watchdog.service into systemd
#   3. Reloads the systemd daemon
#   4. Enables both services for auto-start on boot
#   5. Starts both services immediately
#   6. Prints status of all drone services
#
# Safe to re-run -- existing services are stopped before reinstalling.

set -euo pipefail

# ── config ────────────────────────────────────────────────────────────────────

PROJECT_DIR="/home/dronepi/unitree_lidar_project/unitree_drone_mapper/flight"
SYSTEMD_DIR="/etc/systemd/system"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── helpers ───────────────────────────────────────────────────────────────────

info()  { echo "  [INFO]  $*"; }
ok()    { echo "  [OK]    $*"; }
fail()  { echo "  [FAIL]  $*"; exit 1; }

# ── pre-flight checks ─────────────────────────────────────────────────────────

echo ""
echo "=================================================="
echo "  DronePi Flight Services Setup"
echo "=================================================="
echo ""

[[ $EUID -eq 0 ]] || fail "Run as root: sudo bash setup_flight_services.sh"

[[ -f "$SCRIPT_DIR/mavros.service"          ]] || fail "mavros.service not found in $SCRIPT_DIR"
[[ -f "$SCRIPT_DIR/drone-watchdog.service"  ]] || fail "drone-watchdog.service not found in $SCRIPT_DIR"
[[ -f "$SCRIPT_DIR/drone_watchdog.py"       ]] || fail "drone_watchdog.py not found in $SCRIPT_DIR"
[[ -d "$PROJECT_DIR"                         ]] || fail "Project directory not found: $PROJECT_DIR"

# ── stop existing services if running ────────────────────────────────────────

info "Stopping existing services if running..."
for svc in drone-watchdog mavros; do
    if systemctl is-active --quiet "$svc" 2>/dev/null; then
        systemctl stop "$svc"
        info "Stopped $svc"
    fi
done

# ── watchdog script is already in PROJECT_DIR (same folder as this script) ───
# No copy needed -- just set permissions.

info "Setting permissions on drone_watchdog.py..."
chown dronepi:dronepi "$PROJECT_DIR/drone_watchdog.py"
chmod 755 "$PROJECT_DIR/drone_watchdog.py"
ok "drone_watchdog.py ready at $PROJECT_DIR/"

# ── install service files ─────────────────────────────────────────────────────

info "Installing service files -> $SYSTEMD_DIR/"
cp "$SCRIPT_DIR/mavros.service"         "$SYSTEMD_DIR/mavros.service"
cp "$SCRIPT_DIR/drone-watchdog.service" "$SYSTEMD_DIR/drone-watchdog.service"
ok "Service files installed"

# ── reload and enable ─────────────────────────────────────────────────────────

info "Reloading systemd daemon..."
systemctl daemon-reload
ok "Daemon reloaded"

info "Enabling services for auto-start on boot..."
systemctl enable mavros.service
systemctl enable drone-watchdog.service
ok "Services enabled"

# ── start now ────────────────────────────────────────────────────────────────

info "Starting mavros.service..."
systemctl start mavros.service
sleep 2
if systemctl is-active --quiet mavros; then
    ok "mavros.service running"
else
    echo "  [WARN]  mavros.service did not start -- check: sudo journalctl -u mavros -n 20"
fi

info "Starting drone-watchdog.service..."
systemctl start drone-watchdog.service
sleep 2
if systemctl is-active --quiet drone-watchdog; then
    ok "drone-watchdog.service running"
else
    echo "  [WARN]  drone-watchdog.service did not start -- check: sudo journalctl -u drone-watchdog -n 20"
fi

# ── summary ───────────────────────────────────────────────────────────────────

echo ""
echo "=================================================="
echo "  All drone services status"
echo "=================================================="
echo ""
sudo systemctl status \
    dronepi-hotspot \
    drone-mesh-server \
    foxglove-bridge \
    mavros \
    drone-watchdog \
    --no-pager --lines=3 2>/dev/null || true

echo ""
echo "=================================================="
echo "  Setup complete."
echo ""
echo "  Boot sequence:"
echo "    hotspot       -> up at ~15s"
echo "    mesh-server   -> up at ~16s"
echo "    foxglove      -> up at ~18s"
echo "    mavros        -> up at ~25s  (Pixhawk connected)"
echo "    watchdog      -> polling /mavros/state"
echo ""
echo "  To activate flight stack:"
echo "    1. Arm the drone on the RC transmitter"
echo "    2. Switch to OFFBOARD mode"
echo "    3. Point-LIO and bag recording start automatically"
echo ""
echo "  To deactivate:"
echo "    Disarm or switch out of OFFBOARD mode"
echo "    (Point-LIO and bag stop, bag closes cleanly)"
echo ""
echo "  View watchdog logs:"
echo "    sudo journalctl -u drone-watchdog -f"
echo ""
echo "  View all drone logs:"
echo "    sudo journalctl -u mavros -u drone-watchdog -u foxglove-bridge -f"
echo "=================================================="
echo ""
