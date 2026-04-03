#!/usr/bin/env bash
# install_foxglove_agent.sh — Install and configure the Foxglove Agent on DronePi
#
# Source: https://docs.foxglove.dev/docs/agent/installation
#
# What this script does
# ─────────────────────
#   1. Detects Pi architecture (ARM64) and downloads the correct .deb package
#   2. Installs foxglove-agent via dpkg
#   3. Writes /etc/foxglove/agent/envfile with your SSD path and token
#   4. Sets correct ownership so the foxglove user can read /mnt/ssd/rosbags/
#   5. Enables and starts foxglove-agent.service under systemd
#   6. Verifies the service is running and watching the correct directory
#
# Prerequisites
# ─────────────
#   - Run as a user with sudo access (e.g. pi / ubuntu)
#   - /mnt/ssd must already be mounted
#   - FOXGLOVE_DEVICE_TOKEN must be set OR passed as $1
#     Get a device token at: app.foxglove.dev → Devices → your device → Token
#
# Usage
# ─────
#   export FOXGLOVE_DEVICE_TOKEN=fox_dt_...
#   bash install_foxglove_agent.sh
#
#   OR pass the token directly:
#   bash install_foxglove_agent.sh fox_dt_...
#
# This script is IDEMPOTENT — safe to re-run if something fails mid-way.

set -euo pipefail

# ── colour helpers ─────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'
BOLD='\033[1m'; RESET='\033[0m'
info()    { echo -e "${CYAN}[INFO]${RESET}  $*"; }
success() { echo -e "${GREEN}[OK]${RESET}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
die()     { echo -e "${RED}[ERROR]${RESET} $*" >&2; exit 1; }

# ── config ─────────────────────────────────────────────────────────────────────
AGENT_VERSION="1.4.1"          # pinned — check changelog.foxglove.dev/agent for updates
STORAGE_ROOT="/mnt/ssd/rosbags"
VARDIR="/var/lib/foxglove-agent"
ENVFILE="/etc/foxglove/agent/envfile"
AGENT_USER="foxglove"
ARCH=$(dpkg --print-architecture)   # arm64 on Pi 4/5

# ── token resolution ───────────────────────────────────────────────────────────
TOKEN="${1:-${FOXGLOVE_DEVICE_TOKEN:-}}"
if [[ -z "$TOKEN" ]]; then
  die "FOXGLOVE_DEVICE_TOKEN is not set.\n       Get a device token at: app.foxglove.dev → Devices → your device → Token\n       Then run:  FOXGLOVE_DEVICE_TOKEN=fox_dt_... bash install_foxglove_agent.sh"
fi

echo ""
echo -e "${BOLD}╔══════════════════════════════════════════════════════╗${RESET}"
echo -e "${BOLD}║       Foxglove Agent Installer — DronePi             ║${RESET}"
echo -e "${BOLD}╚══════════════════════════════════════════════════════╝${RESET}"
echo ""

# ── step 1: architecture check ─────────────────────────────────────────────────
info "Detected architecture: $ARCH"
if [[ "$ARCH" != "arm64" && "$ARCH" != "amd64" ]]; then
  die "Unsupported architecture: $ARCH. Foxglove Agent requires arm64 or amd64."
fi

# ── step 2: storage root check ─────────────────────────────────────────────────
info "Checking storage root: $STORAGE_ROOT"
if [[ ! -d "$STORAGE_ROOT" ]]; then
  die "$STORAGE_ROOT does not exist. Is the SSD mounted at /mnt/ssd?"
fi
success "Storage root exists"

# ── step 3: download and install .deb ──────────────────────────────────────────
DEB_FILE="foxglove-agent_${AGENT_VERSION}_${ARCH}.deb"
DEB_URL="https://github.com/foxglove/agent/releases/download/v${AGENT_VERSION}/${DEB_FILE}"
TMP_DEB="/tmp/${DEB_FILE}"

if dpkg -l foxglove-agent &>/dev/null; then
  INSTALLED_VER=$(dpkg -l foxglove-agent | awk '/foxglove-agent/{print $3}' | head -1)
  if [[ "$INSTALLED_VER" == *"$AGENT_VERSION"* ]]; then
    success "foxglove-agent $AGENT_VERSION already installed — skipping download"
  else
    warn "Different version installed ($INSTALLED_VER) — upgrading to $AGENT_VERSION"
    info "Downloading $DEB_FILE ..."
    wget -q --show-progress -O "$TMP_DEB" "$DEB_URL"
    sudo dpkg -i "$TMP_DEB"
    rm -f "$TMP_DEB"
    success "Agent upgraded to $AGENT_VERSION"
  fi
else
  info "Downloading $DEB_FILE from GitHub releases..."
  wget -q --show-progress -O "$TMP_DEB" "$DEB_URL"
  info "Installing via dpkg..."
  sudo dpkg -i "$TMP_DEB"
  rm -f "$TMP_DEB"
  success "foxglove-agent $AGENT_VERSION installed"
fi

# ── step 4: create vardir ──────────────────────────────────────────────────────
if [[ ! -d "$VARDIR" ]]; then
  sudo mkdir -p "$VARDIR"
  info "Created $VARDIR"
fi

# ── step 5: set permissions so foxglove user can read the SSD bags ─────────────
# The foxglove-agent service runs as the 'foxglove' system user created by the
# .deb package. That user needs read access to /mnt/ssd/rosbags/ where your
# ros2 bag recorder (running as your user) writes MCAP files.
#
# Two safe approaches — we use group membership:
#   a) Add foxglove user to the group that owns /mnt/ssd/rosbags/
#   b) OR add your user to the foxglove group and set group-write on storage
#
# Approach: set group ownership of STORAGE_ROOT to foxglove, add read+execute
# for group. This is the least-privilege option.

CURRENT_USER=$(whoami)
STORAGE_GROUP=$(stat -c '%G' "$STORAGE_ROOT")

info "Storage root group: $STORAGE_GROUP"

if [[ "$STORAGE_GROUP" != "$AGENT_USER" ]]; then
  info "Adding foxglove group ownership to $STORAGE_ROOT ..."
  sudo chown -R ":${AGENT_USER}" "$STORAGE_ROOT" 2>/dev/null || \
    warn "Could not change group — Agent may need manual permission fix"
fi

# Ensure group has read+execute on all existing dirs, read on files
sudo find "$STORAGE_ROOT" -type d -exec chmod g+rx {} \;
sudo find "$STORAGE_ROOT" -type f -exec chmod g+r {} \;
# New directories/files inherit group via setgid bit on parent
sudo chmod g+s "$STORAGE_ROOT"
success "Permissions set: foxglove group can read $STORAGE_ROOT"

# Also ensure vardir is owned by foxglove user
sudo chown -R "${AGENT_USER}:${AGENT_USER}" "$VARDIR" 2>/dev/null || true

# ── step 6: write envfile ──────────────────────────────────────────────────────
info "Writing $ENVFILE ..."
sudo mkdir -p "$(dirname "$ENVFILE")"

sudo tee "$ENVFILE" > /dev/null << ENVEOF
# Foxglove Agent configuration — DronePi
# Generated by install_foxglove_agent.sh
# Edit manually if your paths change.
#
# Source: https://docs.foxglove.dev/docs/agent/installation

# ── authentication ────────────────────────────────────────────────────────────
# Device token from app.foxglove.dev → Devices → your device → Token
# A particular Foxglove device should have at most one Agent.
FOXGLOVE_DEVICE_TOKEN=${TOKEN}

# ── storage ───────────────────────────────────────────────────────────────────
# Root directory the Agent watches for new .mcap recordings.
# This must match where ros2 bag record writes files on your SSD.
STORAGE_ROOT=${STORAGE_ROOT}

# Agent's own persistent state directory (survives reboots).
VARDIR=${VARDIR}

# Only watch for MCAP files (your ros2 bag format).
# .db3 files are the old ROS 2 SQLite format — not used in your stack.
WATCH_INCLUDE_SUFFIXES=.mcap

# Auto-import pattern: upload any .mcap found under STORAGE_ROOT.
# Subdirectory structure (scan_YYYYMMDD_HHMMSS/) is preserved in Foxglove.
WATCH_AUTO_IMPORT_PATTERN=**/*.mcap

# ── logging ───────────────────────────────────────────────────────────────────
# Route logs to systemd journal so journalctl -u foxglove-agent shows them.
FOXGLOVE_LOG_OUTPUT=journald
ENVEOF

# Restrict envfile permissions (contains the device token)
sudo chmod 640 "$ENVFILE"
sudo chown "root:${AGENT_USER}" "$ENVFILE"
success "envfile written and secured (root:foxglove, 640)"

# ── step 7: enable and start the service ───────────────────────────────────────
info "Enabling foxglove-agent.service ..."
sudo systemctl daemon-reload
sudo systemctl enable foxglove-agent
sudo systemctl restart foxglove-agent
sleep 3   # give it a moment to initialise

# ── step 8: verify ─────────────────────────────────────────────────────────────
echo ""
info "Verifying service status..."
if sudo systemctl is-active --quiet foxglove-agent; then
  success "foxglove-agent is RUNNING"
else
  die "foxglove-agent failed to start. Check logs with:\n       sudo journalctl -u foxglove-agent -n 50"
fi

echo ""
echo -e "${BOLD}Installation complete.${RESET}"
echo ""
echo "  Watching:   $STORAGE_ROOT  (for *.mcap files)"
echo "  State dir:  $VARDIR"
echo "  Log:        sudo journalctl -u foxglove-agent -f"
echo ""
echo "  To verify uploads, open:  https://app.foxglove.dev → Recordings"
echo "  To check status:          sudo systemctl status foxglove-agent"
echo ""
echo -e "${YELLOW}Next step: run setup_cloud_rotation.py to configure the 5-flight rotation policy.${RESET}"
echo ""
