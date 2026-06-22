#!/usr/bin/env bash
# ============================================================================
# setup_vm.sh — One-shot setup for the Mission Control GUI on the operator VM
# ============================================================================
# Run this ON THE VM (192.168.0.100), not on the Jetson.
#
# What it does:
#   1. Installs PyQt5 + ROS2 humble deps (assumes ros-humble-ros-base is already
#      installed)
#   2. Generates an SSH key if missing and copies it to the Jetson for
#      passwordless login
#   3. Exports the env vars the GUI uses
#
# Usage:
#   chmod +x setup_vm.sh
#   ./setup_vm.sh               # uses defaults (lunabotics@192.168.0.200)
#   JETSON_USER=foo JETSON_HOST=10.0.0.1 ./setup_vm.sh
# ============================================================================

set -euo pipefail

JETSON_USER="${JETSON_USER:-lunabotics}"
JETSON_HOST="${JETSON_HOST:-192.168.0.200}"

echo ""
echo "=========================================="
echo " Mission Control GUI — VM Setup"
echo "=========================================="
echo " Jetson: ${JETSON_USER}@${JETSON_HOST}"
echo ""

# 1) System packages -------------------------------------------------- #
echo "[1/4] Installing PyQt5 + tools..."
sudo apt-get update -qq
sudo apt-get install -y -qq python3-pyqt5 python3-pip openssh-client iputils-ping

# 2) ROS2 sanity ------------------------------------------------------ #
echo ""
echo "[2/4] Checking ROS2 Humble..."
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "  [ERROR] /opt/ros/humble/setup.bash not found."
    echo "  Install ros-humble-ros-base first:"
    echo "    sudo apt install ros-humble-ros-base"
    exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
echo "  [OK] ROS2 Humble found ($(ros2 --version 2>&1 | head -1 || echo unknown))"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"

# 3) SSH key auth ----------------------------------------------------- #
echo ""
echo "[3/4] SSH key-based auth to Jetson..."
if [ ! -f "$HOME/.ssh/id_ed25519" ] && [ ! -f "$HOME/.ssh/id_rsa" ]; then
    echo "  Generating SSH key (no passphrase)..."
    ssh-keygen -t ed25519 -N "" -f "$HOME/.ssh/id_ed25519"
fi

# Test passwordless first
if ssh -o BatchMode=yes -o ConnectTimeout=5 "${JETSON_USER}@${JETSON_HOST}" "echo ok" >/dev/null 2>&1; then
    echo "  [OK] Passwordless SSH already works"
else
    echo "  Copying key to Jetson (you will be prompted for the Jetson password ONE last time)..."
    ssh-copy-id "${JETSON_USER}@${JETSON_HOST}"
    if ssh -o BatchMode=yes -o ConnectTimeout=5 "${JETSON_USER}@${JETSON_HOST}" "echo ok" >/dev/null 2>&1; then
        echo "  [OK] Passwordless SSH works"
    else
        echo "  [WARN] Passwordless SSH still failing — check ~/.ssh/authorized_keys on Jetson"
    fi
fi

# 4) env hint --------------------------------------------------------- #
echo ""
echo "[4/4] Setting Jetson connection in run_gui.sh..."
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ ! -f "$SCRIPT_DIR/run_gui.sh" ]; then
    echo "  [ERROR] $SCRIPT_DIR/run_gui.sh missing — did you sync the whole folder?"
    exit 1
fi
chmod +x "$SCRIPT_DIR/run_gui.sh"
# Patch the defaults to match what the user requested (idempotent).
sed -i -E \
    -e "s|^export JETSON_USER=.*|export JETSON_USER=\"\${JETSON_USER:-${JETSON_USER}}\"|" \
    -e "s|^export JETSON_HOST=.*|export JETSON_HOST=\"\${JETSON_HOST:-${JETSON_HOST}}\"|" \
    "$SCRIPT_DIR/run_gui.sh"
echo "  [OK] run_gui.sh patched: JETSON_USER=${JETSON_USER}  JETSON_HOST=${JETSON_HOST}"

echo ""
echo "=========================================="
echo " Setup done."
echo ""
echo " To launch the GUI:"
echo "     ./run_gui.sh"
echo ""
echo " Quick sanity check (run on VM AFTER bringup is up on Jetson):"
echo "     source /opt/ros/humble/setup.bash"
echo "     ros2 topic list | grep -E 'mission|pico|estop'"
echo "=========================================="
