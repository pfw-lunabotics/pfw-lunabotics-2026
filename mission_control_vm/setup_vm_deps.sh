#!/usr/bin/env bash
# ============================================================================
# setup_vm_deps.sh — Install OS packages required by the Mission Control GUI.
# ============================================================================
# THIS IS THE ONLY SCRIPT IN THIS FOLDER THAT NEEDS INTERNET.
# Run it ONCE on the operator VM, at home / on a network. After that, every
# other script in this folder is fully offline.
#
# Run this ON THE VM (not on the Jetson).
#
# Usage:
#   chmod +x setup_vm_deps.sh
#   ./setup_vm_deps.sh
# ============================================================================

set -euo pipefail

echo "=========================================="
echo " VM dependency install  (requires internet)"
echo "=========================================="

# Sanity: bail early if we obviously cannot reach apt mirrors. We never want
# this to hang at competition.
if ! ping -c 1 -W 2 archive.ubuntu.com >/dev/null 2>&1; then
    echo "[WARN] archive.ubuntu.com unreachable. If you are already at the"
    echo "       competition with no internet, you don't need this script —"
    echo "       you only need it to be run ONCE at home."
    echo "       Continuing anyway in case you have a private mirror..."
fi

echo "[1/2] apt update + install PyQt5, ssh client, ping..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    python3-pyqt5 \
    python3-pip \
    openssh-client \
    iputils-ping

echo ""
echo "[2/2] ROS2 Humble check..."
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "  [ERROR] /opt/ros/humble/setup.bash not found."
    echo "  Install ros-humble-ros-base separately (also needs internet)."
    exit 1
fi
echo "  [OK] ROS2 Humble found"

echo ""
echo "=========================================="
echo " VM deps done. From now on you can be offline."
echo ""
echo " Next:"
echo "   ./setup_ethernet.sh    # configure static IP on this VM (no internet)"
echo "   ./setup_ssh_keys.sh    # passwordless ssh to Jetson (no internet)"
echo "   ./run_gui.sh           # launch the GUI"
echo "=========================================="
