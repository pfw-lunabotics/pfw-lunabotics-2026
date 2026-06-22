#!/usr/bin/env bash
# ============================================================================
# run_gui.sh — Launcher for the Mission Control GUI (runs on the operator VM)
# ============================================================================
# This script:
#   1. Sources ROS2 Humble (so rclpy + std_msgs/geometry_msgs work)
#   2. Sets ROS_DOMAIN_ID (must match the Jetson — default 0)
#   3. Exports Jetson connection env vars (override here or via shell env)
#   4. Launches mission_control.py
#
# You do NOT need to source any workspace install/setup.bash on the VM:
#   - The GUI imports only std_msgs / geometry_msgs / std_srvs (ros-humble-ros-base).
#   - When the GUI SSHes into the Jetson to run bringup or ad-hoc commands,
#     it sources /opt/ros/humble/setup.bash AND ~/pfw-lunabotics/install/setup.bash
#     inside that remote shell itself.
#
# Usage:
#   ./run_gui.sh
#
# Override Jetson on the fly:
#   JETSON_HOST=10.0.0.42 ./run_gui.sh
# ============================================================================

set -e

# --- ROS2 environment (VM side) ---
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
else
    echo "[run_gui.sh] ERROR: /opt/ros/humble/setup.bash not found."
    echo "  Install ROS2 Humble first:  sudo apt install ros-humble-ros-base"
    exit 1
fi

# Must match the Jetson — Jetson uses ROS_DOMAIN_ID=0 by default.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# --- Jetson connection (edit these or set via env) ---
export JETSON_USER="${JETSON_USER:-lunabotics}"
export JETSON_HOST="${JETSON_HOST:-192.168.0.200}"
export JETSON_WS="${JETSON_WS:-/home/lunabotics/pfw-lunabotics}"

echo "[run_gui.sh] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  Jetson=$JETSON_USER@$JETSON_HOST"

cd "$(dirname "$0")"
exec python3 ./mission_control.py
