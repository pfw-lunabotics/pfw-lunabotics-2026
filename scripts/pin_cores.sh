#!/usr/bin/env bash
# ============================================================================
# pin_cores.sh — Pin ROS2 Nodes to Dedicated CPU Cores
# ============================================================================
# Jetson Orin Nano has 6 cores. Without pinning, all ROS2 nodes fight for
# the same cores, and the kernel scheduler makes bad decisions (moves
# Point-LIO to a core that just got a Nav2 costmap update, etc.)
#
# This script assigns each subsystem to specific cores so they CAN'T
# starve each other:
#
#   Core 0:     Point-LIO (highest priority — localization is everything)
#   Core 1:     EKF + robot_state_publisher + joint_state_publisher
#   Core 2-3:   Nav2 (controller, planner, costmap, BT, behavior server)
#   Core 4:     Perception (obstacle detector, zone detector, quality monitor)
#   Core 5:     Everything else (control relay, pico bridge, watchdog, ROS2 daemon)
#
# Usage:
#   ./scripts/pin_cores.sh          # run after competition.sh, once nodes are up
#   ./scripts/pin_cores.sh --dry    # show what would happen without doing it
# ============================================================================

DRY_RUN=false
if [ "${1:-}" = "--dry" ]; then
    DRY_RUN=true
    echo "[DRY RUN — showing plan only]"
    echo ""
fi

PASS_COUNT=0
FAIL_COUNT=0

pin() {
    local PATTERN="$1"
    local CPUSET="$2"
    local LABEL="$3"

    # Find PIDs matching the pattern
    PIDS=$(pgrep -f "$PATTERN" 2>/dev/null || true)

    if [ -z "$PIDS" ]; then
        echo "  [ -- ] $LABEL — not running"
        return
    fi

    for PID in $PIDS; do
        COMM=$(cat /proc/$PID/comm 2>/dev/null || echo "?")
        if [ "$DRY_RUN" = true ]; then
            echo "  [PLAN] $LABEL (PID $PID, $COMM) → cores $CPUSET"
            PASS_COUNT=$((PASS_COUNT + 1))
        else
            if taskset -apc "$CPUSET" "$PID" > /dev/null 2>&1; then
                echo "  [ OK ] $LABEL (PID $PID) → cores $CPUSET"
                PASS_COUNT=$((PASS_COUNT + 1))
            else
                echo "  [FAIL] $LABEL (PID $PID) → cores $CPUSET"
                FAIL_COUNT=$((FAIL_COUNT + 1))
            fi
        fi
    done
}

set_priority() {
    local PATTERN="$1"
    local NICE_VAL="$2"
    local LABEL="$3"

    PIDS=$(pgrep -f "$PATTERN" 2>/dev/null || true)

    if [ -z "$PIDS" ]; then
        return
    fi

    for PID in $PIDS; do
        if [ "$DRY_RUN" = true ]; then
            echo "  [PLAN] $LABEL (PID $PID) → nice $NICE_VAL"
        else
            renice "$NICE_VAL" -p "$PID" > /dev/null 2>&1 || true
        fi
    done
}

echo "========================================="
echo "  CPU CORE PINNING — 6 Core Layout"
echo "========================================="
echo ""
echo "  Core 0:   Point-LIO (localization)"
echo "  Core 1:   EKF + TF publishers"
echo "  Core 2-3: Nav2 (planner, controller, costmap)"
echo "  Core 4:   Perception pipeline"
echo "  Core 5:   Control + Pico + watchdog + misc"
echo ""

# ------------------------------------------------------------------ #
# Core 0: Point-LIO (most critical — must not be starved)
# ------------------------------------------------------------------ #
echo "--- Core 0: Localization (Point-LIO) ---"
pin "pointlio_mapping" "0" "Point-LIO"

# ------------------------------------------------------------------ #
# Core 1: EKF + TF tree
# ------------------------------------------------------------------ #
echo "--- Core 1: EKF + TF ---"
pin "ekf_node\|ekf_filter_node" "1" "EKF"
pin "robot_state_publisher" "1" "robot_state_publisher"
pin "joint_state_publisher" "1" "joint_state_publisher"
pin "static_transform_publisher" "1" "static TF publisher"

# ------------------------------------------------------------------ #
# Core 2-3: Nav2 (needs 2 cores — costmap + planner are heavy)
# ------------------------------------------------------------------ #
echo "--- Core 2-3: Nav2 ---"
pin "controller_server" "2,3" "Nav2 controller"
pin "planner_server" "2,3" "Nav2 planner"
pin "bt_navigator" "2,3" "Nav2 BT navigator"
pin "behavior_server" "2,3" "Nav2 behavior server"
pin "map_server" "2,3" "Nav2 map server"
pin "lifecycle_manager" "2,3" "Nav2 lifecycle manager"
pin "amcl" "2,3" "AMCL (if running)"

# ------------------------------------------------------------------ #
# Core 4: Perception
# ------------------------------------------------------------------ #
echo "--- Core 4: Perception ---"
pin "unified_obstacle_detector" "4" "Obstacle detector"
pin "zone_detector" "4" "Zone detector"
pin "localization_quality_monitor" "4" "Localization quality"

# ------------------------------------------------------------------ #
# Core 5: Control + everything else
# ------------------------------------------------------------------ #
echo "--- Core 5: Control + misc ---"
pin "cmd_vel_relay" "5" "cmd_vel relay"
pin "safety_monitor" "5" "Safety monitor"
pin "system_watchdog" "5" "System watchdog"
pin "pico_bridge" "5" "Pico bridge"
pin "servo_driver" "5" "Servo driver"
pin "teleop_keyboard" "5" "Teleop keyboard"
pin "mission_controller" "5" "Mission controller"
pin "excavation_bridge" "5" "Excavation bridge"
pin "unitree_lidar_ros2" "5" "LiDAR driver"

# ------------------------------------------------------------------ #
# Process priorities (lower nice = higher priority)
# ------------------------------------------------------------------ #
echo ""
echo "--- Setting priorities ---"
set_priority "pointlio_mapping" "-10" "Point-LIO (high priority)"
set_priority "ekf_node\|ekf_filter_node" "-5" "EKF (above normal)"
set_priority "controller_server" "-3" "Nav2 controller (above normal)"
set_priority "unified_obstacle_detector" "-2" "Perception (above normal)"

echo ""
echo "========================================="
echo "  Pinned: $PASS_COUNT   Failed: $FAIL_COUNT"
echo "========================================="
