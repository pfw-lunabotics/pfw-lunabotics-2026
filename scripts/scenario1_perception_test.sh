#!/usr/bin/env bash
# ============================================================================
# scenario1_perception_test.sh — Perception + Obstacle Avoidance Sand Test
# ============================================================================
# ONE COMMAND to launch:
#   1. Configure LiDAR network (192.168.1.2 on eth0)
#   2. Verify LiDAR is publishing
#   3. Verify Pico serial is connected
#   4. Launch full hardware stack (no autonomy, no Nav2 goals)
#   5. Launch reactive wander with perception pipeline
#
# The robot drives forward, perception detects rocks (>=20cm) and craters
# (>=20cm), and the robot avoids them using sector-based obstacle avoidance.
#
# Usage:
#   ./scripts/scenario1_perception_test.sh
#   ./scripts/scenario1_perception_test.sh --no-localization    # skip Point-LIO
#   ./scripts/scenario1_perception_test.sh --duration 120       # run for 120s (default 300)
#   ./scripts/scenario1_perception_test.sh --speed 0.15         # slower (default 0.20)
#
# Tmux shortcuts:
#   Ctrl+b <arrow>   — switch panes
#   Ctrl+b z         — zoom/unzoom current pane
#   Ctrl+b d         — detach (keeps running)
#   tmux attach -t perception_test  — reattach
#
# To kill:
#   tmux kill-session -t perception_test
# ============================================================================

set -euo pipefail

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

SESSION="perception_test"
WORKSPACE="/home/lunabotics/pfw-lunabotics"

LIDAR_IP="192.168.1.62"
JETSON_IP="192.168.1.2"
NET_IFACE="enP8p1s0"

# --- Defaults ---
USE_LOCALIZATION="true"
DURATION="300"
SPEED="0.20"

# --- Parse arguments ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-localization)
            USE_LOCALIZATION="false"
            shift
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --speed)
            SPEED="$2"
            shift 2
            ;;
        *)
            echo -e "${RED}Unknown argument: $1${NC}"
            exit 1
            ;;
    esac
done

echo ""
echo -e "${BOLD}============================================${NC}"
echo -e "${BOLD}  SCENARIO 1: PERCEPTION + AVOIDANCE TEST${NC}"
echo -e "${BOLD}============================================${NC}"
echo -e "  Duration: ${DURATION}s | Speed: ${SPEED} m/s"
echo -e "  Localization: ${USE_LOCALIZATION}"
echo ""

# ------------------------------------------------------------------ #
# STEP 1: Network setup — ensure Jetson IP is on eth0
# ------------------------------------------------------------------ #
echo -e "${BOLD}[1/5] Network Setup${NC}"

# Bring interface up
sudo ip link set "$NET_IFACE" up 2>/dev/null || true

# Add IP if not already present
if ip addr show "$NET_IFACE" 2>/dev/null | grep -q "$JETSON_IP"; then
    echo -e "  ${GREEN}[OK]${NC} Jetson IP $JETSON_IP already on $NET_IFACE"
else
    echo -e "  ${YELLOW}[FIX]${NC} Adding $JETSON_IP/24 to $NET_IFACE..."
    sudo ip addr add "${JETSON_IP}/24" dev "$NET_IFACE" 2>/dev/null || true
    sleep 1
    if ip addr show "$NET_IFACE" 2>/dev/null | grep -q "$JETSON_IP"; then
        echo -e "  ${GREEN}[OK]${NC} IP added successfully"
    else
        echo -e "  ${RED}[FAIL]${NC} Could not add IP — check ethernet interface name"
        echo -e "  Available interfaces: $(ls /sys/class/net/)"
        exit 1
    fi
fi

# ------------------------------------------------------------------ #
# STEP 2: LiDAR connectivity check
# ------------------------------------------------------------------ #
echo ""
echo -e "${BOLD}[2/5] LiDAR Connectivity${NC}"

LIDAR_OK=false
for attempt in 1 2 3; do
    if ping -c 1 -W 2 "$LIDAR_IP" > /dev/null 2>&1; then
        LIDAR_OK=true
        break
    fi
    echo -e "  ${YELLOW}[RETRY]${NC} Ping attempt $attempt/3..."
    sleep 1
done

if $LIDAR_OK; then
    echo -e "  ${GREEN}[OK]${NC} LiDAR responding at $LIDAR_IP"
else
    echo -e "  ${RED}[FAIL]${NC} LiDAR not reachable at $LIDAR_IP"
    echo -e "  Check: ethernet cable plugged in? LiDAR powered on?"
    echo -e "  LiDAR needs ~10s after power-on to become reachable."
    echo ""
    read -rp "  Press Enter to continue anyway (or Ctrl+C to abort)... "
fi

# ------------------------------------------------------------------ #
# STEP 3: Pico serial check
# ------------------------------------------------------------------ #
echo ""
echo -e "${BOLD}[3/5] Pico Motor Controller${NC}"

PICO_SYMLINK="/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6647c156730ab24-if00"
PICO_OK=false

if [ -e "$PICO_SYMLINK" ]; then
    PICO_DEV=$(readlink -f "$PICO_SYMLINK")
    echo -e "  ${GREEN}[OK]${NC} Pico found: $PICO_SYMLINK -> $PICO_DEV"
    PICO_OK=true
elif [ -e /dev/ttyACM0 ]; then
    echo -e "  ${YELLOW}[WARN]${NC} Pico symlink missing, but /dev/ttyACM0 exists (probably Pico)"
    PICO_OK=true
else
    echo -e "  ${RED}[FAIL]${NC} No Pico USB device found"
    echo -e "  Check: USB cable plugged in? Pico powered?"
    echo ""
    read -rp "  Press Enter to continue anyway (or Ctrl+C to abort)... "
fi

# Check dialout group
if groups | grep -qE '(dialout|plugdev)'; then
    echo -e "  ${GREEN}[OK]${NC} Serial access (dialout group)"
else
    echo -e "  ${YELLOW}[WARN]${NC} Not in dialout group — may need: sudo usermod -aG dialout \$USER"
fi

# ------------------------------------------------------------------ #
# STEP 4: ROS2 + workspace sourcing check
# ------------------------------------------------------------------ #
echo ""
echo -e "${BOLD}[4/5] ROS2 Environment${NC}"

# Source ROS2 and workspace (disable -eu temporarily — setup.bash scripts
# reference unset variables and return non-zero internally)
set +eu
source /opt/ros/humble/setup.bash 2>/dev/null
source "$WORKSPACE/install/setup.bash" 2>/dev/null
set -eu

if ! command -v ros2 &>/dev/null; then
    echo -e "  ${RED}[FAIL]${NC} Cannot source /opt/ros/humble/setup.bash"
    exit 1
fi
echo -e "  ${GREEN}[OK]${NC} ROS2 Humble sourced"

if [ -d "$WORKSPACE/install/lunabotics_navigation" ]; then
    echo -e "  ${GREEN}[OK]${NC} Workspace sourced"
else
    echo -e "  ${RED}[FAIL]${NC} Workspace not built — run: cd $WORKSPACE && colcon build --symlink-install"
    exit 1
fi

# DDS config
if [ -f "$WORKSPACE/fastrtps_no_shm.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE/fastrtps_no_shm.xml"
    echo -e "  ${GREEN}[OK]${NC} FastRTPS no-SHM config loaded"
fi

# ------------------------------------------------------------------ #
# STEP 5: Launch in tmux
# ------------------------------------------------------------------ #
echo ""
echo -e "${BOLD}[5/5] Launching Stack${NC}"

# Kill existing session
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Check tmux
if ! command -v tmux &> /dev/null; then
    echo "  Installing tmux..."
    sudo apt-get update -qq && sudo apt-get install -y -qq tmux
fi

# --- Source preamble for each tmux pane ---
SOURCE_CMD="source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash"
FASTRTPS_EXPORT=""
if [ -f "$WORKSPACE/fastrtps_no_shm.xml" ]; then
    FASTRTPS_EXPORT="export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE/fastrtps_no_shm.xml"
fi
PREAMBLE="$SOURCE_CMD"
if [ -n "$FASTRTPS_EXPORT" ]; then
    PREAMBLE="$PREAMBLE && $FASTRTPS_EXPORT"
fi

# --- Build bringup command ---
# use_point_lio:=true  → hardware mode (no Gazebo, use_sim_time=false)
# use_autonomy:=false  → no mission controller (we use reactive wander instead)
# use_localization     → Point-LIO (true) or static TF (false)
LAUNCH_CMD="ros2 launch lunabotics_navigation bringup.launch.py"
LAUNCH_CMD="$LAUNCH_CMD use_point_lio:=true"
LAUNCH_CMD="$LAUNCH_CMD use_autonomy:=false"
LAUNCH_CMD="$LAUNCH_CMD use_localization:=$USE_LOCALIZATION"
LAUNCH_CMD="$LAUNCH_CMD debug:=true"

# --- Reactive wander command (perception mode) ---
WANDER_CMD="python3 /home/lunabotics/pfw-lunabotics/src/lunabotics_navigation/lunabotics_navigation/reactive_wander.py --ros-args"
WANDER_CMD="$WANDER_CMD -p use_perception:=true"
WANDER_CMD="$WANDER_CMD -p duration:=${DURATION}.0"
WANDER_CMD="$WANDER_CMD -p forward_speed:=$SPEED"
WANDER_CMD="$WANDER_CMD -p debug_viz:=true"
WANDER_CMD="$WANDER_CMD -p range_max:=3.0"
WANDER_CMD="$WANDER_CMD -p stop_dist:=0.55"
WANDER_CMD="$WANDER_CMD -p slow_dist:=1.0"

# --- Monitor script ---
MONITOR_SCRIPT='
echo "Waiting 10s for nodes..."
sleep 10
while true; do
    clear
    echo "====== PERCEPTION TEST MONITOR ($(date +%H:%M:%S)) ======"
    echo ""

    echo "--- LiDAR ---"
    timeout 8 ros2 topic hz /unilidar/cloud --window 3 2>&1 | head -1 || echo "  NO DATA"
    echo ""

    echo "--- IMU ---"
    timeout 8 ros2 topic hz /unilidar/imu --window 3 2>&1 | head -1 || echo "  NO DATA"
    echo ""

    echo "--- Perception Pipeline ---"
    timeout 8 ros2 topic hz /perception/unified_obstacles --window 3 2>&1 | head -1 || echo "  NO DATA"
    echo ""

    echo "--- Obstacle Count (last msg) ---"
    OBS=$(timeout 8 ros2 topic echo /perception/hazard_summary --once 2>/dev/null | grep -E "rock_count|crater_count|obstacle_count|min_distance|path_blocked" || echo "  (no data)")
    echo "  $OBS"
    echo ""

    echo "--- Control ---"
    timeout 8 ros2 topic hz /cmd_vel_safe --window 3 2>&1 | head -1 || echo "  idle"
    echo ""

    echo "--- Pico Status ---"
    PICO=$(timeout 5 ros2 topic echo /pico/status --once 2>/dev/null | head -1 || echo "  (no status)")
    echo "  $PICO"
    echo ""

    echo "--- System Health ---"
    HEALTH=$(timeout 5 ros2 topic echo /system/health --once 2>/dev/null | head -1 || echo "  (no watchdog)")
    echo "  $HEALTH"
    echo ""

    echo "--- Localization ---"
    QUAL=$(timeout 5 ros2 topic echo /perception/localization_quality --once 2>/dev/null | head -1 || echo "  (no data)")
    echo "  Quality: $QUAL"
    echo ""

    echo "[Refreshes every 15s. Ctrl+c to stop.]"
    sleep 15
done
'

# ------------------------------------------------------------------ #
# Create tmux session
# Layout:
#   ┌──────────────────────────┬──────────────────────────┐
#   │ 0: ROS2 bringup         │ 1: Monitor               │
#   ├──────────────────────────┼──────────────────────────┤
#   │ 2: Reactive Wander      │ 3: Teleop (E-stop)       │
#   └──────────────────────────┴──────────────────────────┘
# ------------------------------------------------------------------ #

# Pane 0: Bringup (top-left)
tmux new-session -d -s "$SESSION" -x 200 -y 50
tmux send-keys -t "$SESSION" "$PREAMBLE && echo '--- BRINGUP (hardware, no autonomy) ---' && $LAUNCH_CMD" Enter

# Pane 1: Monitor (top-right)
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "$PREAMBLE && $MONITOR_SCRIPT" Enter

# Pane 2: Reactive Wander (bottom-left) — delayed start to let bringup initialize
tmux split-window -v -t "$SESSION:0.0"
tmux send-keys -t "$SESSION" "$PREAMBLE && echo '--- REACTIVE WANDER (perception mode) ---' && echo 'Waiting 15s for bringup + LiDAR...' && sleep 15 && $WANDER_CMD" Enter

# Pane 3: Teleop for E-stop (bottom-right)
tmux split-window -v -t "$SESSION:0.1"
tmux send-keys -t "$SESSION" "$PREAMBLE && echo '--- TELEOP (E-stop with e key) ---' && echo 'Waiting 12s for bringup...' && sleep 12 && ros2 run lunabotics_control teleop_keyboard" Enter

# Disable localization E-stop (scenario 1 is perception-only, no Nav2 goals)
# Must set params BEFORE grace period ends (15s) so safety monitor never triggers.
# IMPORTANT: set +eu is required — setup.bash scripts reference unset variables
# and return non-zero internally (same reason the main script does set +eu above).
(
    set +eu
    sleep 12
    source /opt/ros/humble/setup.bash 2>/dev/null
    source "$WORKSPACE/install/setup.bash" 2>/dev/null
    # Disable all safety checks — wander has its own E-stop via teleop
    ros2 param set /safety_monitor require_localization false 2>/dev/null || true
    ros2 param set /safety_monitor require_sensors false 2>/dev/null || true
    echo "Safety overrides applied for scenario 1"
) &

# Auto-pin CPU cores
(
    sleep 15
    if [ -f "$WORKSPACE/scripts/pin_cores.sh" ]; then
        bash "$WORKSPACE/scripts/pin_cores.sh" 2>&1 | tee /tmp/pin_cores.log
    fi
) &

# Focus wander pane
tmux select-pane -t "$SESSION:0.2"

echo ""
echo -e "${GREEN}${BOLD}Stack launched in tmux session '$SESSION'${NC}"
echo ""
echo -e "  ${CYAN}Pane 0${NC} (top-left):     Bringup (LiDAR + perception + control + localization)"
echo -e "  ${CYAN}Pane 1${NC} (top-right):    Monitor (topic rates, obstacle counts, health)"
echo -e "  ${CYAN}Pane 2${NC} (bottom-left):  Reactive Wander (perception mode, ASCII radar)"
echo -e "  ${CYAN}Pane 3${NC} (bottom-right): Teleop (press ${RED}e${NC} for E-STOP)"
echo ""
echo -e "  Detection thresholds: rocks >= 20cm, craters >= 20cm"
echo -e "  Wander duration: ${DURATION}s at ${SPEED} m/s"
echo ""
echo -e "  ${BOLD}Ctrl+b <arrow>${NC} = switch panes | ${BOLD}Ctrl+b z${NC} = zoom pane"
echo ""

tmux attach -t "$SESSION"
