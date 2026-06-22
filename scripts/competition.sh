#!/usr/bin/env bash
# ============================================================================
# competition.sh — Lunabotics 2026 Competition Launch
# ============================================================================
# Single script to launch the full autonomy stack in a tmux session.
# All panes in one terminal window — no need for multiple SSH sessions.
#
# Layout:
#   ┌──────────────────────────┬──────────────────────────┐
#   │ 0: ROS2 bringup launch  │ 1: Topic monitor         │
#   ├──────────────────────────┼──────────────────────────┤
#   │ 2: Teleop (manual ctrl) │ 3: Mission control       │
#   └──────────────────────────┴──────────────────────────┘
#
# Usage:
#   ./scripts/competition.sh                    # full autonomy
#   ./scripts/competition.sh --teleop-only      # skip autonomy, just teleop
#   ./scripts/competition.sh --no-localization   # skip Point-LIO (dead reckoning)
#
# Tmux shortcuts:
#   Ctrl+b <arrow>   — switch panes
#   Ctrl+b z         — zoom/unzoom current pane
#   Ctrl+b d         — detach (everything keeps running)
#   tmux attach -t luna  — reattach
#
# To kill everything:
#   tmux kill-session -t luna
#   (or Ctrl+c in pane 0, then exit each pane)
# ============================================================================

set -euo pipefail

SESSION="luna"
WORKSPACE="/home/lunabotics/pfw-lunabotics"

# --- Parse arguments ---
USE_AUTONOMY="true"
USE_LOCALIZATION="true"
ARENA_LAYOUT="A"
EXTRA_ARGS=""

for arg in "$@"; do
    case "$arg" in
        --teleop-only)
            USE_AUTONOMY="false"
            ;;
        --no-localization)
            USE_LOCALIZATION="false"
            ;;
        A|a)
            ARENA_LAYOUT="A"
            ;;
        B|b)
            ARENA_LAYOUT="B"
            ;;
        *)
            EXTRA_ARGS="$EXTRA_ARGS $arg"
            ;;
    esac
done

# --- Check tmux ---
if ! command -v tmux &> /dev/null; then
    echo "tmux not installed. Installing..."
    sudo apt-get update -qq && sudo apt-get install -y -qq tmux
fi

# --- Kill existing session if any ---
tmux kill-session -t "$SESSION" 2>/dev/null || true

# --- Source commands (used in every pane) ---
SOURCE_CMD="source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash"

# Export FastRTPS no-SHM config if available
FASTRTPS_EXPORT=""
if [ -f "$WORKSPACE/fastrtps_no_shm.xml" ]; then
    FASTRTPS_EXPORT="export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE/fastrtps_no_shm.xml"
fi

PREAMBLE="$SOURCE_CMD"
if [ -n "$FASTRTPS_EXPORT" ]; then
    PREAMBLE="$PREAMBLE && $FASTRTPS_EXPORT"
fi

# --- Build launch command ---
LAUNCH_CMD="ros2 launch lunabotics_navigation bringup.launch.py"
LAUNCH_CMD="$LAUNCH_CMD use_point_lio:=true"
LAUNCH_CMD="$LAUNCH_CMD use_autonomy:=$USE_AUTONOMY"
LAUNCH_CMD="$LAUNCH_CMD use_localization:=$USE_LOCALIZATION"
LAUNCH_CMD="$LAUNCH_CMD arena_layout:=$ARENA_LAYOUT"
if [ -n "$EXTRA_ARGS" ]; then
    LAUNCH_CMD="$LAUNCH_CMD $EXTRA_ARGS"
fi

# --- Topic monitor script (runs in pane 1) ---
# Loops forever checking key topic rates and last messages
MONITOR_SCRIPT='
echo "Waiting 8s for nodes to start..."
sleep 8
while true; do
    clear
    echo "====== TOPIC MONITOR ($(date +%H:%M:%S)) ======"
    echo ""

    echo "--- LiDAR ---"
    timeout 2 ros2 topic hz /unilidar/cloud --window 5 2>&1 | head -1 || echo "  /unilidar/cloud: NO DATA"
    echo ""

    echo "--- IMU ---"
    timeout 2 ros2 topic hz /unilidar/imu --window 5 2>&1 | head -1 || echo "  /unilidar/imu: NO DATA"
    echo ""

    echo "--- Localization ---"
    timeout 2 ros2 topic hz /odometry/filtered --window 5 2>&1 | head -1 || echo "  /odometry/filtered: NO DATA"
    echo ""

    echo "--- Perception ---"
    timeout 2 ros2 topic hz /perception/unified_obstacles --window 5 2>&1 | head -1 || echo "  /perception/unified_obstacles: NO DATA"
    ZONE=$(timeout 1 ros2 topic echo /perception/current_zone --once 2>/dev/null | head -1 || echo "  no zone data")
    echo "  Zone: $ZONE"
    echo ""

    echo "--- Control ---"
    timeout 2 ros2 topic hz /cmd_vel_safe --window 5 2>&1 | head -1 || echo "  /cmd_vel_safe: idle"
    echo ""

    echo "--- Mission ---"
    STATUS=$(timeout 1 ros2 topic echo /mission/status --once 2>/dev/null | head -2 || echo "  no status")
    echo "  $STATUS"
    STATE=$(timeout 1 ros2 topic echo /mission/state --once 2>/dev/null | head -1 || echo "  no state")
    echo "  State: $STATE"
    echo ""

    echo "--- System Health ---"
    HEALTH=$(timeout 1 ros2 topic echo /system/health --once 2>/dev/null | head -1 || echo "  no watchdog")
    echo "  Health: $HEALTH"
    ESTOP=$(timeout 1 ros2 topic echo /estop --once 2>/dev/null | head -1 || echo "  not published")
    echo "  E-Stop: $ESTOP"
    echo ""

    echo "--- Last Watchdog Events ---"
    timeout 2 ros2 topic echo /system/watchdog_log --once 2>/dev/null | head -3 || echo "  (none)"
    echo ""

    echo "[Refreshes every 10s. Ctrl+c to stop monitor.]"
    sleep 10
done
'

# --- Mission control helper (runs in pane 3) ---
MISSION_SCRIPT='
echo ""
echo "======================================"
echo "  MISSION CONTROL"
echo "======================================"
echo ""
echo "STEP 1: Set starting pose (where judges placed the robot)"
echo "  setpose X Y YAW  — e.g. setpose 3.0 1.0 90"
echo "    X,Y = arena position (center=0,0), YAW = degrees (0=right, 90=up, 180=left, 270=down)"
echo ""
echo "STEP 2 (optional): Confirm arena layout"
echo "  setlayout A  — default pit (berm Y<0)"
echo "  setlayout B  — MIRROR pit (berm Y>0)"
echo ""
echo "STEP 3: Start the mission"
echo "  start  — Begin autonomous mission"
echo ""
echo "Other commands:"
echo "  stop   — Abort mission"
echo "  reset  — Reset to IDLE"
echo "  estop  — Emergency stop"
echo "  clear  — Clear e-stop"
echo "  status — Check mission status"
echo "  zone   — Check current zone"
echo "  nodes  — List active nodes"
echo "  quit   — Exit"
echo ""

while true; do
    read -rp "mission> " cmd args
    case "$cmd" in
        setpose)
            # Parse: setpose X Y YAW_DEG
            read -r sx sy syaw <<< "$args"
            if [ -z "$sx" ] || [ -z "$sy" ] || [ -z "$syaw" ]; then
                echo "Usage: setpose X Y YAW_DEG"
                echo "  Example: setpose 3.0 1.0 90   (placed at x=3, y=1, facing up)"
                echo "  YAW: 0=right(+X), 90=up(+Y), 180=left(-X), 270=down(-Y)"
            else
                echo "Setting start pose: ($sx, $sy, ${syaw}°)..."
                ros2 param set /mission_controller start_x "$sx" 2>/dev/null
                ros2 param set /mission_controller start_y "$sy" 2>/dev/null
                ros2 param set /mission_controller start_yaw_deg "$syaw" 2>/dev/null
                echo "Start pose set. Now run: start"
            fi
            ;;
        setlayout)
            layout=$(echo "$args" | tr "[:lower:]" "[:upper:]" | awk "{print \$1}")
            if [ "$layout" != "A" ] && [ "$layout" != "B" ]; then
                echo "Usage: setlayout A|B"
                echo "  A = default pit (berm at Y<0)"
                echo "  B = mirror pit  (berm at Y>0)"
            else
                echo "Setting arena layout: $layout"
                ros2 param set /mission_controller arena_layout "$layout" 2>/dev/null
                echo "Layout set. Re-applied at /mission/start time too."
            fi
            ;;
        start)
            echo "Starting mission..."
            ros2 service call /mission/start std_srvs/srv/Trigger
            ;;
        stop)
            echo "Stopping mission..."
            ros2 service call /mission/stop std_srvs/srv/Trigger
            ;;
        reset)
            echo "Resetting mission..."
            ros2 service call /mission/reset std_srvs/srv/Trigger
            ;;
        estop)
            echo "E-STOP ACTIVATED"
            ros2 topic pub /estop std_msgs/msg/Bool "{data: true}" --once
            ;;
        clear)
            echo "Clearing e-stop..."
            ros2 topic pub /estop std_msgs/msg/Bool "{data: false}" --once
            ;;
        status)
            echo "Mission status:"
            timeout 2 ros2 topic echo /mission/status --once 2>/dev/null || echo "  (no status)"
            timeout 2 ros2 topic echo /mission/state --once 2>/dev/null || echo "  (no state)"
            ;;
        zone)
            echo "Current zone:"
            timeout 2 ros2 topic echo /perception/current_zone --once 2>/dev/null || echo "  (no zone data)"
            ;;
        nodes)
            ros2 node list 2>/dev/null
            ;;
        topics)
            ros2 topic list 2>/dev/null
            ;;
        quit|exit|q)
            break
            ;;
        "")
            ;;
        *)
            echo "Unknown: $cmd"
            ;;
    esac
done
'

# ------------------------------------------------------------------ #
# Create tmux session with 4 panes
# ------------------------------------------------------------------ #
echo "Launching competition stack..."
echo "  Autonomy: $USE_AUTONOMY"
echo "  Localization: $USE_LOCALIZATION"
echo ""

# Pane 0: ROS2 bringup (top-left)
tmux new-session -d -s "$SESSION" -x 200 -y 50
tmux send-keys -t "$SESSION" "$PREAMBLE && echo '--- PANE 0: ROS2 BRINGUP ---' && $LAUNCH_CMD" Enter

# Pane 1: Topic monitor (top-right)
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "$PREAMBLE && $MONITOR_SCRIPT" Enter

# Pane 2: Teleop (bottom-left)
tmux split-window -v -t "$SESSION:0.0"
tmux send-keys -t "$SESSION" "$PREAMBLE && echo '--- PANE 2: TELEOP ---' && echo 'Waiting 10s for bringup...' && sleep 10 && ros2 run lunabotics_control teleop_keyboard" Enter

# Pane 3: Mission control (bottom-right)
tmux split-window -v -t "$SESSION:0.1"
tmux send-keys -t "$SESSION" "$PREAMBLE && $MISSION_SCRIPT" Enter

# ------------------------------------------------------------------ #
# Auto-pin CPU cores after nodes are up
# ------------------------------------------------------------------ #
(
    sleep 12  # wait for nodes to start
    echo "Auto-pinning CPU cores..."
    bash "$WORKSPACE/scripts/pin_cores.sh" 2>&1 | tee /tmp/pin_cores.log
) &

# Focus on mission control pane
tmux select-pane -t "$SESSION:0.3"

echo "tmux session '$SESSION' created with 4 panes."
echo ""
echo "Attach with:  tmux attach -t $SESSION"
echo "Kill with:    tmux kill-session -t $SESSION"
echo ""

# Attach to the session
tmux attach -t "$SESSION"
