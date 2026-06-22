#!/usr/bin/env bash
# ============================================================================
# debug.sh — Interactive Debug Menu for Lunabotics 2026
# ============================================================================
# Companion to docs/DEBUGGING.md. Runs the diagnostic commands for each
# common failure mode without requiring the operator to remember them.
#
# Usage:
#   ./scripts/debug.sh
#
# Run this in a SEPARATE terminal/SSH session from the main scenario script —
# it doesn't interfere with a running mission.
# ============================================================================

set -u

WORKSPACE="/home/lunabotics/pfw-lunabotics"
LIDAR_IP="192.168.1.62"
JETSON_IP="192.168.1.2"
NET_IFACE="enP8p1s0"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ----- Source ROS2 (suppress complaints if already sourced) -----
if [ -z "${ROS_DISTRO:-}" ]; then
    source /opt/ros/humble/setup.bash 2>/dev/null || true
fi
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash" 2>/dev/null || true
fi
if [ -f "$WORKSPACE/fastrtps_no_shm.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE/fastrtps_no_shm.xml"
fi

pause() {
    echo ""
    read -rp "Press Enter to return to menu... "
}

header() {
    echo ""
    echo -e "${BOLD}${CYAN}===== $1 =====${NC}"
    echo ""
}

ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $*"; }
fail() { echo -e "  ${RED}[FAIL]${NC} $*"; }

# ----------------------------------------------------------------------------
# Top-level: run-everything quick health
# ----------------------------------------------------------------------------
run_all() {
    header "FULL SYSTEM SNAPSHOT"

    # Network
    if ip addr show "$NET_IFACE" 2>/dev/null | grep -q "$JETSON_IP"; then
        ok "Jetson IP $JETSON_IP on $NET_IFACE"
    else
        fail "Jetson IP $JETSON_IP NOT on $NET_IFACE"
    fi
    if ping -c 1 -W 1 "$LIDAR_IP" >/dev/null 2>&1; then
        ok "LiDAR reachable at $LIDAR_IP"
    else
        fail "LiDAR UNREACHABLE at $LIDAR_IP"
    fi

    # Pico USB
    if ls /dev/serial/by-id/usb-MicroPython_Board* >/dev/null 2>&1; then
        ok "Pico USB symlink present: $(ls /dev/serial/by-id/usb-MicroPython_Board* 2>/dev/null | head -1)"
    elif ls /dev/ttyACM* >/dev/null 2>&1; then
        warn "Pico symlink missing but $(ls /dev/ttyACM* | head -1) exists"
    else
        fail "No /dev/ttyACM* — Pico not detected"
    fi

    # Servo USB
    if [ -e /dev/ttyACM1 ]; then
        ok "Servo adapter /dev/ttyACM1 present"
    else
        warn "No /dev/ttyACM1 — deposition servo will not work"
    fi

    # ROS2 daemon
    if ros2 node list >/dev/null 2>&1; then
        local nodes=$(ros2 node list 2>/dev/null | wc -l)
        ok "ROS2 daemon up — $nodes nodes visible"
    else
        fail "ROS2 daemon not reachable"
    fi

    echo ""
    echo "Topic rates (3s window each):"
    for topic in /unilidar/cloud /unilidar/imu /point_lio/odom /odometry/filtered \
                 /perception/unified_obstacles /cmd_vel /cmd_vel_safe; do
        local out=$(timeout 3 ros2 topic hz "$topic" --window 5 2>&1 | head -1)
        if echo "$out" | grep -q "average rate"; then
            ok "$topic — $(echo "$out" | sed 's/^[[:space:]]*//')"
        else
            fail "$topic — NO DATA"
        fi
    done

    echo ""
    echo "E-stop latched values:"
    timeout 1 ros2 topic echo /estop --once 2>/dev/null | grep -E "^data:" || echo "  /estop: not yet published"
    timeout 1 ros2 topic echo /safety/estop --once 2>/dev/null | grep -E "^data:" || echo "  /safety/estop: not yet published"

    echo ""
    echo "Relay status:"
    timeout 1 ros2 topic echo /control/status --once 2>/dev/null | grep -E "^data:" || echo "  no relay status"

    echo ""
    echo "Pico status:"
    timeout 1 ros2 topic echo /pico/status --once 2>/dev/null | grep -E "^data:" || echo "  no pico status"

    echo ""
    echo "Mission state:"
    timeout 1 ros2 topic echo /mission/state --once 2>/dev/null | grep -E "^data:" || echo "  no mission state"
    timeout 1 ros2 topic echo /perception/current_zone --once 2>/dev/null | grep -E "^data:" || echo "  no zone"

    pause
}

# ----------------------------------------------------------------------------
# Section 1: Network
# ----------------------------------------------------------------------------
debug_network() {
    while true; do
        header "NETWORK DEBUG"
        echo "  1) Show interface state ($NET_IFACE)"
        echo "  2) Ping LiDAR ($LIDAR_IP)"
        echo "  3) Fix: re-add Jetson IP to interface"
        echo "  4) Bring interface up"
        echo "  5) Show all interfaces"
        echo "  6) Check WiFi connection"
        echo "  b) Back"
        echo ""
        read -rp "  > " choice
        case "$choice" in
            1) echo ""; ip addr show "$NET_IFACE" 2>/dev/null || fail "Interface not found"; pause ;;
            2) echo ""; ping -c 3 "$LIDAR_IP" || true; pause ;;
            3) sudo ip link set "$NET_IFACE" up
               sudo ip addr add "${JETSON_IP}/24" dev "$NET_IFACE" 2>/dev/null
               ip addr show "$NET_IFACE" | grep "$JETSON_IP" && ok "Added" || fail "Failed"
               pause ;;
            4) sudo ip link set "$NET_IFACE" up; ok "Interface up"; pause ;;
            5) echo ""; ls /sys/class/net/; echo ""; ip addr; pause ;;
            6) echo ""; nmcli connection show --active 2>/dev/null || true
               echo ""; hostname -I; pause ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

# ----------------------------------------------------------------------------
# Section 2: Pico
# ----------------------------------------------------------------------------
debug_pico() {
    while true; do
        header "PICO DEBUG"
        echo "  1) List Pico USB devices"
        echo "  2) Watch /pico/status (Ctrl+C to stop)"
        echo "  3) Watch /pico/encoders (Ctrl+C to stop)"
        echo "  4) Test serial permissions"
        echo "  5) Fix: grant rw on /dev/ttyACM*"
        echo "  6) Show pico_bridge params"
        echo "  7) Run pico_bridge standalone (kill autonomy first!)"
        echo "  8) Test motors directly (0.1 m/s forward 2s — REMOVE WHEELS FIRST)"
        echo "  b) Back"
        echo ""
        read -rp "  > " choice
        case "$choice" in
            1) echo ""; ls -l /dev/ttyACM* 2>/dev/null
               echo ""; ls -l /dev/serial/by-id/ 2>/dev/null; pause ;;
            2) ros2 topic echo /pico/status || true ;;
            3) ros2 topic echo /pico/encoders || true ;;
            4) groups; pause ;;
            5) sudo chmod 666 /dev/ttyACM* 2>/dev/null && ok "Permissions granted"; pause ;;
            6) ros2 param dump /pico_bridge 2>/dev/null || fail "pico_bridge not running"; pause ;;
            7) echo -e "${YELLOW}This will fight any running pico_bridge. Continue? (y/N)${NC}"
               read -r yn
               if [ "$yn" = "y" ]; then
                   ros2 run lunabotics_control pico_bridge \
                       --ros-args --params-file "$WORKSPACE/src/lunabotics_control/config/pico_params.yaml"
               fi ;;
            8) echo -e "${RED}WARNING: ROBOT WILL ATTEMPT TO MOVE FORWARD!${NC}"
               read -rp "  WHEELS OFF GROUND / ROBOT SECURED? (yes/no): " yn
               if [ "$yn" = "yes" ]; then
                   ros2 topic pub /estop std_msgs/msg/Bool "{data: false}" --once >/dev/null
                   ros2 topic pub /safety/estop std_msgs/msg/Bool "{data: false}" --once >/dev/null
                   sleep 0.3
                   ( timeout 2 ros2 topic pub --rate 10 /cmd_vel_safe geometry_msgs/msg/Twist \
                       "{linear: {x: 0.1}, angular: {z: 0.0}}" >/dev/null 2>&1 ) &
                   sleep 0.7
                   echo "Pico status mid-test:"
                   ros2 topic echo /pico/status --once 2>/dev/null | grep data
                   wait
                   ros2 topic pub /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0}}" --once >/dev/null
                   ok "Test complete. Wheels should have spun. If not, check the BLD-510B drivers + motor power."
               fi
               pause ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

# ----------------------------------------------------------------------------
# Section 3: LiDAR / IMU
# ----------------------------------------------------------------------------
debug_lidar() {
    while true; do
        header "LIDAR / IMU DEBUG"
        echo "  1) Topic rates"
        echo "  2) Watch /unilidar/cloud (a single sample)"
        echo "  3) Watch /unilidar/imu (live)"
        echo "  4) Check LiDAR ROS driver node"
        echo "  5) UDP port check (6101/6201)"
        echo "  6) Restart LiDAR driver standalone (kill bringup first!)"
        echo "  b) Back"
        echo ""
        read -rp "  > " choice
        case "$choice" in
            1) echo ""
               echo "  /unilidar/cloud:"; timeout 3 ros2 topic hz /unilidar/cloud 2>&1 | head -1
               echo "  /unilidar/imu:";   timeout 3 ros2 topic hz /unilidar/imu 2>&1 | head -1
               pause ;;
            2) timeout 3 ros2 topic echo /unilidar/cloud --once 2>/dev/null | head -20 || fail "no cloud"; pause ;;
            3) ros2 topic echo /unilidar/imu || true ;;
            4) ros2 node list 2>/dev/null | grep -i lidar || warn "no lidar node found"; pause ;;
            5) sudo ss -tunlp 2>/dev/null | grep -E "6101|6201" || echo "  (ports not in use)"; pause ;;
            6) echo -e "${YELLOW}This will start a new driver. Continue? (y/N)${NC}"
               read -r yn
               if [ "$yn" = "y" ]; then
                   ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args \
                       -p initialize_type:=2 \
                       -p lidar_ip:="$LIDAR_IP" -p local_ip:="$JETSON_IP" \
                       -p lidar_port:=6101 -p local_port:=6201 \
                       -p cloud_topic:=unilidar/cloud -p imu_topic:=unilidar/imu \
                       -p cloud_frame:=unilidar_lidar -p imu_frame:=unilidar_imu
               fi ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

# ----------------------------------------------------------------------------
# Section 4: Localization
# ----------------------------------------------------------------------------
debug_loc() {
    while true; do
        header "LOCALIZATION DEBUG"
        echo "  1) Quality + confidence snapshot"
        echo "  2) Point-LIO rate"
        echo "  3) EKF rate"
        echo "  4) TF: map -> base_footprint"
        echo "  5) TF: base_link -> unilidar_lidar"
        echo "  6) Generate full TF tree (frames.pdf)"
        echo "  7) Lower confidence threshold (helps stuck WAIT_LOCALIZATION)"
        echo "  8) Extend localization wait timeout"
        echo "  b) Back"
        echo ""
        read -rp "  > " choice
        case "$choice" in
            1) timeout 1 ros2 topic echo /perception/localization_quality --once 2>/dev/null
               timeout 1 ros2 topic echo /perception/localization_confidence --once 2>/dev/null
               pause ;;
            2) timeout 3 ros2 topic hz /point_lio/odom 2>&1 | head -1; pause ;;
            3) timeout 3 ros2 topic hz /odometry/filtered 2>&1 | head -1; pause ;;
            4) timeout 3 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -15; pause ;;
            5) timeout 3 ros2 run tf2_ros tf2_echo base_link unilidar_lidar 2>&1 | head -15; pause ;;
            6) ros2 run tf2_tools view_frames -o /tmp/frames; ok "/tmp/frames.pdf generated"; pause ;;
            7) read -rp "  New threshold (default 0.3): " v
               ros2 param set /mission_controller min_localization_confidence "$v"
               pause ;;
            8) read -rp "  New timeout in seconds (default 30): " v
               ros2 param set /mission_controller localization_wait_timeout "$v"
               pause ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

# ----------------------------------------------------------------------------
# Section 5: Nav2
# ----------------------------------------------------------------------------
debug_nav() {
    while true; do
        header "NAV2 DEBUG"
        echo "  1) /cmd_vel rate (must be 10 Hz during a goal)"
        echo "  2) Nav2 lifecycle states"
        echo "  3) Force Nav2 lifecycle startup"
        echo "  4) Clear costmaps"
        echo "  5) List Nav2 services"
        echo "  6) Show use_sim_time on controller_server + costmaps"
        echo "  7) Slow Nav2 down (max_vel_x = 0.25)"
        echo "  b) Back"
        echo ""
        read -rp "  > " choice
        case "$choice" in
            1) timeout 3 ros2 topic hz /cmd_vel 2>&1 | head -1; pause ;;
            2) for n in controller_server planner_server bt_navigator behavior_server map_server; do
                   state=$(ros2 lifecycle get /$n 2>/dev/null | head -1)
                   echo "  $n: ${state:-(not running)}"
               done
               pause ;;
            3) ros2 service call /lifecycle_manager_navigation/startup std_srvs/srv/Trigger; pause ;;
            4) ros2 service call /local_costmap/local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap 2>/dev/null
               ros2 service call /global_costmap/global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap 2>/dev/null
               ok "Costmaps cleared"; pause ;;
            5) ros2 service list 2>/dev/null | grep -E "nav|costmap|controller|planner"; pause ;;
            6) for n in /controller_server /local_costmap/local_costmap /global_costmap/global_costmap; do
                   v=$(ros2 param get "$n" use_sim_time 2>/dev/null | tail -1)
                   echo "  $n use_sim_time: $v"
               done
               echo ""
               echo "  (On hardware, all three MUST be False.)"
               pause ;;
            7) ros2 param set /controller_server FollowPath.max_vel_x 0.25
               ros2 param set /controller_server FollowPath.max_speed_xy 0.25
               ok "Nav2 max linear speed set to 0.25 m/s"; pause ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

# ----------------------------------------------------------------------------
# Section 6: Control / motion
# ----------------------------------------------------------------------------
debug_control() {
    while true; do
        header "CONTROL / MOTION DEBUG"
        echo "  1) Relay status"
        echo "  2) /cmd_vel_safe rate"
        echo "  3) E-stop values on both topics"
        echo "  4) CLEAR e-stop (both topics)"
        echo "  5) Manual move: 0.1 m/s forward 2s (CHECK WHEELS OFF GROUND!)"
        echo "  6) Manual turn: 0.4 rad/s left 2s"
        echo "  7) Toggle invert_linear"
        echo "  8) Toggle invert_angular"
        echo "  9) Show pico_bridge invert params"
        echo "  b) Back"
        echo ""
        read -rp "  > " choice
        case "$choice" in
            1) timeout 1 ros2 topic echo /control/status --once 2>/dev/null; pause ;;
            2) timeout 3 ros2 topic hz /cmd_vel_safe 2>&1 | head -1; pause ;;
            3) timeout 1 ros2 topic echo /estop --once 2>/dev/null
               timeout 1 ros2 topic echo /safety/estop --once 2>/dev/null
               pause ;;
            4) ros2 topic pub /estop std_msgs/msg/Bool "{data: false}" --once
               ros2 topic pub /safety/estop std_msgs/msg/Bool "{data: false}" --once
               ok "Both e-stop topics cleared"; pause ;;
            5) echo -e "${RED}WARNING: ROBOT WILL ATTEMPT TO MOVE FORWARD${NC}"
               read -rp "  WHEELS OFF GROUND? (yes/no): " yn
               if [ "$yn" = "yes" ]; then
                   ros2 topic pub /estop std_msgs/msg/Bool "{data: false}" --once >/dev/null
                   ( timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
                       "{linear: {x: 0.1}, angular: {z: 0.0}}" >/dev/null 2>&1 ) &
                   sleep 2.2
                   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}" --once >/dev/null
                   ok "Done"
               fi
               pause ;;
            6) echo -e "${RED}WARNING: ROBOT WILL ATTEMPT TO TURN LEFT${NC}"
               read -rp "  WHEELS OFF GROUND? (yes/no): " yn
               if [ "$yn" = "yes" ]; then
                   ros2 topic pub /estop std_msgs/msg/Bool "{data: false}" --once >/dev/null
                   ( timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
                       "{linear: {x: 0.0}, angular: {z: 0.4}}" >/dev/null 2>&1 ) &
                   sleep 2.2
                   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.0}}" --once >/dev/null
                   ok "Done"
               fi
               pause ;;
            7) cur=$(ros2 param get /pico_bridge invert_linear 2>/dev/null | tail -1)
               if [[ "$cur" == *"True"* ]]; then
                   ros2 param set /pico_bridge invert_linear false; ok "invert_linear = false"
               else
                   ros2 param set /pico_bridge invert_linear true; ok "invert_linear = true"
               fi; pause ;;
            8) cur=$(ros2 param get /pico_bridge invert_angular 2>/dev/null | tail -1)
               if [[ "$cur" == *"True"* ]]; then
                   ros2 param set /pico_bridge invert_angular false; ok "invert_angular = false"
               else
                   ros2 param set /pico_bridge invert_angular true; ok "invert_angular = true"
               fi; pause ;;
            9) ros2 param get /pico_bridge invert_linear  2>/dev/null
               ros2 param get /pico_bridge invert_angular 2>/dev/null
               pause ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

# ----------------------------------------------------------------------------
# Section 7: Perception
# ----------------------------------------------------------------------------
debug_perception() {
    while true; do
        header "PERCEPTION DEBUG"
        echo "  1) /perception/unified_obstacles rate"
        echo "  2) Current zone"
        echo "  3) Localization confidence + quality"
        echo "  4) Show perception params (RANSAC threshold, height filters)"
        echo "  b) Back"
        echo ""
        read -rp "  > " choice
        case "$choice" in
            1) timeout 3 ros2 topic hz /perception/unified_obstacles 2>&1 | head -1; pause ;;
            2) timeout 1 ros2 topic echo /perception/current_zone --once 2>/dev/null; pause ;;
            3) timeout 1 ros2 topic echo /perception/localization_confidence --once 2>/dev/null
               timeout 1 ros2 topic echo /perception/localization_quality --once 2>/dev/null
               pause ;;
            4) ros2 param dump /unified_obstacle_detector 2>/dev/null | head -40; pause ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

# ----------------------------------------------------------------------------
# Section 8: Excavation / actuator / servo
# ----------------------------------------------------------------------------
debug_excavation() {
    while true; do
        header "EXCAVATION / ACTUATOR / SERVO DEBUG"
        echo "  1) Excavation status"
        echo "  2) Trigger dig service"
        echo "  3) Trigger stow service"
        echo "  4) Trigger dump service"
        echo "  5) Manual belt motor PWM (BELT OFF GROUND!)"
        echo "  6) Manual actuator: raise (continuous +100 for 10s)"
        echo "  7) Manual actuator: lower (continuous -100 for 10s)"
        echo "  8) Manual servo: dump position"
        echo "  9) Manual servo: stow position"
        echo " 10) Weight sensor reading"
        echo "  b) Back"
        echo ""
        read -rp "  > " choice
        case "$choice" in
            1) timeout 1 ros2 topic echo /excavation/status --once 2>/dev/null; pause ;;
            2) ros2 service call /excavation/dig std_srvs/srv/Trigger; pause ;;
            3) ros2 service call /excavation/stow std_srvs/srv/Trigger; pause ;;
            4) ros2 service call /excavation/dump std_srvs/srv/Trigger; pause ;;
            5) read -rp "  PWM (5000-65535, 6000-10000 is normal): " p
               echo "  Sending for 5s, then 0..."
               ( timeout 5 bash -c "while true; do ros2 topic pub /excavation/motor std_msgs/msg/Int32 \"{data: -$p}\" --once; sleep 0.1; done" >/dev/null 2>&1 ) &
               sleep 5.5
               ros2 topic pub /excavation/motor std_msgs/msg/Int32 "{data: 0}" --once
               ok "Belt stopped"; pause ;;
            6) ( timeout 10 ros2 topic pub --rate 10 /actuator/command std_msgs/msg/Int32 "{data: 100}" >/dev/null 2>&1 ) &
               echo "  Raising actuator for 10s..."
               wait
               ros2 topic pub /actuator/command std_msgs/msg/Int32 "{data: 0}" --once
               pause ;;
            7) ( timeout 10 ros2 topic pub --rate 10 /actuator/command std_msgs/msg/Int32 "{data: -100}" >/dev/null 2>&1 ) &
               echo "  Lowering actuator for 10s..."
               wait
               ros2 topic pub /actuator/command std_msgs/msg/Int32 "{data: 0}" --once
               pause ;;
            8) ros2 topic pub /deposition/tilt std_msgs/msg/String "{data: '107.0,4500'}" --once
               ok "Dump command sent"; pause ;;
            9) ros2 topic pub /deposition/tilt std_msgs/msg/String "{data: '0.0,4500'}" --once
               ok "Stow command sent"; pause ;;
            10) timeout 2 ros2 topic echo /deposition/weight --once 2>/dev/null || echo "  no weight data"; pause ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

# ----------------------------------------------------------------------------
# Section 9: Mission
# ----------------------------------------------------------------------------
debug_mission() {
    while true; do
        header "MISSION DEBUG"
        echo "  1) Show state + status"
        echo "  2) Show all mission params"
        echo "  3) Reset to IDLE"
        echo "  4) Stop running mission"
        echo "  5) Force-clear e-stop"
        echo "  6) Start mission"
        echo "  7) Set start pose"
        echo "  8) Set arena layout (A/B)"
        echo "  b) Back"
        echo ""
        read -rp "  > " choice
        case "$choice" in
            1) timeout 1 ros2 topic echo /mission/state --once 2>/dev/null
               timeout 1 ros2 topic echo /mission/status --once 2>/dev/null
               pause ;;
            2) ros2 param dump /mission_controller 2>/dev/null | head -60; pause ;;
            3) ros2 service call /mission/reset std_srvs/srv/Trigger; pause ;;
            4) ros2 service call /mission/stop std_srvs/srv/Trigger; pause ;;
            5) ros2 topic pub /estop std_msgs/msg/Bool "{data: false}" --once
               ros2 topic pub /safety/estop std_msgs/msg/Bool "{data: false}" --once
               ok "E-stop cleared"; pause ;;
            6) ros2 service call /mission/start std_srvs/srv/Trigger; pause ;;
            7) read -rp "  X: " x; read -rp "  Y: " y; read -rp "  YAW (deg): " yaw
               ros2 param set /mission_controller start_x "$x"
               ros2 param set /mission_controller start_y "$y"
               ros2 param set /mission_controller start_yaw_deg "$yaw"
               ok "Set"; pause ;;
            8) read -rp "  Layout (A or B): " l
               ros2 param set /mission_controller arena_layout "$l"
               ok "Set"; pause ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

# ----------------------------------------------------------------------------
# Section 10: ROS2 / workspace
# ----------------------------------------------------------------------------
debug_ros() {
    while true; do
        header "ROS2 / WORKSPACE DEBUG"
        echo "  1) ros2 doctor"
        echo "  2) ros2 node list"
        echo "  3) ros2 topic list"
        echo "  4) ros2 service list"
        echo "  5) ros2 daemon restart"
        echo "  6) Show ROS_DOMAIN_ID + RMW"
        echo "  7) Rebuild workspace (full)"
        echo "  8) Show latest log directory"
        echo "  9) Tail mission_controller log"
        echo "  b) Back"
        echo ""
        read -rp "  > " choice
        case "$choice" in
            1) ros2 doctor 2>&1 | head -30; pause ;;
            2) ros2 node list 2>/dev/null; pause ;;
            3) ros2 topic list 2>/dev/null; pause ;;
            4) ros2 service list 2>/dev/null; pause ;;
            5) ros2 daemon stop; sleep 1; ros2 daemon start; ok "Restarted"; pause ;;
            6) echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
               echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
               echo "  ROS_DISTRO=${ROS_DISTRO:-unknown}"
               echo "  FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-unset}"
               pause ;;
            7) (cd "$WORKSPACE" && colcon build --symlink-install 2>&1 | tail -10); pause ;;
            8) ls -la ~/.ros/log/latest/ 2>/dev/null | head -20; pause ;;
            9) tail -F ~/.ros/log/latest/mission_controller-*.log 2>/dev/null || warn "no mission_controller log" ;;
            b|B) return ;;
            *) ;;
        esac
    done
}

# ----------------------------------------------------------------------------
# Main menu
# ----------------------------------------------------------------------------
main_menu() {
    while true; do
        clear
        echo -e "${BOLD}============================================${NC}"
        echo -e "${BOLD}  LUNABOTICS DEBUG MENU${NC}"
        echo -e "${BOLD}============================================${NC}"
        echo "  See docs/DEBUGGING.md for full doc."
        echo ""
        echo -e "${BOLD}  Quick:${NC}"
        echo "    0) Full system snapshot (run this first)"
        echo ""
        echo -e "${BOLD}  By subsystem:${NC}"
        echo "    1) Network (LiDAR IP, ethernet, WiFi)"
        echo "    2) Pico (USB, /pico/status, motors)"
        echo "    3) LiDAR / IMU (/unilidar/*, driver)"
        echo "    4) Localization (Point-LIO, EKF, TF)"
        echo "    5) Nav2 (controller, planner, lifecycle, costmap)"
        echo "    6) Control / motion (relay, /cmd_vel, /cmd_vel_safe, e-stop)"
        echo "    7) Perception (obstacles, zone)"
        echo "    8) Excavation / actuator / servo"
        echo "    9) Mission (state, services, params)"
        echo "   10) ROS2 / workspace"
        echo ""
        echo "    q) Quit"
        echo ""
        read -rp "  > " choice
        case "$choice" in
            0) run_all ;;
            1) debug_network ;;
            2) debug_pico ;;
            3) debug_lidar ;;
            4) debug_loc ;;
            5) debug_nav ;;
            6) debug_control ;;
            7) debug_perception ;;
            8) debug_excavation ;;
            9) debug_mission ;;
            10) debug_ros ;;
            q|Q) echo "Exiting."; exit 0 ;;
            *) ;;
        esac
    done
}

main_menu
