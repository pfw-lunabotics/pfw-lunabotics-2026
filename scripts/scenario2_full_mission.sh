#!/usr/bin/env bash
# ============================================================================
# scenario2_full_mission.sh — Full Autonomous Mission Sand Test
# ============================================================================
# ONE COMMAND to launch full autonomous cycle:
#   Navigate → Lower actuator → Excavate → Raise actuator →
#   Navigate to berm → Align rear → Deposit → Repeat
#
# Sets up: LiDAR network, Pico serial, ROS2 stack, then launches in tmux
# with mission control pane for setpose + start.
#
# Usage:
#   ./scripts/scenario2_full_mission.sh                              # default: arena A, facing W
#   ./scripts/scenario2_full_mission.sh A                            # explicit layout A
#   ./scripts/scenario2_full_mission.sh B                            # MIRROR pit (berm Y flipped)
#   ./scripts/scenario2_full_mission.sh A --facing W                 # facing W (toward berm)
#   ./scripts/scenario2_full_mission.sh B --facing N --pos 3.0 1.0   # B, facing N, custom start pos
#   ./scripts/scenario2_full_mission.sh A --facing 135               # numeric yaw (degrees CCW from +X)
#   ./scripts/scenario2_full_mission.sh A --no-localization          # skip Point-LIO
#   ./scripts/scenario2_full_mission.sh B --auto-start               # auto-start 25s after launch
#
# Flags:
#   --facing N|S|E|W|<deg>    Cardinal direction the robot FRONT points.
#                             N=90, S=270, E=0, W=180 (default W=180, toward berm).
#                             Applied via `ros2 param set start_yaw_deg` ~15s
#                             after launch, before the bringup is ready for `start`.
#   --pos X Y                 Start position in arena frame. Default (3.0, 0.0).
#                             Optional — beams self-correct up to 0.5 m.
#
# Arena layout selection:
#   - LiDAR cannot tell layouts apart from the start zone (mirror pits look
#     geometrically identical until the robot is past the obstacle field).
#   - Operator picks A or B by visual inspection of which pit was assigned.
#   - Layout A: berm in -Y half of construction zone (default, matches
#     simulator + the "primary" UCF pit).
#   - Layout B: berm in +Y half (the mirror pit).
#   - Layout can also be changed at runtime via `setlayout A|B` in the
#     mission control pane (BEFORE running `start`).
#
# Tmux shortcuts:
#   Ctrl+b <arrow>   — switch panes
#   Ctrl+b z         — zoom/unzoom current pane
#   Ctrl+b d         — detach (keeps running)
#   tmux attach -t full_mission  — reattach
#
# To kill:
#   tmux kill-session -t full_mission
# ============================================================================

set -euo pipefail

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

SESSION="full_mission"
WORKSPACE="/home/lunabotics/pfw-lunabotics"

LIDAR_IP="192.168.1.62"
JETSON_IP="192.168.1.2"
NET_IFACE="enP8p1s0"

# --- Defaults ---
USE_LOCALIZATION="true"
AUTO_START="false"
ARENA_LAYOUT="A"
# Facing + position — empty means "leave at mission_params.yaml default"
START_YAW_DEG=""
START_X=""
START_Y=""
FACING_LETTER=""   # cosmetic, printed back to operator

# Convert N|S|E|W to degrees (CCW from +X). Empty if not a cardinal letter.
facing_to_yaw() {
    case "$(echo "$1" | tr '[:lower:]' '[:upper:]')" in
        E) echo "0"   ;;
        N) echo "90"  ;;
        W) echo "180" ;;
        S) echo "270" ;;
        *) echo ""    ;;
    esac
}

print_usage() {
    cat <<EOF
Usage: $0 [A|B] [--facing N|S|E|W|<deg>] [--pos X Y] [--no-localization] [--auto-start]

  A | B              arena layout (A=TOP, berm at +Y; B=BOTTOM, berm at -Y)
  --facing N|S|E|W   cardinal direction robot FRONT points (or numeric yaw deg)
  --pos X Y          start position in arena frame (default 3.0 0.0)
  --no-localization  skip Point-LIO (debug only)
  --auto-start       call /mission/start 25 s after launch

Examples:
  $0 A --facing W
  $0 B --facing N --pos 3.0 1.0
  $0 A --facing 135
EOF
}

# --- Parse arguments ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        A|a)
            ARENA_LAYOUT="A"
            shift
            ;;
        B|b)
            ARENA_LAYOUT="B"
            shift
            ;;
        --facing)
            if [[ -z "${2:-}" ]]; then
                echo -e "${RED}--facing needs an argument (N|S|E|W or a number)${NC}"
                exit 1
            fi
            FACING_LETTER="$2"
            yaw="$(facing_to_yaw "$2")"
            if [[ -n "$yaw" ]]; then
                START_YAW_DEG="$yaw"
            elif [[ "$2" =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
                START_YAW_DEG="$2"
            else
                echo -e "${RED}--facing must be N|S|E|W or a numeric yaw in degrees (got '$2')${NC}"
                exit 1
            fi
            shift 2
            ;;
        --pos)
            if [[ -z "${2:-}" ]] || [[ -z "${3:-}" ]]; then
                echo -e "${RED}--pos needs two arguments: X Y${NC}"
                exit 1
            fi
            if ! [[ "$2" =~ ^-?[0-9]+(\.[0-9]+)?$ ]] || ! [[ "$3" =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
                echo -e "${RED}--pos X Y must be numbers (got '$2' '$3')${NC}"
                exit 1
            fi
            START_X="$2"
            START_Y="$3"
            shift 3
            ;;
        --no-localization)
            USE_LOCALIZATION="false"
            shift
            ;;
        --auto-start)
            AUTO_START="true"
            shift
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown argument: $1${NC}"
            print_usage
            exit 1
            ;;
    esac
done

echo ""
echo -e "${BOLD}============================================${NC}"
echo -e "${BOLD}  SCENARIO 2: FULL AUTONOMOUS MISSION TEST${NC}"
echo -e "${BOLD}============================================${NC}"
echo -e "  Arena layout: ${BOLD}${ARENA_LAYOUT}${NC}"
if [ "$ARENA_LAYOUT" = "B" ]; then
    echo -e "    ${YELLOW}MIRROR PIT — berm Y flipped to +1.3${NC}"
else
    echo -e "    Default pit — berm Y at -1.3"
fi
if [ -n "$START_YAW_DEG" ]; then
    if [ -n "$FACING_LETTER" ]; then
        echo -e "  Facing: ${BOLD}${FACING_LETTER}${NC} (yaw=${START_YAW_DEG}°) — applied via param set ~15s after launch"
    else
        echo -e "  Facing: yaw=${BOLD}${START_YAW_DEG}°${NC} — applied via param set ~15s after launch"
    fi
else
    echo -e "  Facing: (default from mission_params.yaml — set later with setfacing in pane 2)"
fi
if [ -n "$START_X" ]; then
    echo -e "  Start pos: ${BOLD}(${START_X}, ${START_Y})${NC} — applied via param set ~15s after launch"
else
    echo -e "  Start pos: (default from mission_params.yaml — beams self-correct up to 0.5 m)"
fi
echo -e "  Localization: ${USE_LOCALIZATION}"
echo -e "  Auto-start: ${AUTO_START}"
echo -e "  Hardware excavation: ENABLED (simulate=false)"
echo ""

# Warn on obvious geometry mistakes that the post-launch ros2 param set can't fix
if [ -n "$FACING_LETTER" ]; then
    fl="$(echo "$FACING_LETTER" | tr '[:lower:]' '[:upper:]')"
    if [ "$ARENA_LAYOUT" = "B" ] && [ "$fl" = "N" ]; then
        echo -e "  ${YELLOW}WARN${NC}: Facing N in arena B = pointing at divider strip ~1.3 m ahead."
        echo -e "         Cycle-1 dig stays in place so it's not fatal, but the first nav-to-berm"
        echo -e "         rotation will sweep the chassis toward the divider — give it clearance."
        echo ""
    fi
    if [ "$ARENA_LAYOUT" = "A" ] && [ "$fl" = "S" ]; then
        echo -e "  ${YELLOW}WARN${NC}: Facing S in arena A = pointing at divider strip ~1.3 m ahead."
        echo ""
    fi
fi

# ------------------------------------------------------------------ #
# STEP 1: Network setup
# ------------------------------------------------------------------ #
echo -e "${BOLD}[1/5] Network Setup${NC}"

sudo ip link set "$NET_IFACE" up 2>/dev/null || true

if ip addr show "$NET_IFACE" 2>/dev/null | grep -q "$JETSON_IP"; then
    echo -e "  ${GREEN}[OK]${NC} Jetson IP $JETSON_IP on $NET_IFACE"
else
    echo -e "  ${YELLOW}[FIX]${NC} Adding $JETSON_IP/24 to $NET_IFACE..."
    sudo ip addr add "${JETSON_IP}/24" dev "$NET_IFACE" 2>/dev/null || true
    sleep 1
    if ip addr show "$NET_IFACE" 2>/dev/null | grep -q "$JETSON_IP"; then
        echo -e "  ${GREEN}[OK]${NC} IP added"
    else
        echo -e "  ${RED}[FAIL]${NC} Could not add IP"
        echo -e "  Available interfaces: $(ls /sys/class/net/)"
        exit 1
    fi
fi

# ------------------------------------------------------------------ #
# STEP 2: LiDAR connectivity
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
    echo -e "  ${GREEN}[OK]${NC} LiDAR at $LIDAR_IP"
else
    echo -e "  ${RED}[FAIL]${NC} LiDAR unreachable at $LIDAR_IP"
    read -rp "  Press Enter to continue anyway... "
fi

# ------------------------------------------------------------------ #
# STEP 3: Pico + Servo serial
# ------------------------------------------------------------------ #
echo ""
echo -e "${BOLD}[3/5] Motor Controllers${NC}"

PICO_SYMLINK="/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6647c156730ab24-if00"

if [ -e "$PICO_SYMLINK" ]; then
    echo -e "  ${GREEN}[OK]${NC} Pico found: $(readlink -f "$PICO_SYMLINK")"
elif [ -e /dev/ttyACM0 ]; then
    echo -e "  ${YELLOW}[WARN]${NC} Pico symlink missing, /dev/ttyACM0 exists"
else
    echo -e "  ${RED}[FAIL]${NC} No Pico USB device"
    read -rp "  Press Enter to continue anyway... "
fi

# Servo (deposition)
if [ -e /dev/ttyACM1 ]; then
    echo -e "  ${GREEN}[OK]${NC} Servo adapter at /dev/ttyACM1"
elif [ -e /dev/ttyUSB0 ]; then
    echo -e "  ${YELLOW}[WARN]${NC} Servo adapter may be at /dev/ttyUSB0 (not /dev/ttyACM1)"
else
    echo -e "  ${YELLOW}[WARN]${NC} No servo adapter found (deposition will be command-only)"
fi

if groups | grep -qE '(dialout|plugdev)'; then
    echo -e "  ${GREEN}[OK]${NC} Serial access OK"
else
    echo -e "  ${YELLOW}[WARN]${NC} Not in dialout group"
fi

# ------------------------------------------------------------------ #
# STEP 4: ROS2 environment
# ------------------------------------------------------------------ #
echo ""
echo -e "${BOLD}[4/5] ROS2 Environment${NC}"

set +eu
source /opt/ros/humble/setup.bash 2>/dev/null || {
    echo -e "  ${RED}[FAIL]${NC} Cannot source ROS2"
    exit 1
}
echo -e "  ${GREEN}[OK]${NC} ROS2 Humble"

if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash" 2>/dev/null
    echo -e "  ${GREEN}[OK]${NC} Workspace"
else
    echo -e "  ${RED}[FAIL]${NC} Workspace not built"
    exit 1
fi
set -eu

if [ -f "$WORKSPACE/fastrtps_no_shm.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE/fastrtps_no_shm.xml"
    echo -e "  ${GREEN}[OK]${NC} FastRTPS no-SHM"
fi

# ------------------------------------------------------------------ #
# STEP 5: Launch in tmux
# ------------------------------------------------------------------ #
echo ""
echo -e "${BOLD}[5/5] Launching Full Mission Stack${NC}"

tmux kill-session -t "$SESSION" 2>/dev/null || true

if ! command -v tmux &> /dev/null; then
    sudo apt-get update -qq && sudo apt-get install -y -qq tmux
fi

# --- Preamble for tmux panes ---
SOURCE_CMD="source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash"
FASTRTPS_EXPORT=""
if [ -f "$WORKSPACE/fastrtps_no_shm.xml" ]; then
    FASTRTPS_EXPORT="export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE/fastrtps_no_shm.xml"
fi
PREAMBLE="$SOURCE_CMD"
if [ -n "$FASTRTPS_EXPORT" ]; then
    PREAMBLE="$PREAMBLE && $FASTRTPS_EXPORT"
fi

# --- Bringup command ---
# use_point_lio:=true  → hardware mode
# use_autonomy:=true   → mission controller + excavation bridge
# simulate:=false      → REAL motor/actuator/servo commands
LAUNCH_CMD="ros2 launch lunabotics_navigation bringup.launch.py"
LAUNCH_CMD="$LAUNCH_CMD use_point_lio:=true"
LAUNCH_CMD="$LAUNCH_CMD use_autonomy:=true"
LAUNCH_CMD="$LAUNCH_CMD simulate:=false"
LAUNCH_CMD="$LAUNCH_CMD use_localization:=$USE_LOCALIZATION"
LAUNCH_CMD="$LAUNCH_CMD arena_layout:=$ARENA_LAYOUT"
LAUNCH_CMD="$LAUNCH_CMD debug:=true"

# --- Topic monitor ---
MONITOR_SCRIPT='
echo "Waiting 10s for nodes..."
sleep 10
while true; do
    clear
    echo "====== FULL MISSION MONITOR ($(date +%H:%M:%S)) ======"
    echo ""

    echo "--- Sensors ---"
    timeout 2 ros2 topic hz /unilidar/cloud --window 5 2>&1 | head -1 || echo "  LiDAR: NO DATA"
    timeout 2 ros2 topic hz /unilidar/imu --window 5 2>&1 | head -1 || echo "  IMU: NO DATA"
    echo ""

    echo "--- Localization ---"
    timeout 2 ros2 topic hz /odometry/filtered --window 5 2>&1 | head -1 || echo "  EKF: NO DATA"
    QUAL=$(timeout 1 ros2 topic echo /perception/localization_quality --once 2>/dev/null | head -1 || echo "  unknown")
    echo "  Quality: $QUAL"
    echo ""

    echo "--- Perception ---"
    timeout 2 ros2 topic hz /perception/unified_obstacles --window 5 2>&1 | head -1 || echo "  NO DATA"
    ZONE=$(timeout 1 ros2 topic echo /perception/current_zone --once 2>/dev/null | head -1 || echo "  unknown")
    echo "  Zone: $ZONE"
    echo ""

    echo "--- Mission ---"
    STATUS=$(timeout 1 ros2 topic echo /mission/status --once 2>/dev/null | grep "data:" | head -1 || echo "  (no status)")
    echo "  $STATUS"
    echo ""

    echo "--- Excavation ---"
    EX_STATE=$(timeout 1 ros2 topic echo /excavation/status --once 2>/dev/null | grep -E "state:|gate_position:" || echo "  (no data)")
    echo "  $EX_STATE"
    echo ""

    echo "--- Control ---"
    timeout 2 ros2 topic hz /cmd_vel_safe --window 5 2>&1 | head -1 || echo "  idle"
    PICO=$(timeout 1 ros2 topic echo /pico/status --once 2>/dev/null | head -1 || echo "  (no status)")
    echo "  Pico: $PICO"
    echo ""

    echo "--- System Health ---"
    HEALTH=$(timeout 1 ros2 topic echo /system/health --once 2>/dev/null | head -1 || echo "  (no watchdog)")
    echo "  $HEALTH"
    echo ""

    echo "[Refreshes every 8s]"
    sleep 8
done
'

# --- Mission control shell ---
MISSION_SCRIPT='
echo ""
echo "======================================"
echo "  MISSION CONTROL — SCENARIO 2"
echo "======================================"
echo ""
echo "Arena layout was launched as: '"$ARENA_LAYOUT"'"
echo "  A = berm in -Y half of construction zone (default)"
echo "  B = MIRROR pit, berm in +Y half"
echo "Use \"setlayout A|B\" if it was wrong (BEFORE start)."
echo ""
echo "RECOMMENDED FLOW (just 2 commands):"
echo "  setarena  A|B           # A=TOP arena, B=BOTTOM arena"
echo "  setfacing N|S|E|W       # cardinal direction the robot FRONT points"
echo "  start                   # localize, dig, navigate, deposit, repeat"
echo ""
echo "POSITION: setpos is OPTIONAL. Default (3.0, 0.0) works because the"
echo "  robot uses the BERM BEAMS as fiducials at the deposit gap mouth."
echo "  If dead-reckoning is off by up to 0.5m (berm_max_beam_correction),"
echo "  mission_controller sends a fix-up Nav2 goal to re-center on beams."
echo "  Only use  setpos X Y  if the measurement is reliable."
echo ""
echo "CONVENTION (post-2026-05-15):"
echo "  +X = east (toward fiducial-rail wall)"
echo "  +Y = north (compass-anchored, same in BOTH arenas)"
echo "  Arena A = TOP physical arena, berm at NW (+Y side)"
echo "  Arena B = BOTTOM physical arena, berm at SW (-Y side)"
echo "  YAW (CCW from +X):  E=0   N=90   W=180   S=270"
echo ""
echo "LOW-LEVEL FALLBACK:"
echo "  setpose X Y YAW_DEG     # raw arena coords"
echo "  setlayout A|B           # raw layout flag"
echo ""
echo "FINAL: start (auto-clears e-stop on both topics first)"
echo ""
echo "The robot will (Scenario 2 step-and-dig flow):"
echo "  1. Wait for localization (up to 30s)"
echo "  2. ALIGN (face toward excavation waypoint → naturally NW/SW/W"
echo "       depending on start_y) + MAP_AHEAD (4s LiDAR mapping)"
echo "  3. STEP-AND-DIG starting right where the robot is:"
echo "       a) Actuator lowers; belt engages at 60% depth (PWM 10500)"
echo "       b) Actuator caps at 85% depth, belt at 21000 PWM"
echo "       c) DWELL 6-10s while weight rises; when ΔW<0.2kg/2s and dwell"
echo "          minimum reached → push DEEPER once (92%) or ADVANCE 0.4m"
echo "       d) Forward obstacle → DODGE (belt off → turn → drive past →"
echo "          turn back → belt on); never turn with belt running"
echo "       e) Exits at arena X<0.5 OR load≥10kg OR time budget"
echo "  4. Raise actuator + stow"
echo "  5. Navigate to berm WAYPOINT (-1.5, ±1.3) — gap mouth, NOT inside square"
echo "  6. Open-loop reverse INTO berm square; LiDAR detects beam pairs:"
echo "       Cycle 1: pass front beams → stop near rear beams (deep deposit)"
echo "       Cycle 2+: stop on first pair (rear at berm edge)"
echo "  7. Tilt deposition servo, wait, stow, repeat"
echo ""
echo "Commands:"
echo "  setarena A|B      — A=TOP arena, B=BOTTOM arena (sets layout flag)"
echo "  setfacing N|S|E|W — cardinal direction robot faces (sets start_yaw)"
echo "  setpos X Y        — set start_x, start_y (defaults to 3.0, 0.0)"
echo "  setpose X Y YAW   — LOW-LEVEL: set starting pose all at once"
echo "  setlayout A|B     — LOW-LEVEL: set arena layout directly"
echo "  show              — print all start-pose params + waypoints"
echo "  diag    — PRE-FLIGHT: verify the motion path is clear"
echo "  testmove— PRE-FLIGHT: drive 0.1 m/s for 3s (bypass Nav2)"
echo "  start   — begin autonomous mission (auto-clears e-stop first)"
echo "  stop    — abort mission"
echo "  reset   — reset to IDLE"
echo "  estop   — emergency stop ALL motors"
echo "  clear   — clear e-stop (both /estop AND /safety/estop)"
echo "  status  — check mission"
echo "  zone    — check current zone"
echo "  dig     — manual: start digging"
echo "  dump    — manual: open deposition"
echo "  stow    — manual: stow everything"
echo "  quit    — exit"
echo ""
echo "RECOMMENDED ORDER FOR FIRST RUN:"
echo "  1. diag       (verify Nav2/relay/pico/sensors all healthy)"
echo "  2. testmove   (verify motion path: cmd_vel → relay → pico → wheels)"
echo "  3. setpose X Y YAW"
echo "  4. setlayout A|B"
echo "  5. start"
echo ""

while true; do
    read -rp "mission> " cmd args
    case "$cmd" in
        setarena)
            layout=$(echo "$args" | tr "[:lower:]" "[:upper:]" | awk "{print \$1}")
            if [ "$layout" != "A" ] && [ "$layout" != "B" ]; then
                echo "Usage: setarena A|B"
                echo "  A = TOP physical arena (berm at NW corner, +Y in our frame)"
                echo "  B = BOTTOM physical arena (berm at SW corner, -Y in our frame)"
            else
                echo "Setting arena: $layout"
                ros2 param set /mission_controller arena_layout "$layout" 2>/dev/null
                if [ "$layout" = "A" ]; then
                    echo "  -> Layout A (TOP). Berm waypoint will be at Y = +1.3."
                    echo "  -> Typical start_y for TOP arena: -1.0 (near divider, southern half of arena)"
                else
                    echo "  -> Layout B (BOTTOM). Berm waypoint will be at Y = -1.3."
                    echo "  -> Typical start_y for BOTTOM arena: +1.0 (near divider, northern half of arena)"
                    echo "  -> WARNING: If setfacing N, robot points at divider (close wall)!"
                fi
            fi
            ;;
        setfacing)
            dir=$(echo "$args" | tr "[:lower:]" "[:upper:]" | awk "{print \$1}")
            case "$dir" in
                E) yaw_deg=0   ;;
                N) yaw_deg=90  ;;
                W) yaw_deg=180 ;;
                S) yaw_deg=270 ;;
                "")
                    echo "Usage: setfacing N|S|E|W"
                    echo "  E (East,  yaw=  0): facing fiducial-rail wall"
                    echo "  N (North, yaw= 90): facing top-of-image direction"
                    echo "  W (West,  yaw=180): facing construction/berm (RECOMMENDED default)"
                    echo "  S (South, yaw=270): facing bottom-of-image direction"
                    yaw_deg=""
                    ;;
                *)
                    # Treat as numeric yaw degrees
                    if [[ "$dir" =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
                        yaw_deg="$dir"
                        echo "Numeric yaw: ${yaw_deg} deg"
                    else
                        echo "Unknown direction: $dir"
                        echo "Use N|S|E|W or a numeric yaw in degrees."
                        yaw_deg=""
                    fi
                    ;;
            esac
            if [ -n "$yaw_deg" ]; then
                echo "Setting start_yaw_deg = $yaw_deg ($dir)"
                ros2 param set /mission_controller start_yaw_deg "$yaw_deg" 2>/dev/null
                cur_layout=$(ros2 param get /mission_controller arena_layout 2>/dev/null | grep -oE "[AB]" | head -1)
                if [ "$cur_layout" = "B" ] && [ "$dir" = "N" ]; then
                    echo "  WARN: Facing N in arena B = pointing at DIVIDER ~1.3m ahead. Forward drive will crash."
                fi
                if [ "$cur_layout" = "A" ] && [ "$dir" = "S" ]; then
                    echo "  WARN: Facing S in arena A = pointing at DIVIDER ~1.3m ahead. Forward drive will crash."
                fi
            fi
            ;;
        setpos)
            read -r sx sy <<< "$args"
            if [ -z "$sx" ] || [ -z "$sy" ]; then
                echo "Usage: setpos X Y"
                echo "  X = arena-frame X (start zone is [+2.05, +4.05]); default 3.0"
                echo "  Y = arena-frame Y (range [-2.285, +2.285]); default 0.0"
                echo "  +Y is north (compass-anchored, same in both arenas)."
                echo "  Typical: setpos 3.0 -1.0   (arena A, near divider/southern half)"
                echo "  Typical: setpos 3.0 +1.0   (arena B, near divider/northern half)"
            else
                echo "Setting start_x=$sx, start_y=$sy"
                ros2 param set /mission_controller start_x "$sx" 2>/dev/null
                ros2 param set /mission_controller start_y "$sy" 2>/dev/null
            fi
            ;;
        setpose)
            read -r sx sy syaw <<< "$args"
            if [ -z "$sx" ] || [ -z "$sy" ] || [ -z "$syaw" ]; then
                echo "Usage: setpose X Y YAW_DEG (low-level, sets all three)"
                echo "  Prefer: setarena + setfacing + setpos"
            else
                echo "Setting start pose: ($sx, $sy, ${syaw}deg)..."
                ros2 param set /mission_controller start_x "$sx" 2>/dev/null
                ros2 param set /mission_controller start_y "$sy" 2>/dev/null
                ros2 param set /mission_controller start_yaw_deg "$syaw" 2>/dev/null
                echo "Done. Now run: start"
            fi
            ;;
        setlayout)
            layout=$(echo "$args" | tr "[:lower:]" "[:upper:]" | awk "{print \$1}")
            if [ "$layout" != "A" ] && [ "$layout" != "B" ]; then
                echo "Usage: setlayout A|B  (low-level; prefer setarena A|B)"
            else
                echo "Setting arena_layout: $layout (low-level)"
                ros2 param set /mission_controller arena_layout "$layout" 2>/dev/null
            fi
            ;;
        show|showpose)
            echo "Current mission_controller starting pose params:"
            for p in arena_layout start_x start_y start_yaw_deg waypoint_berm_x waypoint_berm_y waypoint_berm_yaw waypoint_excavation_x waypoint_excavation_y waypoint_excavation_yaw; do
                val=$(ros2 param get /mission_controller "$p" 2>/dev/null | tail -1)
                printf "  %-26s = %s\n" "$p" "$val"
            done
            ;;
        start)
            echo "Auto-clearing e-stop on both topics first..."
            ros2 topic pub /estop std_msgs/msg/Bool "{data: false}" --once >/dev/null 2>&1
            ros2 topic pub /safety/estop std_msgs/msg/Bool "{data: false}" --once >/dev/null 2>&1
            sleep 0.3
            echo "Starting mission..."
            ros2 service call /mission/start std_srvs/srv/Trigger
            ;;
        stop)
            echo "Stopping mission..."
            ros2 service call /mission/stop std_srvs/srv/Trigger
            ;;
        reset)
            echo "Resetting..."
            ros2 service call /mission/reset std_srvs/srv/Trigger
            ;;
        estop)
            echo "E-STOP"
            ros2 topic pub /estop std_msgs/msg/Bool "{data: true}" --once
            ;;
        clear|clearestop)
            echo "Clearing /estop AND /safety/estop..."
            ros2 topic pub /estop std_msgs/msg/Bool "{data: false}" --once
            ros2 topic pub /safety/estop std_msgs/msg/Bool "{data: false}" --once
            echo "Done. Both topics cleared."
            ;;
        diag)
            echo ""
            echo "============================================"
            echo "  MOTION-PATH PRE-FLIGHT (takes ~12s)"
            echo "============================================"
            echo ""
            echo "[1/8] /estop latched value:"
            timeout 1 ros2 topic echo /estop --once 2>/dev/null | grep data || echo "  (not yet published, assumed False)"
            echo ""
            echo "[2/8] /safety/estop latched value:"
            timeout 1 ros2 topic echo /safety/estop --once 2>/dev/null | grep data || echo "  (not yet published)"
            echo ""
            echo "[3/8] /control/status (relay state — must be RUNNING):"
            timeout 1 ros2 topic echo /control/status --once 2>/dev/null | grep data || echo "  ** NO RELAY STATUS — cmd_vel_relay not running **"
            echo ""
            echo "[4/8] /cmd_vel_safe publish rate (relay output — must be ~20 Hz even at idle):"
            timeout 3 ros2 topic hz /cmd_vel_safe --window 20 2>&1 | head -1
            echo ""
            echo "[5/8] /pico/status (Pico USB link — must include OK):"
            timeout 1 ros2 topic echo /pico/status --once 2>/dev/null | grep data || echo "  ** NO PICO STATUS — pico_bridge not running or USB dead **"
            echo ""
            echo "[6/8] /odometry/filtered rate (EKF — must be >0 Hz):"
            timeout 3 ros2 topic hz /odometry/filtered --window 10 2>&1 | head -1
            echo ""
            echo "[7/8] /perception/unified_obstacles rate (perception — must be ~2 Hz):"
            timeout 3 ros2 topic hz /perception/unified_obstacles --window 4 2>&1 | head -1
            echo ""
            echo "[8/8] Nav2 active state:"
            timeout 2 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>/dev/null | tail -3 || echo "  Could not query Nav2 lifecycle — may still be coming up"
            echo ""
            echo "INTERPRETATION:"
            echo "  - Relay status MUST start with RUNNING (not ESTOP / WATCHDOG)."
            echo "  - /cmd_vel_safe MUST be ~20 Hz. If 0 Hz, relay is not publishing."
            echo "  - Pico status MUST include OK L:0 R:0 EX:0 or similar."
            echo "  - All e-stop values should be data: false."
            echo "  - If any FAIL: run clearestop then re-check, or fix the listed subsystem."
            echo ""
            ;;
        testmove)
            echo ""
            echo "============================================"
            echo "  MOTION TEST — bypass Nav2 (3 seconds)"
            echo "============================================"
            echo "Clearing e-stop on both topics..."
            ros2 topic pub /estop std_msgs/msg/Bool "{data: false}" --once >/dev/null 2>&1
            ros2 topic pub /safety/estop std_msgs/msg/Bool "{data: false}" --once >/dev/null 2>&1
            sleep 0.3
            echo "Publishing 0.1 m/s linear to /cmd_vel at 10 Hz for 3s..."
            echo "  Watch for: (a) /control/status switches to RUNNING with lv>0"
            echo "             (b) /pico/status shows L:>0 R:>0"
            echo "             (c) WHEELS ACTUALLY TURN"
            echo ""
            ( timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 ) &
            TEST_PID=$!
            sleep 0.8
            echo "--- snapshot @0.8s ---"
            timeout 1 ros2 topic echo /control/status --once 2>/dev/null | grep data || echo "  no relay status"
            timeout 1 ros2 topic echo /pico/status --once 2>/dev/null | grep data || echo "  no pico status"
            wait $TEST_PID 2>/dev/null
            echo "--- stopping (publish zero) ---"
            ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once >/dev/null 2>&1
            echo "Done. If wheels did not turn but pico status showed L>0 R>0,"
            echo "the issue is downstream of pico_bridge (motor power, wiring, drivers)."
            echo "If pico status stayed L:0 R:0, the issue is upstream (relay e-stop/watchdog)."
            echo ""
            ;;
        status)
            timeout 2 ros2 topic echo /mission/status --once 2>/dev/null || echo "(no status)"
            timeout 2 ros2 topic echo /mission/state --once 2>/dev/null || echo "(no state)"
            ;;
        zone)
            timeout 2 ros2 topic echo /perception/current_zone --once 2>/dev/null || echo "(no zone)"
            ;;
        dig)
            echo "Manual dig..."
            ros2 service call /excavation/dig std_srvs/srv/Trigger
            ;;
        dump)
            echo "Manual dump..."
            ros2 service call /excavation/dump std_srvs/srv/Trigger
            ;;
        stow)
            echo "Manual stow..."
            ros2 service call /excavation/stow std_srvs/srv/Trigger
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
# Layout:
#   +-----------------------------+-----------------------------+
#   | 0: ROS2 Bringup            | 1: Topic Monitor            |
#   +-----------------------------+-----------------------------+
#   | 2: Mission Control          | 3: Mission Log (view-only)  |
#   +-----------------------------+-----------------------------+
# ------------------------------------------------------------------ #

# Pane 0: Bringup (top-left)
tmux new-session -d -s "$SESSION" -x 200 -y 50
tmux send-keys -t "$SESSION" "$PREAMBLE && echo '--- FULL MISSION BRINGUP (hardware mode) ---' && $LAUNCH_CMD" Enter

# Pane 1: Monitor (top-right)
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "$PREAMBLE && $MONITOR_SCRIPT" Enter

# Pane 2: Mission Control (bottom-left)
tmux split-window -v -t "$SESSION:0.0"
tmux send-keys -t "$SESSION" "$PREAMBLE && echo 'Waiting 12s for bringup...' && sleep 12 && $MISSION_SCRIPT" Enter

# Pane 3: Teleop (OBSERVER mode by default — press 'm' to take manual control)
tmux split-window -v -t "$SESSION:0.1"
tmux send-keys -t "$SESSION" "$PREAMBLE && echo 'Waiting 12s for bringup...' && sleep 12 && echo '--- TELEOP (OBSERVER mode — press m for manual, e for E-STOP) ---' && ros2 run lunabotics_control teleop_keyboard" Enter

# Auto-pin CPU cores
(
    sleep 15
    if [ -f "$WORKSPACE/scripts/pin_cores.sh" ]; then
        bash "$WORKSPACE/scripts/pin_cores.sh" 2>&1 | tee /tmp/pin_cores.log
    fi
) &

# ------------------------------------------------------------------ #
# Apply --facing / --pos via ros2 param set once mission_controller
# is alive. This must complete BEFORE the --auto-start branch fires
# its /mission/start service call, otherwise the mission starts with
# the stale defaults from mission_params.yaml.
#
# We poll for the param service instead of using a fixed sleep so the
# operator can launch on a slow Jetson / cold cache without racing.
# ------------------------------------------------------------------ #
if [ -n "$START_YAW_DEG" ] || [ -n "$START_X" ]; then
    (
        set +eu
        source /opt/ros/humble/setup.bash 2>/dev/null
        source "$WORKSPACE/install/setup.bash" 2>/dev/null
        if [ -f "$WORKSPACE/fastrtps_no_shm.xml" ]; then
            export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE/fastrtps_no_shm.xml"
        fi
        # Wait up to ~30 s for mission_controller's parameter service to appear.
        ready=false
        for i in $(seq 1 15); do
            sleep 2
            if timeout 1 ros2 service list 2>/dev/null \
                 | grep -q "^/mission_controller/set_parameters$"; then
                ready=true
                break
            fi
        done
        if ! $ready; then
            echo "[param-set] WARN: /mission_controller/set_parameters never appeared — params NOT applied" >> /tmp/luna_param_set.log
            exit 0
        fi
        {
            echo "[param-set] applying CLI-supplied start params..."
            if [ -n "$START_YAW_DEG" ]; then
                ros2 param set /mission_controller start_yaw_deg "$START_YAW_DEG"
            fi
            if [ -n "$START_X" ]; then
                ros2 param set /mission_controller start_x "$START_X"
                ros2 param set /mission_controller start_y "$START_Y"
            fi
            echo "[param-set] verify:"
            ros2 param get /mission_controller start_yaw_deg
            ros2 param get /mission_controller start_x
            ros2 param get /mission_controller start_y
            ros2 param get /mission_controller arena_layout
        } 2>&1 >> /tmp/luna_param_set.log
    ) &
fi

# Auto-start mission if requested
if [ "$AUTO_START" = "true" ]; then
    (
        set +eu
        sleep 25
        echo "Auto-starting mission..."
        source /opt/ros/humble/setup.bash 2>/dev/null
        source "$WORKSPACE/install/setup.bash" 2>/dev/null
        ros2 service call /mission/start std_srvs/srv/Trigger
    ) &
fi

# ------------------------------------------------------------------ #
# Web dashboard — access from mission control room via browser
# ttyd serves the tmux session on port 8080 (fully interactive)
# ------------------------------------------------------------------ #
WEB_PORT=8080
if command -v ttyd &> /dev/null; then
    # Kill any previous ttyd
    pkill -f "ttyd.*full_mission" 2>/dev/null || true
    sleep 0.5
    ttyd -p "$WEB_PORT" -t fontSize=14 -t theme='{"background":"#1e1e1e"}' tmux attach -t "$SESSION" &
    TTYD_PID=$!
    echo -e "  ${GREEN}[OK]${NC} Web dashboard: http://$(hostname -I | awk '{print $1}'):${WEB_PORT}"
else
    echo -e "  ${YELLOW}[SKIP]${NC} ttyd not installed — no web dashboard"
    echo -e "         Install: sudo apt install -y ttyd"
fi

# Focus mission control pane
tmux select-pane -t "$SESSION:0.2"

echo ""
echo -e "${GREEN}${BOLD}Full mission stack launched in tmux session '$SESSION'${NC}"
echo ""
echo -e "  ${CYAN}Pane 0${NC} (top-left):     Bringup (all subsystems, hardware excavation)"
echo -e "  ${CYAN}Pane 1${NC} (top-right):    Monitor (sensors, zones, mission state, excavation)"
echo -e "  ${CYAN}Pane 2${NC} (bottom-left):  ${BOLD}Mission Control${NC} (setpose, start — hands-free after start)"
echo -e "  ${CYAN}Pane 3${NC} (bottom-right): Teleop (OBSERVER mode — press 'm' for manual, 'e' for E-STOP)"
echo ""
echo -e "  ${BOLD}WEB:${NC}   Open ${CYAN}http://$(hostname -I | awk '{print $1}'):${WEB_PORT}${NC} in any browser"
echo -e "         Fully interactive — switch panes, type commands, press 'e' for E-stop"
echo ""
echo -e "  ${BOLD}LAYOUT:${NC} Launched with arena_layout=${CYAN}${ARENA_LAYOUT}${NC}"
echo -e "         (use ${CYAN}setlayout B${NC} in pane 2 if it's the mirror pit)"
echo -e "  ${BOLD}FIRST:${NC} In pane 2, run: ${CYAN}setpose 3.5 1.5 180${NC}"
echo -e "  ${BOLD}THEN:${NC}  Run: ${CYAN}start${NC}"
echo ""
echo -e "  Hardware mode: actuator + belt + servo commands are REAL"
echo ""

tmux attach -t "$SESSION"
