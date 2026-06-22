#!/usr/bin/env bash
# ============================================================================
# preflight.sh — Lunabotics 2026 Hardware Pre-Flight Check
# ============================================================================
# Run BEFORE competition launch to verify all hardware is connected and ready.
# Takes ~10 seconds. Reports pass/fail for each subsystem.
#
# Usage:
#   ./scripts/preflight.sh
# ============================================================================

# No set -e/-u — checks intentionally test commands that may fail

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

PASS="${GREEN}[PASS]${NC}"
FAIL="${RED}[FAIL]${NC}"
WARN="${YELLOW}[WARN]${NC}"
INFO="${CYAN}[INFO]${NC}"

PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0

pass() { echo -e "  ${PASS} $1"; PASS_COUNT=$((PASS_COUNT + 1)); }
fail() { echo -e "  ${FAIL} $1"; FAIL_COUNT=$((FAIL_COUNT + 1)); }
warn() { echo -e "  ${WARN} $1"; WARN_COUNT=$((WARN_COUNT + 1)); }
info() { echo -e "  ${INFO} $1"; }

WORKSPACE="/home/lunabotics/pfw-lunabotics"

# --- DDS config (must be set before any ros2 command) ---
if [ -f "$WORKSPACE/fastrtps_no_shm.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE/fastrtps_no_shm.xml"
fi

echo ""
echo -e "${BOLD}========================================${NC}"
echo -e "${BOLD}  LUNABOTICS 2026 — PRE-FLIGHT CHECK${NC}"
echo -e "${BOLD}========================================${NC}"
echo ""

# ------------------------------------------------------------------ #
# 1. ROS2 Environment
# ------------------------------------------------------------------ #
echo -e "${BOLD}1. ROS2 Environment${NC}"

if [ -n "${ROS_DISTRO:-}" ]; then
    pass "ROS2 distro: $ROS_DISTRO"
else
    fail "ROS2 not sourced — run: source /opt/ros/humble/setup.bash"
fi

if [ -f "$WORKSPACE/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WORKSPACE/install/setup.bash" 2>/dev/null
    pass "Workspace built (install/setup.bash exists)"
else
    fail "Workspace not built — run: cd $WORKSPACE && colcon build --symlink-install"
fi

# Check critical packages (timeout protects against DDS hangs)
# Poke the daemon first so pkg list doesn't hang
timeout 3 ros2 daemon start >/dev/null 2>&1 || true
PKG_LIST=$(timeout 10 ros2 pkg list 2>/dev/null || echo "")
if [ -z "$PKG_LIST" ]; then
    fail "ros2 pkg list timed out — check DDS config / daemon"
else
    REQUIRED_PKGS=(
        lunabotics_navigation
        lunabotics_perception
        lunabotics_control
        lunabotics_localization
        lunabotics_autonomy
        lunabotics_description
    )
    MISSING_PKGS=()
    for pkg in "${REQUIRED_PKGS[@]}"; do
        if ! echo "$PKG_LIST" | grep -q "^${pkg}$"; then
            MISSING_PKGS+=("$pkg")
        fi
    done
    if [ ${#MISSING_PKGS[@]} -eq 0 ]; then
        pass "All ${#REQUIRED_PKGS[@]} lunabotics packages found"
    else
        fail "Missing packages: ${MISSING_PKGS[*]}"
    fi

    # Check hardware-specific packages
    if echo "$PKG_LIST" | grep -q "^point_lio$"; then
        pass "Point-LIO package found"
    else
        warn "Point-LIO package not found (needed for localization)"
    fi

    if echo "$PKG_LIST" | grep -q "^unitree_lidar_ros2$"; then
        pass "Unitree LiDAR driver package found"
    else
        warn "Unitree LiDAR driver not found (needed for L2)"
    fi
fi

echo ""

# ------------------------------------------------------------------ #
# 2. Network — Unitree L2 LiDAR
# ------------------------------------------------------------------ #
echo -e "${BOLD}2. Unitree L2 LiDAR (Network)${NC}"

LIDAR_IP="192.168.1.62"
JETSON_IP="192.168.1.2"

# Check Jetson has the right IP on the LiDAR subnet
if ip addr show 2>/dev/null | grep -q "$JETSON_IP"; then
    pass "Jetson IP $JETSON_IP configured"
else
    fail "Jetson IP $JETSON_IP not found — configure ethernet interface"
    info "Try: sudo ip addr add $JETSON_IP/24 dev enP8p1s0"
fi

# Ping LiDAR
if ping -c 2 -W 1 "$LIDAR_IP" > /dev/null 2>&1; then
    pass "LiDAR responding at $LIDAR_IP"
else
    fail "LiDAR not reachable at $LIDAR_IP — check ethernet cable"
fi

echo ""

# ------------------------------------------------------------------ #
# 3. Pico Motor Controller (USB Serial)
# ------------------------------------------------------------------ #
echo -e "${BOLD}3. Pico Motor Controller (USB)${NC}"

PICO_SYMLINK="/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6647c156730ab24-if00"

if [ -e "$PICO_SYMLINK" ]; then
    pass "Pico USB symlink found"
    PICO_DEV=$(readlink -f "$PICO_SYMLINK")
    info "  -> resolves to $PICO_DEV"
elif [ -e /dev/ttyACM0 ]; then
    warn "Pico symlink missing, but /dev/ttyACM0 exists (may be Pico)"
else
    fail "No Pico USB device found — check USB cable"
fi

# Check user is in dialout group (needed for serial access)
if groups | grep -qE '(dialout|plugdev)'; then
    pass "User in dialout/plugdev group (serial access OK)"
else
    warn "User not in dialout group — may need: sudo usermod -aG dialout $USER"
fi

echo ""

# ------------------------------------------------------------------ #
# 4. Servo (Deposition) — USB Serial
# ------------------------------------------------------------------ #
echo -e "${BOLD}4. Deposition Servo (USB)${NC}"

if [ -e /dev/ttyACM1 ]; then
    pass "Servo serial device /dev/ttyACM1 found"
else
    warn "Servo /dev/ttyACM1 not found (OK if not using deposition)"
fi

echo ""

# ------------------------------------------------------------------ #
# 5. Jetson Performance Mode
# ------------------------------------------------------------------ #
echo -e "${BOLD}5. Jetson Performance${NC}"

# CPU governor
CPU_GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "unknown")
if [ "$CPU_GOV" = "performance" ]; then
    pass "CPU governor: performance (max clocks)"
else
    warn "CPU governor: $CPU_GOV — run: sudo ./scripts/jetson_max_performance.sh"
fi

# GPU frequency
GPU_CUR=$(cat /sys/devices/platform/bus@0/17000000.gpu/devfreq/17000000.gpu/cur_freq 2>/dev/null || echo "0")
GPU_MAX=$(cat /sys/devices/platform/bus@0/17000000.gpu/devfreq/17000000.gpu/max_freq 2>/dev/null || echo "0")
if [ "$GPU_CUR" -ge "$GPU_MAX" ] 2>/dev/null && [ "$GPU_MAX" -gt 0 ] 2>/dev/null; then
    pass "GPU frequency: max ($(( GPU_CUR / 1000000 )) MHz)"
else
    GPU_CUR_MHZ=$(( GPU_CUR / 1000000 ))
    GPU_MAX_MHZ=$(( GPU_MAX / 1000000 ))
    warn "GPU at ${GPU_CUR_MHZ}MHz / ${GPU_MAX_MHZ}MHz — run: sudo ./scripts/jetson_max_performance.sh"
fi

echo ""

# ------------------------------------------------------------------ #
# 6. System Resources
# ------------------------------------------------------------------ #
echo -e "${BOLD}6. System Resources${NC}"

# CPU temperature (Jetson-specific)
if [ -f /sys/devices/virtual/thermal/thermal_zone0/temp ]; then
    TEMP=$(cat /sys/devices/virtual/thermal/thermal_zone0/temp)
    TEMP_C=$((TEMP / 1000))
    if [ "$TEMP_C" -lt 70 ]; then
        pass "CPU temperature: ${TEMP_C}°C"
    elif [ "$TEMP_C" -lt 85 ]; then
        warn "CPU temperature: ${TEMP_C}°C (warm — check cooling)"
    else
        fail "CPU temperature: ${TEMP_C}°C (OVERHEATING)"
    fi
fi

# Available memory
MEM_AVAIL=$(awk '/MemAvailable/ {printf "%.0f", $2/1024}' /proc/meminfo)
if [ "$MEM_AVAIL" -gt 2000 ]; then
    pass "Available memory: ${MEM_AVAIL} MB"
elif [ "$MEM_AVAIL" -gt 1000 ]; then
    warn "Available memory: ${MEM_AVAIL} MB (low — close other apps)"
else
    fail "Available memory: ${MEM_AVAIL} MB (CRITICAL — will OOM)"
fi

# Disk space
DISK_AVAIL=$(df -BM "$WORKSPACE" | tail -1 | awk '{print $4}' | tr -d 'M')
if [ "$DISK_AVAIL" -gt 1000 ]; then
    pass "Disk space: ${DISK_AVAIL} MB free"
else
    warn "Disk space: ${DISK_AVAIL} MB free (low — clear old bags)"
fi

echo ""

# ------------------------------------------------------------------ #
# 7. ROS2 DDS / Middleware
# ------------------------------------------------------------------ #
echo -e "${BOLD}7. ROS2 Middleware${NC}"

if [ -n "${RMW_IMPLEMENTATION:-}" ]; then
    info "RMW: $RMW_IMPLEMENTATION"
else
    info "RMW: default (likely rmw_fastrtps_cpp)"
fi

if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    if [ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
        pass "FastRTPS config: $FASTRTPS_DEFAULT_PROFILES_FILE"
    else
        warn "FASTRTPS_DEFAULT_PROFILES_FILE set but file missing"
    fi
fi

# Check no-shared-memory XML (important for Jetson stability)
if [ -f "$WORKSPACE/fastrtps_no_shm.xml" ]; then
    if [ "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" = "$WORKSPACE/fastrtps_no_shm.xml" ]; then
        pass "Shared memory disabled (fastrtps_no_shm.xml active)"
    else
        warn "fastrtps_no_shm.xml exists but not set as FASTRTPS_DEFAULT_PROFILES_FILE"
        info "Add to ~/.bashrc: export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE/fastrtps_no_shm.xml"
    fi
fi

echo ""

# ------------------------------------------------------------------ #
# Summary
# ------------------------------------------------------------------ #
echo -e "${BOLD}========================================${NC}"
TOTAL=$((PASS_COUNT + FAIL_COUNT + WARN_COUNT))
echo -e "  ${GREEN}${PASS_COUNT} passed${NC}  ${RED}${FAIL_COUNT} failed${NC}  ${YELLOW}${WARN_COUNT} warnings${NC}  (${TOTAL} checks)"

if [ "$FAIL_COUNT" -eq 0 ]; then
    echo ""
    echo -e "  ${GREEN}${BOLD}PRE-FLIGHT PASSED — ready to launch${NC}"
    echo -e "  Run: ${CYAN}./scripts/competition.sh${NC}"
else
    echo ""
    echo -e "  ${RED}${BOLD}PRE-FLIGHT FAILED — fix ${FAIL_COUNT} issue(s) above${NC}"
fi
echo -e "${BOLD}========================================${NC}"
echo ""

exit "$FAIL_COUNT"
