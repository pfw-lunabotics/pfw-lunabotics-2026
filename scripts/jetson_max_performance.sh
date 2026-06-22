#!/usr/bin/env bash
# ============================================================================
# jetson_max_performance.sh — Lock Jetson Orin Nano to Maximum Performance
# ============================================================================
# The Jetson defaults to power-saving mode:
#   - CPU governor: schedutil (scales down when "idle" — but ROS2 looks idle
#     between callbacks, so CPU clocks drop RIGHT before a heavy callback)
#   - GPU: 306 MHz (max is 1020 MHz)
#   - Power mode: possibly 7W instead of 15W
#
# This script locks everything to maximum. Run ONCE after boot, before
# launching any ROS2 nodes.
#
# Usage:
#   sudo ./scripts/jetson_max_performance.sh
# ============================================================================

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "Must run as root: sudo $0"
    exit 1
fi

echo "========================================="
echo "  JETSON MAX PERFORMANCE MODE"
echo "========================================="
echo ""

# ------------------------------------------------------------------ #
# 1. Power mode → 15W (MAXN for Orin Nano)
# ------------------------------------------------------------------ #
echo "1. Setting power mode to 15W (MAXN)..."
if command -v nvpmodel &> /dev/null; then
    nvpmodel -m 0  # Mode 0 = 15W on Orin Nano
    echo "   Power mode: $(nvpmodel -q 2>/dev/null | head -1)"
else
    echo "   nvpmodel not found — skipping (check JetPack install)"
fi

# ------------------------------------------------------------------ #
# 2. CPU governor → performance (lock all cores to max frequency)
# ------------------------------------------------------------------ #
echo "2. Locking CPU governor to 'performance'..."
for gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance > "$gov" 2>/dev/null || true
done

# Verify
FREQ=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null)
MAX=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null)
echo "   CPU0 freq: ${FREQ}kHz / ${MAX}kHz max"
GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)
echo "   Governor: $GOV"

# ------------------------------------------------------------------ #
# 3. GPU → max frequency
# ------------------------------------------------------------------ #
echo "3. Locking GPU to max frequency..."
GPU_PATH="/sys/devices/platform/bus@0/17000000.gpu/devfreq/17000000.gpu"
if [ -d "$GPU_PATH" ]; then
    GPU_MAX=$(cat "$GPU_PATH/max_freq")
    echo "$GPU_MAX" > "$GPU_PATH/min_freq" 2>/dev/null || true
    echo "userspace" > "$GPU_PATH/governor" 2>/dev/null || true
    echo "$GPU_MAX" > "$GPU_PATH/userspace/set_freq" 2>/dev/null || true
    CUR=$(cat "$GPU_PATH/cur_freq")
    echo "   GPU freq: ${CUR}Hz / ${GPU_MAX}Hz max"
else
    echo "   GPU devfreq path not found — trying jetson_clocks"
fi

# ------------------------------------------------------------------ #
# 4. jetson_clocks — lock EMC (memory bus) and other clocks
# ------------------------------------------------------------------ #
echo "4. Running jetson_clocks..."
if command -v jetson_clocks &> /dev/null; then
    jetson_clocks 2>/dev/null || echo "   jetson_clocks returned non-zero (may be partial)"
    echo "   jetson_clocks applied"
else
    echo "   jetson_clocks not found"
fi

# ------------------------------------------------------------------ #
# 5. Disable kernel CPU idle states (prevent deep sleep between callbacks)
# ------------------------------------------------------------------ #
echo "5. Disabling CPU idle states..."
for cpu_dir in /sys/devices/system/cpu/cpu*/cpuidle/state*/disable; do
    echo 1 > "$cpu_dir" 2>/dev/null || true
done
echo "   CPU idle states disabled"

# ------------------------------------------------------------------ #
# 6. Set scheduler tuning for real-time workloads
# ------------------------------------------------------------------ #
echo "6. Tuning kernel scheduler..."
# Reduce scheduler migration cost — helps ROS2 callbacks that wake frequently
echo 100000 > /proc/sys/kernel/sched_migration_cost_ns 2>/dev/null || true
# Allow near-real-time tasks to run without throttling
echo -1 > /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null || true
echo "   Scheduler tuned for real-time workloads"

echo ""
echo "========================================="
echo "  DONE — Jetson locked to max performance"
echo "========================================="
echo ""

# Summary
echo "Status:"
echo "  CPU gov:  $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
echo "  CPU freq: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)kHz"
echo "  GPU freq: $(cat $GPU_PATH/cur_freq 2>/dev/null || echo 'unknown')Hz"
echo ""
