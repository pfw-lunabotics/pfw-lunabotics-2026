#!/usr/bin/env bash
# ============================================================================
# setup_ethernet.sh — Configure the wired ethernet link for VM <-> Jetson.
# ============================================================================
# NO INTERNET REQUIRED. Uses tools already on the system (ip, nmcli).
#
# Auto-detects which machine you're on:
#   - If interface `enP8p1s0` exists -> JETSON side    (sets 192.168.0.200/24)
#   - Otherwise                       -> VM side        (sets 192.168.0.100/24)
#
# Run this on BOTH sides (Jetson and VM). It is idempotent.
#
# Usage:
#   chmod +x setup_ethernet.sh
#   sudo ./setup_ethernet.sh                 # auto-detect side
#   sudo ./setup_ethernet.sh jetson          # force Jetson side
#   sudo ./setup_ethernet.sh vm              # force VM side
#   sudo ./setup_ethernet.sh vm eth0         # force VM side, force iface
# ============================================================================

set -euo pipefail

JETSON_IP="192.168.0.200"
VM_IP="192.168.0.100"
NETMASK="24"
JETSON_IFACE="enP8p1s0"   # Tegra ethernet name on this robot

if [ "$(id -u)" -ne 0 ]; then
    echo "Re-running with sudo..."
    exec sudo -E "$0" "$@"
fi

# ---------- detect side ----------
SIDE="${1:-auto}"
FORCED_IFACE="${2:-}"

if [ "$SIDE" = "auto" ]; then
    if ip link show "$JETSON_IFACE" >/dev/null 2>&1; then
        SIDE="jetson"
    else
        SIDE="vm"
    fi
fi

case "$SIDE" in
    jetson)
        IFACE="${FORCED_IFACE:-$JETSON_IFACE}"
        IP_ADDR="$JETSON_IP"
        PEER_LABEL="VM ($VM_IP)"
        ;;
    vm)
        # On the VM, pick the first non-loopback non-virtual ethernet-ish iface.
        if [ -n "$FORCED_IFACE" ]; then
            IFACE="$FORCED_IFACE"
        else
            IFACE=$(ip -o link show \
                | awk -F': ' '{print $2}' \
                | grep -vE '^(lo|docker|virbr|br-|veth|wl|usb|l4tbr|can)' \
                | head -1)
        fi
        if [ -z "$IFACE" ]; then
            echo "[ERROR] could not find an ethernet interface on the VM."
            echo "        Pass it explicitly: sudo ./setup_ethernet.sh vm <iface>"
            ip -br link show
            exit 1
        fi
        IP_ADDR="$VM_IP"
        PEER_LABEL="Jetson ($JETSON_IP)"
        ;;
    *)
        echo "Usage: $0 [auto|jetson|vm] [iface]"
        exit 2
        ;;
esac

echo "=========================================="
echo " Ethernet setup"
echo "   side  : $SIDE"
echo "   iface : $IFACE"
echo "   ip    : $IP_ADDR/$NETMASK"
echo "   peer  : $PEER_LABEL"
echo "=========================================="

# ---------- configure ----------
# Prefer nmcli (persistent across reboot). Fall back to plain `ip` if nmcli
# is absent (transient — survives until reboot, fine for a competition run).
CON_NAME="luna-direct-$IFACE"

if command -v nmcli >/dev/null 2>&1 && systemctl is-active --quiet NetworkManager 2>/dev/null; then
    echo "[nmcli] using NetworkManager (persistent across reboot)"
    # Delete any prior connection that owns this iface so we don't fight.
    nmcli -t -f NAME,DEVICE con show \
      | awk -F: -v d="$IFACE" '$2==d {print $1}' \
      | while read -r old; do
          if [ "$old" != "$CON_NAME" ]; then
              echo "  removing previous connection on $IFACE: $old"
              nmcli con delete "$old" >/dev/null 2>&1 || true
          fi
      done
    nmcli con delete "$CON_NAME" >/dev/null 2>&1 || true
    nmcli con add type ethernet ifname "$IFACE" con-name "$CON_NAME" \
        ipv4.method manual \
        ipv4.addresses "$IP_ADDR/$NETMASK" \
        ipv6.method ignore \
        autoconnect yes
    nmcli con up "$CON_NAME"
else
    echo "[ip] NetworkManager not active — using transient \`ip addr\`."
    echo "     (lost on reboot; rerun this script after reboot.)"
    ip addr flush dev "$IFACE" || true
    ip addr add "$IP_ADDR/$NETMASK" dev "$IFACE"
    ip link set "$IFACE" up
fi

echo ""
echo "[result] $IFACE is now:"
ip -br addr show "$IFACE"

# ---------- reachability ----------
if [ "$SIDE" = "vm" ]; then
    PEER_IP="$JETSON_IP"
else
    PEER_IP="$VM_IP"
fi

echo ""
echo "[ping] testing $PEER_IP (waiting up to 5s)..."
if ping -c 2 -W 2 "$PEER_IP" >/dev/null 2>&1; then
    echo "  [OK] $PEER_IP is reachable over the cable."
else
    echo "  [INFO] $PEER_IP did not answer yet."
    echo "         This is fine if you haven't run setup_ethernet.sh on the"
    echo "         OTHER end yet. Run it there too, then re-test:"
    echo "             ping $PEER_IP"
fi

echo ""
echo "Done."
