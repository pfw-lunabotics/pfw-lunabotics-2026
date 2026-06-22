#!/usr/bin/env bash
# ============================================================================
# setup_ssh_keys.sh — Passwordless SSH from VM to Jetson.
# ============================================================================
# NO INTERNET REQUIRED (uses ssh-keygen and ssh-copy-id, already installed).
#
# You only need to run this ONCE, after the ethernet link is up.
# Run this ON THE VM (not the Jetson).
#
# Usage:
#   chmod +x setup_ssh_keys.sh
#   ./setup_ssh_keys.sh
#   JETSON_USER=lunabotics JETSON_HOST=192.168.0.200 ./setup_ssh_keys.sh
# ============================================================================

set -euo pipefail

JETSON_USER="${JETSON_USER:-lunabotics}"
JETSON_HOST="${JETSON_HOST:-192.168.0.200}"

echo "=========================================="
echo " SSH key setup  ->  ${JETSON_USER}@${JETSON_HOST}"
echo "=========================================="

# 1) Generate a key if neither type exists.
if [ ! -f "$HOME/.ssh/id_ed25519" ] && [ ! -f "$HOME/.ssh/id_rsa" ]; then
    echo "[1/3] no existing ssh key — generating ed25519 (no passphrase)"
    mkdir -p "$HOME/.ssh"; chmod 700 "$HOME/.ssh"
    ssh-keygen -t ed25519 -N "" -f "$HOME/.ssh/id_ed25519"
else
    echo "[1/3] ssh key already exists — keeping it"
fi

# 2) Test if passwordless already works.
echo "[2/3] testing passwordless login..."
if ssh -o BatchMode=yes -o ConnectTimeout=5 \
      -o StrictHostKeyChecking=accept-new \
      "${JETSON_USER}@${JETSON_HOST}" "echo ok" >/dev/null 2>&1; then
    echo "  [OK] passwordless ssh already works"
else
    echo "  not yet — running ssh-copy-id (you will be prompted for the"
    echo "  Jetson password ONE last time)"
    ssh-copy-id -o StrictHostKeyChecking=accept-new \
        "${JETSON_USER}@${JETSON_HOST}"
fi

# 3) Verify.
echo "[3/3] verifying..."
if ssh -o BatchMode=yes -o ConnectTimeout=5 \
      "${JETSON_USER}@${JETSON_HOST}" "hostname && whoami" 2>/dev/null; then
    echo "  [OK] passwordless ssh works."
else
    echo "  [WARN] still failing. Things to check:"
    echo "    - Is the ethernet link up? ping ${JETSON_HOST}"
    echo "    - Is ssh enabled on the Jetson? sudo systemctl status ssh"
    echo "    - On the Jetson: ls -la ~/.ssh/authorized_keys"
fi

echo ""
echo "Done. Now launch the GUI: ./run_gui.sh"
