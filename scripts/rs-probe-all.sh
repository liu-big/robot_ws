#!/bin/bash
# 探测整机 8×灵足：can0 抬升 1~4，can1 拉杆 5~8
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== can0 抬升 ID 1~4 ==="
CAN_IFACE=can0 "$SCRIPT_DIR/rs-probe.sh" 01 02 03 04

echo ""
echo "=== can1 拉杆 ID 5~8 ==="
CAN_IFACE=can1 "$SCRIPT_DIR/rs-probe.sh" 05 06 07 08
