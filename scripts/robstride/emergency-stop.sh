#!/bin/bash
# 灵足 RS03 紧急停转 — 杀 ROS 进程 + 失能 ID 1~8
set -eo pipefail
# shellcheck source=../_paths.sh
source "$(cd "$(dirname "$0")/.." && pwd)/_paths.sh"

IFACE="${CAN_IFACE:-can0}"
echo "=== 灵足紧急停转 ==="
pkill -9 leg1_dual_ros2 2>/dev/null || true
pkill -9 -f rs_motor_ros2 2>/dev/null || true
rm -f /tmp/leg1_dual_ros2.lock
sleep 0.3

"$CAN_DIR/hub-up.sh" "$IFACE" 2>/dev/null || true
"$ROBSTRIDE_DIR/disable.sh" 01 02 03 04 05 06 07 08
echo "[✓] 已失能 ID 1~8。若还在转请立即断 24V。"
