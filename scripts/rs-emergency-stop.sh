#!/bin/bash
# 灵足 RS03 紧急停转 — 杀 ROS 进程 + 失能 can0/can1 全部 ID
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== 灵足紧急停转 ==="
pkill -9 quad_rs_ros2 2>/dev/null || true
pkill -9 leg1_dual_ros2 2>/dev/null || true
pkill -9 -f rs_motor_ros2 2>/dev/null || true
rm -f /tmp/quad_rs_ros2.lock /tmp/leg1_dual_ros2.lock
sleep 0.3

"$SCRIPT_DIR/can-hub-up.sh" can0 can1 2>/dev/null || true
CAN_IFACE=can0 "$SCRIPT_DIR/rs-disable.sh" 01 02 03 04
CAN_IFACE=can1 "$SCRIPT_DIR/rs-disable.sh" 05 06 07 08

echo "[✓] can0/can1 已失能。若还在转请立即断 24V。"
