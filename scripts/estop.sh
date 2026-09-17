#!/bin/bash
# 全机急停：ROS 急停话题 + 杀节点 + CAN 失能/清零
# 用法: ~/robot_ws/scripts/estop.sh
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=== 全机急停 ==="

pkill -9 -f 'ros2 topic pub.*quad/wheels' 2>/dev/null || true
pkill -9 -f 'ros2 topic pub.*c620' 2>/dev/null || true
pkill -9 -f 'lib/rs_motor_ros2' 2>/dev/null || true
pkill -9 quad_rs_ros2 2>/dev/null || true
pkill -9 c620_quad_ros2 2>/dev/null || true
pkill -9 quad_teleop_ros2 2>/dev/null || true
rm -f /tmp/quad_rs_ros2.lock /tmp/c620_quad_ros2.lock /tmp/quad_teleop_ros2.lock
sleep 0.3

if command -v ros2 &>/dev/null && [[ -f "$WS_DIR/install/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  # shellcheck source=/dev/null
  source "$WS_DIR/install/setup.bash" 2>/dev/null || true
  # shellcheck source=/dev/null
  source "$SCRIPT_DIR/ros-env.sh" 2>/dev/null || true
  set -u
  "$SCRIPT_DIR/quad-pub.sh" /quad/e_stop 2>/dev/null || true
  "$SCRIPT_DIR/quad-pub.sh" /quad/wheels/stop 2>/dev/null || true
fi

"$SCRIPT_DIR/can-hub-up.sh" can0 can1 can2 2>/dev/null || true

HOST_ID="${HOST_ID:-FF}"
for IFACE in can0 can1; do
  case "$IFACE" in
    can0) ids=(01 02 03 04) ;;
    can1) ids=(05 06 07 08) ;;
  esac
  for MID in "${ids[@]}"; do
    stop_id="$(printf '%08X' $(( (4 << 24) | (0x$HOST_ID << 8) | (16#$MID) )) )"
    cansend "$IFACE" "${stop_id}#0000000000000000" 2>/dev/null || true
  done
  echo "CAN $IFACE: 灵足 1~8 已发失能帧"
done

C620_CAN="${C620_CAN:-can2}"
if ip link show "$C620_CAN" &>/dev/null; then
  sudo ip link set "$C620_CAN" up 2>/dev/null || true
  for _ in $(seq 1 50); do
    cansend "$C620_CAN" 200#0000000000000000 2>/dev/null || break
  done
  echo "CAN $C620_CAN: C620 已发零电流"
fi

echo "若仍在转 → 立即断 24V"
