#!/bin/bash
# C620 急停 — 杀节点 + CAN 发 0A
set -euo pipefail

echo "=== C620 急停 ==="
pkill -9 -f '/lib/rs_motor_ros2/c620_ros2' 2>/dev/null || true
pkill -9 -f 'ros2 run rs_motor_ros2 c620_ros2' 2>/dev/null || true

if command -v ros2 &>/dev/null && [ -f "$HOME/robot_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/robot_ws/install/setup.bash" 2>/dev/null || true
  ros2 topic pub /c620/stop std_msgs/msg/Empty "{}" --once 2>/dev/null || true
  ros2 topic pub /c620/velocity_command std_msgs/msg/Float64 '{data: 0.0}' --once 2>/dev/null || true
  ros2 topic pub /c620/command std_msgs/msg/Float64 '{data: 0.0}' --once 2>/dev/null || true
fi

CAN_IF="${C620_CAN:-can1}"
if ip link show "$CAN_IF" &>/dev/null; then
  sudo ip link set "$CAN_IF" up 2>/dev/null || true
  for i in $(seq 1 100); do
    cansend "$CAN_IF" 200#0000000000000000 2>/dev/null || break
  done
  echo "CAN $CAN_IF: 已发 100 帧 0A"
else
  echo "$CAN_IF 不存在，跳过 CAN"
fi

echo "若仍在转 → 立即断 C620 **24V 电源**"
echo "完成"
