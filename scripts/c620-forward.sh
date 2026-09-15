#!/bin/bash
# 前两轮（leg1+leg2 / CAN ID1+ID2）独立前进 — 需 c620_quad_ros2 已启动
# 用法: c620-forward.sh [速度rad/s，默认0.15]
set -euo pipefail
V="${1:-0.15}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${HOME}/robot_ws/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${HOME}/robot_ws/install/setup.bash"
fi

echo "前进 leg1+leg2 (ID1+ID2) 速度=${V} rad/s（各轮独立控制）"
echo "停止: Ctrl+C 后发零速，或 ~/robot_ws/scripts/c620-estop.sh"
exec ros2 topic pub /quad/wheels/cmd std_msgs/msg/Float64MultiArray \
  "{data: [${V}, ${V}, 0.0, 0.0]}" \
  --rate 100 \
  --qos-reliability reliable \
  --qos-durability transient_local
