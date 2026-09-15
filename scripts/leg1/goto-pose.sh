#!/bin/bash
# 跳到预设姿态（需 leg1_dual_ros2 已运行）
# 用法: leg1-goto-pose.sh neutral|stand|crouch
set -eo pipefail

POSE="${1:-stand}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_DIR/install/setup.bash"
fi

case "$POSE" in
  neutral|stand|crouch) ;;
  *)
    echo "用法: $0 neutral|stand|crouch"
    exit 1
    ;;
esac

if ros2 node list 2>/dev/null | grep -q quad_rs_node; then
  DRIVER="quad_rs_ros2"
elif ros2 node list 2>/dev/null | grep -q leg1_dual_node; then
  DRIVER="leg1_dual_ros2"
else
  echo "[错误] 灵足节点未运行 → ~/robot_ws/scripts/run-leg1-all.sh"
  exit 1
fi

echo "=== goto $POSE ($DRIVER) ==="
ros2 topic pub "/leg1/goto/${POSE}" std_msgs/msg/Empty '{}' --once \
  --qos-reliability reliable --qos-durability transient_local

sleep 3
echo "=== 反馈 ==="
ros2 topic echo /leg1/lift/state --once 2>/dev/null | grep -A1 "position:" || true
ros2 topic echo /leg1/crouch/state --once 2>/dev/null | grep -A1 "position:" || true
echo ""
echo "锁定: ros2 topic pub /leg1/hold std_msgs/msg/Empty '{}' --once \
  --qos-reliability reliable --qos-durability transient_local"
