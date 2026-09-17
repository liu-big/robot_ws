#!/bin/bash
# 四足整机预设姿态（需 quad_rs_ros2 已运行）
# 用法: quad-goto-pose.sh neutral|stand|crouch
set -eo pipefail

POSE="${1:-stand}"
WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_DIR/install/setup.bash"
fi
# shellcheck source=/dev/null
source "$WS_DIR/scripts/ros-env.sh" 2>/dev/null || true

case "$POSE" in
  neutral|stand|crouch) ;;
  *)
    echo "用法: $0 neutral|stand|crouch"
    exit 1
    ;;
esac

if ! ros2 node list 2>/dev/null | grep -q quad_rs_node; then
  echo "[错误] quad_rs_ros2 未运行 → ~/robot_ws/scripts/run-quad-all.sh"
  exit 1
fi

echo "=== quad goto $POSE ==="
"$WS_DIR/scripts/quad-pub.sh" "/quad/goto/${POSE}"
sleep 3
ros2 topic echo /quad/joint_states --once 2>/dev/null | head -20 || true
