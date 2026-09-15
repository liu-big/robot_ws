#!/bin/bash
# leg1 双灵足（抬升 can0 ID1 + 拉杆 can1 ID5）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=== 停止旧进程 ==="
pkill -9 leg1_dual_ros2 2>/dev/null || true
pkill -9 -f rs_motor_ros2 2>/dev/null || true
pkill -9 leg1_dual_pose 2>/dev/null || true
sleep 1
rm -f /tmp/leg1_dual_ros2.lock

"$SCRIPT_DIR/can-hub-up.sh" can0 can1

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_DIR/install/setup.bash"
fi

echo
echo "=== leg1_dual_ros2 (can0 抬升=1, can1 拉杆=5) ==="
echo "话题:"
echo "  /leg1/lift/command    抬升目标角 (rad)"
echo "  /leg1/crouch/command  趴下/起身目标角 (rad)"
echo "  /leg1/lift/state      /leg1/crouch/state"
echo "  /leg1/hold            双关节锁定当前位置"
echo "  /leg1/goto/neutral    预设：休息位"
echo "  /leg1/goto/stand      预设：微站立"
echo "  /leg1/goto/crouch     预设：微蹲（趴下关节更高）"
echo
echo "示例:"
echo "  ~/robot_ws/scripts/leg1-goto-pose.sh stand"
echo "  ~/robot_ws/scripts/leg1-pose-ros.sh"
echo

exec ros2 run rs_motor_ros2 leg1_dual_ros2 --ros-args \
  --params-file "$WS_DIR/config/leg1_dual_params.yaml"
