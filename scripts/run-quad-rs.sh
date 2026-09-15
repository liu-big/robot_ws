#!/bin/bash
# 四足 8×RS03（抬升 can0 + 拉杆 can1）
# 台架默认 active_legs=[1]；全机: 改 config/quad_rs_params.yaml 为 [1,2,3,4]
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=== 停止旧灵足节点 ==="
pkill -9 quad_rs_ros2 2>/dev/null || true
pkill -9 leg1_dual_ros2 2>/dev/null || true
pkill -9 -f rs_motor_ros2 2>/dev/null || true
sleep 1
rm -f /tmp/quad_rs_ros2.lock /tmp/leg1_dual_ros2.lock

"$SCRIPT_DIR/can-hub-up.sh" can0 can1

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_DIR/install/setup.bash"
fi

echo ""
echo "=== quad_rs_ros2 (can0 抬升 + can1 拉杆) ==="
echo "  话题: /leg{N}/lift|crouch/*  /quad/goto/*  /quad/joint_states"
echo "  leg1 兼容: /leg1/goto/stand  /leg1/hold"
echo "  参数: config/quad_rs_params.yaml"
echo ""

exec ros2 run rs_motor_ros2 quad_rs_ros2 --ros-args \
  --params-file "$WS_DIR/config/quad_rs_params.yaml"
