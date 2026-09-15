#!/bin/bash
# 启动 c620_quad_ros2（leg1+leg4 前轮），自动拉起 can2
# 用法: ~/robot_ws/scripts/run-c620-quad.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
IFACE="${1:-can2}"

echo "=== 停止旧进程 ==="
pkill -9 -f 'lib/rs_motor_ros2/c620_quad_ros2' 2>/dev/null || true
pkill -9 -f 'ros2 topic pub.*quad/wheels' 2>/dev/null || true
rm -f /tmp/c620_quad_ros2.lock
sleep 1

echo "=== 拉起 $IFACE ==="
"$SCRIPT_DIR/can/hub-up.sh" "$IFACE"

if ! ip link show "$IFACE" 2>/dev/null | grep -q 'state UP'; then
  echo "[错误] $IFACE 未 UP，请检查 USB-CAN 连接"
  exit 1
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
# shellcheck source=/dev/null
source "$WS_DIR/install/setup.bash"

echo
echo "=== 启动 c620_quad_ros2 ==="
echo "前进测试（另开终端，leg1+leg2 / ID1+ID2）："
echo "  ~/robot_ws/scripts/c620-forward.sh 0.15"
echo "或："
echo "  ros2 topic pub /quad/wheels/cmd std_msgs/msg/Float64MultiArray \\"
echo "    '{data: [0.15, 0.15, 0.0, 0.0]}' --rate 100 \\"
echo "    --qos-reliability reliable --qos-durability transient_local"
echo
exec ros2 run rs_motor_ros2 c620_quad_ros2 --ros-args \
  --params-file "$WS_DIR/config/c620_quad_params.yaml"
