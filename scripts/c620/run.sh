#!/bin/bash
# 一键启动 C620 节点（默认 can1）
# 用法: ~/robot_ws/scripts/run-c620.sh [can1]
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
IFACE="${1:-can1}"

echo "=== 停止旧进程 ==="
pkill -9 c620_ros2 2>/dev/null || true
sleep 1
rm -f /tmp/c620_ros2.lock
# 清理可能堵塞的 CAN TX 队列
sudo ip link set "$IFACE" down 2>/dev/null || true
sleep 0.3

echo "=== 启动 $IFACE ==="
"$SCRIPT_DIR/can-hub-up.sh" "$IFACE"

if ! cansend "$IFACE" 200#0000000000000000 2>/dev/null; then
  echo "[错误] CAN 发送失败，队列可能堵塞。正在重试 down/up ..."
  sudo ip link set "$IFACE" down
  sleep 0.5
  sudo ip link set "$IFACE" up
  sleep 0.5
  cansend "$IFACE" 200#0000000000000000
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_DIR/install/setup.bash"
fi

echo
echo "=== 启动 c620_ros2 (can=$IFACE) ==="
echo "另开终端发速度:"
echo "  source $WS_DIR/install/setup.bash"
echo "  ros2 topic pub /c620/velocity_command std_msgs/msg/Float64 '{data: 5.0}' --rate 20"
echo
exec ros2 run rs_motor_ros2 c620_ros2 --ros-args \
  -p "can_interface:=${IFACE}" \
  -p can_iface_reset_enable:=true
