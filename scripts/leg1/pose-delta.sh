#!/bin/bash
# 通过 ROS 话题做小幅度位姿（需 leg1_dual_ros2 已运行）
# 用法: leg1-pose-ros.sh [lift_delta_rad] [crouch_delta_rad]
# 默认: 抬升 +0.08, 趴下 +0.15
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LIFT_D="${1:-0.08}"
CROUCH_D="${2:-0.15}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_DIR/install/setup.bash"
fi

read_pos() {
  local topic="$1"
  python3 - "$topic" <<'PY'
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

topic = sys.argv[1]
rclpy.init()
node = Node("leg1_pose_reader")
done = {"v": None}

def cb(msg: JointState):
    if msg.position:
        done["v"] = float(msg.position[0])

sub = node.create_subscription(JointState, topic, cb, 10)
import time
t0 = time.time()
while done["v"] is None and time.time() - t0 < 3.0:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
if done["v"] is None:
    sys.exit(1)
print(done["v"])
PY
}

if ! ros2 node list 2>/dev/null | grep -q leg1_dual_node; then
  echo "[错误] leg1_dual_ros2 未运行。先执行:"
  echo "  ~/robot_ws/scripts/run-leg1-dual.sh"
  exit 1
fi

echo "=== 读取当前位置 ==="
LIFT_POS="$(read_pos /leg1/lift/state)"
CROUCH_POS="$(read_pos /leg1/crouch/state)"
echo "  抬升  ${LIFT_POS} rad"
echo "  趴下  ${CROUCH_POS} rad"

LIFT_TGT="$(python3 -c "print(${LIFT_POS} + ${LIFT_D})")"
CROUCH_TGT="$(python3 -c "print(${CROUCH_POS} + ${CROUCH_D})")"

echo ""
echo "=== 发送目标 (Δ 抬升=${LIFT_D}, 趴下=${CROUCH_D}) ==="
echo "  抬升  → ${LIFT_TGT} rad"
echo "  趴下  → ${CROUCH_TGT} rad"

ros2 topic pub /leg1/lift/command std_msgs/msg/Float64 "{data: ${LIFT_TGT}}" --once \
  --qos-reliability reliable --qos-durability transient_local
ros2 topic pub /leg1/crouch/command std_msgs/msg/Float64 "{data: ${CROUCH_TGT}}" --once \
  --qos-reliability reliable --qos-durability transient_local

echo ""
echo "等待到位..."
sleep 3
echo "=== 反馈 ==="
read_pos /leg1/lift/state | xargs -I{} echo "  抬升  {} rad"
read_pos /leg1/crouch/state | xargs -I{} echo "  趴下  {} rad"
echo ""
echo "锁定: ros2 topic pub /leg1/hold std_msgs/msg/Empty '{}' --once"
