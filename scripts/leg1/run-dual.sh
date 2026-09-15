#!/bin/bash
# 第一条腿双灵足 ROS 节点（can0, ID 1 + 5）
set -eo pipefail
# shellcheck source=../_paths.sh
source "$(cd "$(dirname "$0")/.." && pwd)/_paths.sh"

pkill -9 leg1_dual_ros2 2>/dev/null || true
pkill -9 -f rs_motor_ros2 2>/dev/null || true
pkill -9 leg1_dual_pose 2>/dev/null || true
sleep 1
rm -f /tmp/leg1_dual_ros2.lock

"$CAN_DIR/hub-up.sh" can0
source_ros

echo "=== leg1_dual_ros2 (can0, lift=1 crouch=5) ==="
echo "  /leg1/lift/command  /leg1/crouch/command"
echo "  /leg1/goto/stand    /leg1/goto/crouch"
echo "  示例: $LEG1_DIR/goto-pose.sh stand"
echo ""

exec ros2 run rs_motor_ros2 leg1_dual_ros2 --ros-args \
  --params-file "$WS_DIR/config/ros/leg1_dual_params.yaml"
