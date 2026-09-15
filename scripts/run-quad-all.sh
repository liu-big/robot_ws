#!/bin/bash
# 四足全机一键启动：quad_rs(8关节) + c620_quad(麦轮) + quad_teleop(遥控)
# 用法: run-quad-all.sh [stop]
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="${WS_DIR}/log/quad"
mkdir -p "$LOG_DIR"

PID_RS="${LOG_DIR}/quad_rs.pid"
PID_C620="${LOG_DIR}/c620_quad.pid"
PID_TEL="${LOG_DIR}/teleop.pid"

stop_all() {
  echo "=== 停止四足全机节点 ==="
  for f in "$PID_RS" "$PID_C620" "$PID_TEL"; do
    [[ -f "$f" ]] && kill "$(cat "$f")" 2>/dev/null || true
  done
  rm -f "$PID_RS" "$PID_C620" "$PID_TEL"
  pkill -9 quad_rs_ros2 2>/dev/null || true
  pkill -9 c620_quad_ros2 2>/dev/null || true
  pkill -9 c620_ros2 2>/dev/null || true
  pkill -9 quad_teleop_ros2 2>/dev/null || true
  pkill -9 leg1_dual_ros2 2>/dev/null || true
  rm -f /tmp/quad_rs_ros2.lock /tmp/c620_quad_ros2.lock /tmp/c620_ros2.lock
  echo "已停止"
}

[[ "${1:-}" == "stop" ]] && stop_all && exit 0

stop_all
sleep 1

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_DIR/install/setup.bash"
fi

"$SCRIPT_DIR/can-hub-up.sh" can0 can1 can3

echo "=== quad_rs_ros2 (8×灵足, active_legs=1~4) ==="
nohup ros2 run rs_motor_ros2 quad_rs_ros2 --ros-args \
  --params-file "$WS_DIR/config/quad_full_params.yaml" \
  >"$LOG_DIR/quad_rs.log" 2>&1 &
echo $! >"$PID_RS"

echo "=== c620_quad_ros2 (can3 麦轮) ==="
nohup ros2 run rs_motor_ros2 c620_quad_ros2 --ros-args \
  --params-file "$WS_DIR/config/c620_quad_params.yaml" \
  >"$LOG_DIR/c620_quad.log" 2>&1 &
echo $! >"$PID_C620"

echo "=== quad_teleop_ros2 (遥控协调) ==="
nohup ros2 run rs_motor_ros2 quad_teleop_ros2 --ros-args \
  --params-file "$WS_DIR/config/quad_teleop_params.yaml" \
  >"$LOG_DIR/teleop.log" 2>&1 &
echo $! >"$PID_TEL"

echo "等待节点就绪..."
for _ in $(seq 1 20); do
  ros2 node list 2>/dev/null | grep -q quad_rs_node && \
  ros2 node list 2>/dev/null | grep -q c620_quad_node && \
  ros2 node list 2>/dev/null | grep -q quad_teleop_node && break
  sleep 1
done

echo ""
echo "=== 四足全机已启动 ==="
echo "  日志: $LOG_DIR/"
echo "  站起: ~/robot_ws/scripts/quad-goto-pose.sh stand"
echo "  遥控: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \\"
echo "          '{linear: {x: 0.3}, angular: {z: 0.0}}' --rate 20 \\"
echo "          --qos-reliability reliable --qos-durability transient_local"
echo "  急停: ros2 topic pub /quad/e_stop std_msgs/msg/Empty '{}' --once \\"
echo "          --qos-reliability reliable --qos-durability transient_local"
echo "  停止: ~/robot_ws/scripts/run-quad-all.sh stop"
