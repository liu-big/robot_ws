#!/bin/bash
# 第一条腿一键启动：quad_rs_ros2(can0+can1) + c620_ros2(can3)
# 用法: run-leg1-all.sh [stop]
# 兼容 leg1 话题: /leg1/goto/*  /leg1/hold  /leg1/lift|crouch/*
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="${WS_DIR}/log/leg1"
PID_RS="${LOG_DIR}/quad_rs.pid"
PID_C620="${LOG_DIR}/c620.pid"

stop_all() {
  echo "=== 停止 leg1 节点 ==="
  [[ -f "$PID_RS" ]] && kill "$(cat "$PID_RS")" 2>/dev/null || true
  [[ -f "$PID_C620" ]] && kill "$(cat "$PID_C620")" 2>/dev/null || true
  rm -f "$PID_RS" "$PID_C620"
  pkill -9 quad_rs_ros2 2>/dev/null || true
  pkill -9 leg1_dual_ros2 2>/dev/null || true
  pkill -9 c620_ros2 2>/dev/null || true
  pkill -9 -f rs_motor_ros2 2>/dev/null || true
  rm -f /tmp/quad_rs_ros2.lock /tmp/leg1_dual_ros2.lock /tmp/c620_ros2.lock
  echo "已停止"
}

[[ "${1:-}" == "stop" ]] && stop_all && exit 0

stop_all
sleep 1
mkdir -p "$LOG_DIR"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_DIR/install/setup.bash"
fi

"$SCRIPT_DIR/can-hub-up.sh" can0 can1 can3

echo "=== 启动 quad_rs_ros2 (抬升 can0 + 拉杆 can1, active_legs=[1]) ==="
nohup ros2 run rs_motor_ros2 quad_rs_ros2 --ros-args \
  --params-file "$WS_DIR/config/quad_rs_params.yaml" \
  >"$LOG_DIR/quad_rs.log" 2>&1 &
echo $! >"$PID_RS"

echo "=== 启动 c620_ros2 (can3, 空载摩擦 ff≥1A) ==="
nohup ros2 run rs_motor_ros2 c620_ros2 --ros-args \
  --params-file "$WS_DIR/config/c620_params.yaml" \
  >"$LOG_DIR/c620.log" 2>&1 &
echo $! >"$PID_C620"

echo "等待节点就绪..."
for _ in $(seq 1 15); do
  ros2 node list 2>/dev/null | grep -q quad_rs_node && \
  ros2 node list 2>/dev/null | grep -q c620_node && break
  sleep 1
done

echo ""
echo "=== 第一条腿已启动 ==="
echo "  日志: $LOG_DIR/quad_rs.log  $LOG_DIR/c620.log"
echo "  姿态: ~/robot_ws/scripts/leg1-goto-pose.sh stand"
echo "  停止: ~/robot_ws/scripts/run-leg1-all.sh stop"
