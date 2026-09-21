#!/usr/bin/env bash
set -eo pipefail
REMOTE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="${ROBOT_WORKSPACE:-/home/a/robot_ws}"

echo "=== 停止旧进程 ==="
pkill -f "${REMOTE_DIR}/server.py" 2>/dev/null || true
# Old web server can survive pkill and block a fresh ROS bridge after robot restart.
if command -v fuser >/dev/null 2>&1; then
  fuser -k 8765/tcp 2>/dev/null || true
else
  lsof -ti:8765 | xargs -r kill -9 2>/dev/null || true
fi
"${ROBOT_WS}/scripts/run-quad-all.sh" stop 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
find /dev/shm -maxdepth 1 -name '*fastrtps*' -delete 2>/dev/null || true
sleep 2

echo "=== 启动机器人节点 ==="
"${ROBOT_WS}/scripts/run-quad-all.sh"

echo "=== 启动手机遥控 Web ==="
nohup "${REMOTE_DIR}/run.sh" --host 0.0.0.0 --port 8765 > /tmp/quad_remote.log 2>&1 &
sleep 4

if curl -sf http://127.0.0.1:8765/health >/dev/null; then
  IP="$(hostname -I | awk '{print $1}')"
  echo ""
  echo "=== 全部就绪 ==="
  echo "  手机访问: http://${IP}:8765/"
  echo "  健康检查: curl http://127.0.0.1:8765/health"
  echo "  遥控日志: tail -f /tmp/quad_remote.log"
else
  echo "遥控服务启动失败，查看 /tmp/quad_remote.log"
  tail -20 /tmp/quad_remote.log
  exit 1
fi
