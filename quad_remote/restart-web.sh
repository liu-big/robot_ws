#!/usr/bin/env bash
# 仅重启手机遥控 Web（机器人节点已运行时使用）
set -eo pipefail
REMOTE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "=== 停止旧 Web 遥控 ==="
pkill -f "${REMOTE_DIR}/server.py" 2>/dev/null || true
if command -v fuser >/dev/null 2>&1; then
  fuser -k 8765/tcp 2>/dev/null || true
else
  lsof -ti:8765 | xargs -r kill -9 2>/dev/null || true
fi
sleep 1

echo "=== 启动 Web 遥控 ==="
nohup "${REMOTE_DIR}/run.sh" --host 0.0.0.0 --port 8765 > /tmp/quad_remote.log 2>&1 &
sleep 3

if curl -sf http://127.0.0.1:8765/health >/dev/null; then
  IP="$(hostname -I | awk '{print $1}')"
  echo "就绪: http://${IP}:8765/"
  echo "健康: curl http://127.0.0.1:8765/health"
else
  echo "启动失败，查看 /tmp/quad_remote.log"
  tail -20 /tmp/quad_remote.log
  exit 1
fi
