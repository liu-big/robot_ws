#!/bin/bash
# 重置麦轮里程计原点
set -eo pipefail
WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_DIR/install/setup.bash"
fi
# shellcheck source=/dev/null
source "$WS_DIR/scripts/ros-env.sh" 2>/dev/null || true

echo "=== reset /quad/odom/reset ==="
ros2 service call /quad/odom/reset std_srvs/srv/Empty {}
