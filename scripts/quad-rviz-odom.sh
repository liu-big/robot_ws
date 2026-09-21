#!/bin/bash
# RViz2：显示 odom TF、轨迹与 /odom
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

RVIZ_CFG="$WS_DIR/config/quad_odom.rviz"
if [[ ! -f "$RVIZ_CFG" ]]; then
  echo "[错误] 缺少 $RVIZ_CFG"
  exit 1
fi
echo "=== RViz2 odom 可视化 ==="
echo "  Fixed Frame: odom"
echo "  轨迹: /odom/path"
echo "  重置: ~/robot_ws/scripts/quad-odom-reset.sh"
exec rviz2 -d "$RVIZ_CFG"
