#!/bin/bash
# 低延迟话题发布（-w 0 不等 DDS 发现，避免卡顿）
# 用法:
#   quad-pub.sh /quad/goto/neutral
#   quad-pub.sh /quad/lift/command std_msgs/msg/Float64 '{data: 0.45}'
#   quad-pub.sh /quad/legs/command std_msgs/msg/Float64MultiArray '{data: [0,0,0,0,0,0,0,0]}'
set -eo pipefail

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TOPIC="${1:?用法: quad-pub.sh TOPIC [TYPE] [YAML]}"

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WS_DIR/install/setup.bash" 2>/dev/null || true
source "$WS_DIR/scripts/ros-env.sh" 2>/dev/null || true
set -u

if [[ $# -ge 3 ]]; then
  TYPE="$2"
  YAML="$3"
elif [[ $# -eq 2 ]]; then
  TYPE="$2"
  YAML='{}'
else
  TYPE="std_msgs/msg/Empty"
  YAML='{}'
fi

exec ros2 topic pub "$TOPIC" "$TYPE" "$YAML" --once ${QUAD_TOPIC_QOS:---qos-reliability reliable -w 0}
