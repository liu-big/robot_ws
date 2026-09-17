#!/usr/bin/env bash
set -eo pipefail
REMOTE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WORKSPACE="${ROBOT_WORKSPACE:-/home/a/robot_ws}"
if [[ " $* " != *" --demo "* ]]; then
  source /opt/ros/humble/setup.bash
  source "$ROBOT_WORKSPACE/install/setup.bash"
  export ROS_LOCALHOST_ONLY=1
  export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOT_WORKSPACE/config/fastdds_udp_only.xml"
fi
export PYTHONPATH="$REMOTE_DIR/.deps${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 "$REMOTE_DIR/server.py" --workspace "$ROBOT_WORKSPACE" "$@"
