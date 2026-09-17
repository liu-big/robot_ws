#!/bin/bash
# 四足 ROS2 环境：禁用 SHM、清理僵尸 fastrtps 锁文件
# 用法: source ~/robot_ws/scripts/ros-env.sh
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_udp_only.xml"
export ROS_LOCALHOST_ONLY=1

# 清理残留的共享内存端口（会导致 open_and_lock_file failed）
if compgen -G "/dev/shm/*fastrtps*" >/dev/null 2>&1; then
  stale=$(find /dev/shm -maxdepth 1 -name '*fastrtps*' 2>/dev/null | wc -l)
  if [[ "$stale" -gt 20 ]]; then
    find /dev/shm -maxdepth 1 -name '*fastrtps*' -delete 2>/dev/null || true
  fi
fi

# ros2 topic pub 默认 -w 1 会等 DDS 发现订阅者，造成卡顿；脚本里用 -w 0 立即发送
export QUAD_TOPIC_QOS="--qos-reliability reliable -w 0"
