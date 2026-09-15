#!/bin/bash
# 转发到统一入口（quad_rs_ros2 + c620_ros2）
exec "$(cd "$(dirname "$0")/.." && pwd)/run-leg1-all.sh" "$@"
