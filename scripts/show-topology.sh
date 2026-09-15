#!/bin/bash
# 显示 CPU 拓扑与推荐亲和性
echo "=== CPU 拓扑 ==="
for i in /sys/devices/system/cpu/cpu[0-9]*; do
    n=$(basename "$i")
    core=$(cat "$i/topology/core_id")
    sib=$(cat "$i/topology/thread_siblings_list")
    echo "CPU${n#cpu}  core=$core  siblings=$sib"
done

echo
echo "=== 推荐分配 (i5-4200U 2C/4T) ==="
echo "CPU0,1  → Linux / ROS2 / SSH / 网络"
echo "CPU1    → Safety (SCHED_FIFO 70)"
echo "CPU2    → CAN IO (SCHED_FIFO 80)"
echo "CPU3    → servo_rt 500Hz (SCHED_FIFO 90)"
echo
echo "=== 当前调度策略 ==="
ps -eLo pid,tid,class,rtprio,psr,comm 2>/dev/null | grep -E 'servo|can|FIFO' | head -10 || true

echo
echo "=== USB 拓扑 ==="
lsusb -t 2>/dev/null || echo "lsusb 不可用"

echo
echo "=== CAN 接口 ==="
ip -br link show type can 2>/dev/null || echo "无 CAN 接口（USB-CAN 未插入）"
