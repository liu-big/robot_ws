#!/bin/bash
# USB-CAN 插入后运行，记录哪个 USB 口对应哪条 CAN
echo "=== dmesg (gs_usb / can) ==="
dmesg 2>/dev/null | grep -iE 'gs_usb|can[0-9]|candle|usb.*can' | tail -20 || sudo dmesg | grep -iE 'gs_usb|can[0-9]|candle' | tail -20

echo
echo "=== CAN 接口详情 ==="
ip -details link show type can 2>/dev/null || echo "无 CAN 接口"

echo
echo "=== USB 设备 ==="
lsusb 2>/dev/null | grep -iE 'can|candle|gs_usb|1d50|16d0' || lsusb | tail -10

echo
echo "=== USB 拓扑 ==="
lsusb -t

echo
echo "提示: 把 can0 固定在 Bus 03 (USB3) 的某个口，can1 固定另一个口，不要换。"
