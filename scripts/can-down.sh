#!/bin/bash
# 关闭 CAN 总线
set -euo pipefail
for IFACE in can0 can1 can2; do
    if ip link show "$IFACE" &>/dev/null; then
        sudo ip link set "$IFACE" down
        echo "$IFACE down"
    fi
done
