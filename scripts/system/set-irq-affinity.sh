#!/bin/bash
# 绑定网卡 IRQ 到指定 CPU（插网线后运行）
# 用法: sudo set-irq-affinity.sh

set -euo pipefail

bind_nic() {
    local iface="$1" cpu="$2"
    local irq path
    path="/sys/class/net/$iface/device/msi_irqs"
    if [[ ! -d "$path" ]]; then
        echo "跳过 $iface: 无 MSI IRQ 或未连接"
        return
    fi
    for irq in "$path"/*; do
        local n=$(basename "$irq")
        echo "$cpu" > "/proc/irq/$n/smp_affinity_list"
        echo "$iface IRQ $n → CPU$cpu"
    done
}

# 有线网卡中断绑到 CPU0，避免与 RT 线程抢 CPU2/3
bind_nic enp2s0 0
bind_nic enp4s0 0

echo "完成。WiFi 保持默认。"
