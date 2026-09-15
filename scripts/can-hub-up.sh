#!/bin/bash
# 灵足 USB_CANHUB 启动
# 用法:
#   can-hub-up.sh          # 默认只启 can0
#   can-hub-up.sh all      # 启 can0~can4
#   can-hub-up.sh can0 can1
set -euo pipefail

BITRATE=1000000
RESTART_MS=100
TXQLEN=1000

if [[ $# -eq 0 ]]; then
    IFACES=(can0)
elif [[ "$1" == "all" ]]; then
    IFACES=(can0 can1 can2 can3 can4)
else
    IFACES=("$@")
fi

load_can_modules() {
    echo "=== 加载 CAN 内核模块 ==="
    sudo modprobe can 2>/dev/null || true
    sudo modprobe can_raw 2>/dev/null || true
    sudo modprobe can_dev 2>/dev/null || true
    if ! lsmod | grep -q '^gs_usb'; then
        echo "  加载 gs_usb ..."
        sudo modprobe gs_usb 2>/dev/null || true
        sleep 2
    else
        echo "  gs_usb 已加载"
    fi
    sleep 0.5
}

iface_exists() {
    ip link show "$1" &>/dev/null
}

reset_usb_for_iface() {
    local IFACE="$1"
    local devpath usb_port
    devpath=$(readlink -f "/sys/class/net/$IFACE/device" 2>/dev/null) || return 1
    usb_port=$(basename "$(dirname "$devpath")")
    echo "  重置 USB 设备 $usb_port ($IFACE) ..."
    echo "$usb_port" | sudo tee /sys/bus/usb/drivers/usb/unbind >/dev/null 2>&1 || return 1
    sleep 2
    echo "$usb_port" | sudo tee /sys/bus/usb/drivers/usb/bind >/dev/null 2>&1 || return 1
    sleep 2
}

bring_up_iface() {
    local IFACE="$1"
    local attempt err

    for attempt in 1 2; do
        if ! iface_exists "$IFACE"; then
            echo "  等待 $IFACE ($attempt/2) ..."
            sleep 1
            continue
        fi

        sudo ip link set "$IFACE" down 2>/dev/null || true

        if ! err=$(sudo ip link set "$IFACE" type can bitrate "$BITRATE" restart-ms "$RESTART_MS" 2>&1); then
            echo "  $IFACE: 设波特率失败 — $err"
        elif ! err=$(sudo ip link set "$IFACE" txqueuelen "$TXQLEN" 2>&1); then
            echo "  $IFACE: txqueuelen 失败 — $err（继续）"
        fi

        if err=$(sudo ip link set "$IFACE" up 2>&1); then
            echo "  $IFACE UP @ ${BITRATE} bps"
            return 0
        fi
        echo "  $IFACE: ip link up 失败 — $err"

        if [[ "$attempt" -eq 1 ]]; then
            reset_usb_for_iface "$IFACE" || true
        fi
    done

    echo "  错误: $IFACE 无法启动"
    return 1
}

find_working_can() {
    local i
    for i in 0 1 2 3 4; do
        if ip -br link show "can$i" 2>/dev/null | grep -q 'UP'; then
            echo "can$i"
        fi
    done
}

load_can_modules

FOUND=0
FAILED_IFACES=()
for IFACE in "${IFACES[@]}"; do
    if bring_up_iface "$IFACE"; then
        FOUND=$((FOUND + 1))
    else
        FAILED_IFACES+=("$IFACE")
    fi
done

for i in 0 1 2 3 4; do
    keep_up=false
    for IFACE in "${IFACES[@]}"; do
        [[ "$IFACE" == "can$i" ]] && keep_up=true && break
    done
    if [[ "$keep_up" == false ]]; then
        sudo ip link set "can$i" down 2>/dev/null || true
    fi
done

echo ""
echo "=== CAN 状态 ==="
ip -br link show type can 2>/dev/null || ip -br link show | grep can || true

if [[ ${#FAILED_IFACES[@]} -gt 0 ]]; then
    echo ""
    echo "以下接口启动失败: ${FAILED_IFACES[*]}"
    WORKING=$(find_working_can | tr '\n' ' ')
    if [[ -n "$WORKING" ]]; then
        echo "当前可用的 CAN: $WORKING"
        echo "若 C620 不在 can1 上，可把线换到可用口，并改 ros 参数 can_interface:=canX"
    fi
    echo "硬件排查:"
    echo "  1) 拔掉 USB HUB 等 5 秒再插"
    echo "  2) sudo dmesg | tail -30"
    echo "  3) ~/robot_ws/scripts/usb-can-map.sh"
fi

if [[ "$FOUND" -eq 0 ]]; then
    exit 1
fi

if [[ ${#FAILED_IFACES[@]} -gt 0 ]]; then
    exit 1
fi
