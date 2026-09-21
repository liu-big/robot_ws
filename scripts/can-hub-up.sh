#!/bin/bash
# 灵足 USB_CANHUB 启动
# 用法:
#   can-hub-up.sh          # 默认只启 can0
#   can-hub-up.sh all      # 启 can2→can0→can1→can3→can4（麦轮口优先）
#   can-hub-up.sh can0 can1
# 多口同时启时按 CAN_START_ORDER 逐个 up，避免 USB 枚举未完成导致 NO-CARRIER。
set -euo pipefail

BITRATE=1000000
RESTART_MS=100
TXQLEN=1000
# 麦轮 can2 最先；腿 can0/can1 其次（与 run-quad-all.sh 节点启动顺序一致）
CAN_START_ORDER=(can2 can0 can1 can3 can4)
IFACE_SETTLE_S=0.6

if [[ $# -eq 0 ]]; then
    REQUESTED=(can0)
elif [[ "$1" == "all" ]]; then
    REQUESTED=(can2 can0 can1 can3 can4)
else
    REQUESTED=("$@")
fi

sort_ifaces() {
    local -a sorted=()
    local want iface
    for want in "${CAN_START_ORDER[@]}"; do
        for iface in "${REQUESTED[@]}"; do
            if [[ "$iface" == "$want" ]]; then
                sorted+=("$iface")
                break
            fi
        done
    done
    for iface in "${REQUESTED[@]}"; do
        local seen=false
        for want in "${sorted[@]}"; do
            [[ "$want" == "$iface" ]] && seen=true && break
        done
        [[ "$seen" == false ]] && sorted+=("$iface")
    done
    IFACES=("${sorted[@]}")
}

sort_ifaces
if [[ ${#IFACES[@]} -gt 1 ]]; then
    echo "  启动顺序: ${IFACES[*]}"
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
        # 多路 USB-CAN 在模块已加载时仍可能未枚举完，首次 up 易出现 NO-CARRIER
        if [[ ${#IFACES[@]} -gt 1 ]]; then
            sleep 2
        else
            sleep 0.5
        fi
    fi
}

iface_exists() {
    ip link show "$1" &>/dev/null
}

iface_operational() {
    local IFACE="$1"
    local state flags
    [[ -r "/sys/class/net/$IFACE/operstate" ]] || return 1
    state=$(<"/sys/class/net/$IFACE/operstate")
    flags=$(ip -d link show "$IFACE" 2>/dev/null | sed -n 's/.*<//p' | tr -d '>')
    [[ "$state" == "up" ]] && [[ "$flags" != *NO-CARRIER* ]]
}

wait_for_ifaces() {
    local IFACE deadline=$((SECONDS + 12)) missing
    while (( SECONDS < deadline )); do
        missing=()
        for IFACE in "${IFACES[@]}"; do
            iface_exists "$IFACE" || missing+=("$IFACE")
        done
        if [[ ${#missing[@]} -eq 0 ]]; then
            return 0
        fi
        echo "  等待接口出现: ${missing[*]} ..."
        sleep 0.5
    done
    echo "  超时: 未找到 ${missing[*]}"
    return 1
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

    for attempt in 1 2 3; do
        if ! iface_exists "$IFACE"; then
            echo "  等待 $IFACE ($attempt/3) ..."
            sleep 1
            continue
        fi

        sudo ip link set "$IFACE" down 2>/dev/null || true

        if ! err=$(sudo ip link set "$IFACE" type can bitrate "$BITRATE" restart-ms "$RESTART_MS" 2>&1); then
            echo "  $IFACE: 设波特率失败 — $err"
        elif ! err=$(sudo ip link set "$IFACE" txqueuelen "$TXQLEN" 2>&1); then
            echo "  $IFACE: txqueuelen 失败 — $err（继续）"
        fi

        if ! err=$(sudo ip link set "$IFACE" up 2>&1); then
            echo "  $IFACE: ip link up 失败 — $err"
        else
            sleep 0.3
            if iface_operational "$IFACE"; then
                echo "  $IFACE UP @ ${BITRATE} bps"
                return 0
            fi
            echo "  $IFACE: 已 up 但 NO-CARRIER，重试 ($attempt/3) ..."
        fi

        if [[ "$attempt" -lt 3 ]]; then
            reset_usb_for_iface "$IFACE" || sleep "$attempt"
        fi
    done

    echo "  错误: $IFACE 无法启动（持续 NO-CARRIER 或不存在）"
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
wait_for_ifaces || true

FOUND=0
FAILED_IFACES=()
for IFACE in "${IFACES[@]}"; do
    if bring_up_iface "$IFACE"; then
        FOUND=$((FOUND + 1))
        if [[ ${#IFACES[@]} -gt 1 ]]; then
            sleep "$IFACE_SETTLE_S"
        fi
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
