#!/bin/bash
# 灵足 RS03 探测（can0，不发运动指令）
#
# 29-bit 扩展帧 ID 格式（与手册 / ROS 例程一致）:
#   (通信类型 << 24) | (主机ID << 8) | 电机ID
# 例: 查询 ID=0x7F, 主机=0xFF → 0x0000FF7F
#
# 用法: rs-probe.sh [motor_id_hex ...]  默认 01 05（leg1）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IFACE="${CAN_IFACE:-can0}"
HOST_ID="${HOST_ID:-FF}"
COMM_GET_ID=0

"$SCRIPT_DIR/can-hub-up.sh" "$IFACE" 2>/dev/null || true

ids=("$@")
if [[ ${#ids[@]} -eq 0 ]]; then
    ids=(01 05)
fi

rs_can_id() {
    local type="$1" host="$2" motor="$3"
    printf '%08X' $(( (type << 24) | (host << 8) | motor ))
}

echo "=== $IFACE 灵足 RS03 探测 (host=0x$HOST_ID) ==="
ip -details link show "$IFACE" | grep -E 'state UP|can state' || true
echo ""
echo "29-bit ID = (type<<24) | (host<<8) | motor"
echo ""

TMPLOG="$(mktemp)"
candump -tz -e "$IFACE" >"$TMPLOG" 2>&1 &
DUMP_PID=$!
sleep 0.2

for mid in "${ids[@]}"; do
    mid="${mid#0x}"
    mid="${mid^^}"
    can_id="$(rs_can_id "$COMM_GET_ID" "0x$HOST_ID" "0x$mid")"
    echo "→ 查询电机 ID=0x$mid  TX 扩展帧 0x${can_id}"
    cansend "$IFACE" "${can_id}#0000000000000000"
    sleep 0.3
done

sleep 1
kill "$DUMP_PID" 2>/dev/null || true

echo ""
if [[ -s "$TMPLOG" ]]; then
    echo "=== 捕获帧 ==="
    cat "$TMPLOG"
    echo ""
    if grep -qvE '0000FF(7F|01|02|FD)' "$TMPLOG" 2>/dev/null; then
        echo "[✓] 有电机应答（见上方 RX 帧，非本机发出的 TX）"
    fi
else
    echo "(无反馈 — 检查供电、接线、HUB 路号)"
fi
rm -f "$TMPLOG"
