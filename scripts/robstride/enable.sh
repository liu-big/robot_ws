#!/bin/bash
# 灵足 RS03 仅使能（不发送速度/运动指令）
# 用法: rs-enable.sh [motor_id_hex]   默认 01
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IFACE="${CAN_IFACE:-can0}"
HOST_ID="${HOST_ID:-FF}"
MID="${1:-01}"
MID="${MID#0x}"
MID="${MID^^}"

rs_id() {
    local type="$1" host="$2" motor="$3"
    printf '%08X' $(( (type << 24) | (host << 8) | motor ))
}

"$SCRIPT_DIR/can-hub-up.sh" "$IFACE" 2>/dev/null || true

echo "=== $IFACE 灵足 RS03 使能（不转） host=0x$HOST_ID motor=0x$MID ==="
ip -details link show "$IFACE" | grep -E 'state UP|can state' || true

TMPLOG="$(mktemp)"
candump -tz -e "$IFACE" >"$TMPLOG" 2>&1 &
DUMP_PID=$!
sleep 0.2

# 先停，清掉上次速度模式残留
stop_id="$(rs_id 4 "0x$HOST_ID" "0x$MID")"
echo "→ 停止/清状态  0x${stop_id}"
cansend "$IFACE" "${stop_id}#0000000000000000"
sleep 0.3

# 通信类型 3: 使能（不发 0x700A 速度等参数）
en_id="$(rs_id 3 "0x$HOST_ID" "0x$MID")"
echo "→ 使能        0x${en_id}"
cansend "$IFACE" "${en_id}#0000000000000000"
sleep 0.5

kill "$DUMP_PID" 2>/dev/null || true

echo ""
if grep -qE '0280|0200' "$TMPLOG" 2>/dev/null; then
    echo "[✓] 电机已使能（反馈已收到）"
    grep -vE "${stop_id}|${en_id}" "$TMPLOG" | head -5 || true
else
    echo "[?] 已发使能帧，请听/看电机是否上劲（保持力矩）"
    cat "$TMPLOG" | head -8 || true
fi
rm -f "$TMPLOG"

echo ""
echo "停机: ~/robot_ws/scripts/rs-disable.sh $MID"
