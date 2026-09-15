#!/bin/bash
# 灵足 RS03 停止/失能
# 用法: rs-disable.sh [motor_id_hex ...]   默认 01 05（leg1）
set -eo pipefail

# shellcheck source=../_paths.sh
source "$(cd "$(dirname "$0")/.." && pwd)/_paths.sh"
IFACE="${CAN_IFACE:-can0}"
HOST_ID="${HOST_ID:-FF}"

"$CAN_DIR/hub-up.sh" "$IFACE" 2>/dev/null || true

ids=("$@")
if [[ ${#ids[@]} -eq 0 ]]; then
  ids=(01 05)
fi

for MID in "${ids[@]}"; do
  MID="${MID#0x}"
  MID="${MID^^}"
  stop_id="$(printf '%08X' $(( (4 << 24) | (0x$HOST_ID << 8) | (16#$MID) )) )"
  echo "→ 失能电机 0x$MID  帧 0x${stop_id}"
  cansend "$IFACE" "${stop_id}#0000000000000000"
done
echo "[✓] 已发送失能帧"
