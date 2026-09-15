#!/bin/bash
# 大疆 C620 电流控制 (can3, 标准帧)
# 用法: c620-send-current.sh <motor_id_1-4> <current_A>
# 例:   c620-send-current.sh 1 0.5   # ID1 输出 0.5A
# 警告: 确保麦轮可自由转动，人在急停位！

set -euo pipefail

MID="${1:?用法: c620-send-current.sh <id_1-4> <current_A>}"
CUR_A="${2:?用法: c620-send-current.sh <id_1-4> <current_A>}"

if (( MID < 1 || MID > 4 )); then
    echo "motor id 必须是 1~4 (在 0x200 帧内)"
    exit 1
fi

# int16: -16384~16384 ↔ -20~20A
# C620 手册: 电流 int16 大端 (高字节在前)
RAW=$(python3 -c "import struct; v=int(float('$CUR_A')/20*16384); v=max(-16384,min(16384,v)); print(struct.pack('>h',v).hex())")
OFFSET=$(( (MID - 1) * 2 ))

# 构建 8 字节 DATA，仅目标电机非零
DATA=$(python3 -c "
raw=bytes.fromhex('$RAW')
off=$OFFSET
d=bytearray(8)
d[off:off+2]=raw
print(''.join(f'{b:02X}' for b in d))
")

IFACE="${C620_CAN:-can3}"
echo "C620 ID=$MID 电流=${CUR_A}A → $IFACE 0x200#$DATA"
read -p "确认发送? [y/N] " -n 1 -r; echo
[[ $REPLY =~ ^[Yy]$ ]] || exit 0

cansend "$IFACE" "200#${DATA}"
echo "已发送。监听反馈: candump -tz $IFACE,20${MID}:7FF"
