#!/bin/bash
# C620 / can1 诊断（参考 RoboMaster 社区 + SocketCAN 排障）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IFACE="${1:-can1}"
LISTEN_SEC=5

echo "=============================================="
echo " C620 $IFACE 诊断"
echo " 网络案例要点:"
echo "  - C620 上电后应周期性发 0x201 (ID=1)"
echo "  - 控制帧 0x200, 电流 int16 大端"
echo "  - 单节点无 ACK → TX 错误 → ERROR-PASSIVE / BUS-OFF"
echo "  - 查: 24V、CAN_H/L/GND、终端电阻(~60Ω)、电调 ID"
echo "=============================================="
echo

"$SCRIPT_DIR/can-hub-up.sh" "$IFACE"

echo "=== 1) 接口状态（发帧前）==="
ip -details -statistics link show "$IFACE" | grep -E 'state UP|can state|bitrate|bus-off|error-pass|NO-CARRIER' || true

echo
echo "=== 2) 被动监听 ${LISTEN_SEC}s（不接指令，应有 0x201）==="
echo "    若完全无帧 → 电调未上电 / 未接 can1 / 线序 / 电阻"
timeout "$LISTEN_SEC" candump -tz "${IFACE},201:7FF" 2>/dev/null | head -20 || true
FRAME_COUNT=$(timeout "$LISTEN_SEC" candump -tz "$IFACE" 2>/dev/null | wc -l | tr -d ' ')
echo "    总帧数(约): ${FRAME_COUNT:-0}"

echo
echo "=== 3) 发 0A 保活 + 再听 3s ==="
cansend "$IFACE" 200#0000000000000000 || echo "    cansend 失败（可能已 BUS-OFF）"
timeout 3 candump -tz "${IFACE},201:7FF" 2>/dev/null | head -10 || true

echo
echo "=== 4) 发后总线状态（看是否 BUS-OFF）==="
ip -details link show "$IFACE" | grep -E 'can state|bus-off|error-pass' || true

echo
echo "=== 5) USB-CAN 映射 ==="
"$SCRIPT_DIR/usb-can-map.sh" 2>/dev/null | tail -15 || true

echo
echo "=== 6) 本机 loopback 自检（与 C620 无关，验证适配器）==="
sudo ip link set "$IFACE" down
sudo ip link set "$IFACE" type can bitrate 1000000 loopback on restart-ms 100
sudo ip link set "$IFACE" up
sleep 0.2
cansend "$IFACE" 123#DEADBEEF
LB=$(timeout 1 candump -tz "$IFACE" 2>/dev/null | wc -l | tr -d ' ')
sudo ip link set "$IFACE" down
sudo ip link set "$IFACE" type can bitrate 1000000 loopback off restart-ms 100
sudo ip link set "$IFACE" up
echo "    loopback 收到帧数: $LB (应为 1，说明 gs_usb 正常)"

echo
echo "=== 7) 手动试转（确认接线后再执行）==="
echo "    cansend $IFACE 200#0064000000000000   # ID1 约 0.5A (大端 0x0064=100)"
echo "    ~/robot_ws/scripts/c620-send-current.sh 1 0.3"
echo
echo "=== 判定 ==="
if [[ "$FRAME_COUNT" -gt 0 ]]; then
  echo "  [OK] 总线上有数据，可继续 ros2 run rs_motor_ros2 c620_ros2"
else
  echo "  [FAIL] 无 0x201 → 优先查硬件，不是 ROS 代码问题"
  echo "  - C620 接 HUB 第 2 路 (can1)，不要接 can0"
  echo "  - 24V 上电，SET 设 ID=1"
  echo "  - PWM 口不要接线（CAN/PWM 二选一）"
  echo "  - HUB can1 拨码电阻 + C620 板载电阻（总线两端各 120Ω）"
  echo "  - 万用表断电测 CAN_H-CAN_L ≈ 60Ω"
fi
