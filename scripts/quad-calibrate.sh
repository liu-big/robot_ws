#!/bin/bash
# 灵足 8 关节零点标定 — 把当前姿态设为 home，并写入 config/quad_home.yaml
#
# 用法:
#   1. 手动把四条腿摆到「中性/休息」姿态
#   2. ~/robot_ws/scripts/quad-calibrate.sh
#   3. 重启 quad_rs 节点使 home 生效
set -eo pipefail

WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${WS_DIR}/config/quad_home.yaml"
QOS="--qos-reliability reliable --qos-durability transient_local"

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
# shellcheck source=/dev/null
source "$WS_DIR/install/setup.bash"
set -u

if ! ros2 node list 2>/dev/null | grep -q quad_rs_node; then
  echo "[错误] quad_rs_ros2 未运行"
  echo "  先: ~/robot_ws/scripts/run-quad-all.sh"
  exit 1
fi

echo "=== 灵足关节标定 ==="
echo "请确认四条腿已在目标「零点」姿态，3 秒后开始记录..."
sleep 3

echo "→ 发送 /quad/calibrate"
ros2 topic pub /quad/calibrate std_msgs/msg/Empty '{}' --once $QOS
sleep 1

TMP="$(mktemp)"
if ! timeout 8 ros2 topic echo /quad/joint_states --once >"$TMP" 2>/dev/null; then
  echo "[错误] 未收到 /quad/joint_states，检查 CAN 与电机上电"
  rm -f "$TMP"
  exit 1
fi

python3 - "$TMP" "$OUT" <<'PY'
import re, sys

src, out = sys.argv[1], sys.argv[2]
text = open(src, encoding="utf-8", errors="replace").read()

name_block = text.split("name:", 1)[1].split("position:", 1)[0]
names = re.findall(r"-\s*(\S+)", name_block)
pos_block = text.split("position:", 1)[1].split("velocity:", 1)[0]
positions = [float(x) for x in re.findall(r"-?\d+\.\d+", pos_block)]

if len(names) != len(positions):
    print(f"[错误] 解析失败 names={len(names)} pos={len(positions)}", file=sys.stderr)
    sys.exit(1)

mapping = {}
for n, p in zip(names, positions):
    # leg1_lift -> home_leg1_lift
    mapping[f"home_{n}"] = p

lines = [
    "# 灵足关节零点标定 — 由 quad-calibrate.sh 自动生成",
    "# command 话题发相对 home 的偏移 (rad)；preset stand/crouch 同理",
    "quad_rs_node:",
    "  ros__parameters:",
]
for leg in range(1, 5):
    for joint in ("lift", "crouch"):
        key = f"home_leg{leg}_{joint}"
        val = mapping.get(key, -999.0)
        lines.append(f"    {key}: {val:.6f}")

open(out, "w", encoding="utf-8").write("\n".join(lines) + "\n")
print(f"已写入 {out}")
for k in sorted(mapping):
    print(f"  {k}: {mapping[k]:.4f} rad")
PY

rm -f "$TMP"

echo ""
echo "=== 下一步 ==="
echo "  重启节点加载标定:"
echo "    ~/robot_ws/scripts/run-quad-all.sh stop"
echo "    ~/robot_ws/scripts/run-quad-all.sh"
echo ""
echo "  测试相对指令（抬升 +0.05 rad）:"
echo "    ros2 topic pub /leg1/lift/command std_msgs/msg/Float64 '{data: 0.05}' --once $QOS"
