# 脚本手册

## 脚本一览（7 个）

| 脚本 | 用途 |
|------|------|
| `run-quad-all.sh` | **一键启动** 8 关节 + 麦轮 + 遥控协调（`stop` 停止） |
| `can-hub-up.sh can0 can1 can2` | 加载 gs_usb、启动 CAN |
| `quad-pub.sh TOPIC [TYPE] [YAML]` | 低延迟发 ROS 话题（`-w 0`） |
| `quad-goto-pose.sh stand\|crouch\|neutral` | 整机预设姿态 |
| `quad-calibrate.sh` | 当前姿态标定为零点 → `config/quad_home.yaml` |
| `estop.sh` | 全机急停（ROS + CAN 失能/清零） |
| `ros-env.sh` | `source` 用：DDS 仅 UDP、清理 SHM |

---

## 每次上电

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
source ~/robot_ws/scripts/ros-env.sh

~/robot_ws/scripts/run-quad-all.sh
```

---

## 常用命令

```bash
# 姿态
~/robot_ws/scripts/quad-goto-pose.sh stand
~/robot_ws/scripts/quad-goto-pose.sh neutral

# 关节同步（相对 home 的 Δ rad）
~/robot_ws/scripts/quad-pub.sh /quad/lift/command std_msgs/msg/Float64 '{data: 0.45}'
~/robot_ws/scripts/quad-pub.sh /quad/crouch/command std_msgs/msg/Float64 '{data: 0.20}'

# 麦轮遥控
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}' --rate 20 \
  --qos-reliability reliable -w 0

# 麦轮直接（leg1+leg2，须 --rate 持续发）
ros2 topic pub /quad/wheels/cmd std_msgs/msg/Float64MultiArray \
  '{data: [0.35, 0.35, 0.0, 0.0]}' --rate 100 --qos-reliability reliable -w 0

# 急停
~/robot_ws/scripts/estop.sh

# 停止节点
~/robot_ws/scripts/run-quad-all.sh stop
```

---

## 标定流程

```bash
# 1. 手动摆好休息姿态
# 2. 标定并写 yaml
~/robot_ws/scripts/quad-calibrate.sh
# 3. 重启节点
~/robot_ws/scripts/run-quad-all.sh stop
~/robot_ws/scripts/run-quad-all.sh
```
