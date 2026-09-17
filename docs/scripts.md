# 脚本手册

本页汇总 `robot_ws/scripts/` 与 `quad_remote/` 的启动、调试、急停流程。整机架构见 [README](../README.md)，手机遥控协议见 [quad_remote 说明](quad_remote.md)。

---

## 脚本一览

### robot_ws/scripts/

| 脚本 | 用途 |
|------|------|
| `run-quad-all.sh` | **一键启动** 8 关节 + 麦轮 + 遥控协调（`stop` 停止） |
| `can-hub-up.sh can0 can1 can2` | 加载 gs_usb、启动 CAN（需 sudo） |
| `quad-pub.sh TOPIC [TYPE] [YAML]` | 低延迟发 ROS 话题（`-w 0`） |
| `quad-goto-pose.sh stand\|crouch\|neutral` | 整机预设姿态 |
| `quad-calibrate.sh` | 当前姿态标定为零点 → `config/quad_home.yaml` |
| `estop.sh` | 全机急停（ROS + CAN 失能/清零） |
| `ros-env.sh` | `source` 用：DDS 仅 UDP、清理 SHM |

### quad_remote/

| 脚本 | 用途 |
|------|------|
| `start-all.sh` | **全栈重启**：停旧进程 → `run-quad-all.sh` → Web `:8765` |
| `run.sh` | 仅启动手机遥控 Web（需 ROS 节点已运行） |

---

## 完整上电流程（推荐）

```bash
# 一条命令：CAN + 三 ROS 节点 + 手机 Web
~/quad_remote/start-all.sh
```

输出示例：

```
=== 全部就绪 ===
  手机访问: http://192.168.x.x:8765/
  健康检查: curl http://127.0.0.1:8765/health
  遥控日志: tail -f /tmp/quad_remote.log
```

上电后站起：

```bash
~/robot_ws/scripts/quad-goto-pose.sh stand
```

---

## 分步启动

```bash
# 1. 停止旧进程
pkill -f quad_remote/server.py 2>/dev/null || true
~/robot_ws/scripts/run-quad-all.sh stop
ros2 daemon stop 2>/dev/null || true
find /dev/shm -maxdepth 1 -name '*fastrtps*' -delete 2>/dev/null || true

# 2. ROS 环境
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
source ~/robot_ws/scripts/ros-env.sh

# 3. 机器人
~/robot_ws/scripts/run-quad-all.sh

# 4. 手机遥控
~/quad_remote/run.sh --host 0.0.0.0 --port 8765

# 5. 检查
curl -s http://127.0.0.1:8765/health
ros2 node list | grep -E 'quad_rs|c620_quad|quad_teleop|quad_web'
```

---

## 常用命令

### 姿态

```bash
~/robot_ws/scripts/quad-goto-pose.sh stand
~/robot_ws/scripts/quad-goto-pose.sh neutral
~/robot_ws/scripts/quad-goto-pose.sh crouch
```

### 关节（相对 home 的 Δ rad）

```bash
~/robot_ws/scripts/quad-pub.sh /quad/lift/command std_msgs/msg/Float64 '{data: 0.08}'
~/robot_ws/scripts/quad-pub.sh /quad/crouch/command std_msgs/msg/Float64 '{data: 0.20}'
```

### 麦轮 — 终端遥控

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}' --rate 20 \
  --qos-reliability reliable -w 0
```

### 麦轮 — 直接轮速（leg1+leg2，须 `--rate` 持续发）

```bash
ros2 topic pub /quad/wheels/cmd std_msgs/msg/Float64MultiArray \
  '{data: [0.35, 0.35, 0.0, 0.0]}' --rate 100 --qos-reliability reliable -w 0
```

### 反馈查看

```bash
ros2 topic echo /quad/joint_states --once
ros2 topic echo /quad/wheel_states --once
ros2 topic hz /cmd_vel
```

---

## 标定流程

```bash
# 1. 节点运行中，手动摆好休息姿态
# 2. 标定并写 yaml
~/robot_ws/scripts/quad-calibrate.sh
# 3. 重启
~/robot_ws/scripts/run-quad-all.sh stop
~/robot_ws/scripts/run-quad-all.sh
```

标定文件：`config/quad_home.yaml`（`run-quad-all.sh` 自动加载）。

---

## 急停与停止

```bash
# 软件急停（杀节点 + CAN 失能）
~/robot_ws/scripts/estop.sh

# 仅停止 ROS 节点（不发 CAN 急停帧）
~/robot_ws/scripts/run-quad-all.sh stop

# 停手机 Web
pkill -f quad_remote/server.py
```

**若电机仍在转 → 立即断 24V。**

---

## 日志

| 组件 | 路径 |
|------|------|
| quad_rs | `~/robot_ws/log/quad/quad_rs.log` |
| c620_quad | `~/robot_ws/log/quad/c620_quad.log` |
| teleop | `~/robot_ws/log/quad/teleop.log` |
| 手机遥控 | `/tmp/quad_remote.log` |

```bash
tail -f ~/robot_ws/log/quad/c620_quad.log
tail -f /tmp/quad_remote.log
```

---

## 编译（改 C++ 后）

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rs_motor_ros2
source install/setup.bash
~/quad_remote/start-all.sh
```
