# 脚本与命令手册

> **can0** 灵足抬升 ID 1~4 ｜ **can1** 灵足拉杆 ID 5~8 ｜ **can2** C620 四轮 ID 1~4  
> 详见 [`config/quadruped.yaml`](../config/quadruped.yaml)

---

## 每次测试前

```bash
source ~/robot_ws/install/setup.bash

# 杀旧进程
~/robot_ws/scripts/run-leg1-all.sh stop

# 启 CAN（can0 + can1 + can2 一起）
~/robot_ws/scripts/can-hub-up.sh can0 can1 can2
ip -br link show can0 can1 can2
```

---

## 脚本一览

### CAN 总线

| 脚本 | 用途 |
|------|------|
| `can-hub-up.sh can0 can1 can2` | **主入口** — 加载 gs_usb、启动 CAN |
| `can-hub-up.sh all` | 启动 can0~can4 |
| `can-down.sh [canX...]` | 关闭 CAN 接口 |
| `usb-can-map.sh` | USB 口 ↔ canX 映射 |

### 启动 ROS 节点

| 脚本 | 用途 |
|------|------|
| `run-quad-all.sh` | **一键后台** 全机：8 腿 + 麦轮 + 遥控协调（`run-quad-all.sh stop` 停止） |
| `run-leg1-all.sh` | **一键后台** 启 quad_rs_ros2 + c620_ros2（台架 active_legs=[1]） |
| `run-quad-rs.sh` | 前台 quad_rs（can0 抬升 + can1 拉杆） |
| `quad-goto-pose.sh` | 整机预设姿态 `/quad/goto/*` |
| `run-leg1-all.sh stop` | 停止全部节点 |
| `run-leg1-dual.sh` | 前台 leg1（can0 ID1 + can1 ID5） |
| `run-c620.sh can2` | 前台麦轮（can2，C620 ID1） |

### 灵足 RS03（无 ROS）

| 脚本 | 用途 |
|------|------|
| `rs-probe.sh 01 05` | 探测电机 ID（不发运动） |
| `rs-probe-all.sh` | can0 探 1~4，can1 探 5~8 |
| `rs-enable.sh 01` | 仅使能单台 |
| `rs-disable.sh 01 05` | 失能（默认 leg1 两台） |
| `rs-emergency-stop.sh` | 急停：杀进程 + 失能 1~8 |

### 第一条腿 · 位姿

| 脚本 | 用途 |
|------|------|
| `leg1-goto-pose.sh stand` | 预设站立（绝对角，见 `leg1_dual_params.yaml`） |
| `leg1-goto-pose.sh crouch` | 预设微蹲 |
| `leg1-goto-pose.sh neutral` | 休息位 |
| `leg1-pose-ros.sh` | 相对微调（默认抬升 +0.08，趴下 +0.15） |

### 大疆 C620

| 脚本 | 用途 |
|------|------|
| `diagnose-c620.sh can2` | C620 连通性诊断 |
| `c620-estop.sh` | 急停（默认 can2） |
| `c620-send-current.sh 1 0.3` | 手动试电流（不经 ROS） |

### 系统 / 实时（可选）

| 脚本 | 用途 |
|------|------|
| `show-topology.sh` | CPU / USB 拓扑 |
| `boot-tuning.sh` | 启动调优 |
| `set-irq-affinity.sh` | IRQ 亲和性 |

---

## ROS 话题速查

### 灵足 leg1（`leg1_dual_ros2`）

```bash
ros2 topic echo /leg1/lift/state --once
ros2 topic echo /leg1/crouch/state --once

# 绝对角
ros2 topic pub /leg1/crouch/command std_msgs/msg/Float64 '{data: 2.65}' --once \
  --qos-reliability reliable --qos-durability transient_local

# 预设姿态
ros2 topic pub /leg1/goto/stand std_msgs/msg/Empty '{}' --once \
  --qos-reliability reliable --qos-durability transient_local

# 锁定
ros2 topic pub /leg1/hold std_msgs/msg/Empty '{}' --once \
  --qos-reliability reliable --qos-durability transient_local
```

### 麦轮（`c620_ros2`）

```bash
ros2 topic echo /c620/state --once

ros2 topic pub /c620/velocity_command std_msgs/msg/Float64 '{data: 0.5}' \
  --rate 20 --qos-reliability reliable --qos-durability transient_local

ros2 topic pub /c620/velocity_command std_msgs/msg/Float64 '{data: 0.0}' \
  --rate 5 --qos-reliability reliable --qos-durability transient_local
```

---

## 推荐流程

```bash
# 1. 编译（改代码后）
cd ~/robot_ws && colcon build --packages-select rs_motor_ros2
source install/setup.bash

# 2. 一键启动
~/robot_ws/scripts/run-leg1-all.sh

# 3. 姿态
~/robot_ws/scripts/leg1-goto-pose.sh stand

# 4. 停
~/robot_ws/scripts/run-leg1-all.sh stop
```

更详细的故障排查见 [`leg1_commands.md`](leg1_commands.md)。
