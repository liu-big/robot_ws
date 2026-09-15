# robot_ws — 机械狗 x86 实时控制工作区

Ubuntu 22.04 + ROS2 Humble + PREEMPT_RT。

**当前阶段：** 四足整机 CAN 已分三路，leg1 台架联调。

---

## CAN 接线（当前）

| CAN | 设备 | 电机 ID | 说明 |
|-----|------|---------|------|
| **can0** | 灵足 RS03 ×4 | **1, 2, 3, 4** | 抬升（内部，带麦轮连杆） |
| **can1** | 灵足 RS03 ×4 | **5, 6, 7, 8** | 拉杆（外部，趴下/站起） |
| **can2** | 大疆 C620 ×4 + M3508 | **1, 2, 3, 4** | 四轮麦轮 |

```
        leg4          leg3
    抬升④ 拉杆⑧    抬升③ 拉杆⑦
          ┌────┐
          │机身│
          └────┘
    抬升① 拉杆⑤    抬升② 拉杆⑥
        leg1          leg2
```

| 腿 | 抬升 (can0) | 拉杆 (can1) | 麦轮 (can2) |
|----|-------------|-------------|-------------|
| leg1 左前 | 1 | 5 | 1 |
| leg2 右前 | 2 | 6 | 2 |
| leg3 右后 | 3 | 7 | 3 |
| leg4 左后 | 4 | 8 | 4 |

详情：[`config/quadruped.yaml`](config/quadruped.yaml) · [`config/can.yaml`](config/can.yaml)

> 启 CAN 时执行 **`can-hub-up.sh can0 can1 can2`**，不要只启 can0。

---

## 快速开始

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

cd ~/robot_ws && colcon build --packages-select rs_motor_ros2
source install/setup.bash

# 一键后台（quad_rs + c620，leg1）
~/robot_ws/scripts/run-leg1-all.sh

# 预设姿态
~/robot_ws/scripts/leg1-goto-pose.sh stand

# 停止
~/robot_ws/scripts/run-leg1-all.sh stop
```

前台分终端：

```bash
~/robot_ws/scripts/can-hub-up.sh can0 can1 can2
~/robot_ws/scripts/run-quad-rs.sh      # 灵足（can0+can1）
~/robot_ws/scripts/run-c620.sh can2    # 麦轮
```

---

## ROS2 节点

| 节点 | CAN | 说明 |
|------|-----|------|
| `quad_rs_ros2` | can0 + can1 | 8 关节，`active_legs` 控制启用哪些腿 |
| `leg1_dual_ros2` | can0 + can1 | leg1 专用（与 quad_rs 二选一） |
| `c620_ros2` | can2 | 单轮速度环（默认 ID=1） |

参数：[`config/quad_rs_params.yaml`](config/quad_rs_params.yaml)

### 主要话题

- `/leg{N}/lift/command` · `/leg{N}/crouch/command` — 关节角 (rad)
- `/leg1/goto/stand` · `/leg1/goto/crouch` — 预设姿态
- `/c620/velocity_command` — 麦轮速度 (rad/s)，须带 QoS

---

## 脚本速查

| 脚本 | 用途 |
|------|------|
| `can-hub-up.sh can0 can1 can2` | 启动三路 CAN |
| `run-leg1-all.sh` | 后台启 quad_rs + c620 |
| `run-quad-rs.sh` | 前台灵足 8 关节 |
| `run-c620.sh can2` | 前台麦轮 |
| `rs-probe-all.sh` | can0 探 1~4，can1 探 5~8 |
| `diagnose-c620.sh can2` | C620 诊断 |
| `rs-emergency-stop.sh` | 灵足急停 |
| `c620-estop.sh` | 麦轮急停 |

完整列表：[`docs/scripts.md`](docs/scripts.md)

---

## 急停

```bash
~/robot_ws/scripts/rs-emergency-stop.sh
~/robot_ws/scripts/c620-estop.sh
```

仍在转 → **断 24V**。

---

## 文档

| 文档 | 内容 |
|------|------|
| [`docs/scripts.md`](docs/scripts.md) | 脚本与命令 |
| [`docs/leg1_commands.md`](docs/leg1_commands.md) | leg1 操作 |
| [`docs/quadruped.md`](docs/quadruped.md) | 四足架构 |

---

## 备份

`/home/a/robot_ws_pkg_20260912.tar.gz`
