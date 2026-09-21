# 第一条腿 · 终端指令手册

> 脚本速查表见 [`scripts.md`](scripts.md)

台架当前接线：

| 关节 | 电机 | CAN | 节点 | 说明 |
|------|------|-----|------|------|
| 抬升 `leg1_lift` | 灵足 RS03 (ID `1`) | **can0** | `leg1_dual_ros2` | 位置/运控保持 |
| 趴下/起身 `leg1_crouch` | 灵足 RS03 (ID `5`) | **can0** | `leg1_dual_ros2` | 同上，双电机单节点 |
| 麦轮 `c620_motor` | 大疆 C620 + M3508 (ID `1`) | **can1** | `c620_ros2` | 速度闭环 + 零速锁位 |

> 旧单电机节点 `rs_motor_ros2`（ID `0x7F`）已不用；**不要与 `leg1_dual_ros2` 同时跑**（抢 can0）。

---

## 0. 每次测试前（终端 0）

```bash
source ~/robot_ws/install/setup.bash

# 杀掉旧进程（避免重复节点抢 CAN）
pkill -9 leg1_dual_ros2 2>/dev/null || true
pkill -9 -f rs_motor_ros2 2>/dev/null || true
pkill -9 c620_ros2 2>/dev/null || true
rm -f /tmp/leg1_dual_ros2.lock /tmp/c620_ros2.lock
sleep 1

# 同时启 can0 + can1（必须两路一起，不要只 canup）
~/robot_ws/scripts/can-hub-up.sh can0 can1

# 确认 UP
ip -br link show can0 can1
```

### 上电自检（不上 ROS）

```bash
# 灵足 can0（有扩展帧即可，具体 ID 视电机而定）
timeout 3 candump can0 | head -5

# 大疆 can1（应有持续 0x201）
timeout 3 candump -tz can1,201:7FF | head -5
```

看不到 `0x201` → 查 C620 **24V、CAN 线、HUB 第 2 路**。

---

## 1. 启动节点（两个终端常驻）

一键后台启动（推荐）：

```bash
~/robot_ws/scripts/run-leg1-all.sh
```

或分终端前台启动：

### 终端 1 — 双灵足（can0 / ID 1 + 5）

```bash
source ~/robot_ws/install/setup.bash
~/robot_ws/scripts/run-leg1-dual.sh
```

等到：`抬升 已使能，保持 x.xxx rad` 和 `趴下 已使能，保持 y.yyy rad`。

### 终端 2 — 麦轮（can1 / 大疆 C620）

```bash
source ~/robot_ws/install/setup.bash
~/robot_ws/scripts/run-c620.sh can1
```

或手动：

```bash
source ~/robot_ws/install/setup.bash
pkill -9 c620_ros2 2>/dev/null || true
sleep 1
rm -f /tmp/c620_ros2.lock

ros2 run rs_motor_ros2 c620_ros2 --ros-args \
  -p can_interface:=can1 \
  -p motor_id:=1 \
  -p gear_ratio:=19.0 \
  -p hold_enable:=true \
  -p can_iface_reset_enable:=true
```

等到：`→ 锁位` 且 `[锁位] err≈0  I≈0  fb_age=0ms`。

---

## 2. ROS 话题速查

### 灵足双关节 `/leg1/lift/*` `/leg1/crouch/*`

| 话题 | 类型 | 说明 |
|------|------|------|
| `/leg1/lift/command` | `std_msgs/Float64` | 抬升目标角 (rad)，电机 ID=1 |
| `/leg1/crouch/command` | `std_msgs/Float64` | 趴下/起身目标角 (rad)，电机 ID=5 |
| `/leg1/lift/state` | `sensor_msgs/JointState` | 抬升 position / velocity / effort |
| `/leg1/crouch/state` | `sensor_msgs/JointState` | 趴下 position / velocity / effort |
| `/leg1/lift/hold` | `std_msgs/Empty` | 抬升锁定当前位置 |
| `/leg1/crouch/hold` | `std_msgs/Empty` | 趴下锁定当前位置 |
| `/leg1/hold` | `std_msgs/Empty` | 两关节同时锁定 |
| `/leg1/goto/neutral` | `std_msgs/Empty` | 跳到休息位（见 `config/leg1_dual_params.yaml`） |
| `/leg1/goto/stand` | `std_msgs/Empty` | 跳到微站立 |
| `/leg1/goto/crouch` | `std_msgs/Empty` | 跳到微蹲 |

### 麦轮 `/c620/*`

| 话题 | 类型 | 说明 |
|------|------|------|
| `/c620/velocity_command` | `std_msgs/Float64` | 轮子角速度 (rad/s) |
| `/c620/state` | `sensor_msgs/JointState` | 位置 / 速度 / 电流 |
| `/c620/stop` | `std_msgs/Empty` | 急停 |

> C620 速度指令必须带 QoS：`--qos-durability transient_local --qos-reliability reliable`

---

## 3. 发指令（终端 3）

```bash
source ~/robot_ws/install/setup.bash
```

### 3.1 双灵足关节

```bash
# ① 读当前角度（必做）
ros2 topic echo /leg1/lift/state --once
ros2 topic echo /leg1/crouch/state --once

# ② 一键小幅度：抬升 +0.08 rad，趴下 +0.15 rad（相对当前）
~/robot_ws/scripts/leg1-pose-ros.sh

# 或手动发绝对目标角（在当前值基础上微调）
ros2 topic pub /leg1/lift/command std_msgs/msg/Float64 '{data: 6.15}' --once
ros2 topic pub /leg1/crouch/command std_msgs/msg/Float64 '{data: 2.55}' --once

# ③ 锁定
ros2 topic pub /leg1/hold std_msgs/msg/Empty "{}" --once
```

单次变化默认限 **±0.25 rad**（`max_command_step_rad`），大跨度需多次 pub。

### 3.2 麦轮

```bash
# 读状态
ros2 topic echo /c620/state --once

# 转轮（先小速度 2 rad/s）
ros2 topic pub /c620/velocity_command std_msgs/msg/Float64 '{data: 2.0}' \
  --rate 20 \
  --qos-durability transient_local \
  --qos-reliability reliable

# 停转 + 锁位（持续发 0，Ctrl+C 停 pub 后仍保持锁位）
ros2 topic pub /c620/velocity_command std_msgs/msg/Float64 '{data: 0.0}' \
  --rate 5 \
  --qos-durability transient_local \
  --qos-reliability reliable

# 急停话题
ros2 topic pub /c620/stop std_msgs/msg/Empty "{}" --once
```

### 3.3 同时监视

```bash
# 终端 A
ros2 topic echo /leg1/lift/state

# 终端 B
ros2 topic echo /leg1/crouch/state

# 终端 C
ros2 topic echo /c620/state
```

---

## 4. 完整测试流程（复制照着做）

```
步骤 0  上电 24V → run-leg1-all.sh
步骤 1  leg1_dual_ros2 + c620_ros2 后台已运行
步骤 3  终端3 leg1-pose-ros.sh 或手动 pub 关节角
步骤 4  麦轮 pub 小速度试转 → pub 0.0 停转
步骤 5  ros2 topic pub /leg1/hold 锁位
步骤 6  Ctrl+C 退出节点，或急停脚本
```

---

## 5. 急停 / 全杀

```bash
# 抬升 RS03
~/robot_ws/scripts/rs-emergency-stop.sh

# C620 麦轮（默认 can1，可 export C620_CAN=can1）
C620_CAN=can1 ~/robot_ws/scripts/c620-estop.sh

# 或一次性全杀
pkill -9 leg1_dual_ros2 2>/dev/null
pkill -9 -f rs_motor_ros2 2>/dev/null
pkill -9 c620_ros2 2>/dev/null
rm -f /tmp/leg1_dual_ros2.lock /tmp/c620_ros2.lock
```

仍在转 → **立即断 24V**。

---

## 6. 诊断

```bash
# C620 连通性
~/robot_ws/scripts/diagnose-c620.sh can1

# 监听 C620 反馈
candump -tz can1,201:7FF

# 灵足探测（当前 ID 1 + 5）
~/robot_ws/scripts/rs-probe.sh 01 05

# USB 口映射
~/robot_ws/scripts/usb-can-map.sh
```

### C620 手动试转（不经 ROS）

```bash
# ID1 约 0.5A
cansend can1 200#0064000000000000

# 或脚本
C620_CAN=can1 ~/robot_ws/scripts/c620-send-current.sh 1 0.3
```

### CAN 无帧 / 锁不住时恢复

```bash
sudo ~/robot_ws/scripts/can-hub-up.sh can0 can1
# 仍无 0x201 → 重插 USB HUB 或重上 C620 24V，再执行上面命令
# 然后重启 c620_ros2
```

---

## 7. 编译（改代码后）

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws
colcon build --packages-select rs_motor_ros2
source install/setup.bash
```

---

## 8. 参数调优

### 双灵足更慢更柔

```bash
ros2 run rs_motor_ros2 leg1_dual_ros2 --ros-args \
  -p can_interface:=can0 \
  -p lift_motor_id:=1 \
  -p crouch_motor_id:=5 \
  -p max_vel_rad_s:=0.10 \
  -p max_accel_rad_s2:=0.3 \
  -p motion_kp:=30.0 \
  -p motion_kd:=2.5
```

### C620 锁位抖动

```bash
ros2 run rs_motor_ros2 c620_ros2 --ros-args \
  -p can_interface:=can1 \
  -p hold_kp:=0.35 \
  -p hold_kd_rpm:=0.035 \
  -p hold_lpf_tau_s:=0.04 \
  -p can_iface_reset_enable:=true
```

### C620 速度环（空载）

```bash
ros2 run rs_motor_ros2 c620_ros2 --ros-args \
  -p can_interface:=can1 \
  -p max_current_a:=1.8 \
  -p vel_kp:=0.25 \
  -p vel_ki:=0.02 \
  -p vel_kd:=0.001
```

---

## 9. 注意事项

1. **必须先启动节点再 pub**，只发话题电机不会动。
2. **can-hub-up 必须带 can0 can1**，单独 `canup`（默认只 can0）会把 can1 关掉。
3. **灵足双关节**：发角度前先看 `position`，单次变化默认限 ±0.25 rad；用 `leg1_dual_ros2`，勿与 `rs_motor_ros2` 同跑。
4. **麦轮**：`velocity_command` 必须带 transient_local + reliable QoS。
5. **杀 C620 进程**：`pkill -9 c620_ros2`（不要用 `pkill -f c620_ros2`）。
6. **日志正常**：`[锁位] err≈0  fb_age=0ms`；异常：`I=2A fb≈0` 或 `fb_age>200ms` → CAN 通信问题。

---

## 相关文档

- [leg1_lift.md](leg1_lift.md)
- [c620_ros.md](c620_ros.md)
- [C620.md](C620.md)
- [USB_CANHUB.md](USB_CANHUB.md)
