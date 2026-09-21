# 第一条腿 · 抬升关节

电机: 灵足 RS03，CAN **0x7F**，**can0**  
节点: `leg1_lift_node`

## 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/leg1/lift/command` | `std_msgs/Float64` | 目标角度 (rad)，节点内 **梯形轨迹平滑** 后下发 |
| `/leg1/lift/state` | `sensor_msgs/JointState` | position / velocity / effort |
| `/leg1/lift/hold` | `std_msgs/Empty` | 锁定当前位置 |

## 平滑控制（参考 MIT Cheetah / Unitree 关节层）

灵足 RS03 **运控模式** ≈ MIT 混合阻抗：`τ ≈ Kp(qDes−q) + Kd(qdDes−qd)`。

节点在 ROS 侧做 **梯形速度规划**，每周期下发 `qDes + qdDes`（位置 + 速度前馈），避免目标角突变：

| 参数 | 默认 | 说明 |
|------|------|------|
| `smooth_enabled` | true | 开启轨迹平滑 |
| `max_vel_rad_s` | 0.4 | 最大角速度 (rad/s) |
| `max_accel_rad_s2` | 1.5 | 最大角加速度 (rad/s²) |
| `motion_kp` | 40 | 刚度（过大易抖） |
| `motion_kd` | 2.0 | 阻尼（配合 kp 抑振） |
| `loop_ms` | 10 | 100Hz 控制环 |

更慢更柔：`max_vel_rad_s:=0.2 max_accel_rad_s2:=0.8 motion_kp:=30 motion_kd:=2.5`

## 操作

**终端 1：**

```bash
source ~/.bashrc
canup

ros2 run rs_motor_ros2 rs_motor_ros2 --ros-args \
  -p motion_kp:=40.0 \
  -p motion_kd:=2.0 \
  -p max_vel_rad_s:=0.4 \
  -p max_accel_rad_s2:=1.5 \
  -p max_command_step_rad:=0.5 \
  -p disable_on_exit:=false
```

等到 `已使能，保持 xxx rad`（**记下这个 xxx**）。

**终端 2：**

```bash
source ~/.bashrc

# 1. 必须先读当前角
ros2 topic echo /leg1/lift/state --once

# 2. 发目标角（节点会平滑逼近，可一次发较远目标）
ros2 topic pub /leg1/lift/command std_msgs/msg/Float64 '{data: 0.5}' --once

# 3. 再读反馈
ros2 topic echo /leg1/lift/state --once

# 4. 继续发目标，不必每次只 ±0.1（仍受 max_command_step_rad 限幅）
ros2 topic pub /leg1/lift/command std_msgs/msg/Float64 '{data: 1.0}' --once
```

**失能：** `~/robot_ws/scripts/rs-disable.sh` 或 `estop`

## 安全

- **禁止**不看 `position` 就发与当前差很大的角（如直接 `-3.4`）→ 即使平滑也会转很久
- `max_command_step_rad` 限制单次 command 目标变化（默认 ±0.5 rad）
- 梯形规划限制 **速度/加速度**，比原先阶跃目标平滑得多
- 要到 -3.4：分多次 pub，或逐步增大 `max_command_step_rad`
