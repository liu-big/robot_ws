# 大疆 C620 · can2 ROS2 控制

> **本机 C620 接 HUB 第 3 路 → `can2`**（can1 硬件异常不可用）

## 快速启动

```bash
pkill -9 -f c620_ros2
source ~/robot_ws/install/setup.bash
~/robot_ws/scripts/can-hub-up.sh can2

# 终端1
ros2 run rs_motor_ros2 c620_ros2 --ros-args -p can_interface:=can2

# 终端2（也要 source）
ros2 topic pub /c620/velocity_command std_msgs/msg/Float64 '{data: 5.0}' --rate 20
```

停转：`Ctrl+C` → `ros2 topic pub ... '{data: 0.0}' --rate 5` 或 `~/robot_ws/scripts/c620-estop.sh`

## 连通性自检

```bash
~/robot_ws/scripts/can-hub-up.sh can2
ip -br link show can2          # 应 UP
~/robot_ws/scripts/diagnose-c620.sh can1   # 应见 0x201
```

## 双路（灵足 + C620）

```bash
~/robot_ws/scripts/can-hub-up.sh can0 can2
# can0: leg1 灵足
# can2: C620 麦轮
```

## 推荐参数（空载）

```bash
ros2 run rs_motor_ros2 c620_ros2 --ros-args \
  -p can_interface:=can2 \
  -p max_current_a:=1.8 \
  -p max_vel_accel_rad_s2:=10.0 \
  -p vel_kp:=1.0 -p vel_ki:=0.05 -p vel_kd:=0.08
```

## 话题

| 话题 | 说明 |
|------|------|
| `/c620/velocity_command` | 麦轮 rad/s |
| `/c620/stop` | 急停 0A |
| `/c620/state` | JointState |

## can1 不可用说明

若 `can-hub-up.sh can1` 报 `No such device`，是 **USB-CAN 第 2 路硬件问题**，请用 can2 并改 `can_interface:=can2`。
