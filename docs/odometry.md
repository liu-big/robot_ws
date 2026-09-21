# 麦轮里程计（quad_odom_ros2）

## 原理

与业界常见方案一致（ros2_control `mecanum_drive_controller`、Nav2 麦轮底盘）：

```
轮速反馈 → 正运动学 FK → 车体 twist (vx, vy, ω)
         → 旋转到 odom 系积分 → nav_msgs/Odometry + tf
```

本实现 **不** 对 C620 单圈 `position` 做 unwrap 积分，而是直接用 **转速**（与控制器相同），避免 0~8191 回绕问题。

## 运动学（与 quad_teleop 一致）

轮序：leg1 FL、leg2 FR、leg3 RR、leg4 RL。  
\(k = \frac{L_x}{2} + \frac{L_y}{2}\)（`wheel_base_x/y`）

逆运动学 IK（teleop 发令）：

\[
\begin{aligned}
w_0 &= v_x - v_y - \omega k \\
w_1 &= v_x + v_y + \omega k \\
w_2 &= v_x + v_y - \omega k \\
w_3 &= v_x - v_y + \omega k
\end{aligned}
\]

正运动学 FK（四轮全在线）：

\[
v_x = \frac{w_0+w_1+w_2+w_3}{4},\quad
v_y = \frac{w_1-w_0+w_2-w_3}{4},\quad
\omega = \frac{-w_0+w_1-w_2+w_3}{4k}
\]

部分轮离线时，对活跃轮的雅可比行做 **最小二乘**（伪逆），与 ros2_control 文档中 “\(v_b = r A^\dagger \omega\)” 思路相同。

## 配置

`config/quad_odom_params.yaml` 必须与 `quad_teleop_params.yaml`、`c620_quad_params.yaml` 对齐：

| 参数 | 说明 |
|------|------|
| `wheel_base_x/y` | 轮距，与 teleop 相同 |
| `active_wheels` | 当前 `[1,2]` 仅前轮 |
| `wheel_sign` | 与 C620 一致，修正镜像安装 |
| `velocity_ema_alpha` | 速度低通（0.35 默认） |
| `publish_tf` | 是否广播 `odom→base_link` |

## 重置与可视化

```bash
# 里程计归零（服务 /quad/odom/reset）
~/robot_ws/scripts/quad-odom-reset.sh

# RViz2：odom 坐标系 + 轨迹 /odom/path
~/robot_ws/scripts/quad-rviz-odom.sh
```

发布话题：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/odom` | `nav_msgs/Odometry` | 位姿 + 车体 twist |
| `/odom/path` | `nav_msgs/Path` | 行驶轨迹（RViz Path） |
| `tf` | `odom → base_link` | 与 `/odom` 同步 |

## 使用

随 `run-quad-all.sh` 自动启动。手动：

```bash
ros2 run rs_motor_ros2 quad_odom_ros2 --ros-args \
  --params-file ~/robot_ws/config/quad_odom_params.yaml
```

查看：

```bash
ros2 topic hz /odom
ros2 topic echo /odom/pose/pose/position --once
ros2 run tf2_ros tf2_echo odom base_link
```

## 精度说明

| 场景 | 预期 |
|------|------|
| 四轮全在线 | vx/vy/ω 可观测性最好 |
| **仅 FL+FR（当前）** | vx 可靠；vy 与 ω 耦合，为最小二乘估计 |
| 原地打滑 / 抬腿 | 里程漂移，需后续 IMU/视觉融合 |
| 长时间积分 | 累积误差，适合短程遥控而非精定位 |

## 日志

`~/robot_ws/log/quad/odom.log`
