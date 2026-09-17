# robot_ws — 四足机械狗 ROS2 工作区

Ubuntu 22.04 + ROS2 Humble。

## CAN 接线

| CAN | 设备 | ID |
|-----|------|-----|
| can0 | 灵足 RS03 抬升 | 1–4 |
| can1 | 灵足 RS03 拉杆 | 5–8 |
| can2 | 大疆 C620 麦轮 | 1–2（当前在线） |

配置：[`config/quadruped.yaml`](config/quadruped.yaml) · [`config/quad_full_params.yaml`](config/quad_full_params.yaml)

---

## 快速开始

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
cd ~/robot_ws && colcon build --packages-select rs_motor_ros2
source install/setup.bash

~/robot_ws/scripts/run-quad-all.sh
```

手机遥控（[`~/quad_remote`](../quad_remote) 或同目录 `quad_remote`）：

```bash
~/quad_remote/start-all.sh
# 手机: http://<主机IP>:8765/
```

---

## 脚本（`scripts/`）

| 脚本 | 用途 |
|------|------|
| `run-quad-all.sh` | 启动/停止全机节点 |
| `can-hub-up.sh can0 can1 can2` | 启动 CAN |
| `quad-pub.sh` | 发 ROS 控制话题 |
| `quad-goto-pose.sh` | 站起/趴下/回零 |
| `quad-calibrate.sh` | 零点标定 |
| `estop.sh` | 急停 |

详见 [`docs/scripts.md`](docs/scripts.md)

---

## ROS 节点

| 节点 | 说明 |
|------|------|
| `quad_rs_ros2` | 8 关节，can0+can1 |
| `c620_quad_ros2` | 麦轮，can2 |
| `quad_teleop_ros2` | `/cmd_vel` → 麦轮 IK |

主要话题：`/quad/lift/command` · `/quad/crouch/command` · `/quad/wheels/cmd` · `/cmd_vel`

---

## 急停

```bash
~/robot_ws/scripts/estop.sh
```

仍在转 → **断 24V**。
