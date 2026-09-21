# robot_ws — Quadruped ROS2 Workspace

Ubuntu 22.04 · ROS2 Humble · RobStride RS03 (8 joints) + DJI C620 mecanum wheels (can2)

Includes onboard control (`src/robstride_ros_sample`), launch scripts (`scripts/`), configs (`config/`), and mobile web teleop ([`quad_remote/`](quad_remote/)).

![QUAD Remote UI](docs/images/quad-remote-ui.png)

---

## Quick Start

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws && colcon build --packages-select rs_motor_ros2
source install/setup.bash

# Full stack: CAN + ROS nodes + mobile web
~/quad_remote/start-all.sh

# Stand up
~/robot_ws/scripts/quad-goto-pose.sh stand

# Phone (landscape): http://<host-ip>:8765/
```

---

## Hardware

| CAN | Device | Motor IDs |
|-----|--------|-----------|
| can0 | RS03 lift | 1–4 |
| can1 | RS03 crouch | 5–8 |
| can2 | C620 wheels | 1–4 (only 1–2 active now) |

---

## Software Stack

| Node | Role |
|------|------|
| `quad_rs_ros2` | 8 joints, MIT-style motion control (运控模式), 200 Hz |
| `c620_quad_ros2` | Wheel velocity PID → current, can2 |
| `quad_teleop_ros2` | `/cmd_vel` → mecanum IK → `/quad/wheels/cmd` |
| `quad_odom_ros2` | Wheel FK + integration → `/odom`, `/odom/path`, TF |
| `quad_remote` | Phone WebSocket UI → `/cmd_vel` + pose buttons |

---

## Control Modes

- **Joints (RS03):** Hybrid impedance \(\tau \approx K_p(q_{des}-q) + K_d(\dot q_{des}-\dot q)\) via `send_motion_hold()`
- **Wheels (C620):** Speed PID + friction feedforward (not MIT protocol)
- **Teleop:** STAND / NEUTRAL / CROUCH presets; dual-stick VX/VY/WZ at 50 Hz

---

## Key Topics

| Topic | Type | Notes |
|-------|------|-------|
| `/cmd_vel` | `Twist` | vx, vy, wz |
| `/quad/wheels/cmd` | `Float64MultiArray` | 4 wheel rim speeds (rad/s) |
| `/quad/wheel_states` | `JointState` | velocity + single-turn angle |
| `/quad/joint_states` | `JointState` | 8 joint feedback |
| `/odom` | `Odometry` | Integrated from wheel velocities |
| `/quad/goto/{stand,neutral,crouch}` | `Empty` | Body poses |

---

## Scripts

| Script | Purpose |
|--------|---------|
| `run-quad-all.sh` | Start/stop all ROS nodes + wheel odom |
| `start-all.sh` (quad_remote) | Full restart incl. web server |
| `quad-goto-pose.sh` | stand / neutral / crouch |
| `quad-calibrate.sh` | Save home pose → `config/quad_home.yaml` |
| `estop.sh` | Emergency stop (ROS + CAN disable) |

See [docs/scripts.md](docs/scripts.md) (Chinese) for full command reference.

---

## Documentation

- [中文 README](README.md)
- [Mobile teleop guide](docs/quad_remote.md)
- [Wheel odometry](docs/odometry.md)
- [quad_remote API](quad_remote/README.md)

---

## Safety

```bash
~/robot_ws/scripts/estop.sh
```

If wheels still spin → **cut 24V power immediately**.

---

**Author:** [liu-big](https://github.com/liu-big)
