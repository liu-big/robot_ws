# quad_remote — 四足手机横屏遥控

**QUAD · REMOTE / 01** — 浏览器双摇杆界面，经 WebSocket 低延迟驱动 ROS2 `/cmd_vel`，一键切换 STAND / NEUTRAL / CROUCH 姿态。

无需安装 App；与 [`robot_ws`](../README.md) 配合使用。

---

## 目录

- [功能概览](#功能概览)
- [系统架构](#系统架构)
- [安装与依赖](#安装与依赖)
- [启动方式](#启动方式)
- [手机操作说明](#手机操作说明)
- [WebSocket 协议](#websocket-协议)
- [ROS 桥接](#ros-桥接)
- [配置与限幅](#配置与限幅)
- [测试](#测试)
- [排障](#排障)
- [源码结构](#源码结构)

---

## 功能概览

| 功能 | 说明 |
|------|------|
| **双摇杆驱动** | 左：VX + VY（前后/横移）；右：WZ（左旋/右旋） |
| **50 Hz 控制** | rAF 定时发 drive，有输入时每帧下发，松手即停 |
| **速度倍率** | 0.40× ~ 1.50× 滑条，默认 0.80×，写入 sessionStorage |
| **姿态按钮** | STAND 站立 / NEUTRAL 中立 / CROUCH 蹲伏 |
| **实时遥测** | 5 Hz 轻量状态 + 周期性全量 joints/wheels |
| **连接状态** | ROS 链路、WebSocket RTT、四轮 FL/FR/RL/RR 速度 |
| **安全** | 竖屏禁驱、详情面板禁驱、断线 stop、多页最新者控权 |
| **DEMO 模式** | `--demo` 无 ROS，模拟状态（开发用） |

---

## 系统架构

```
┌──────────────────────────────────────────────────────────────┐
│  手机浏览器  landscape                                         │
│  static/index.html + app.js                                  │
│    · 双摇杆 50Hz                                               │
│    · SPEED / POSE 按钮                                         │
└───────────────────────────┬──────────────────────────────────┘
                            │ WebSocket  ws://host:8765/ws
                            ▼
┌──────────────────────────────────────────────────────────────┐
│  server.py (aiohttp)                                         │
│    · Latest 最新值存储（线程安全）                              │
│    · 非阻塞 ping/pong                                          │
│    · 5 Hz 状态下发                                             │
└───────────────────────────┬──────────────────────────────────┘
                            │
                            ▼
┌──────────────────────────────────────────────────────────────┐
│  ros_bridge.py (rclpy 独立线程)                               │
│    · 50 Hz 发布 /cmd_vel                                       │
│    · 姿态 → /quad/teleop/{stand,neutral,crouch}              │
│    · 订阅 /quad/body_mode、joint_states、wheel_states         │
└───────────────────────────┬──────────────────────────────────┘
                            ▼
              quad_teleop_ros2  →  c620_quad_ros2 / quad_rs_ros2
```

**驱动路径：** 摇杆 → `/cmd_vel` → 麦轮 IK → `/quad/wheels/cmd` → C620 速度 PID

**姿态路径：** 按钮 → `/quad/teleop/stand` 等 → `/quad/goto/stand` → 8 关节 MIT 类运控

---

## 安装与依赖

```bash
pip install -r requirements.txt
```

| 包 | 用途 |
|----|------|
| `aiohttp` | HTTP + WebSocket 服务 |
| `PyYAML` | 读取 `robot_ws` 速度限幅配置 |

运行时还需：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
```

`run.sh` 会自动设置 `PYTHONPATH`（含 `.deps/`）、`ROS_LOCALHOST_ONLY=1` 和 FastDDS UDP 配置。

---

## 启动方式

### 与 robot_ws 一键启动（推荐）

```bash
~/quad_remote/start-all.sh
```

内部调用 `robot_ws/scripts/run-quad-all.sh` + 本服务 `:8765`。

### 单独启动 Web（robot 节点需已运行）

```bash
~/quad_remote/run.sh --host 0.0.0.0 --port 8765
```

### 演示模式（无硬件）

```bash
~/quad_remote/run.sh --demo --host 0.0.0.0 --port 8765
```

### 访问地址

```bash
hostname -I | awk '{print "http://" $1 ":8765/"}'
```

手机 **横屏** 打开；建议加入主屏幕快捷方式。

### 健康检查

```bash
curl -s http://127.0.0.1:8765/health
# 期望: "ok": true, "ros_connected": true, "teleop_connected": true
```

---

## 手机操作说明

### 顶部状态栏

| 指示 | 含义 |
|------|------|
| **ROS 已连接** | `quad_web_bridge` 已连上 `/cmd_vel` 订阅链 |
| **ROS 等待** | teleop 未就绪或 DDS 未图通 |
| **WS xxx ms** | WebSocket 往返延迟 |
| **DEMO** | 演示模式，无真实硬件 |

### 摇杆区

- **TRANSLATION（左）**：上=前进、下=后退、左/右=横移；映射为 `-VY` / `-VX`（与机身坐标一致）
- **ROTATION（右）**：左/右=原地转；仅 X 轴有效
- 死区约 8%，小幅度推杆有起步地板速度（`min_body` / `min_yaw` ≈ 0.4× 倍率）
- **松手即停**：摇杆回中后发送 stop，电机指令清零

### 中央区域

- **模式**：STAND / NEUTRAL / CROUCH / UNKNOWN（来自 `/quad/body_mode`）
- **CMD VX VY WZ**：当前下发指令数值
- **俯视图**：点击打开 **LIVE TELEMETRY** 面板（8 关节 + 4 轮表格）

### 底部

- **SPEED 滑条**：整体速度倍率，影响 vx/vy/wz 上限
- **姿态按钮**：切换整机预设；会 reset 摇杆并立即发 pose 消息

### 限制

- 竖屏：显示提示，**禁止驱动**
- 打开详情 dialog：**禁止驱动**（防误触）
- 页面 hidden / 断线：自动 stop + 重连（指数退避，最长 5s）

---

## WebSocket 协议

连接：`ws://<host>:8765/ws`

### 服务端 → 客户端

**hello**（连接后首包）

```json
{
  "type": "hello",
  "limits": {"vx": 0.5, "vy": 0.5, "wz": 1.0},
  "base_limits": {"vx": 0.8, "vy": 0.8, "wz": 1.5},
  "control_hz": 50,
  "state_hz": 5,
  "speed_min": 0.4,
  "speed_max": 1.5,
  "min_body": 0.4,
  "min_yaw": 0.4,
  "demo": false
}
```

**state**（约 5 Hz；每 20  tick 含 joints/wheels 全量）

```json
{
  "type": "state",
  "mode": "stand",
  "vx": 0.0, "vy": 0.0, "wz": 0.0,
  "ros_connected": true,
  "teleop_connected": true,
  "can_control": true,
  "ping": 42
}
```

**pong**

```json
{"type": "pong", "t": 1234567890.1}
```

### 客户端 → 服务端

| type | 字段 | 说明 |
|------|------|------|
| `drive` | `vx`, `vy`, `wz` | 速度指令（float，50Hz） |
| `pose` | `mode` | `stand` / `neutral` / `crouch` |
| `stop` | — | 立即停止，清零 drive |
| `ping` | `t`, `rtt?` | 测延迟 |

无效消息会导致 socket 关闭并 stop。

---

## ROS 桥接

`ros_bridge.py` 节点名：`quad_web_bridge`

| 方向 | 话题 | QoS |
|------|------|-----|
| 发布 | `/cmd_vel` | RELIABLE, depth 1 |
| 发布 | `/quad/teleop/{stand,neutral,crouch}` | RELIABLE |
| 订阅 | `/quad/body_mode` | 同步 UI 模式显示 |
| 订阅 | `/quad/joint_states` | 关节遥测 |
| 订阅 | `/quad/wheel_states` | 轮子遥测 |

- 控制环：**50 Hz** 读 `Latest.tick()` 发 Twist
- `ros_connected`：`/cmd_vel` 有订阅者即为 true
- 超时：1.2 s 无 drive 则发零速（与 teleop `cmd_timeout_ms: 600` 配合）

---

## 配置与限幅

启动时读取 `robot_ws/config/quad_teleop_params.yaml`：

| 参数 | 默认 | UI 中体现 |
|------|------|-----------|
| max_vx / max_vy | 0.8 m/s | `base_limits` × SPEED 倍率 |
| max_omega | 1.5 rad/s | 同上 |
| active_wheels | [1, 2] | 后两轮显示 OFF |

Web 端 `limits` = 基值 × **1.5**（SPEED 滑条最大倍率上限）。

---

## 测试

```bash
cd ~/quad_remote
PYTHONPATH=.:.deps python3 -m pytest tests/ -q
```

覆盖：`core.py` 最新值逻辑、`server.py` WebSocket 路由与状态率。

---

## 排障

| 现象 | 处理 |
|------|------|
| ROS 未连接 | 先 `run-quad-all.sh`；查 `ros2 node list \| grep teleop` |
| 无控制权 | 关闭其他标签页；刷新后重连 |
| 摇杆无效 | 确认横屏、未开详情、非 DEMO |
| WS 延迟 >500ms | 换 5GHz WiFi；靠近 AP；查 `/tmp/quad_remote.log` |
| 轮子方向反 | 改 `c620_quad_params.yaml` 的 `wheel_sign` |

```bash
tail -f /tmp/quad_remote.log
curl http://127.0.0.1:8765/health
ros2 topic hz /cmd_vel
```

---

## 源码结构

```
quad_remote/
├── server.py        # aiohttp Web + WS  handler
├── core.py          # Latest 存储、限幅、tick
├── ros_bridge.py    # rclpy 桥
├── run.sh           # 启动入口
├── start-all.sh     # robot_ws + Web 一键
├── static/
│   ├── index.html   # QUAD Remote UI
│   ├── app.js       # 摇杆、WS、遥测
│   └── style.css
├── tests/
└── requirements.txt
```

---

**仓库：** 本目录同步于 [liu-big/robot_ws](https://github.com/liu-big/robot_ws/tree/main/quad_remote) · 也可独立放在 `~/quad_remote`
