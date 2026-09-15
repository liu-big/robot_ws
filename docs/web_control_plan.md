# Web 遥控开发计划

> 目标：浏览器遥控四足机械狗 — 8 关节（灵足 RS03）+ 4 麦轮（大疆 C620）  
> 当前硬件：8×灵足在线；麦轮 **ID1+ID2** 在线（ID3/4 待接）  
> 状态：**驱动层 + 遥控协调层已就绪**，Web 端待开发

---

## 1. 系统架构

```
┌─────────────┐     WebSocket      ┌──────────────────┐
│  浏览器 UI   │ ◄──────────────► │ rosbridge_server │  (待部署)
│  摇杆/按钮   │                   │ 或 Foxglove      │
└─────────────┘                   └────────┬─────────┘
                                           │ ROS2 话题
                                  ┌────────▼─────────┐
                                  │ quad_teleop_ros2 │  ✅ 已实现
                                  │  cmd_vel 麦轮 IK │
                                  │  pose → /quad/goto│
                                  └────────┬─────────┘
                          ┌────────────────┼────────────────┐
                          ▼                ▼                ▼
                   quad_rs_ros2      c620_quad_ros2    (未来步态)
                   8×灵足关节         4×麦轮电流
                   can0 + can1        can2
```

### 已实现节点

| 节点 | 作用 | 启动 |
|------|------|------|
| `quad_rs_ros2` | 8 关节， `active_legs` 控制使能哪些腿 | `run-quad-rs.sh` / `run-quad-all.sh` |
| `c620_quad_ros2` | 4 轮单帧 0x200，`active_wheels` 控制在线轮 | `run-quad-all.sh` |
| `quad_teleop_ros2` | `/cmd_vel` 麦轮 IK + 姿态转发 | `run-quad-all.sh` |

台架单腿仍可用：`run-leg1-all.sh`（`active_legs: [1]` + 单轮 `c620_ros2`）

---

## 2. ROS 话题契约（Web / 遥控统一 API）

### 输入（Web / 手柄 → 机器人）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 麦轮运动：`linear.x/y`，`angular.z` |
| `/quad/teleop/pose` | `std_msgs/String` | `"stand"` / `"crouch"` / `"neutral"` |
| `/quad/teleop/stand` | `std_msgs/Empty` | 一键站起 |
| `/quad/teleop/crouch` | `std_msgs/Empty` | 一键趴下 |
| `/quad/teleop/neutral` | `std_msgs/Empty` | 中性姿态 |
| `/quad/e_stop` | `std_msgs/Empty` | 急停：停轮 + 锁关节 |
| `/quad/goto/{stand,crouch,neutral}` | `std_msgs/Empty` | 直接调姿态（可跳过 teleop） |
| `/quad/wheels/cmd` | `Float64MultiArray` | 4 轮角速度 rad/s `[FL,FR,RR,RL]` |
| `/leg{N}/lift/command` | `Float64` | 单关节弧度（高级调试） |

**QoS**：订阅/发布均需 `reliable` + `transient_local`（与现有脚本一致）。

### 输出（监控）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/quad/joint_states` | `JointState` | 8 腿关节聚合 |
| `/quad/wheel_states` | `JointState` | 4 麦轮状态 |
| `/leg{N}/lift/state` | `JointState` | 单腿抬升反馈 |

---

## 3. 一键启动与命令行遥控

```bash
# 全机（8 腿 + 2 轮 + 遥控协调）
~/robot_ws/scripts/run-quad-all.sh

# 站起
~/robot_ws/scripts/quad-goto-pose.sh stand

# 前进（麦轮）
source ~/robot_ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  --rate 20 --qos-reliability reliable --qos-durability transient_local

# 急停
ros2 topic pub /quad/e_stop std_msgs/msg/Empty '{}' --once \
  --qos-reliability reliable --qos-durability transient_local

# 停止全机
~/robot_ws/scripts/run-quad-all.sh stop
```

---

## 4. Web 端开发阶段

### Phase A — 基础连通（1~2 天）

- [ ] 安装 `rosbridge_server`：`sudo apt install ros-humble-rosbridge-suite`
- [ ] 启动：`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- [ ] 浏览器用 `roslibjs` 订阅 `/quad/joint_states`、`/quad/wheel_states`
- [ ] 验证：页面显示 8 关节 + 轮速实时数据

### Phase B — 控制面板（2~3 天）

- [ ] 姿态按钮：发布 `/quad/teleop/stand|crouch|neutral`
- [ ] 虚拟摇杆：发布 `/cmd_vel`（linear.x + angular.z）
- [ ] 红色急停：发布 `/quad/e_stop`
- [ ] 速度滑块上限（读 `quad_teleop_params.yaml` 的 max_vx 等）

### Phase C — 安全与体验（1~2 天）

- [ ] `cmd_vel` 超时：teleop 节点已实现 300ms 超时停轮
- [ ] Web 断连检测：停止发送 cmd_vel
- [ ] 仅站立模式允许麦轮（协调层加 `body_mode` 检查）
- [ ] 日志面板：订阅 `/rosout` 或写专用 `/quad/status`

### Phase D — 可选增强

- [ ] Foxglove Studio 替代自研 UI（更快落地）
- [ ] 游戏手柄：`joy_node` → `teleop_twist_joy` → `/cmd_vel`
- [ ] 录像：`ros2 bag record /quad/joint_states /quad/wheel_states`
- [ ] 四轮齐全后：`c620_quad_params.yaml` → `active_wheels: [1,2,3,4]`

---

## 5. Web 技术栈建议

| 组件 | 推荐 | 说明 |
|------|------|------|
| 通信 | **rosbridge WebSocket** | `ws://<robot_ip>:9090` |
| 前端 | Vue3 / React 或纯 HTML+JS | 虚拟摇杆可用 nipplejs |
| 部署 | 静态文件 nginx 或机器人本机 | 不跑在实时环内 |
| 认证 | 内网 VPN / 简单 token | 量产前再加 |

### 最小 Web 发布示例（roslibjs）

```javascript
const ros = new ROSLIB.Ros({ url: 'ws://192.168.x.x:9090' });
const cmdVel = new ROSLIB.Topic({
  ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist'
});
// 摇杆回调
cmdVel.publish(new ROSLIB.Message({
  linear: { x: vx, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: omega }
}));
```

---

## 6. 配置文件索引

| 文件 | 用途 |
|------|------|
| `config/quad_full_params.yaml` | 8 腿全使能 `active_legs: [1,2,3,4]` |
| `config/quad_rs_params.yaml` | 台架单腿 `active_legs: [1]` |
| `config/c620_quad_params.yaml` | 麦轮 `active_wheels: [1,2]` |
| `config/c620_params.yaml` | 单轮台架 `motor_id: 1` |
| `config/quad_teleop_params.yaml` | 遥控限速、轮距 |
| `config/quadruped.yaml` | 整机拓扑与 ID 表 |

---

## 7. 硬件待办

- [ ] 麦轮 ID3、ID4 接线 + SET 设 ID
- [ ] 四轮齐全后改 `active_wheels: [1, 2, 3, 4]`
- [ ] 麦轮 ID 与腿号对应：leg1→ID1 … leg4→ID4（见 `quadruped.yaml`）

---

## 8. 相关文档

- [quadruped.md](quadruped.md) — 整机拓扑
- [scripts.md](scripts.md) — 脚本索引
- [leg1_commands.md](leg1_commands.md) — 台架命令
- [c620_ros.md](c620_ros.md) — 单轮驱动说明
