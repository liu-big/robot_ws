# 手机遥控 QUAD Remote

横屏 Web 双摇杆遥控四足麦轮与姿态。源码：`quad_remote/`（或 `~/quad_remote`）。

![界面截图](images/quad-remote-ui.png)

---

## 快速开始

```bash
~/quad_remote/start-all.sh
# 手机横屏打开 http://<主机IP>:8765/
~/robot_ws/scripts/quad-goto-pose.sh stand
```

---

## 界面说明

| 区域 | 功能 |
|------|------|
| **TRANSLATION** 左摇杆 | VX 前后 + VY 横移 |
| **ROTATION** 右摇杆 | WZ 左旋/右旋 |
| **中央 CMD** | 当前 VX / VY / WZ 数值 |
| **俯视图 FL/FR/RL/RR** | 四轮速度；点击进入详细遥测 |
| **SPEED** | 0.40× ~ 1.50× 速度倍率 |
| **STAND / NEUTRAL / CROUCH** | 整机姿态 |
| **ROS 已连接** | teleop 链路就绪 |
| **WS ms** | WebSocket 延迟 |

**注意：** 必须横屏；竖屏禁驱。打开详情面板时禁驱。松手即停。

---

## 控制链路

```
摇杆 50Hz → WebSocket → server.py → ros_bridge.py → /cmd_vel
                                              ↓
                                    quad_teleop_ros2
                                              ↓
                              /quad/wheels/cmd → c620_quad (can2)

姿态按钮 → /quad/teleop/{stand,neutral,crouch}
              → /quad/goto/* → quad_rs (can0+can1, MIT 类运控)
```

---

## 实时性

| 环节 | 频率 |
|------|------|
| 浏览器 drive | 50 Hz |
| `/cmd_vel` 发布 | 50 Hz |
| teleop 轮速 | 100 Hz |
| 状态下发 | 5 Hz（全量 joints/wheels 约每 4 s） |

---

## 健康检查

```bash
curl -s http://127.0.0.1:8765/health | python3 -m json.tool
```

期望：`ok: true`，`ros_connected: true`，`teleop_connected: true`。

---

## 排障

| 现象 | 处理 |
|------|------|
| ROS 未连接 | `run-quad-all.sh`；查 `ros2 node list` |
| 摇杆无反应 | 横屏、关详情、刷新页面 |
| WS 延迟高 | 5GHz WiFi；`tail -f /tmp/quad_remote.log` |
| 轮子弱 | 调 SPEED；查 `config/c620_quad_params.yaml` |

完整协议与 API 见 [`quad_remote/README.md`](../quad_remote/README.md)。
