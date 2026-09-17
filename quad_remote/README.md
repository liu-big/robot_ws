# quad_remote — 四足手机遥控

横屏双摇杆 Web 遥控，经 WebSocket → ROS2 `/cmd_vel` → `quad_teleop_ros2`。

## 依赖

```bash
pip install -r requirements.txt
```

## 启动

与 `robot_ws` 一起一键启动：

```bash
~/quad_remote/start-all.sh
```

或单独启动（需 `run-quad-all.sh` 已运行）：

```bash
~/quad_remote/run.sh --host 0.0.0.0 --port 8765
```

手机访问：`http://<主机IP>:8765/`（横屏）

## 架构

```
浏览器 → WebSocket :8765 → server.py → ros_bridge.py → /cmd_vel
姿态按钮 → /quad/teleop/{stand,neutral,crouch}
```

## 测试

```bash
PYTHONPATH=.:.deps python3 -m pytest tests/ -q
```
