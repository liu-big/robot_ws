# 大疆 C620 · can2 ROS2 控制

> **C620 四轮接 HUB 第 3 路 → `can2`**

## 启动

```bash
~/robot_ws/scripts/can-hub-up.sh can2
~/robot_ws/scripts/run-c620.sh can2
```

或参数：

```bash
ros2 run rs_motor_ros2 c620_ros2 --ros-args -p can_interface:=can2
```

## 诊断

```bash
~/robot_ws/scripts/diagnose-c620.sh can2
timeout 3 candump -tz can2,201:7FF | head -5
```

## 与灵足联调

```bash
~/robot_ws/scripts/can-hub-up.sh can0 can1 can2
~/robot_ws/scripts/run-quad-rs.sh    # can0 抬升 + can1 拉杆
~/robot_ws/scripts/run-c620.sh can2  # 麦轮
```

## 急停

```bash
~/robot_ws/scripts/c620-estop.sh
# 或 export C620_CAN=can2
```
