# 第四条腿测试指南

> **已过时** — 当前请用 [`scripts.md`](scripts.md) 与 [`quadruped.md`](quadruped.md)。整机 ID：leg4 抬升=4，拉杆=8。

## 机械与 CAN 分配

```
can0 (灵足 RS03 ×1, 扩展帧 29-bit)
  └── ID 0x7F  当前单电机台架（出厂默认）

规划（后续接上）:
  ├── ID 0x01  leg4_joint   腿关节
  └── ID 0x02  leg4_lift    抬腿
can1 → 大疆 C620 麦轮 (ID 1)  未接
```

**当前接线**: 仅 **can0** 一路、**一台** RS03（ID **0x7F**）。第二台 RS03 / C620 未接。

配置文件: `config/leg4.yaml`, `config/can.yaml`

## 手册

| 设备 | 文件 | 摘要 |
|------|------|------|
| USB_CANHUB | `3668968132USB_CANHUB使用说明V1.0(1).pdf` | `docs/USB_CANHUB.md` |
| 灵足 RS03 | `product_manual_robStride03-9c54db7a.pdf` | `docs/RS03.md` |
| 大疆 C620 | `RoboMaster C620无刷电机调速器使用说明（中英日）V1.01.pdf` | `docs/C620.md` |

## RS03 协议摘要

| 项目 | 值 |
|------|-----|
| 波特率 | **1 Mbps** |
| 帧类型 | **扩展 29-bit** |
| 主机 ID | `0xFF`（官方 ROS 例程） |
| 使能 | 通信类型 3 |
| 停止 | 通信类型 4 |
| 运控 | 通信类型 1（角度/角速度/力矩/Kp/Kd） |
| 反馈 | 通信类型 2 |
| ID 设置 | 参数 `CAN_ID`（0x2009）或上位机 |

## C620 协议摘要 (V1.01)

| 项目 | 值 |
|------|-----|
| 波特率 | **1 Mbps** |
| 帧类型 | **标准 11-bit** |
| 控制帧 | `0x200` (ID1~4), `0x1FF` (ID5~8) |
| 反馈帧 | `0x200 + esc_id` (ID1→`0x201`) |
| 电流指令 | int16, -16384~16384 → **±20A** |
| ID 设置 | C620 SET 键，范围 1~8 |
| 终端电阻 | C620 拨码开关 + 总线规范 |

## 测试顺序

### 1. 只启 can0

```bash
~/robot_ws/scripts/can-hub-up.sh        # 默认只启 can0
~/robot_ws/scripts/rs-probe.sh          # 探测 0x01/0x02，不发运动指令
```

### 2. 灵足 RS03 — 关节 (can0)

```bash
candump -tz -e can0          # 另开终端
~/robot_ws/scripts/run-leg1-dual.sh  # 当前台架 leg1；leg4 待 quad 节点
```

改测抬腿电机: 编辑 `main.cpp` 中 `0x01` → `0x02`，重新 `colcon build`。

### 3. 大疆 C620 — 麦轮 (can1) — 当前未接

```bash
# C620 接上后:
~/robot_ws/scripts/can-hub-up.sh can0 can1
candump -tz can1             # 应见 0x201 反馈 (ID=1)
~/robot_ws/scripts/c620-send-current.sh 1 0.3   # 小电流试转
```

### 4. 安全

- C620 必须 **PWM/CAN 模式** 二选一，不能同时插线
- 24V 供电，先空载再带载
- 急停：断电或发送零电流 `cansend can1 200#0000000000000000`
