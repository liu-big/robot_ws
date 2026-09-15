# 灵足 USB_CANHUB 使用说明（Linux）

手册原件: `~/robot_ws/3668968132USB_CANHUB使用说明V1.0(1).pdf`

相关电机手册:
- 灵足 RS03 → `product_manual_robStride03-9c54db7a.pdf`（`docs/RS03.md`）
- 大疆 C620 → `RoboMaster C620无刷电机调速器使用说明（中英日）V1.01.pdf`（`docs/C620.md`）

## 设备概要

| 项目 | 说明 |
|------|------|
| 厂商 | 北京市灵足时代科技有限公司 ([robstride.com](https://www.robstride.com)) |
| 供电 | Type-C USB2.0 + 5V |
| CAN 路数 | **5 路** → Linux 下 `can0` ~ `can4` |
| 驱动 | **gs_usb**（candleLight 兼容，免驱） |
| 波特率 | **1 Mbps** |
| 终端电阻 | 拨码开关 **ON** = 接入 120Ω |
| 接线 | 2.54-3P：H=CAN_H, L=CAN_L, GND |

## 快速启动

```bash
# 插入 USB_CANHUB 后
~/robot_ws/scripts/usb-can-map.sh    # 查看映射
~/robot_ws/scripts/can-hub-up.sh can0 can1   # 启动 CAN（推荐）
~/robot_ws/scripts/can-hub-up.sh all         # 启动 can0~can4

# 测试
candump -tz can0
cansend can0 123#DEADBEEF
```

## 与 RobStride 官方例程

仓库已克隆到: `~/robot_ws/src/robstride_ros_sample`

官方 README: [RobStride/robstride_ros_sample](https://github.com/RobStride/robstride_ros_sample)

例程默认使用 **`can4`** 接口（与 HUB 第 5 路对应）:

```cpp
motor(RobStrideMotor("can4", 0xFF, 0x01, 0));
```

> 注意: 该例程依赖 **ROS2 Humble**。当前系统未安装 ROS2，若要用此例程需先装 ROS2。

## 硬件注意事项

1. **120Ω 终端电阻**: 总线两端各一个；HUB 拨码 ON 可启用一路，按实际拓扑配置
2. **USB 口**: 固定插在 **USB 3.0 (Bus 03)**，不要换口
3. **先不要接电机**: 确认 `candump` 能起来再通电

## Windows 上位机（可选）

手册 4.2~4.7 节: 用 .exe 上位机配置 5 路 CAN、1M 波特率、发指令测电机。Linux 开发可跳过。
