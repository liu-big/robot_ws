# 四足机械狗 · 电机分布与协调控制路线

## 整机电机 ID（你已设好）

俯视整机，**内部**为抬升（带麦轮连杆），**外部**为拉杆（趴下/站起）：

```
        leg4 (左后)          leg3 (右后)
        抬升④  拉杆⑧          抬升③  拉杆⑦
              ┌──────┐
              │ 机身 │
              └──────┘
        抬升①  拉杆⑤          抬升②  拉杆⑥
        leg1 (左前)          leg2 (右前)
```

| 腿 | 位置 | 抬升 RS03（内） | 拉杆 RS03（外） | 麦轮 C620 |
|----|------|-----------------|-----------------|-----------|
| leg1 | 左前 | **1** | **5** | **1** |
| leg2 | 右前 | **2** | **6** | **2** |
| leg3 | 右后 | **3** | **7** | **3** |
| leg4 | 左后 | **4** | **8** | **4** |

- **ID 1~4**：抬升关节，驱动带轮子的腿杆  
- **ID 5~8**：外部拉杆，负责**趴下 / 站起**（机身高度）  
- **C620 ID 1~4**：四条腿的麦轮，与腿号对应  

配置文件：`config/quadruped.yaml`、`config/motors.yaml`

---

## CAN 接线（三路 HUB）

| CAN | 灵足 / 大疆 | 电机 ID |
|-----|-------------|---------|
| **can0** | 灵足 抬升 | 1, 2, 3, 4 |
| **can1** | 灵足 拉杆 | 5, 6, 7, 8 |
| **can2** | 大疆 C620 麦轮 | 1, 2, 3, 4 |

ROS 节点 `quad_rs_ros2` 使用 **lift_can_interface=can0**、**crouch_can_interface=can1**。  
`c620_ros2` 默认 **can2**。

探测：

```bash
~/robot_ws/scripts/can-hub-up.sh can0 can1 can2
~/robot_ws/scripts/rs-probe-all.sh
~/robot_ws/scripts/diagnose-c620.sh can2
```

---

## ROS 话题（现状 → 目标）

### 现已可用（leg1）

```text
/leg1/lift/command      抬升 ID=1
/leg1/crouch/command    拉杆 ID=5
/leg1/goto/stand        预设站立
/leg1/goto/crouch       预设微蹲
/c620/velocity_command  leg1 麦轮
```

### 协调层（后续开发）

```text
/quad/body_pose         整体站/蹲/高度指令
/quad/legs/command      8 关节目标（或保留 /legN/*）
/quad/wheels/cmd        4 轮速度 / 全向
/quad/joint_states      整机反馈
```

---

## 软件分期

1. **Phase 1（完成）** — leg1 双 RS03 + 单 C620，`leg1_dual_ros2`  
2. **Phase 2** — can0 接满 8 RS03，`rs-probe` 全通过  
3. **Phase 3（进行中）** — `quad_rs_ros2`：单进程控 8 关节，话题 `/leg1..4/lift|crouch/*`，台架 `active_legs: [1]`  
4. **Phase 4** — `c620_quad_ros2`：单进程控 4 轮  
5. **Phase 5** — `quad_coord_node`：同步四腿站起/趴下、再接步态  

---

## leg1 快速测试（当前）

```bash
~/robot_ws/scripts/run-leg1-all.sh          # 后台启双节点
~/robot_ws/scripts/leg1-goto-pose.sh stand  # 预设站立
~/robot_ws/scripts/leg1-goto-pose.sh crouch # 预设微蹲
```

姿态标定值：`config/leg1_dual_params.yaml`（仅 leg1；其余腿接上后各建一份或并入 `quad_params.yaml`）。
