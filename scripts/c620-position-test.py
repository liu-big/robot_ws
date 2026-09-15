#!/usr/bin/env python3
"""leg1+leg4 发布速度并读位置反馈 — 位置测试"""
import os
import subprocess
import time
import threading

DURATION = 4.0
VEL = 0.20
WS = os.path.expanduser("~/robot_ws")


def ros_env_cmd(inner):
    return (
        f"source /opt/ros/humble/setup.bash && "
        f"source {WS}/install/setup.bash && {inner}"
    )


def read_states(samples, stop):
    proc = subprocess.Popen(
        ["bash", "-lc", ros_env_cmd(
            "ros2 topic echo /quad/wheel_states --qos-reliability reliable")],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    field = None
    names = []
    pos = []
    vel = []
    for line in proc.stdout:
        if stop.is_set():
            break
        line = line.rstrip()
        if line == "name:":
            names = []
        elif line.startswith("- leg") and line.endswith("_wheel"):
            names.append(line[2:])
        elif line == "position:":
            field = "pos"
            pos = []
        elif line == "velocity:":
            field = "vel"
            vel = []
        elif line == "effort:":
            field = None
        elif line == "---":
            if len(names) >= 2 and len(pos) >= 2:
                samples.append({
                    "t": time.time(),
                    "names": list(names),
                    "pos": pos[:2],
                    "vel": vel[:2] if len(vel) >= 2 else [],
                })
            names = []
            pos = []
            vel = []
            field = None
        elif field == "pos" and line.startswith("- "):
            pos.append(float(line[2:]))
        elif field == "vel" and line.startswith("- "):
            vel.append(float(line[2:]))
    proc.kill()


def main():
    samples = []
    stop = threading.Event()
    th = threading.Thread(target=read_states, args=(samples, stop), daemon=True)
    th.start()
    time.sleep(4.0)

    if not samples:
        print("未收到 /quad/wheel_states，请先启动 c620_quad_ros2")
        stop.set()
        return 1

    p0 = samples[-1]
    t_pub = time.time()
    print("=== 位置测试：发布 leg1+leg4 前进 ===")
    print(f"速度指令: {VEL} rad/s × {DURATION}s  话题 data=[leg1,0,0,leg4]")
    print(f"起始位置: {dict(zip(p0['names'], [f'{x:.4f}' for x in p0['pos']]))}")

    pub = subprocess.Popen(
        ["bash", "-lc", ros_env_cmd(
            f"ros2 topic pub /quad/wheels/cmd std_msgs/msg/Float64MultiArray "
            f"'{{data: [{VEL}, 0.0, 0.0, {VEL}]}}' --rate 50 "
            f"--qos-reliability reliable --qos-durability transient_local")],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(DURATION)
    pub.kill()
    subprocess.run(
        ["bash", "-lc", ros_env_cmd(
            "ros2 topic pub /quad/wheels/cmd std_msgs/msg/Float64MultiArray "
            "'{data: [0.0, 0.0, 0.0, 0.0]}' --once "
            "--qos-reliability reliable --qos-durability transient_local")],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(0.8)
    stop.set()
    th.join(timeout=1)

    if len(samples) < 2:
        print("采样不足")
        return 1

    during = [s for s in samples if s["t"] >= t_pub]
    p1 = during[-1] if during else samples[-1]
    p0t = [s for s in samples if s["t"] < t_pub]
    p0 = p0t[-1] if p0t else samples[0]
    print(f"结束位置: {dict(zip(p1['names'], [f'{x:.4f}' for x in p1['pos']]))}")
    print("（注：position 为转子角，转多圈会回绕；看位移请用 velocity 或 candump）")
    d1 = p1["pos"][0] - p0["pos"][0]
    d2 = p1["pos"][1] - p0["pos"][1]
    print(
        f"转子角变化 Δ{p0['names'][0]}={d1:+.4f} rad  "
        f"Δ{p0['names'][1]}={d2:+.4f} rad  "
        f"差={abs(d1 - d2):.4f} rad"
    )
    if during:
        for i, nm in enumerate(p0["names"]):
            vs = [s["vel"][i] for s in during if len(s.get("vel", [])) > i]
            if vs:
                print(f"运动中 {nm} 均速={sum(vs)/len(vs):+.3f} rad/s")
    if p1.get("vel"):
        print(f"末速: {dict(zip(p1['names'], [f'{v:+.3f}' for v in p1['vel']]))}")
    print("=== 测试结束 ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
