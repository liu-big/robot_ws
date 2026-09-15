#!/usr/bin/env python3
"""C620 双轮同步测试 — 用法: ros2 环境 + c620_quad 节点已启动后运行本脚本"""
import struct
import subprocess
import time
import socket
import select
import threading

G = 19.0
TARGET = 0.20
# 与 c620_quad_params.yaml 中 wheel_sign 保持一致
SIGN = [-1.0, -1.0]


def wrpm(r):
    return r / G * 6.28318 / 60


def can_loop(samples, stop):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind(("can2",))
    s.setblocking(False)
    lw1 = lw2 = None
    while not stop.is_set():
        r, _, _ = select.select([s], [], [], 0.005)
        if not r:
            continue
        t = time.time() - samples["t0"]
        while True:
            try:
                f = s.recv(16)
            except BlockingIOError:
                break
            cid = struct.unpack("=I", f[:4])[0] & 0x7FF
            d = f[8:16]
            raw = struct.unpack(">h", d[2:4])[0]
            if cid == 0x201:
                lw1 = wrpm(raw) * SIGN[0]
            elif cid == 0x202:
                lw2 = wrpm(raw) * SIGN[1]
            if lw1 is not None and lw2 is not None:
                samples["can"].append((t, lw1, lw2, abs(lw1 - lw2)))
                lw1 = lw2 = None


def ros_loop(samples, stop):
    proc = subprocess.Popen(
        ["ros2", "topic", "echo", "/quad/wheel_states",
         "--qos-reliability", "reliable"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
    )
    field = None
    vel = []
    eff = []
    for line in proc.stdout:
        if stop.is_set():
            break
        line = line.rstrip()
        if line == "velocity:":
            field = "vel"
            vel = []
        elif line == "effort:":
            field = "eff"
            eff = []
        elif line == "---":
            if len(vel) >= 2 and len(eff) >= 2:
                t = time.time() - samples["t0"]
                samples["ros"].append(
                    (t, vel[0], vel[1], abs(vel[0] - vel[1]), eff[0], eff[1])
                )
            field = None
        elif field == "vel" and line.startswith("- "):
            vel.append(float(line[2:]))
        elif field == "eff" and line.startswith("- "):
            eff.append(float(line[2:]))
    proc.kill()


def ph(label, data, a, b, effort=False):
    ds = [d for d in data if a <= d[0] < b]
    if not ds:
        print(f"{label}: 无数据")
        return
    w1 = sum(x[1] for x in ds) / len(ds)
    w2 = sum(x[2] for x in ds) / len(ds)
    diffs = [x[3] for x in ds]
    msg = (
        f"{label}: leg1={w1:+.3f} leg2={w2:+.3f} "
        f"差avg={sum(diffs)/len(diffs):.3f} 差max={max(diffs):.3f} rad/s"
    )
    if effort:
        msg += (
            f" | I1={sum(x[4] for x in ds)/len(ds):+.2f}A "
            f"I2={sum(x[5] for x in ds)/len(ds):+.2f}A"
        )
    print(msg)


def main():
    pub = subprocess.Popen(
        ["ros2", "topic", "pub", "/quad/wheels/cmd",
         "std_msgs/msg/Float64MultiArray",
         f"{{data: [{TARGET}, {TARGET}, 0.0, 0.0]}}",
         "--rate", "50",
         "--qos-reliability", "reliable",
         "--qos-durability", "transient_local"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    samples = {"t0": time.time(), "can": [], "ros": []}
    stop = threading.Event()
    threading.Thread(target=can_loop, args=(samples, stop), daemon=True).start()
    threading.Thread(target=ros_loop, args=(samples, stop), daemon=True).start()
    time.sleep(5.5)
    stop.set()
    pub.kill()
    subprocess.run(
        ["ros2", "topic", "pub", "/quad/wheels/cmd",
         "std_msgs/msg/Float64MultiArray",
         "{data: [0.0, 0.0, 0.0, 0.0]}",
         "--once",
         "--qos-reliability", "reliable",
         "--qos-durability", "transient_local"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    print(f"=== 同步测试 {TARGET} rad/s × 5s (sign={SIGN}) ===")
    for tag, data in [("CAN", samples["can"]), ("ROS", samples["ros"])]:
        print(f"--- {tag} ---")
        ph("起步 0~0.8s", data, 0, 0.8, tag == "ROS")
        ph("过渡 0.8~2s", data, 0.8, 2.0, tag == "ROS")
        ph("稳态 2~5s", data, 2.0, 5.0, tag == "ROS")
        if data:
            diffs = [x[3] for x in data]
            print(
                f"全程: 差avg={sum(diffs)/len(diffs):.3f} "
                f"差max={max(diffs):.3f} rad/s (n={len(data)})"
            )


if __name__ == "__main__":
    main()
