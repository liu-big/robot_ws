#!/bin/bash
# 每次开机后应用实时调优（RT 内核启动后执行）
echo -1 > /sys/module/usbcore/parameters/autosuspend 2>/dev/null || true
if command -v cpupower >/dev/null; then
    cpupower frequency-set -g performance >/dev/null 2>&1 || true
fi
