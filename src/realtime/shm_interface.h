#pragma once
// RL policy ↔ servo_rt 共享内存接口（后续扩展为 mmap 或 POSIX shm）

#include <cstdint>
#include <atomic>

struct JointCommand {
    float pos;
    float vel;
    float torque;
};

struct JointState {
    float pos;
    float vel;
    float torque;
    float temperature;
};

static constexpr int kNumJoints = 8;  // 4x M3508 + 4x RS03

struct RobotCommand {
    std::atomic<uint64_t> seq;
    JointCommand joints[kNumJoints];
};

struct RobotState {
    std::atomic<uint64_t> seq;
    JointState joints[kNumJoints];
    uint32_t cycle_overrun_us;
    uint32_t can0_tx_errors;
    uint32_t can1_tx_errors;
};
