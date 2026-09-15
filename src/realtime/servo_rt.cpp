// servo_rt — 500 Hz 实时电机环骨架
// 编译: make -C ~/robot_ws/src/realtime
// 运行: sudo chrt -f 90 taskset -c 3 ./servo_rt

#include "shm_interface.h"

#include <sched.h>
#include <time.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cerrno>

static constexpr int kCpu = 3;
static constexpr int kFifoPrio = 90;
static constexpr int kRateHz = 500;
static constexpr long kPeriodNs = 1000000000L / kRateHz;

static bool set_realtime(int cpu, int prio) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0) {
        perror("sched_setaffinity");
        return false;
    }
    struct sched_param sp{};
    sp.sched_priority = prio;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("sched_setscheduler");
        return false;
    }
    return true;
}

static void sleep_until(timespec& next) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    next.tv_nsec += kPeriodNs;
    if (next.tv_nsec >= 1000000000L) {
        next.tv_nsec -= 1000000000L;
        next.tv_sec += 1;
    }
}

int main() {
    if (!set_realtime(kCpu, kFifoPrio)) {
        fprintf(stderr, "需要 realtime 组权限: sudo usermod -aG realtime $USER\n");
        return 1;
    }

    RobotCommand cmd{};
    RobotState state{};
    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);

    uint64_t cycle = 0;
    uint64_t overruns = 0;

    printf("servo_rt 启动: %d Hz, CPU%d, SCHED_FIFO %d\n", kRateHz, kCpu, kFifoPrio);

    while (true) {
        timespec start{};
        clock_gettime(CLOCK_MONOTONIC, &start);

        // TODO: 从共享内存读 RobotCommand
        // TODO: SocketCAN 发帧到 can0 / can1
        // TODO: 收反馈写入 RobotState

        state.seq.store(++cycle);

        timespec end{};
        clock_gettime(CLOCK_MONOTONIC, &end);
        long elapsed_ns = (end.tv_sec - start.tv_sec) * 1000000000L
                        + (end.tv_nsec - start.tv_nsec);
        if (elapsed_ns > kPeriodNs) {
            ++overruns;
            state.cycle_overrun_us = static_cast<uint32_t>(elapsed_ns / 1000);
        }

        if (cycle % (kRateHz * 5) == 0) {
            printf("cycle=%llu overruns=%llu\n",
                   static_cast<unsigned long long>(cycle),
                   static_cast<unsigned long long>(overruns));
        }

        sleep_until(next);
    }
    return 0;
}
