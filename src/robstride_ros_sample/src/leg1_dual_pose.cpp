#include "motor_ros2/motor_cfg.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_stop{false};

void on_sigint(int) { g_stop.store(true); }

struct SmoothJointTrajectory {
  float pos{0.f};
  float vel{0.f};

  void reset(float p) {
    pos = p;
    vel = 0.f;
  }

  void step(float target, float dt, float max_vel, float max_accel) {
    if (dt <= 0.f) {
      return;
    }
    const float err = target - pos;
    if (std::abs(err) < 1e-4f) {
      pos = target;
      vel = 0.f;
      return;
    }
    const float direction = (err >= 0.f) ? 1.f : -1.f;
    const float stop_vel = std::sqrt(2.f * max_accel * std::abs(err));
    const float desired_vel = direction * std::min(max_vel, stop_vel);
    const float dv_max = max_accel * dt;
    float dv = desired_vel - vel;
    if (dv > dv_max) {
      dv = dv_max;
    } else if (dv < -dv_max) {
      dv = -dv_max;
    }
    vel += dv;
    pos += vel * dt;
    if ((target - pos) * direction <= 0.f) {
      pos = target;
      vel = 0.f;
    }
  }
};

bool init_motor(RobStrideMotor &m, float kp, float kd, const char *label) {
  std::printf("[*] 初始化 %s (ID=%u)...\n", label, m.motor_id);
  if (!m.init_motion_mode(kp, kd)) {
    std::fprintf(stderr, "[!] %s init_motion_mode 失败\n", label);
    return false;
  }
  for (int i = 0; i < 8; ++i) {
    m.send_motion_hold(m.get_position(), 0.f, kp, kd);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::printf("[✓] %s 当前 %.3f rad  T=%.1f°C\n", label, m.get_position(),
              m.get_temperature());
  return true;
}

float parse_arg(int argc, char **argv, int idx, float default_v) {
  if (idx >= argc) {
    return default_v;
  }
  return static_cast<float>(std::atof(argv[idx]));
}

} // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  const std::string iface = "can0";
  const uint8_t lift_id = 1;
  const uint8_t crouch_id = 5;
  const float lift_delta = parse_arg(argc, argv, 1, 0.08f);
  const float crouch_delta = parse_arg(argc, argv, 2, 0.15f);
  const float kp = 35.f;
  const float kd = 2.f;
  const float max_vel = 0.12f;
  const float max_accel = 0.4f;
  const int hz = 100;
  const float dt = 1.f / static_cast<float>(hz);

  std::printf("=== leg1 双电机小幅度位姿 ===\n");
  std::printf("can0 | 抬升 ID=%u Δ=%+.3f rad | 趴下 ID=%u Δ=%+.3f rad\n",
              lift_id, lift_delta, crouch_id, crouch_delta);
  std::printf("v_max=%.2f rad/s  a_max=%.2f  kp=%.0f kd=%.1f\n", max_vel,
              max_accel, kp, kd);

  RobStrideMotor lift(iface, 0xFF, lift_id, 3);
  RobStrideMotor crouch(iface, 0xFF, crouch_id, 3);

  if (!init_motor(lift, kp, kd, "抬升") || !init_motor(crouch, kp, kd, "趴下")) {
    return 1;
  }

  const float lift_start = lift.get_position();
  const float crouch_start = crouch.get_position();
  const float lift_target = lift_start + lift_delta;
  const float crouch_target = crouch_start + crouch_delta;

  SmoothJointTrajectory lift_traj;
  SmoothJointTrajectory crouch_traj;
  lift_traj.reset(lift_start);
  crouch_traj.reset(crouch_start);

  std::printf("→ 目标: 抬升 %.3f→%.3f  趴下 %.3f→%.3f\n", lift_start,
              lift_target, crouch_start, crouch_target);

  bool reached = false;
  int tick = 0;
  while (!g_stop.load()) {
    lift_traj.step(lift_target, dt, max_vel, max_accel);
    crouch_traj.step(crouch_target, dt, max_vel, max_accel);

    lift.send_motion_hold(lift_traj.pos, lift_traj.vel, kp, kd);
    crouch.send_motion_hold(crouch_traj.pos, crouch_traj.vel, kp, kd);

    if (++tick % hz == 0) {
      std::printf("  抬升 %.3f→%.3f  趴下 %.3f→%.3f\n", lift.get_position(),
                  lift_target, crouch.get_position(), crouch_target);
    }

    const float lift_err = std::abs(lift_target - lift.get_position());
    const float crouch_err = std::abs(crouch_target - crouch.get_position());
    const bool traj_done =
        std::abs(lift_traj.pos - lift_target) < 0.005f &&
        std::abs(crouch_traj.pos - crouch_target) < 0.005f &&
        std::abs(lift_traj.vel) < 0.02f && std::abs(crouch_traj.vel) < 0.02f;
    if (!reached && traj_done &&
        lift_err < 0.05f && crouch_err < 0.05f) {
      reached = true;
      std::printf("[✓] 到位: 抬升 %.3f rad  趴下 %.3f rad  (保持中, Ctrl+C 退出)\n",
                  lift.get_position(), crouch.get_position());
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / hz));
  }

  std::printf("\n[*] 保持当前位置后退出\n");
  for (int i = 0; i < 20; ++i) {
    lift.send_motion_hold(lift.get_position(), 0.f, kp, kd);
    crouch.send_motion_hold(crouch.get_position(), 0.f, kp, kd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
