#include "motor_ros2/c620_motor.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <csignal>
#include <cstdio>
#include <fcntl.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <sys/file.h>
#include <thread>
#include <unistd.h>

namespace {
std::atomic<bool> g_sigint{false};
void on_sigint(int) { g_sigint.store(true); }

float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

struct PidResult {
  float out{0.f};
  float p{0.f};
  float i{0.f};
  float d{0.f};
  float err{0.f};
};

/// 速度环 PID：默认 P-only（vel_kd=0），D 需低通后再启用
struct Pid {
  float kp{0.f};
  float ki{0.f};
  float kd{0.f};
  float integral{0.f};
  float last_meas{0.f};
  float out_limit{10.f};

  void reset(float measured = 0.f) {
    integral = 0.f;
    last_meas = measured;
  }

  PidResult step(float target, float measured, float dt) {
    PidResult r;
    if (dt <= 0.f) {
      return r;
    }
    r.err = target - measured;
    integral += r.err * dt;
    if (ki > 1e-8f) {
      integral = clampf(integral, -out_limit / ki, out_limit / ki);
    }
    r.p = kp * r.err;
    r.i = ki * integral;
    r.d = 0.f;
    if (kd > 1e-8f) {
      const float d_meas = (measured - last_meas) / dt;
      r.d = -kd * d_meas;
    }
    last_meas = measured;
    r.out = clampf(r.p + r.i + r.d, -out_limit, out_limit);
    return r;
  }
};

struct ContinuousRotorAngle {
  float rad{0.f};
  uint16_t last_raw{0};
  bool init{false};

  float raw_to_rad(uint16_t raw) {
    return static_cast<float>(raw) / static_cast<float>(C620Motor::kAngleRawMax) *
           static_cast<float>(2.0 * M_PI);
  }

  float update(uint16_t raw) {
    if (!init) {
      rad = raw_to_rad(raw);
      last_raw = raw;
      init = true;
      return rad;
    }
    int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(last_raw);
    if (delta > C620Motor::kAngleRawMax / 2) {
      delta -= C620Motor::kAngleRawMax + 1;
    } else if (delta < -C620Motor::kAngleRawMax / 2) {
      delta += C620Motor::kAngleRawMax + 1;
    }
    rad += static_cast<float>(delta) / static_cast<float>(C620Motor::kAngleRawMax) *
           static_cast<float>(2.0 * M_PI);
    last_raw = raw;
    return rad;
  }
};

struct HoldStepResult {
  float out{0.f};
  float p{0.f};
  float d{0.f};
  float err{0.f};
};

/// 锁位：位置弹簧 + 转速阻尼（无积分）
struct SpringDamperHold {
  float kp{0.12f};
  float kd_rpm{0.004f};
  float deadband_rad{0.025f};
  float slew_a_per_s{15.f};
  float lpf_tau_s{0.05f};
  float last_out{0.f};

  void reset() { last_out = 0.f; }

  HoldStepResult step(float target_rad, float meas_rad, float meas_rpm, float dt,
                      float out_limit) {
    HoldStepResult r;
    r.err = target_rad - meas_rad;
    if (std::abs(r.err) < deadband_rad) {
      r.err = 0.f;
    }
    r.p = kp * r.err;
    r.d = -kd_rpm * meas_rpm;
    float out = r.p + r.d;
    out = clampf(out, -out_limit, out_limit);
    if (dt > 0.f && lpf_tau_s > 0.f) {
      const float alpha = dt / (lpf_tau_s + dt);
      out = last_out + alpha * (out - last_out);
    }
    if (dt > 0.f && slew_a_per_s > 0.f) {
      const float max_dv = slew_a_per_s * dt;
      out = clampf(out, last_out - max_dv, last_out + max_dv);
    }
    last_out = out;
    r.out = out;
    return r;
  }
};

struct RpmRamp {
  float value{0.f};

  void reset(float initial_rpm = 0.f) { value = initial_rpm; }

  float step(float target, float dt, float max_delta_rpm_s) {
    if (dt <= 0.f) {
      return value;
    }
    const float dv = max_delta_rpm_s * dt;
    const float err = target - value;
    if (std::abs(err) <= dv) {
      value = target;
    } else {
      value += (err > 0.f) ? dv : -dv;
    }
    return value;
  }
};

/// 摩擦前馈：库仑 + 粘性（随 |ramp_rpm| 线性增，不封顶）
/// 旧版在 full_rpm 处饱和 → 中高速 ff 恒定，改目标转速几乎只靠 PID 微调
float friction_feedforward(float ramp_rpm, float coulomb_a, float viscous_a,
                           float viscous_ref_rpm, float min_rpm) {
  const float mag = std::abs(ramp_rpm);
  if (mag < min_rpm) {
    return 0.f;
  }
  float ff = coulomb_a;
  if (viscous_ref_rpm > 1.f && viscous_a > 1e-6f) {
    ff += viscous_a * mag / viscous_ref_rpm;
  }
  return std::copysign(ff, ramp_rpm);
}

const char *mode_name(int mode) {
  switch (mode) {
  case 0:
    return "HOLD";
  case 1:
    return "VELOCITY";
  case 2:
    return "BRAKING";
  default:
    return "?";
  }
}

bool acquire_single_instance_lock() {
  const int fd = open("/tmp/c620_ros2.lock", O_CREAT | O_RDWR, 0644);
  if (fd < 0) {
    return true;
  }
  if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
    std::fprintf(stderr, "已有 c620_ros2 在运行 → pkill -9 c620_ros2\n");
    close(fd);
    return false;
  }
  return true;
}

} // namespace

class C620Node : public rclcpp::Node {
public:
  C620Node() : Node("c620_node") {
    const std::string can_iface =
        declare_parameter("can_interface", "can3");
    motor_id_ = static_cast<uint8_t>(declare_parameter("motor_id", 1));
    gear_ratio_ = declare_parameter("gear_ratio", 19.0);
    control_hz_ = declare_parameter("control_hz", 500);
    can_tx_hz_ = declare_parameter("can_tx_hz", 250);
    can_tx_idle_hz_ = declare_parameter("can_tx_idle_hz", 10);
    log_hz_ = declare_parameter("log_hz", 10);
    tx_fail_recover_threshold_ =
        declare_parameter("tx_fail_recover_threshold", 80);
    can_iface_reset_enable_ =
        declare_parameter("can_iface_reset_enable", false);
    can_iface_reset_min_s_ =
        declare_parameter("can_iface_reset_min_s", 120);
    cmd_timeout_ms_ = declare_parameter("cmd_timeout_ms", 1000);

    max_current_a_ = declare_parameter("max_current_a", 2.5);
    max_wheel_rad_s_ = declare_parameter("max_vel_rad_s", 1.0);
    vel_deadband_rad_s_ = declare_parameter("vel_deadband_rad_s", 0.05);
    rpm_ramp_rpm_s_ = declare_parameter("rpm_ramp_rpm_s", 400.0);
    brake_ramp_rpm_s_ = declare_parameter("brake_ramp_rpm_s", 400.0);
    brake_enter_hold_rpm_ = declare_parameter("brake_enter_hold_rpm", 15.0);
    brake_settle_ms_ = declare_parameter("brake_settle_ms", 300);

    vel_kp_ = declare_parameter("vel_kp", 0.025);
    vel_ki_ = declare_parameter("vel_ki", 0.012);
    vel_kd_ = declare_parameter("vel_kd", 0.0);
    vel_friction_coulomb_a_ =
        declare_parameter("vel_friction_coulomb_a", 0.4);
    vel_friction_a_ = declare_parameter("vel_friction_a", 0.9);
    vel_friction_full_rpm_ =
        declare_parameter("vel_friction_full_rpm", 145.0);
    vel_friction_min_rpm_ =
        declare_parameter("vel_friction_min_rpm", 1.0);

    hold_enable_ = declare_parameter("hold_enable", true);
    hold_kp_ = declare_parameter("hold_kp", 0.12);
    hold_kd_rpm_ = declare_parameter("hold_kd_rpm", 0.004);
    hold_deadband_rad_ = declare_parameter("hold_deadband_rad", 0.025);
    hold_slew_a_per_s_ = declare_parameter("hold_slew_a_per_s", 15.0);
    hold_lpf_tau_s_ = declare_parameter("hold_lpf_tau_s", 0.05);
    hold_max_current_a_ = declare_parameter("hold_max_current_a", 0.8);
    hold_active_rpm_ = declare_parameter("hold_active_rpm", 40.0);
    hold_stall_ms_ = declare_parameter("hold_stall_ms", 500);
    hold_heartbeat_ms_ = declare_parameter("hold_heartbeat_ms", 2000);

    joint_name_ = declare_parameter("joint_name", "c620_motor");
    vel_cmd_topic_ =
        declare_parameter("velocity_command_topic", "/c620/velocity_command");
    stop_topic_ = declare_parameter("stop_topic", "/c620/stop");
    state_topic_ = declare_parameter("state_topic", "/c620/state");

    can_iface_ = can_iface;
    motor_ = std::make_unique<C620Motor>(can_iface, motor_id_);
    last_iface_reset_tp_ = std::chrono::steady_clock::now();
    if (!motor_->is_ready()) {
      RCLCPP_ERROR(get_logger(), "CAN 打开失败: %s", can_iface.c_str());
    }

    speed_pid_.kp = static_cast<float>(vel_kp_);
    speed_pid_.ki = static_cast<float>(vel_ki_);
    speed_pid_.kd = static_cast<float>(vel_kd_);
    speed_pid_.out_limit = static_cast<float>(max_current_a_);

    hold_.kp = static_cast<float>(hold_kp_);
    hold_.kd_rpm = static_cast<float>(hold_kd_rpm_);
    hold_.deadband_rad = static_cast<float>(hold_deadband_rad_);
    hold_.slew_a_per_s = static_cast<float>(hold_slew_a_per_s_);
    hold_.lpf_tau_s = static_cast<float>(hold_lpf_tau_s_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

    velocity_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
        vel_cmd_topic_, qos,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          const float v = clampf(static_cast<float>(msg->data),
                                 static_cast<float>(-max_wheel_rad_s_),
                                 static_cast<float>(max_wheel_rad_s_));
          const float prev = target_wheel_rad_s_.load();
          target_wheel_rad_s_.store(v);
          have_cmd_.store(true);
          last_cmd_ns_.store(now().nanoseconds());
          if (std::abs(v - prev) > 0.02f) {
            pid_integral_reset_.store(true);
            RCLCPP_INFO(get_logger(), "速度目标 %.3f rad/s (wheel rpm≈%.0f)",
                        v, wheel_to_rotor_rpm(v));
          }
        });

    stop_sub_ = create_subscription<std_msgs::msg::Empty>(
        stop_topic_, qos, [this](const std_msgs::msg::Empty::SharedPtr) {
          stop_now("stop");
        });

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>(state_topic_, 10);
    worker_ = std::thread(&C620Node::control_loop, this);

    RCLCPP_INFO(get_logger(),
                "C620 can=%s | ctrl=%dHz TX=%dHz | vel Kp=%.4f Ki=%.4f Kd=%.4f "
                "Imax=%.2fA ff=%.2f+%.2fA@%.0frpm ramp=%.0frpm/s max=%.2f rad/s | %s",
                can_iface.c_str(), control_hz_, can_tx_hz_, vel_kp_, vel_ki_,
                vel_kd_, max_current_a_, vel_friction_coulomb_a_, vel_friction_a_,
                vel_friction_full_rpm_, rpm_ramp_rpm_s_, max_wheel_rad_s_,
                hold_enable_ ? "HOLD enabled" : "HOLD disabled");
  }

  ~C620Node() override {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    if (motor_) {
      motor_->set_current_a(0.f);
      motor_->transmit(true);
    }
  }

private:
  enum class Mode : int { Hold = 0, Velocity = 1, Braking = 2 };

  float wheel_to_rotor_rpm(float wheel_rad_s) const {
    return wheel_rad_s * static_cast<float>(gear_ratio_) * 60.f /
           static_cast<float>(2.0 * M_PI);
  }

  float rotor_to_wheel_rad_s(float rotor_rpm) const {
    return rotor_rpm / static_cast<float>(gear_ratio_) *
           static_cast<float>(2.0 * M_PI) / 60.f;
  }

  bool has_active_velocity_target() const {
    if (!have_cmd_.load()) {
      return false;
    }
    if (cmd_timeout_ms_ > 0) {
      const int64_t age_ms =
          (now().nanoseconds() - last_cmd_ns_.load()) / 1000000;
      if (age_ms > cmd_timeout_ms_) {
        return false;
      }
    }
    return std::abs(target_wheel_rad_s_.load()) >=
           static_cast<float>(vel_deadband_rad_s_);
  }

  void stop_now(const char *reason) {
    have_cmd_.store(false);
    target_wheel_rad_s_.store(0.f);
    RCLCPP_WARN(get_logger(), "停止 (%s) → BRAKING", reason);
  }

  void enter_braking(float meas_rpm, Mode &mode, float &ramp_rpm) {
    if (mode != Mode::Braking) {
      mode = Mode::Braking;
      speed_pid_.reset(meas_rpm);
      rpm_ramp_.reset(meas_rpm);
      ramp_rpm = meas_rpm;
      brake_low_rpm_ticks_ = 0;
      RCLCPP_INFO(get_logger(), "→ BRAKING (meas_rpm=%.0f)", meas_rpm);
    }
  }

  void enter_hold(float rotor_rad, Mode &mode, float &hold_target_rad,
                  bool &hold_latched) {
    mode = Mode::Hold;
    hold_target_rad = rotor_rad;
    hold_latched = true;
    hold_.reset();
    speed_pid_.reset();
    rpm_ramp_.reset(0.f);
    brake_low_rpm_ticks_ = 0;
    RCLCPP_INFO(get_logger(), "→ HOLD (rotor=%.3f rad)", hold_target_rad);
  }

  void enter_velocity(float meas_rpm, Mode &mode, float &ramp_rpm) {
    if (mode != Mode::Velocity) {
      mode = Mode::Velocity;
      speed_pid_.reset(meas_rpm);
      rpm_ramp_.reset(meas_rpm);
      ramp_rpm = meas_rpm;
      brake_low_rpm_ticks_ = 0;
      RCLCPP_INFO(get_logger(), "→ VELOCITY (meas_rpm=%.0f)", meas_rpm);
    }
  }

  /// 库仑 ff 固定；粘性 ff 随目标转速变。PID 限幅只预留库仑，避免中高速 ff 吃光余量
  float compute_motion_current(float ramp, float meas, float dt,
                               PidResult &pid_out, float &ff_out) {
    if (pid_integral_reset_.exchange(false)) {
      speed_pid_.integral = 0.f;
    }
    ff_out = friction_feedforward(
        ramp, static_cast<float>(vel_friction_coulomb_a_),
        static_cast<float>(vel_friction_a_),
        static_cast<float>(vel_friction_full_rpm_),
        static_cast<float>(vel_friction_min_rpm_));
    const float pid_limit = std::max(
        0.5f, static_cast<float>(max_current_a_) -
                  static_cast<float>(vel_friction_coulomb_a_));
    speed_pid_.out_limit = pid_limit;
    pid_out = speed_pid_.step(ramp, meas, dt);
    return clampf(pid_out.out + ff_out, -static_cast<float>(max_current_a_),
                  static_cast<float>(max_current_a_));
  }

  void control_loop() {
    const int hz = std::max(100, control_hz_);
    const auto period = std::chrono::microseconds(1000000 / hz);
    const float dt = 1.f / static_cast<float>(hz);

    float cmd_current = 0.f;
    float target_rpm = 0.f;
    float ramp_rpm = 0.f;
    float hold_target_rad = 0.f;
    float hold_err = 0.f;
    float hold_p = 0.f;
    float hold_d = 0.f;
    float friction_ff = 0.f;
    PidResult pid{};
    Mode mode = hold_enable_ ? Mode::Hold : Mode::Braking;
    bool hold_latched = false;
    int hold_stall_ticks = 0;
    int hold_heartbeat_ticks = 0;
    int log_div = 0;
    int tx_tick = 0;
    int recover_cooldown = 0;
    int recover_backoff_s = 2;
    brake_low_rpm_ticks_ = 0;

    const int tx_hz = std::max(50, std::min(can_tx_hz_, hz));
    const int tx_interval = std::max(1, hz / tx_hz);
    const int idle_tx_hz = std::max(5, can_tx_idle_hz_);
    const int idle_tx_interval = std::max(1, hz / idle_tx_hz);
    const int log_interval = std::max(1, hz / std::max(1, log_hz_));
    const int brake_settle_ticks =
        std::max(1, static_cast<int>(brake_settle_ms_ * hz / 1000));
    bool last_tx_ok = true;
    bool hold_initialized = false;

    while (running_.load() && !g_sigint.load()) {
      const auto t0 = std::chrono::steady_clock::now();

      const auto fb = motor_->poll_feedback();
      const float meas_rpm = static_cast<float>(fb.speed_rpm);
      const float rotor_rad = rotor_angle_.update(fb.angle_raw);
      const float meas_wheel = rotor_to_wheel_rad_s(meas_rpm);
      const float wheel_tgt = target_wheel_rad_s_.load();

      if (!hold_initialized && hold_enable_) {
        hold_target_rad = rotor_rad;
        hold_latched = true;
        hold_initialized = true;
      }

      const bool want_velocity = has_active_velocity_target();

      if (cmd_timeout_ms_ > 0 && have_cmd_.load() &&
          (now().nanoseconds() - last_cmd_ns_.load()) / 1000000 >
              cmd_timeout_ms_) {
        target_wheel_rad_s_.store(0.f);
        have_cmd_.store(false);
      }

      if (want_velocity) {
        enter_velocity(meas_rpm, mode, ramp_rpm);
        target_rpm = wheel_to_rotor_rpm(wheel_tgt);
        ramp_rpm = rpm_ramp_.step(target_rpm, dt,
                                  static_cast<float>(rpm_ramp_rpm_s_));
        cmd_current =
            compute_motion_current(ramp_rpm, meas_rpm, dt, pid, friction_ff);
        hold_latched = false;
      } else if (mode == Mode::Velocity) {
        enter_braking(meas_rpm, mode, ramp_rpm);
      }

      if (!want_velocity && mode == Mode::Braking) {
        target_rpm = 0.f;
        ramp_rpm = rpm_ramp_.step(0.f, dt, static_cast<float>(brake_ramp_rpm_s_));
        cmd_current =
            compute_motion_current(ramp_rpm, meas_rpm, dt, pid, friction_ff);

        if (std::abs(meas_rpm) <
            static_cast<float>(brake_enter_hold_rpm_)) {
          ++brake_low_rpm_ticks_;
        } else {
          brake_low_rpm_ticks_ = 0;
        }

        if (hold_enable_ &&
            brake_low_rpm_ticks_ >= brake_settle_ticks) {
          enter_hold(rotor_rad, mode, hold_target_rad, hold_latched);
        } else if (!hold_enable_ &&
                   brake_low_rpm_ticks_ >= brake_settle_ticks) {
          mode = Mode::Hold;
          cmd_current = 0.f;
          hold_latched = false;
          pid = {};
        }
      }

      if (!want_velocity && mode == Mode::Hold && hold_enable_) {
        if (!hold_latched) {
          enter_hold(rotor_rad, mode, hold_target_rad, hold_latched);
        }
        const auto hold = hold_.step(hold_target_rad, rotor_rad, meas_rpm, dt,
                                     static_cast<float>(hold_max_current_a_));
        hold_err = hold.err;
        hold_p = hold.p;
        hold_d = hold.d;
        cmd_current = hold.out;
        friction_ff = 0.f;
        target_rpm = 0.f;
        ramp_rpm = 0.f;
        pid = {};

        const float abs_cmd = std::abs(cmd_current);
        const float abs_fb = std::abs(fb.current_a);
        const int fb_age = motor_->fb_age_ms();
        const bool fb_stale = fb_age < 0 || fb_age > 200;
        if (abs_cmd > 0.3f && abs_fb < abs_cmd * 0.25f &&
            std::abs(meas_rpm) < 25.f && !fb_stale) {
          ++hold_stall_ticks;
        } else {
          hold_stall_ticks = 0;
        }
        const int stall_limit =
            std::max(1, static_cast<int>(hold_stall_ms_ * hz / 1000));
        if (hold_stall_ticks >= stall_limit) {
          RCLCPP_WARN(get_logger(),
                      "锁位输出 %.2fA 但反馈 %.2fA → 重锁当前角",
                      cmd_current, fb.current_a);
          hold_target_rad = rotor_rad;
          hold_err = 0.f;
          hold_.reset();
          cmd_current = 0.f;
          hold_stall_ticks = 0;
        }
      } else if (!want_velocity && mode == Mode::Hold && !hold_enable_) {
        cmd_current = 0.f;
      }

      const float i_limit =
          (mode == Mode::Velocity || mode == Mode::Braking)
              ? static_cast<float>(max_current_a_)
              : static_cast<float>(hold_max_current_a_);
      cmd_current = clampf(cmd_current, -i_limit, i_limit);
      motor_->set_current_a(cmd_current);

      const bool motion_active =
          (mode == Mode::Velocity || mode == Mode::Braking);
      const bool need_tx = motion_active || hold_enable_;
      bool tx_ok = last_tx_ok;
      bool hold_disturbed = false;
      if (mode == Mode::Hold && hold_latched) {
        hold_disturbed =
            std::abs(hold_err) > static_cast<float>(hold_deadband_rad_) ||
            std::abs(meas_rpm) > static_cast<float>(hold_active_rpm_);
      }
      const int tx_period =
          motion_active ? tx_interval
                        : (hold_disturbed ? tx_interval : idle_tx_interval);
      bool force_tx = motion_active;
      if (mode == Mode::Hold && hold_enable_) {
        const int hb_ticks =
            std::max(1, static_cast<int>(hold_heartbeat_ms_ * hz / 1000));
        if (++hold_heartbeat_ticks >= hb_ticks) {
          hold_heartbeat_ticks = 0;
          const int fb_age = motor_->fb_age_ms();
          if (!last_tx_ok || motor_->tx_fail_streak() > 0 ||
              fb_age < 0 || fb_age > 400) {
            force_tx = true;
          }
        }
      } else {
        hold_heartbeat_ticks = 0;
      }
      if (need_tx && (force_tx || ++tx_tick >= tx_period)) {
        tx_tick = 0;
        tx_ok = motor_->transmit(force_tx);
        last_tx_ok = tx_ok;
        bool recovered = false;
        if (!tx_ok &&
            motor_->tx_fail_streak() >=
                static_cast<uint32_t>(tx_fail_recover_threshold_) &&
            recover_cooldown <= 0) {
          recovered = try_recover_can(tx_ok);
          recover_cooldown = hz * recover_backoff_s;
          recover_backoff_s = std::min(recover_backoff_s * 2, 60);
        } else if (tx_ok) {
          recover_backoff_s = 2;
        }
        if (recovered && hold_latched) {
          hold_target_rad = rotor_rad;
          hold_err = 0.f;
          hold_.reset();
          hold_stall_ticks = 0;
          RCLCPP_INFO(get_logger(), "CAN 恢复 → 重锁 (rotor=%.3f rad)",
                      hold_target_rad);
        }
      }
      if (recover_cooldown > 0) {
        --recover_cooldown;
      }

      if (++log_div >= log_interval) {
        log_div = 0;
        const int fb_age = motor_->fb_age_ms();
        if (mode == Mode::Hold) {
          RCLCPP_INFO(get_logger(),
                      "mode=HOLD wheel_tgt=%.3f hold_tgt=%.3f rotor=%.3f "
                      "pos_err=%.4f hold_P=%.3f hold_D=%.3f meas_rpm=%.0f "
                      "cmd=%.3fA fb=%.3fA T=%u fb_age=%dms%s",
                      wheel_tgt, hold_target_rad, rotor_rad, hold_err, hold_p,
                      hold_d, meas_rpm, cmd_current, fb.current_a,
                      fb.temperature_c, fb_age, tx_ok ? "" : " TX_FAIL");
        } else {
          RCLCPP_INFO(get_logger(),
                      "mode=%s wheel_tgt=%.3f tgt_rpm=%.0f ramp_rpm=%.0f "
                      "meas_rpm=%.0f err_rpm=%.0f P=%.3f I=%.3f D=%.3f "
                      "ff=%.3f cmd=%.3fA fb=%.3fA T=%u fb_age=%dms%s",
                      mode_name(static_cast<int>(mode)), wheel_tgt, target_rpm,
                      ramp_rpm, meas_rpm, pid.err, pid.p, pid.i, pid.d,
                      friction_ff, cmd_current, fb.current_a, fb.temperature_c,
                      fb_age, tx_ok ? "" : " TX_FAIL");
        }
        publish_state(rotor_rad, meas_wheel, fb);
      }

      const auto elapsed = std::chrono::steady_clock::now() - t0;
      if (elapsed < period) {
        std::this_thread::sleep_for(period - elapsed);
      }
    }

    motor_->set_current_a(0.f);
    motor_->transmit(true);
  }

  bool try_recover_can(bool &tx_ok) {
    RCLCPP_WARN(get_logger(), "CAN TX 连续失败 → 重连 socket (%s)",
                can_iface_.c_str());
    motor_->drain_rx(64);
    for (int attempt = 0; attempt < 4; ++attempt) {
      motor_->recover_socket();
      usleep(50000 * (attempt + 1));
      tx_ok = motor_->transmit(true);
      if (tx_ok) {
        RCLCPP_INFO(get_logger(), "socket 重连成功");
        return true;
      }
    }

    if (!can_iface_reset_enable_) {
      RCLCPP_ERROR(get_logger(),
                   "socket 重连仍失败。请手动: sudo ~/robot_ws/scripts/"
                   "can-hub-up.sh %s  然后重启节点",
                   can_iface_.c_str());
      return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto since_reset = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_iface_reset_tp_);
    if (since_reset.count() < can_iface_reset_min_s_) {
      RCLCPP_WARN(get_logger(),
                  "跳过接口重置（冷却 %lds），仅 socket 已重试",
                  can_iface_reset_min_s_ - since_reset.count());
      return false;
    }

    if (!reset_can_interface_via_script()) {
      RCLCPP_ERROR(get_logger(),
                   "接口重置失败（需 sudo 免密）。请手动: sudo "
                   "~/robot_ws/scripts/can-hub-up.sh %s",
                   can_iface_.c_str());
      return false;
    }

    last_iface_reset_tp_ = now;
    motor_->recover_socket();
    usleep(200000);
    tx_ok = motor_->transmit(true);
    if (tx_ok) {
      RCLCPP_INFO(get_logger(), "CAN 接口重置成功");
      return true;
    }
    RCLCPP_ERROR(get_logger(), "接口重置后 TX 仍失败");
    return false;
  }

  bool reset_can_interface_via_script() const {
    const char *home = std::getenv("HOME");
    if (home == nullptr) {
      return false;
    }
    const std::string script = std::string(home) +
                               "/robot_ws/scripts/can-hub-up.sh " + can_iface_;
    RCLCPP_WARN(get_logger(), "重置 CAN %s（sudo -n %s）", can_iface_.c_str(),
                script.c_str());
    const std::string cmd = "sudo -n " + script + " >/dev/null 2>&1";
    return std::system(cmd.c_str()) == 0;
  }

  void publish_state(float rotor_rad, float wheel_rad_s,
                     const C620Motor::Feedback &fb) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = {joint_name_};
    js.position = {rotor_rad / static_cast<double>(gear_ratio_)};
    js.velocity = {wheel_rad_s};
    js.effort = {fb.current_a};
    state_pub_->publish(js);
  }

  uint8_t motor_id_{1};
  int control_hz_{500};
  int can_tx_hz_{250};
  int can_tx_idle_hz_{10};
  int log_hz_{10};
  int tx_fail_recover_threshold_{80};
  bool can_iface_reset_enable_{false};
  int can_iface_reset_min_s_{120};
  std::chrono::steady_clock::time_point last_iface_reset_tp_{};
  std::string can_iface_;
  int cmd_timeout_ms_{1000};
  double gear_ratio_{19.0};
  double max_current_a_{2.5};
  double max_wheel_rad_s_{1.0};
  double vel_deadband_rad_s_{0.05};
  double rpm_ramp_rpm_s_{400.0};
  double brake_ramp_rpm_s_{400.0};
  double brake_enter_hold_rpm_{15.0};
  int brake_settle_ms_{300};
  bool hold_enable_{true};
  double hold_kp_{0.12};
  double hold_kd_rpm_{0.004};
  double hold_deadband_rad_{0.025};
  double hold_slew_a_per_s_{15.0};
  double hold_lpf_tau_s_{0.05};
  double hold_max_current_a_{0.8};
  double hold_active_rpm_{40.0};
  int hold_stall_ms_{500};
  int hold_heartbeat_ms_{2000};
  double vel_kp_{0.018};
  double vel_ki_{0.008};
  double vel_kd_{0.0};
  double vel_friction_coulomb_a_{0.4};
  double vel_friction_a_{0.9};
  double vel_friction_full_rpm_{145.0};
  double vel_friction_min_rpm_{1.0};
  int brake_low_rpm_ticks_{0};
  std::string joint_name_;
  std::string vel_cmd_topic_;
  std::string stop_topic_;
  std::string state_topic_;

  std::atomic<bool> running_{true};
  std::atomic<bool> have_cmd_{false};
  std::atomic<bool> pid_integral_reset_{false};
  std::atomic<float> target_wheel_rad_s_{0.f};
  std::atomic<int64_t> last_cmd_ns_{0};

  ContinuousRotorAngle rotor_angle_;
  Pid speed_pid_;
  SpringDamperHold hold_;
  RpmRamp rpm_ramp_;
  std::thread worker_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  std::unique_ptr<C620Motor> motor_;
};

int main(int argc, char **argv) {
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  if (!acquire_single_instance_lock()) {
    return 1;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<C620Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
