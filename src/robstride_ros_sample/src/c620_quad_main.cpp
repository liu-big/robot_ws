#include "motor_ros2/c620_motor.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fcntl.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <sys/file.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
std::atomic<bool> g_sigint{false};
void on_sigint(int) { g_sigint.store(true); }

float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

struct Pid {
  float kp{0.025f};
  float ki{0.012f};
  float kd{0.f};
  float integral{0.f};
  float last_meas{0.f};
  float out_limit{3.f};

  void reset(float m = 0.f) {
    integral = 0.f;
    last_meas = m;
  }

  float step(float target, float measured, float dt) {
    if (dt <= 0.f) {
      return 0.f;
    }
    const float err = target - measured;
    integral += err * dt;
    if (ki > 1e-8f) {
      integral = clampf(integral, -out_limit / ki, out_limit / ki);
    }
    float d = 0.f;
    if (kd > 1e-8f) {
      d = -kd * (measured - last_meas) / dt;
    }
    last_meas = measured;
    return clampf(kp * err + ki * integral + d, -out_limit, out_limit);
  }
};

float friction_ff(float ramp_rpm, float coulomb_a, float viscous_a,
                  float viscous_ref_rpm, float min_rpm) {
  const float mag = std::abs(ramp_rpm);
  if (mag < min_rpm) {
    return 0.f;
  }
  // 库仑 + 随转速线性增加的粘性前馈，使高速指令有明显更大电流
  float ff = coulomb_a;
  if (viscous_ref_rpm > 1.f && viscous_a > 1e-6f) {
    ff += viscous_a * mag / viscous_ref_rpm;
  }
  return std::copysign(ff, ramp_rpm);
}

bool acquire_lock() {
  const int fd = open("/tmp/c620_quad_ros2.lock", O_CREAT | O_RDWR, 0644);
  if (fd < 0) {
    return true;
  }
  if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
    std::fprintf(stderr, "已有 c620_quad_ros2 在运行\n");
    close(fd);
    return false;
  }
  return true;
}
} // namespace

struct WheelChannel {
  uint8_t motor_id{0};
  bool active{false};
  float sign{1.f};
  float target_wheel_rad_s{0.f};
  float last_cmd_wheel_rad_s{0.f};
  float ramp_rpm{0.f};
  Pid pid;
  std::string joint_name;
};

/// 四麦轮驱动 — 单 CAN 帧 0x200，话题 /quad/wheels/cmd
class C620QuadNode : public rclcpp::Node {
public:
  C620QuadNode() : Node("c620_quad_node") {
    const std::string can_iface =
        declare_parameter("can_interface", "can2");
    control_hz_ = declare_parameter("control_hz", 500);
    can_tx_hz_ = declare_parameter("can_tx_hz", 250);
    gear_ratio_ = declare_parameter("gear_ratio", 19.0);
    max_current_a_ = declare_parameter("max_current_a", 3.0);
    max_wheel_rad_s_ = declare_parameter("max_vel_rad_s", 1.0);
    vel_deadband_rad_s_ = declare_parameter("vel_deadband_rad_s", 0.05);
    rpm_ramp_rpm_s_ = declare_parameter("rpm_ramp_rpm_s", 400.0);
    vel_kp_ = declare_parameter("vel_kp", 0.025);
    vel_ki_ = declare_parameter("vel_ki", 0.012);
    vel_kd_ = declare_parameter("vel_kd", 0.0);
    vel_friction_coulomb_a_ = declare_parameter("vel_friction_coulomb_a", 1.0);
    vel_friction_a_ = declare_parameter("vel_friction_a", 0.3);
    vel_friction_full_rpm_ = declare_parameter("vel_friction_full_rpm", 145.0);
    vel_friction_min_rpm_ = declare_parameter("vel_friction_min_rpm", 1.0);
    cmd_timeout_ms_ = declare_parameter("cmd_timeout_ms", 500);
    sync_wheels_ = declare_parameter("sync_wheels", true);
    sync_gain_a_per_rpm_ = declare_parameter("sync_gain_a_per_rpm", 0.012);
    sync_corr_limit_a_ = declare_parameter("sync_corr_limit_a", 1.5);
    sync_catchup_frac_ = declare_parameter("sync_catchup_frac", 0.55);
    sync_catchup_min_rpm_ = declare_parameter("sync_catchup_min_rpm", 25.0);
    sync_vel_band_rpm_ = declare_parameter("sync_vel_band_rpm", 12.0);
    sync_reset_delta_rad_s_ =
        declare_parameter("sync_reset_delta_rad_s", 0.08);
    startup_current_a_ = declare_parameter("startup_current_a", 0.8);
    stop_brake_a_ = declare_parameter("stop_brake_a", 0.4);
    stop_brake_min_rpm_ = declare_parameter("stop_brake_min_rpm", 25.0);

    std::vector<int64_t> active_wheels =
        declare_parameter("active_wheels", std::vector<int64_t>{1, 2, 3, 4});
    auto wheel_sign_param =
        declare_parameter("wheel_sign", std::vector<double>{1, 1, 1, 1});
    auto wheel_motor_ids_param = declare_parameter(
        "wheel_motor_ids", std::vector<int64_t>{1, 2, 3, 4});

    bus_ = std::make_unique<C620Bus>(can_iface);
    if (!bus_->is_ready()) {
      RCLCPP_ERROR(get_logger(), "CAN 打开失败: %s", can_iface.c_str());
    }

    for (int leg = 1; leg <= 4; ++leg) {
      auto &w = wheels_[leg - 1];
      const size_t idx = static_cast<size_t>(leg - 1);
      int64_t mid = static_cast<int64_t>(leg);
      if (idx < wheel_motor_ids_param.size()) {
        mid = wheel_motor_ids_param[idx];
      }
      w.motor_id = static_cast<uint8_t>(mid);
      w.joint_name = "leg" + std::to_string(leg) + "_wheel";
      w.active = false;
      if (mid >= 1 && mid <= 4) {
        for (int64_t id : active_wheels) {
          if (id == leg) {
            w.active = true;
            break;
          }
        }
      }
      w.pid.kp = static_cast<float>(vel_kp_);
      w.pid.ki = static_cast<float>(vel_ki_);
      w.pid.kd = static_cast<float>(vel_kd_);
      w.pid.out_limit = static_cast<float>(max_current_a_);
      if (!sync_pid_initialized_) {
        sync_pid_.kp = static_cast<float>(vel_kp_);
        sync_pid_.ki = static_cast<float>(vel_ki_);
        sync_pid_.kd = static_cast<float>(vel_kd_);
        sync_pid_.out_limit = static_cast<float>(max_current_a_);
        sync_pid_initialized_ = true;
      }
      if (static_cast<size_t>(leg - 1) < wheel_sign_param.size()) {
        w.sign = static_cast<float>(wheel_sign_param[leg - 1]);
        if (std::abs(w.sign) < 1e-6f) {
          w.sign = 1.f;
        }
      }
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    wheels_cmd_sub_ =
        create_subscription<std_msgs::msg::Float64MultiArray>(
            "/quad/wheels/cmd", qos,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
              apply_wheel_cmd_array(msg);
            });

    for (int leg = 1; leg <= 4; ++leg) {
      const std::string topic =
          "/leg" + std::to_string(leg) + "/wheel/velocity_command";
      per_leg_subs_.push_back(create_subscription<std_msgs::msg::Float64>(
          topic, qos,
          [this, leg](const std_msgs::msg::Float64::SharedPtr msg) {
            if (!wheels_[leg - 1].active) {
              return;
            }
            wheels_[leg - 1].target_wheel_rad_s =
                clampf(static_cast<float>(msg->data),
                       static_cast<float>(-max_wheel_rad_s_),
                       static_cast<float>(max_wheel_rad_s_));
            have_cmd_.store(true);
            last_cmd_ns_.store(now().nanoseconds());
          }));
    }

    stop_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/quad/wheels/stop", qos,
        [this](const std_msgs::msg::Empty::SharedPtr) { stop_all(); });

    state_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("/quad/wheel_states", 10);

    worker_ = std::thread(&C620QuadNode::control_loop, this);

    std::string active_list;
    for (auto &w : wheels_) {
      if (w.active) {
        active_list += w.joint_name + "(id" +
                       std::to_string(w.motor_id) + ",sgn=" +
                       (w.sign >= 0.f ? "+" : "") +
                       std::to_string(static_cast<int>(w.sign)) + ") ";
      }
    }
    RCLCPP_INFO(get_logger(),
                "c620_quad can=%s | active wheels: %s| "
                "ff=%.2f+%.2fA@%.0frpm max=%.1fA kp=%.3f sync=%d",
                can_iface.c_str(), active_list.c_str(),
                vel_friction_coulomb_a_, vel_friction_a_,
                vel_friction_full_rpm_, max_current_a_, vel_kp_,
                static_cast<int>(sync_wheels_));
  }

  ~C620QuadNode() override {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    if (bus_) {
      bus_->zero_all();
      bus_->transmit(true);
    }
  }

private:
  float wheel_to_rpm(float wheel_rad_s) const {
    return wheel_rad_s * static_cast<float>(gear_ratio_) * 60.f /
           static_cast<float>(2.0 * M_PI);
  }

  float rpm_to_wheel(float rpm) const {
    return rpm / static_cast<float>(gear_ratio_) *
           static_cast<float>(2.0 * M_PI) / 60.f;
  }

  void sync_reset_ramp() {
    float sum = 0.f;
    int n = 0;
    for (auto &w : wheels_) {
      if (!w.active) {
        continue;
      }
      const auto fb = bus_->feedback(w.motor_id);
      sum += static_cast<float>(fb.speed_rpm) * w.sign;
      ++n;
      w.pid.reset(static_cast<float>(fb.speed_rpm) * w.sign);
    }
    shared_ramp_rpm_ = (n > 0) ? (sum / static_cast<float>(n)) : 0.f;
    sync_pid_.reset(shared_ramp_rpm_);
  }

  void maybe_sync_reset_on_target_change() {
    float max_delta = 0.f;
    for (auto &w : wheels_) {
      if (!w.active) {
        continue;
      }
      max_delta = std::max(
          max_delta, std::abs(w.target_wheel_rad_s - w.last_cmd_wheel_rad_s));
      w.last_cmd_wheel_rad_s = w.target_wheel_rad_s;
    }
    if (max_delta >= static_cast<float>(sync_reset_delta_rad_s_)) {
      sync_reset_ramp();
    }
  }

  void apply_wheel_cmd_array(
      const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    for (size_t i = 0; i < 4 && i < msg->data.size(); ++i) {
      if (!wheels_[i].active) {
        continue;
      }
      wheels_[i].target_wheel_rad_s =
          clampf(static_cast<float>(msg->data[i]),
                 static_cast<float>(-max_wheel_rad_s_),
                 static_cast<float>(max_wheel_rad_s_));
    }
    bool any_drive = false;
    for (auto &w : wheels_) {
      if (!w.active) {
        continue;
      }
      if (std::abs(w.target_wheel_rad_s) >=
          static_cast<float>(vel_deadband_rad_s_)) {
        any_drive = true;
        break;
      }
    }
    if (!any_drive) {
      shared_ramp_rpm_ = 0.f;
      sync_pid_.reset(0.f);
      for (auto &w : wheels_) {
        w.ramp_rpm = 0.f;
        if (w.active) {
          w.pid.reset(0.f);
        }
      }
    } else {
      maybe_sync_reset_on_target_change();
    }
    have_cmd_.store(true);
    last_cmd_ns_.store(now().nanoseconds());
  }

  void stop_all() {
    for (auto &w : wheels_) {
      w.target_wheel_rad_s = 0.f;
      w.last_cmd_wheel_rad_s = 0.f;
      w.pid.reset(0.f);
    }
    shared_ramp_rpm_ = 0.f;
    sync_pid_.reset(0.f);
    have_cmd_.store(true);
    last_cmd_ns_.store(now().nanoseconds());
    RCLCPP_WARN(get_logger(), "quad wheels stop");
  }

  bool cmd_stale() const {
    if (!have_cmd_.load()) {
      return true;
    }
    if (cmd_timeout_ms_ <= 0) {
      return false;
    }
    const int64_t age_ms =
        (now().nanoseconds() - last_cmd_ns_.load()) / 1000000;
    return age_ms > cmd_timeout_ms_;
  }

  void control_loop() {
    const int hz = std::max(100, control_hz_);
    const auto period = std::chrono::microseconds(1000000 / hz);
    const float dt = 1.f / static_cast<float>(hz);
    const int tx_interval = std::max(1, hz / std::max(50, can_tx_hz_));
    int tx_tick = 0;
    int log_div = 0;
    const int log_interval = std::max(1, hz / 10);

    while (running_.load() && !g_sigint.load()) {
      const auto t0 = std::chrono::steady_clock::now();

      bus_->drain_rx(64);

      const bool stale = cmd_stale();
      if (stale) {
        for (auto &w : wheels_) {
          w.target_wheel_rad_s = 0.f;
        }
      }

      bus_->zero_all();
      bool any_motion = false;

      struct WheelStep {
        WheelChannel *wheel{nullptr};
        float meas_rpm{0.f};
        float target_rpm{0.f};
        bool moving{false};
      };
      std::array<WheelStep, 4> steps{};
      int active_n = 0;
      int moving_n = 0;
      float sum_target_rpm = 0.f;
      float sum_meas_moving = 0.f;
      float sum_abs_target = 0.f;
      float min_abs_meas = 1e9f;

      for (auto &w : wheels_) {
        if (!w.active) {
          continue;
        }
        auto &st = steps[active_n++];
        st.wheel = &w;
        const auto fb = bus_->feedback(w.motor_id);
        st.meas_rpm = static_cast<float>(fb.speed_rpm) * w.sign;
        st.target_rpm = wheel_to_rpm(w.target_wheel_rad_s);
        st.moving = std::abs(w.target_wheel_rad_s) >=
                    static_cast<float>(vel_deadband_rad_s_);
        if (st.moving) {
          any_motion = true;
          ++moving_n;
          sum_target_rpm += st.target_rpm;
          sum_meas_moving += st.meas_rpm;
          sum_abs_target += std::abs(st.target_rpm);
          min_abs_meas =
              std::min(min_abs_meas, std::abs(st.meas_rpm));
        }
      }
      if (moving_n == 0) {
        min_abs_meas = 0.f;
      }

      const float dv = static_cast<float>(rpm_ramp_rpm_s_) * dt;
      const bool cmd_live = !stale;
      const bool any_drive = cmd_live && moving_n > 0;
      // PID 用满电流上限；摩擦前馈另计，总电流在下方 clamp 到 max_a
      const float pid_limit = static_cast<float>(max_current_a_);
      const float max_a = static_cast<float>(max_current_a_);
      const float brake_a = static_cast<float>(stop_brake_a_);
      const float brake_min_rpm = static_cast<float>(stop_brake_min_rpm_);

      std::array<float, 4> wheel_cmds{};

      if (!any_drive) {
        shared_ramp_rpm_ = 0.f;
        sync_pid_.reset(0.f);
        for (auto &w : wheels_) {
          w.ramp_rpm = 0.f;
          if (w.active) {
            w.pid.reset(0.f);
          }
        }
        for (int i = 0; i < active_n; ++i) {
          float cmd = 0.f;
          const float meas = steps[i].meas_rpm;
          if (brake_a > 1e-4f && std::abs(meas) > brake_min_rpm) {
            cmd = std::copysign(brake_a, -meas);
          }
          wheel_cmds[i] = clampf(cmd, -max_a, max_a);
          if (std::abs(meas) > 5.f) {
            any_motion = true;
          }
        }
      } else if (sync_wheels_ && moving_n >= 2 && active_n >= 2) {
        const float avg_target =
            sum_target_rpm / static_cast<float>(moving_n);
        const float ramp_err = avg_target - shared_ramp_rpm_;
        if (std::abs(ramp_err) <= dv) {
          shared_ramp_rpm_ = avg_target;
        } else {
          shared_ramp_rpm_ += (ramp_err > 0.f) ? dv : -dv;
        }

        const float avg_meas =
            sum_meas_moving / static_cast<float>(moving_n);
        const float avg_abs_target =
            sum_abs_target / static_cast<float>(moving_n);
        const float sync_gain = static_cast<float>(sync_gain_a_per_rpm_);
        const float sync_limit = static_cast<float>(sync_corr_limit_a_);
        const float sync_band = static_cast<float>(sync_vel_band_rpm_);

        if (avg_abs_target > 1.f) {
          shared_ramp_rpm_ =
              clampf(shared_ramp_rpm_, -avg_abs_target * 1.05f,
                     avg_abs_target * 1.05f);
        }
        for (int i = 0; i < active_n; ++i) {
          steps[i].wheel->ramp_rpm = shared_ramp_rpm_;
        }

        const float shared_ff = friction_ff(
            shared_ramp_rpm_, static_cast<float>(vel_friction_coulomb_a_),
            static_cast<float>(vel_friction_a_),
            static_cast<float>(vel_friction_full_rpm_),
            static_cast<float>(vel_friction_min_rpm_));
        sync_pid_.out_limit = pid_limit;
        float base =
            sync_pid_.step(shared_ramp_rpm_, avg_meas, dt) + shared_ff;
        if (startup_current_a_ > 1e-4f &&
            std::abs(shared_ramp_rpm_) >= 5.f &&
            std::abs(avg_meas) < 20.f) {
          const float kick = std::copysign(
              static_cast<float>(startup_current_a_), shared_ramp_rpm_);
          if (std::abs(base) < std::abs(kick)) {
            base = kick;
          }
        }
        base = clampf(base, -max_a, max_a);

        bool trim_ok = true;
        for (int i = 0; i < active_n; ++i) {
          if (!steps[i].moving ||
              std::abs(steps[i].meas_rpm) < sync_band) {
            trim_ok = false;
            break;
          }
        }

        for (int i = 0; i < active_n; ++i) {
          auto &st = steps[i];
          if (!st.moving) {
            wheel_cmds[i] = 0.f;
            continue;
          }
          float cmd = base;
          if (trim_ok) {
            const float trim = clampf(
                -sync_gain * (st.meas_rpm - avg_meas), -sync_limit, sync_limit);
            cmd = clampf(cmd + trim, -max_a, max_a);
          }
          wheel_cmds[i] = cmd;
          if (std::abs(shared_ramp_rpm_) > 1.f ||
              std::abs(st.wheel->target_wheel_rad_s) > 0.01f) {
            any_motion = true;
          }
        }
      } else {
        for (int i = 0; i < active_n; ++i) {
          auto &st = steps[i];
          auto &w = *st.wheel;

          if (!st.moving) {
            w.ramp_rpm = 0.f;
            w.pid.reset(st.meas_rpm);
            wheel_cmds[i] = 0.f;
            continue;
          }

          float ramp_rpm = w.ramp_rpm;
          const float err = st.target_rpm - ramp_rpm;
          ramp_rpm += (std::abs(err) <= dv) ? err : ((err > 0.f) ? dv : -dv);
          w.ramp_rpm = ramp_rpm;

          const float ff = friction_ff(
              ramp_rpm, static_cast<float>(vel_friction_coulomb_a_),
              static_cast<float>(vel_friction_a_),
              static_cast<float>(vel_friction_full_rpm_),
              static_cast<float>(vel_friction_min_rpm_));
          w.pid.out_limit = pid_limit;
          const float pid_out = w.pid.step(ramp_rpm, st.meas_rpm, dt);
          float cmd = clampf(pid_out + ff, -max_a, max_a);
          if (startup_current_a_ > 1e-4 &&
              std::abs(ramp_rpm) >= 3.f && std::abs(st.meas_rpm) < 25.f) {
            const float kick =
                std::copysign(static_cast<float>(startup_current_a_), ramp_rpm);
            if (std::abs(cmd) < std::abs(kick)) {
              cmd = kick;
            }
          }
          wheel_cmds[i] = cmd;
          if (std::abs(ramp_rpm) > 1.f ||
              std::abs(w.target_wheel_rad_s) > 0.01f) {
            any_motion = true;
          }
        }
      }

      for (int i = 0; i < active_n; ++i) {
        bus_->set_current_a(steps[i].wheel->motor_id,
                            wheel_cmds[i] * steps[i].wheel->sign);
      }

      if (any_motion || ++tx_tick >= tx_interval) {
        tx_tick = 0;
        bus_->transmit(any_motion);
      }

      if (++log_div >= log_interval) {
        log_div = 0;
        publish_state();
      }

      const auto elapsed = std::chrono::steady_clock::now() - t0;
      if (elapsed < period) {
        std::this_thread::sleep_for(period - elapsed);
      }
    }

    bus_->zero_all();
    bus_->transmit(true);
  }

  void publish_state() {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    for (auto &w : wheels_) {
      if (!w.active) {
        continue;
      }
      const auto fb = bus_->feedback(w.motor_id);
      js.name.push_back(w.joint_name);
      js.position.push_back(static_cast<double>(fb.angle_raw) /
                            C620Motor::kAngleRawMax * 2.0 * M_PI /
                            gear_ratio_);
      js.velocity.push_back(rpm_to_wheel(static_cast<float>(fb.speed_rpm)) *
                            static_cast<double>(w.sign));
      js.effort.push_back(fb.current_a);
    }
    if (!js.name.empty()) {
      state_pub_->publish(js);
    }
  }

  std::array<WheelChannel, 4> wheels_{};
  std::unique_ptr<C620Bus> bus_;
  std::atomic<bool> running_{true};
  std::atomic<bool> have_cmd_{false};
  std::atomic<int64_t> last_cmd_ns_{0};
  std::thread worker_;

  int control_hz_{500};
  int can_tx_hz_{250};
  double gear_ratio_{19.0};
  double max_current_a_{3.0};
  double max_wheel_rad_s_{1.0};
  double vel_deadband_rad_s_{0.05};
  double rpm_ramp_rpm_s_{400.0};
  double vel_kp_{0.025};
  double vel_ki_{0.012};
  double vel_kd_{0.0};
  double vel_friction_coulomb_a_{1.0};
  double vel_friction_a_{0.3};
  double vel_friction_full_rpm_{145.0};
  double vel_friction_min_rpm_{1.0};
  int cmd_timeout_ms_{500};
  bool sync_wheels_{true};
  double sync_gain_a_per_rpm_{0.012};
  double sync_corr_limit_a_{1.5};
  double sync_catchup_frac_{0.55};
  double sync_catchup_min_rpm_{25.0};
  double sync_vel_band_rpm_{12.0};
  double sync_reset_delta_rad_s_{0.08};
  double startup_current_a_{0.8};
  double stop_brake_a_{0.4};
  double stop_brake_min_rpm_{25.0};
  float shared_ramp_rpm_{0.f};
  Pid sync_pid_{};
  bool sync_pid_initialized_{false};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      wheels_cmd_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr>
      per_leg_subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
};

int main(int argc, char **argv) {
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  if (!acquire_lock()) {
    return 1;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<C620QuadNode>());
  rclcpp::shutdown();
  return 0;
}
