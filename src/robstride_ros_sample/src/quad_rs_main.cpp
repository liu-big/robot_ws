#include "motor_ros2/motor_cfg.h"

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
#include <string>
#include <sys/file.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
std::atomic<bool> g_sigint{false};

void on_sigint(int) { g_sigint.store(true); }

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
    dv = std::max(-dv_max, std::min(dv_max, dv));
    vel += dv;
    pos += vel * dt;
    if ((target - pos) * direction <= 0.f) {
      pos = target;
      vel = 0.f;
    }
  }
};

bool acquire_single_instance_lock() {
  const int fd = open("/tmp/quad_rs_ros2.lock", O_CREAT | O_RDWR, 0644);
  if (fd < 0) {
    return true;
  }
  if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
    std::fprintf(stderr, "已有 quad_rs_ros2 在运行 → pkill -9 quad_rs_ros2\n");
    close(fd);
    return false;
  }
  return true;
}

} // namespace

struct JointAxis {
  std::unique_ptr<RobStrideMotor> motor;
  SmoothJointTrajectory trajectory;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
  std::atomic<double> user_target_rad{0.0};
  std::atomic<bool> has_command{false};
  float home_rad{0.f};
  int runaway_count{0};
  std::string joint_name;
  std::string log_label;
  bool enabled{false};
};

struct LegPair {
  int leg_index{0};
  JointAxis lift;
  JointAxis crouch;
  uint8_t lift_motor_id{0};
  uint8_t crouch_motor_id{0};
  bool active{false};
};

/// 四足 8×RS03（抬升 can0 ID1~4，拉杆 can1 ID5~8），话题 /leg{N}/lift|crouch/*
class QuadRsNode : public rclcpp::Node {
public:
  QuadRsNode() : Node("quad_rs_node") {
    const std::string lift_can =
        declare_parameter("lift_can_interface", "can0");
    const std::string crouch_can =
        declare_parameter("crouch_can_interface", "can1");
    loop_ms_ = declare_parameter("loop_ms", 10);
    motion_kp_ = declare_parameter("motion_kp", 35.0);
    motion_kd_ = declare_parameter("motion_kd", 2.0);
    smooth_enabled_ = declare_parameter("smooth_enabled", true);
    max_vel_rad_s_ = declare_parameter("max_vel_rad_s", 0.15);
    max_accel_rad_s2_ = declare_parameter("max_accel_rad_s2", 0.5);
    runaway_vel_rad_s_ = declare_parameter("runaway_vel_rad_s", 2.0);
    runaway_count_limit_ = declare_parameter("runaway_count_limit", 5);
    max_command_step_rad_ = declare_parameter("max_command_step_rad", 0.25);
    disable_on_exit_ = declare_parameter("disable_on_exit", false);
    can_debug_ = declare_parameter("can_debug", false);
    preset_mode_ = declare_parameter("preset_mode", "absolute");

    std::vector<int64_t> active_legs =
        declare_parameter("active_legs", std::vector<int64_t>{1});

    legs_[0].leg_index = 1;
    legs_[0].lift_motor_id =
        static_cast<uint8_t>(declare_parameter("leg1_lift_motor_id", 1));
    legs_[0].crouch_motor_id =
        static_cast<uint8_t>(declare_parameter("leg1_crouch_motor_id", 5));
    legs_[1].leg_index = 2;
    legs_[1].lift_motor_id =
        static_cast<uint8_t>(declare_parameter("leg2_lift_motor_id", 2));
    legs_[1].crouch_motor_id =
        static_cast<uint8_t>(declare_parameter("leg2_crouch_motor_id", 6));
    legs_[2].leg_index = 3;
    legs_[2].lift_motor_id =
        static_cast<uint8_t>(declare_parameter("leg3_lift_motor_id", 3));
    legs_[2].crouch_motor_id =
        static_cast<uint8_t>(declare_parameter("leg3_crouch_motor_id", 7));
    legs_[3].leg_index = 4;
    legs_[3].lift_motor_id =
        static_cast<uint8_t>(declare_parameter("leg4_lift_motor_id", 4));
    legs_[3].crouch_motor_id =
        static_cast<uint8_t>(declare_parameter("leg4_crouch_motor_id", 8));

    load_pose_array("pose_neutral_lift", pose_neutral_lift_);
    load_pose_array("pose_neutral_crouch", pose_neutral_crouch_);
    load_pose_array("pose_stand_lift", pose_stand_lift_);
    load_pose_array("pose_stand_crouch", pose_stand_crouch_);
    load_pose_array("pose_crouch_lift", pose_crouch_lift_);
    load_pose_array("pose_crouch_crouch", pose_crouch_crouch_);
    load_sign_array("lift_preset_sign", lift_preset_sign_);
    load_sign_array("crouch_preset_sign", crouch_preset_sign_);

    auto cmd_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

    for (auto &leg : legs_) {
      leg.active = false;
      for (int64_t id : active_legs) {
        if (id == leg.leg_index) {
          leg.active = true;
          break;
        }
      }
      if (!leg.active) {
        continue;
      }

      const int n = leg.leg_index;
      leg.lift.joint_name = "leg" + std::to_string(n) + "_lift";
      leg.crouch.joint_name = "leg" + std::to_string(n) + "_crouch";
      leg.lift.log_label = "leg" + std::to_string(n) + "_抬升";
      leg.crouch.log_label = "leg" + std::to_string(n) + "_拉杆";
      leg.lift.enabled = true;
      leg.crouch.enabled = true;

      leg.lift.motor = std::make_unique<RobStrideMotor>(
          lift_can, 0xFF, leg.lift_motor_id, 3);
      leg.crouch.motor = std::make_unique<RobStrideMotor>(
          crouch_can, 0xFF, leg.crouch_motor_id, 3);
      leg.lift.motor->set_can_debug(can_debug_);
      leg.crouch.motor->set_can_debug(can_debug_);

      const std::string prefix = "/leg" + std::to_string(n);
      leg.lift.state_pub = create_publisher<sensor_msgs::msg::JointState>(
          prefix + "/lift/state", 10);
      leg.crouch.state_pub = create_publisher<sensor_msgs::msg::JointState>(
          prefix + "/crouch/state", 10);

      cmd_subs_.push_back(create_subscription<std_msgs::msg::Float64>(
          prefix + "/lift/command", cmd_qos,
          [this, n](const std_msgs::msg::Float64::SharedPtr msg) {
            apply_command(legs_[n - 1].lift, static_cast<float>(msg->data));
          }));
      cmd_subs_.push_back(create_subscription<std_msgs::msg::Float64>(
          prefix + "/crouch/command", cmd_qos,
          [this, n](const std_msgs::msg::Float64::SharedPtr msg) {
            apply_command(legs_[n - 1].crouch, static_cast<float>(msg->data));
          }));
      hold_subs_.push_back(create_subscription<std_msgs::msg::Empty>(
          prefix + "/lift/hold", cmd_qos,
          [this, n](const std_msgs::msg::Empty::SharedPtr) {
            latch_hold(legs_[n - 1].lift);
          }));
      hold_subs_.push_back(create_subscription<std_msgs::msg::Empty>(
          prefix + "/crouch/hold", cmd_qos,
          [this, n](const std_msgs::msg::Empty::SharedPtr) {
            latch_hold(legs_[n - 1].crouch);
          }));

      if (n == 1) {
        hold_subs_.push_back(create_subscription<std_msgs::msg::Empty>(
            "/leg1/hold", cmd_qos, [this](const std_msgs::msg::Empty::SharedPtr) {
              latch_hold(legs_[0].lift);
              latch_hold(legs_[0].crouch);
              RCLCPP_INFO(get_logger(), "leg1 hold");
            }));
        preset_subs_.push_back(create_subscription<std_msgs::msg::Empty>(
            "/leg1/goto/neutral", cmd_qos,
            [this](const std_msgs::msg::Empty::SharedPtr) {
              apply_leg_preset(1, "neutral");
            }));
        preset_subs_.push_back(create_subscription<std_msgs::msg::Empty>(
            "/leg1/goto/stand", cmd_qos,
            [this](const std_msgs::msg::Empty::SharedPtr) {
              apply_leg_preset(1, "stand");
            }));
        preset_subs_.push_back(create_subscription<std_msgs::msg::Empty>(
            "/leg1/goto/crouch", cmd_qos,
            [this](const std_msgs::msg::Empty::SharedPtr) {
              apply_leg_preset(1, "crouch");
            }));
      }

      RCLCPP_INFO(get_logger(), "leg%d 启用 抬升ID=%u 拉杆ID=%u", n,
                  leg.lift_motor_id, leg.crouch_motor_id);
    }

    joint_state_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("/quad/joint_states", 10);

    quad_hold_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/quad/hold", cmd_qos, [this](const std_msgs::msg::Empty::SharedPtr) {
          for (auto &leg : legs_) {
            if (!leg.active) {
              continue;
            }
            latch_hold(leg.lift);
            latch_hold(leg.crouch);
          }
          RCLCPP_INFO(get_logger(), "quad hold → 全部活跃关节锁定");
        });

    preset_subs_.push_back(create_subscription<std_msgs::msg::Empty>(
        "/quad/goto/neutral", cmd_qos,
        [this](const std_msgs::msg::Empty::SharedPtr) {
          apply_preset("neutral");
        }));
    preset_subs_.push_back(create_subscription<std_msgs::msg::Empty>(
        "/quad/goto/stand", cmd_qos, [this](const std_msgs::msg::Empty::SharedPtr) {
          apply_preset("stand");
        }));
    preset_subs_.push_back(create_subscription<std_msgs::msg::Empty>(
        "/quad/goto/crouch", cmd_qos,
        [this](const std_msgs::msg::Empty::SharedPtr) {
          apply_preset("crouch");
        }));

    worker_ = std::thread(&QuadRsNode::control_loop, this);

    RCLCPP_INFO(get_logger(),
                "quad_rs lift=%s crouch=%s | kp=%.1f kd=%.2f v<=%.2f | preset=%s",
                lift_can.c_str(), crouch_can.c_str(), motion_kp_, motion_kd_,
                max_vel_rad_s_, preset_mode_.c_str());
  }

  ~QuadRsNode() override {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    if (disable_on_exit_) {
      for (auto &leg : legs_) {
        if (!leg.active) {
          continue;
        }
        if (leg.lift.motor) {
          leg.lift.motor->Disenable_Motor(0);
        }
        if (leg.crouch.motor) {
          leg.crouch.motor->Disenable_Motor(0);
        }
      }
    }
  }

private:
  void load_pose_array(const std::string &name, std::array<double, 4> &out) {
    auto vals = declare_parameter(name, std::vector<double>{});
    for (size_t i = 0; i < 4 && i < vals.size(); ++i) {
      out[i] = vals[i];
    }
  }

  void load_sign_array(const std::string &name, std::array<double, 4> &out) {
    auto vals = declare_parameter(name, std::vector<double>{1, 1, 1, 1});
    for (size_t i = 0; i < 4 && i < vals.size(); ++i) {
      out[i] = vals[i];
    }
  }

  void preset_deltas(const char *name, float &d_lift, float &d_crouch) const {
    d_lift = 0.f;
    d_crouch = 0.f;
    const float n_lift = static_cast<float>(pose_neutral_lift_[0]);
    const float n_crouch = static_cast<float>(pose_neutral_crouch_[0]);
    if (std::string(name) == "stand") {
      d_lift = static_cast<float>(pose_stand_lift_[0]) - n_lift;
      d_crouch = static_cast<float>(pose_stand_crouch_[0]) - n_crouch;
    } else if (std::string(name) == "crouch") {
      d_lift = static_cast<float>(pose_crouch_lift_[0]) - n_lift;
      d_crouch = static_cast<float>(pose_crouch_crouch_[0]) - n_crouch;
    }
  }

  float resolve_preset_target(const JointAxis &axis, int leg_idx, bool is_lift,
                              const char *name) const {
    if (preset_mode_ == "relative") {
      float d_lift = 0.f;
      float d_crouch = 0.f;
      preset_deltas(name, d_lift, d_crouch);
      const float sign = static_cast<float>(
          is_lift ? lift_preset_sign_[leg_idx]
                  : crouch_preset_sign_[leg_idx]);
      const float delta = is_lift ? d_lift : d_crouch;
      return axis.home_rad + sign * delta;
    }
    const std::array<double, 4> *lift = nullptr;
    const std::array<double, 4> *crouch = nullptr;
    get_preset_arrays(name, &lift, &crouch);
    if (is_lift) {
      return static_cast<float>((*lift)[leg_idx]);
    }
    return static_cast<float>((*crouch)[leg_idx]);
  }

  void apply_pose_command(JointAxis &axis, float requested) {
    axis.user_target_rad.store(requested);
    axis.has_command.store(true);
    RCLCPP_INFO(get_logger(), "%s pose → %.3f rad (pos=%.3f)",
                axis.log_label.c_str(), requested, axis.motor->get_position());
  }

  void get_preset_arrays(const char *name, const std::array<double, 4> **lift,
                         const std::array<double, 4> **crouch) const {
    if (std::string(name) == "neutral") {
      *lift = &pose_neutral_lift_;
      *crouch = &pose_neutral_crouch_;
    } else if (std::string(name) == "stand") {
      *lift = &pose_stand_lift_;
      *crouch = &pose_stand_crouch_;
    } else {
      *lift = &pose_crouch_lift_;
      *crouch = &pose_crouch_crouch_;
    }
  }

  void apply_leg_preset(int leg_index, const char *name) {
    auto &leg = legs_[leg_index - 1];
    if (!leg.active) {
      return;
    }
    const int i = leg_index - 1;
    apply_pose_command(leg.lift, resolve_preset_target(leg.lift, i, true, name));
    apply_pose_command(leg.crouch,
                       resolve_preset_target(leg.crouch, i, false, name));
    RCLCPP_INFO(get_logger(), "→ leg%d preset %s", leg_index, name);
  }

  void apply_preset(const char *name) {
    for (auto &leg : legs_) {
      if (!leg.active) {
        continue;
      }
      const int i = leg.leg_index - 1;
      apply_pose_command(leg.lift, resolve_preset_target(leg.lift, i, true, name));
      apply_pose_command(leg.crouch,
                         resolve_preset_target(leg.crouch, i, false, name));
    }
    RCLCPP_INFO(get_logger(), "→ quad preset %s (%s)", name,
                preset_mode_.c_str());
  }

  void apply_command(JointAxis &axis, float requested) {
    const float prev = static_cast<float>(axis.user_target_rad.load());
    const float delta = requested - prev;
    float clamped = requested;
    if (std::abs(delta) > max_command_step_rad_) {
      clamped =
          prev + std::copysign(static_cast<float>(max_command_step_rad_), delta);
      RCLCPP_WARN(get_logger(),
                  "%s command %.3f 超出单步 ±%.2f → %.3f",
                  axis.log_label.c_str(), requested, max_command_step_rad_,
                  clamped);
    }
    axis.user_target_rad.store(clamped);
    axis.has_command.store(true);
    RCLCPP_INFO(get_logger(), "%s command → %.3f rad", axis.log_label.c_str(),
                clamped);
  }

  void latch_hold(JointAxis &axis) {
    const float pos = axis.motor->get_position();
    axis.user_target_rad.store(pos);
    axis.trajectory.reset(pos);
    axis.has_command.store(false);
    axis.runaway_count = 0;
  }

  bool init_axis(JointAxis &axis, float kp, float kd) {
    if (!axis.motor->init_motion_mode(kp, kd)) {
      RCLCPP_ERROR(get_logger(), "%s init_motion_mode 失败",
                   axis.log_label.c_str());
      return false;
    }
    for (int i = 0; i < 10; ++i) {
      axis.motor->send_motion_hold(axis.motor->get_position(), 0.f, kp, kd);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    latch_hold(axis);
    axis.home_rad = axis.motor->get_position();
    RCLCPP_INFO(get_logger(), "%s 已使能，home=%.3f rad",
                axis.log_label.c_str(), axis.home_rad);
    return true;
  }

  void control_loop() {
    const float kp = static_cast<float>(motion_kp_);
    const float kd = static_cast<float>(motion_kd_);

    for (auto &leg : legs_) {
      if (!leg.active) {
        continue;
      }
      if (!init_axis(leg.lift, kp, kd) || !init_axis(leg.crouch, kp, kd)) {
        return;
      }
    }

    const auto loop_period = std::chrono::milliseconds(loop_ms_);
    const float dt = static_cast<float>(loop_ms_) / 1000.f;
    int aggregate_div = 0;

    while (running_.load() && !g_sigint.load()) {
      const auto t0 = std::chrono::steady_clock::now();
      for (auto &leg : legs_) {
        if (!leg.active) {
          continue;
        }
        step_axis(leg.lift, dt, kp, kd);
        step_axis(leg.crouch, dt, kp, kd);
      }
      if (++aggregate_div >= std::max(1, 1000 / loop_ms_)) {
        aggregate_div = 0;
        publish_aggregate_state();
      }
      const auto elapsed = std::chrono::steady_clock::now() - t0;
      if (elapsed < loop_period) {
        std::this_thread::sleep_for(loop_period - elapsed);
      }
    }
  }

  void step_axis(JointAxis &axis, float dt, float kp, float kd) {
    const float user_target = static_cast<float>(axis.user_target_rad.load());
    float cmd_pos = user_target;
    float cmd_vel = 0.f;

    if (smooth_enabled_) {
      axis.trajectory.step(user_target, dt,
                           static_cast<float>(max_vel_rad_s_),
                           static_cast<float>(max_accel_rad_s2_));
      cmd_pos = axis.trajectory.pos;
      cmd_vel = axis.trajectory.vel;
    } else {
      axis.trajectory.reset(user_target);
    }

    auto [pos, vel, torque, temp] =
        axis.motor->send_motion_hold(cmd_pos, cmd_vel, kp, kd);

    if (!axis.has_command.load() && std::abs(vel) > runaway_vel_rad_s_) {
      ++axis.runaway_count;
      if (axis.runaway_count >= runaway_count_limit_) {
        latch_hold(axis);
      }
    } else {
      axis.runaway_count = 0;
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = {axis.joint_name};
    js.position = {static_cast<double>(pos)};
    js.velocity = {static_cast<double>(vel)};
    js.effort = {static_cast<double>(torque)};
    axis.state_pub->publish(js);
  }

  void publish_aggregate_state() {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    for (auto &leg : legs_) {
      if (!leg.active) {
        continue;
      }
      js.name.push_back(leg.lift.joint_name);
      js.position.push_back(leg.lift.motor->get_position());
      js.velocity.push_back(leg.lift.motor->get_velocity());
      js.effort.push_back(leg.lift.motor->get_torque());
      js.name.push_back(leg.crouch.joint_name);
      js.position.push_back(leg.crouch.motor->get_position());
      js.velocity.push_back(leg.crouch.motor->get_velocity());
      js.effort.push_back(leg.crouch.motor->get_torque());
    }
    if (!js.name.empty()) {
      joint_state_pub_->publish(js);
    }
  }

  std::array<LegPair, 4> legs_{};
  std::array<double, 4> pose_neutral_lift_{6.05, 6.05, 6.05, 6.05};
  std::array<double, 4> pose_neutral_crouch_{2.45, 2.45, 2.45, 2.45};
  std::array<double, 4> pose_stand_lift_{6.13, 6.13, 6.13, 6.13};
  std::array<double, 4> pose_stand_crouch_{2.65, 2.65, 2.65, 2.65};
  std::array<double, 4> pose_crouch_lift_{5.97, 5.97, 5.97, 5.97};
  std::array<double, 4> pose_crouch_crouch_{2.78, 2.78, 2.78, 2.78};

  double motion_kp_{35.0};
  double motion_kd_{2.0};
  bool smooth_enabled_{true};
  double max_vel_rad_s_{0.15};
  double max_accel_rad_s2_{0.5};
  double runaway_vel_rad_s_{2.0};
  int runaway_count_limit_{5};
  double max_command_step_rad_{0.25};
  bool disable_on_exit_{false};
  bool can_debug_{false};
  std::string preset_mode_{"absolute"};
  std::array<double, 4> lift_preset_sign_{1, 1, 1, 1};
  std::array<double, 4> crouch_preset_sign_{1, 1, 1, 1};
  int loop_ms_{10};

  std::atomic<bool> running_{true};
  std::thread worker_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr quad_hold_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> cmd_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr> hold_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr> preset_subs_;
};

int main(int argc, char **argv) {
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  if (!acquire_single_instance_lock()) {
    return 1;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadRsNode>());
  rclcpp::shutdown();
  return 0;
}
