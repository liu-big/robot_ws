#include "motor_ros2/motor_cfg.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
#include <unistd.h>

namespace {
std::atomic<bool> g_sigint{false};

void on_sigint(int) { g_sigint.store(true); }

/// 梯形速度规划（MIT Cheetah / Unitree 关节层常用做法）
/// 输出平滑 qDes + qdDes，配合运控模式 kp/kd 实现柔顺跟踪
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

} // namespace

/// 第1条腿 · 抬升关节 — 运控保持：目标锁定，无指令不追反馈
class Leg1LiftNode : public rclcpp::Node {
public:
  Leg1LiftNode() : Node("leg1_lift_node") {
    const std::string can_iface =
        declare_parameter("can_interface", "can0");
    const uint8_t motor_id =
        static_cast<uint8_t>(declare_parameter("motor_id", 0x7F));
    loop_ms_ = declare_parameter("loop_ms", 10);
    motion_kp_ = declare_parameter("motion_kp", 40.0);
    motion_kd_ = declare_parameter("motion_kd", 2.0);
    smooth_enabled_ = declare_parameter("smooth_enabled", true);
    max_vel_rad_s_ = declare_parameter("max_vel_rad_s", 0.4);
    max_accel_rad_s2_ = declare_parameter("max_accel_rad_s2", 1.5);
    runaway_vel_rad_s_ = declare_parameter("runaway_vel_rad_s", 2.0);
    runaway_count_limit_ = declare_parameter("runaway_count_limit", 5);
    max_command_step_rad_ = declare_parameter("max_command_step_rad", 0.5);
    disable_on_exit_ = declare_parameter("disable_on_exit", false);
    can_debug_ = declare_parameter("can_debug", false);
    joint_name_ = declare_parameter("joint_name", "leg1_lift");
    cmd_topic_ = declare_parameter("command_topic", "/leg1/lift/command");
    state_topic_ = declare_parameter("state_topic", "/leg1/lift/state");
    hold_topic_ = declare_parameter("hold_topic", "/leg1/lift/hold");

    motor_ = std::make_unique<RobStrideMotor>(can_iface, 0xFF, motor_id, 3);
    motor_->set_can_debug(can_debug_);

    cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
        cmd_topic_, 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          apply_command(static_cast<float>(msg->data));
        });

    hold_sub_ = create_subscription<std_msgs::msg::Empty>(
        hold_topic_, 10, [this](const std_msgs::msg::Empty::SharedPtr) {
          latch_hold_at_current();
          RCLCPP_INFO(get_logger(), "hold → 锁定当前位置，仍使能");
        });

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>(state_topic_, 10);
    worker_ = std::thread(&Leg1LiftNode::control_loop, this);

    RCLCPP_INFO(get_logger(),
                "leg1 抬升 | kp=%.1f kd=%.2f | smooth=%s v<=%.2f a<=%.2f | 等 %s",
                motion_kp_, motion_kd_, smooth_enabled_ ? "on" : "off",
                max_vel_rad_s_, max_accel_rad_s2_, cmd_topic_.c_str());
  }

  ~Leg1LiftNode() override {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    if (motor_ && disable_on_exit_) {
      motor_->Disenable_Motor(0);
    }
  }

private:
  void apply_command(float requested) {
    const float prev_target =
        static_cast<float>(user_target_rad_.load());
    const float delta = requested - prev_target;
    float clamped = requested;
    if (std::abs(delta) > max_command_step_rad_) {
      clamped = prev_target + std::copysign(
                                  static_cast<float>(max_command_step_rad_),
                                  delta);
      RCLCPP_WARN(get_logger(),
                  "command %.3f 超出单步 ±%.2f (target=%.3f) → 限幅为 %.3f",
                  requested, max_command_step_rad_, prev_target, clamped);
    }
    user_target_rad_.store(clamped);
    has_command_.store(true);
    RCLCPP_INFO(get_logger(), "command → %.3f rad (pos=%.3f)", clamped,
                motor_->get_position());
  }

  void latch_hold_at_current() {
    const float pos = motor_->get_position();
    user_target_rad_.store(pos);
    trajectory_.reset(pos);
    has_command_.store(false);
  }

  void control_loop() {
    if (!motor_->init_motion_mode(static_cast<float>(motion_kp_),
                                  static_cast<float>(motion_kd_))) {
      RCLCPP_ERROR(get_logger(), "init_motion_mode 失败，检查供电/CAN/接线");
    }

    // 多帧保持，确保 position 来自电机反馈而非初始 0
    for (int i = 0; i < 10; ++i) {
      motor_->send_motion_hold(motor_->get_position(), 0.0f,
                               static_cast<float>(motion_kp_),
                               static_cast<float>(motion_kd_));
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const float hold = motor_->get_position();
    user_target_rad_.store(hold);
    trajectory_.reset(hold);
    has_command_.store(false);
    runaway_count_ = 0;

    RCLCPP_INFO(get_logger(), "已使能，保持 %.3f rad", hold);

    const auto loop_period = std::chrono::milliseconds(loop_ms_);
    while (running_.load() && !g_sigint.load()) {
      const auto t0 = std::chrono::steady_clock::now();
      const float user_target =
          static_cast<float>(user_target_rad_.load());
      float cmd_pos = user_target;
      float cmd_vel = 0.f;

      if (smooth_enabled_) {
        trajectory_.step(user_target, static_cast<float>(loop_ms_) / 1000.f,
                         static_cast<float>(max_vel_rad_s_),
                         static_cast<float>(max_accel_rad_s2_));
        cmd_pos = trajectory_.pos;
        cmd_vel = trajectory_.vel;
      } else {
        trajectory_.reset(user_target);
      }

      auto [pos, vel, torque, temp] = motor_->send_motion_hold(
          cmd_pos, cmd_vel, static_cast<float>(motion_kp_),
          static_cast<float>(motion_kd_));

      if (!has_command_.load() && std::abs(vel) > runaway_vel_rad_s_) {
        ++runaway_count_;
        if (runaway_count_ >= runaway_count_limit_) {
          RCLCPP_WARN(get_logger(), "失控 vel=%.2f → 锁定 pos=%.3f", vel, pos);
          user_target_rad_.store(pos);
          trajectory_.reset(pos);
          has_command_.store(false);
          runaway_count_ = 0;
        }
      } else {
        runaway_count_ = 0;
      }

      publish_state(pos, vel, torque, user_target_rad_.load(), cmd_pos, cmd_vel);
      const auto elapsed = std::chrono::steady_clock::now() - t0;
      if (elapsed < loop_period) {
        std::this_thread::sleep_for(loop_period - elapsed);
      }
    }
  }

  void publish_state(float pos, float vel, float torque, double user_target,
                     float cmd_pos, float cmd_vel) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = {joint_name_};
    js.position = {static_cast<double>(pos)};
    js.velocity = {static_cast<double>(vel)};
    js.effort = {static_cast<double>(torque)};
    state_pub_->publish(js);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "goal=%.3f cmd=%.3f/%.2frad/s pos=%.3f vel=%.3f "
                         "torque=%.2f active=%d",
                         user_target, cmd_pos, cmd_vel, pos, vel, torque,
                         has_command_.load());
  }

  double motion_kp_{40.0};
  double motion_kd_{2.0};
  bool smooth_enabled_{true};
  double max_vel_rad_s_{0.4};
  double max_accel_rad_s2_{1.5};
  double runaway_vel_rad_s_{2.0};
  int runaway_count_limit_{5};
  double max_command_step_rad_{0.5};
  bool disable_on_exit_{false};
  bool can_debug_{false};
  int loop_ms_{20};
  int runaway_count_{0};
  std::string joint_name_;
  std::string cmd_topic_;
  std::string state_topic_;
  std::string hold_topic_;

  std::atomic<bool> running_{true};
  std::atomic<bool> has_command_{false};
  std::atomic<double> user_target_rad_{0.0};
  SmoothJointTrajectory trajectory_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr hold_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  std::thread worker_;
  std::unique_ptr<RobStrideMotor> motor_;
};

int main(int argc, char **argv) {
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Leg1LiftNode>());
  rclcpp::shutdown();
  return 0;
}
