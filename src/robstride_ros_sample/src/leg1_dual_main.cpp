#include "motor_ros2/motor_cfg.h"

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

bool acquire_single_instance_lock() {
  const int fd = open("/tmp/leg1_dual_ros2.lock", O_CREAT | O_RDWR, 0644);
  if (fd < 0) {
    return true;
  }
  if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
    std::fprintf(stderr, "已有 leg1_dual_ros2 在运行 → pkill -9 leg1_dual_ros2\n");
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
  int runaway_count{0};
  std::string joint_name;
  std::string log_label;
};

/// 第1条腿 · 双灵足（can0）：抬升 ID=1 + 趴下/起身 ID=5
class Leg1DualNode : public rclcpp::Node {
public:
  Leg1DualNode() : Node("leg1_dual_node") {
    const std::string lift_can =
        declare_parameter("lift_can_interface", "can0");
    const std::string crouch_can =
        declare_parameter("crouch_can_interface", "can1");
    const uint8_t lift_id =
        static_cast<uint8_t>(declare_parameter("lift_motor_id", 1));
    const uint8_t crouch_id =
        static_cast<uint8_t>(declare_parameter("crouch_motor_id", 5));
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

    lift_.joint_name = declare_parameter("lift_joint_name", "leg1_lift");
    crouch_.joint_name = declare_parameter("crouch_joint_name", "leg1_crouch");
    lift_.log_label = "抬升";
    crouch_.log_label = "趴下";

    const std::string lift_cmd =
        declare_parameter("lift_command_topic", "/leg1/lift/command");
    const std::string crouch_cmd =
        declare_parameter("crouch_command_topic", "/leg1/crouch/command");
    const std::string lift_state =
        declare_parameter("lift_state_topic", "/leg1/lift/state");
    const std::string crouch_state =
        declare_parameter("crouch_state_topic", "/leg1/crouch/state");
    const std::string lift_hold =
        declare_parameter("lift_hold_topic", "/leg1/lift/hold");
    const std::string crouch_hold =
        declare_parameter("crouch_hold_topic", "/leg1/crouch/hold");
    const std::string all_hold =
        declare_parameter("hold_topic", "/leg1/hold");
    const std::string topic_goto_neutral =
        declare_parameter("goto_neutral_topic", "/leg1/goto/neutral");
    const std::string topic_goto_stand =
        declare_parameter("goto_stand_topic", "/leg1/goto/stand");
    const std::string topic_goto_crouch =
        declare_parameter("goto_crouch_topic", "/leg1/goto/crouch");

    pose_neutral_lift_rad_ =
        declare_parameter("pose_neutral_lift_rad", 6.05);
    pose_neutral_crouch_rad_ =
        declare_parameter("pose_neutral_crouch_rad", 2.45);
    pose_stand_lift_rad_ = declare_parameter("pose_stand_lift_rad", 6.13);
    pose_stand_crouch_rad_ =
        declare_parameter("pose_stand_crouch_rad", 2.65);
    pose_crouch_lift_rad_ = declare_parameter("pose_crouch_lift_rad", 5.97);
    pose_crouch_crouch_rad_ =
        declare_parameter("pose_crouch_crouch_rad", 2.78);

    lift_.motor =
        std::make_unique<RobStrideMotor>(lift_can, 0xFF, lift_id, 3);
    crouch_.motor =
        std::make_unique<RobStrideMotor>(crouch_can, 0xFF, crouch_id, 3);
    lift_.motor->set_can_debug(can_debug_);
    crouch_.motor->set_can_debug(can_debug_);

    lift_.state_pub =
        create_publisher<sensor_msgs::msg::JointState>(lift_state, 10);
    crouch_.state_pub =
        create_publisher<sensor_msgs::msg::JointState>(crouch_state, 10);

    // 与 ros2 topic pub --once 默认 QoS 兼容（reliable + transient_local）
    auto cmd_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

    lift_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
        lift_cmd, cmd_qos,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          apply_command(lift_, static_cast<float>(msg->data));
        });
    crouch_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
        crouch_cmd, cmd_qos,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          apply_command(crouch_, static_cast<float>(msg->data));
        });
    lift_hold_sub_ = create_subscription<std_msgs::msg::Empty>(
        lift_hold, cmd_qos, [this](const std_msgs::msg::Empty::SharedPtr) {
          latch_hold(lift_);
          RCLCPP_INFO(get_logger(), "抬升 hold → 锁定当前位置");
        });
    crouch_hold_sub_ = create_subscription<std_msgs::msg::Empty>(
        crouch_hold, cmd_qos, [this](const std_msgs::msg::Empty::SharedPtr) {
          latch_hold(crouch_);
          RCLCPP_INFO(get_logger(), "趴下 hold → 锁定当前位置");
        });
    all_hold_sub_ = create_subscription<std_msgs::msg::Empty>(
        all_hold, cmd_qos, [this](const std_msgs::msg::Empty::SharedPtr) {
          latch_hold(lift_);
          latch_hold(crouch_);
          RCLCPP_INFO(get_logger(), "双关节 hold → 锁定当前位置");
        });
    goto_neutral_sub_ = create_subscription<std_msgs::msg::Empty>(
        topic_goto_neutral, cmd_qos,
        [this](const std_msgs::msg::Empty::SharedPtr) {
          apply_preset_pose("neutral", pose_neutral_lift_rad_,
                            pose_neutral_crouch_rad_);
        });
    goto_stand_sub_ = create_subscription<std_msgs::msg::Empty>(
        topic_goto_stand, cmd_qos,
        [this](const std_msgs::msg::Empty::SharedPtr) {
          apply_preset_pose("stand", pose_stand_lift_rad_,
                            pose_stand_crouch_rad_);
        });
    goto_crouch_sub_ = create_subscription<std_msgs::msg::Empty>(
        topic_goto_crouch, cmd_qos,
        [this](const std_msgs::msg::Empty::SharedPtr) {
          apply_preset_pose("crouch", pose_crouch_lift_rad_,
                            pose_crouch_crouch_rad_);
        });

    worker_ = std::thread(&Leg1DualNode::control_loop, this);

    RCLCPP_INFO(get_logger(),
                "leg1 抬升 can=%s ID=%u | 拉杆 can=%s ID=%u | kp=%.1f kd=%.2f "
                "v<=%.2f",
                lift_can.c_str(), lift_id, crouch_can.c_str(), crouch_id,
                motion_kp_, motion_kd_, max_vel_rad_s_);
  }

  ~Leg1DualNode() override {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    if (disable_on_exit_) {
      if (lift_.motor) {
        lift_.motor->Disenable_Motor(0);
      }
      if (crouch_.motor) {
        crouch_.motor->Disenable_Motor(0);
      }
    }
  }

private:
  void apply_pose_command(JointAxis &axis, float requested) {
    axis.user_target_rad.store(requested);
    axis.has_command.store(true);
    RCLCPP_INFO(get_logger(), "%s pose → %.3f rad (pos=%.3f)",
                axis.log_label.c_str(), requested, axis.motor->get_position());
  }

  void apply_preset_pose(const char *name, double lift_rad, double crouch_rad) {
    apply_pose_command(lift_, static_cast<float>(lift_rad));
    apply_pose_command(crouch_, static_cast<float>(crouch_rad));
    RCLCPP_INFO(get_logger(), "→ preset %s: lift=%.3f crouch=%.3f", name,
                lift_rad, crouch_rad);
  }

  void apply_command(JointAxis &axis, float requested) {
    const float prev = static_cast<float>(axis.user_target_rad.load());
    const float delta = requested - prev;
    float clamped = requested;
    if (std::abs(delta) > max_command_step_rad_) {
      clamped =
          prev + std::copysign(static_cast<float>(max_command_step_rad_), delta);
      RCLCPP_WARN(get_logger(),
                  "%s command %.3f 超出单步 ±%.2f (target=%.3f) → %.3f",
                  axis.log_label.c_str(), requested, max_command_step_rad_,
                  prev, clamped);
    }
    axis.user_target_rad.store(clamped);
    axis.has_command.store(true);
    RCLCPP_INFO(get_logger(), "%s command → %.3f rad (pos=%.3f)",
                axis.log_label.c_str(), clamped, axis.motor->get_position());
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
    RCLCPP_INFO(get_logger(), "%s 已使能，保持 %.3f rad",
                axis.log_label.c_str(), axis.motor->get_position());
    return true;
  }

  void control_loop() {
    const float kp = static_cast<float>(motion_kp_);
    const float kd = static_cast<float>(motion_kd_);
    if (!init_axis(lift_, kp, kd) || !init_axis(crouch_, kp, kd)) {
      return;
    }

    const auto loop_period = std::chrono::milliseconds(loop_ms_);
    const float dt = static_cast<float>(loop_ms_) / 1000.f;

    while (running_.load() && !g_sigint.load()) {
      const auto t0 = std::chrono::steady_clock::now();
      step_axis(lift_, dt, kp, kd);
      step_axis(crouch_, dt, kp, kd);
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
        RCLCPP_WARN(get_logger(), "%s 失控 vel=%.2f → 锁定 %.3f",
                    axis.log_label.c_str(), vel, pos);
        latch_hold(axis);
      }
    } else {
      axis.runaway_count = 0;
    }

    publish_state(axis, pos, vel, torque, user_target, cmd_pos, cmd_vel);
  }

  void publish_state(JointAxis &axis, float pos, float vel, float torque,
                     float user_target, float cmd_pos, float cmd_vel) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = {axis.joint_name};
    js.position = {static_cast<double>(pos)};
    js.velocity = {static_cast<double>(vel)};
    js.effort = {static_cast<double>(torque)};
    axis.state_pub->publish(js);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "%s goal=%.3f cmd=%.3f/%.2f pos=%.3f vel=%.3f",
                         axis.log_label.c_str(), user_target, cmd_pos, cmd_vel,
                         pos, vel);
  }

  JointAxis lift_;
  JointAxis crouch_;
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
  int loop_ms_{10};
  double pose_neutral_lift_rad_{6.05};
  double pose_neutral_crouch_rad_{2.45};
  double pose_stand_lift_rad_{6.13};
  double pose_stand_crouch_rad_{2.65};
  double pose_crouch_lift_rad_{5.97};
  double pose_crouch_crouch_rad_{2.78};

  std::atomic<bool> running_{true};
  std::thread worker_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lift_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr crouch_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr lift_hold_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr crouch_hold_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr all_hold_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr goto_neutral_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr goto_stand_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr goto_crouch_sub_;
};

int main(int argc, char **argv) {
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  if (!acquire_single_instance_lock()) {
    return 1;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Leg1DualNode>());
  rclcpp::shutdown();
  return 0;
}
