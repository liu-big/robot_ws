#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace {
std::atomic<bool> g_sigint{false};
void on_sigint(int) { g_sigint.store(true); }

float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}
} // namespace

/// 四足遥控协调：/cmd_vel → 麦轮 IK，/quad/teleop/pose → 身体姿态
class QuadTeleopNode : public rclcpp::Node {
public:
  QuadTeleopNode() : Node("quad_teleop_node") {
    wheel_base_x_ = declare_parameter("wheel_base_x", 0.30);
    wheel_base_y_ = declare_parameter("wheel_base_y", 0.20);
    max_vx_ = declare_parameter("max_vx", 0.5);
    max_vy_ = declare_parameter("max_vy", 0.5);
    max_omega_ = declare_parameter("max_omega", 1.0);
    max_wheel_rad_s_ = declare_parameter("max_wheel_rad_s", 1.0);
    cmd_timeout_ms_ = declare_parameter("cmd_timeout_ms", 300);
    publish_hz_ = declare_parameter("publish_hz", 50);

    std::vector<int64_t> active_wheels =
        declare_parameter("active_wheels", std::vector<int64_t>{1, 2, 3, 4});
    for (int64_t id : active_wheels) {
      if (id >= 1 && id <= 4) {
        wheel_active_[static_cast<size_t>(id - 1)] = true;
      }
    }

    auto qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", qos, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          vx_ = clampf(static_cast<float>(msg->linear.x),
                       static_cast<float>(-max_vx_),
                       static_cast<float>(max_vx_));
          vy_ = clampf(static_cast<float>(msg->linear.y),
                       static_cast<float>(-max_vy_),
                       static_cast<float>(max_vy_));
          omega_ = clampf(static_cast<float>(msg->angular.z),
                          static_cast<float>(-max_omega_),
                          static_cast<float>(max_omega_));
          have_vel_.store(true);
          last_vel_ns_.store(now().nanoseconds());
        });

    pose_sub_ = create_subscription<std_msgs::msg::String>(
        "/quad/teleop/pose", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          publish_pose(msg->data);
        });

    for (const char *pose : {"neutral", "stand", "crouch"}) {
      const std::string topic = std::string("/quad/teleop/") + pose;
      pose_btn_subs_.push_back(create_subscription<std_msgs::msg::Empty>(
          topic, qos, [this, pose](const std_msgs::msg::Empty::SharedPtr) {
            publish_pose(pose);
          }));
    }

    estop_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/quad/e_stop", qos, [this](const std_msgs::msg::Empty::SharedPtr) {
          estop();
        });

    wheels_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/quad/wheels/cmd", qos);
    wheels_stop_pub_ =
        create_publisher<std_msgs::msg::Empty>("/quad/wheels/stop", qos);
    quad_hold_pub_ =
        create_publisher<std_msgs::msg::Empty>("/quad/hold", qos);

    for (const char *pose : {"neutral", "stand", "crouch"}) {
      pose_pubs_[pose] =
          create_publisher<std_msgs::msg::Empty>("/quad/goto/" + std::string(pose), qos);
    }

    timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / std::max(10, publish_hz_)),
        [this]() { on_timer(); });

    RCLCPP_INFO(get_logger(),
                "quad_teleop | cmd_vel → /quad/wheels/cmd | pose → /quad/goto/*");
  }

private:
  void publish_pose(const std::string &name) {
    if (pose_pubs_.count(name) == 0) {
      RCLCPP_WARN(get_logger(), "未知姿态: %s", name.c_str());
      return;
    }
    std_msgs::msg::Empty msg;
    pose_pubs_.at(name)->publish(msg);
    RCLCPP_INFO(get_logger(), "→ body pose %s", name.c_str());
  }

  void estop() {
    vx_ = vy_ = omega_ = 0.f;
    have_vel_.store(false);
    std_msgs::msg::Empty empty;
    wheels_stop_pub_->publish(empty);
    quad_hold_pub_->publish(empty);
    RCLCPP_WARN(get_logger(), "E-STOP");
  }

  void mecanum_ik(float vx, float vy, float omega,
                  std::array<float, 4> &out) const {
    // leg1 FL, leg2 FR, leg3 RR, leg4 RL
    const float lx = static_cast<float>(wheel_base_x_) * 0.5f;
    const float ly = static_cast<float>(wheel_base_y_) * 0.5f;
    const float k = lx + ly;
    out[0] = vx - vy - omega * k;
    out[1] = vx + vy + omega * k;
    out[2] = vx + vy - omega * k;
    out[3] = vx - vy + omega * k;
    for (float &v : out) {
      v = clampf(v, static_cast<float>(-max_wheel_rad_s_),
                 static_cast<float>(max_wheel_rad_s_));
    }
  }

  void on_timer() {
    if (cmd_timeout_ms_ > 0 && have_vel_.load()) {
      const int64_t age_ms =
          (now().nanoseconds() - last_vel_ns_.load()) / 1000000;
      if (age_ms > cmd_timeout_ms_) {
        vx_ = vy_ = omega_ = 0.f;
        have_vel_.store(false);
      }
    }

    std::array<float, 4> wheel_vel{};
    mecanum_ik(vx_, vy_, omega_, wheel_vel);

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(4);
    for (size_t i = 0; i < 4; ++i) {
      msg.data[i] = wheel_active_[i] ? wheel_vel[i] : 0.0;
    }
    wheels_pub_->publish(msg);
  }

  double wheel_base_x_{0.30};
  double wheel_base_y_{0.20};
  double max_vx_{0.5};
  double max_vy_{0.5};
  double max_omega_{1.0};
  double max_wheel_rad_s_{1.0};
  int cmd_timeout_ms_{300};
  int publish_hz_{50};

  float vx_{0.f};
  float vy_{0.f};
  float omega_{0.f};
  std::array<bool, 4> wheel_active_{{true, true, true, true}};

  std::atomic<bool> have_vel_{false};
  std::atomic<int64_t> last_vel_ns_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pose_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr> pose_btn_subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr estop_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheels_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr wheels_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr quad_hold_pub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr>
      pose_pubs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
