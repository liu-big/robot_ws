/// 麦轮里程计：/quad/wheel_states 轮速 → 车体 twist → 积分 → /odom + tf
/// FK 与 quad_teleop_main.cpp 的 mecanum_ik 互逆；参考 ros2_control
/// mecanum_drive_controller::Odometry 的积分方式。
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace {

constexpr int kWheelCount = 4;

double normalize_angle(double a) {
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

/// 麦轮 FK 矩阵行（与 quad_teleop IK 一致）
/// w_i = J_i · [vx, vy, omega]^T
void wheel_jacobian_row(int wheel_index, double k, double row[3]) {
  static const double signs[4][2] = {
      {-1.0, -1.0}, {1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}};
  row[0] = 1.0;
  row[1] = signs[wheel_index][0];
  row[2] = signs[wheel_index][1] * k;
}

bool solve_normal_equations(double ata[3][3], double atb[3], double x[3]) {
  // 3×3 Gaussian elimination with partial pivoting
  double a[3][4];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      a[i][j] = ata[i][j];
    }
    a[i][3] = atb[i];
  }
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 3; ++row) {
      if (std::abs(a[row][col]) > std::abs(a[pivot][col])) {
        pivot = row;
      }
    }
    if (std::abs(a[pivot][col]) < 1e-9) {
      return false;
    }
    if (pivot != col) {
      for (int j = 0; j < 4; ++j) {
        std::swap(a[pivot][j], a[col][j]);
      }
    }
    const double div = a[col][col];
    for (int j = col; j < 4; ++j) {
      a[col][j] /= div;
    }
    for (int row = 0; row < 3; ++row) {
      if (row == col) {
        continue;
      }
      const double factor = a[row][col];
      for (int j = col; j < 4; ++j) {
        a[row][j] -= factor * a[col][j];
      }
    }
  }
  for (int i = 0; i < 3; ++i) {
    x[i] = a[i][3];
  }
  return true;
}

bool fk_mecanum(const std::array<double, kWheelCount> &wheel_rad_s,
                const std::array<bool, kWheelCount> &active, double k, double out[3]) {
  int active_count = 0;
  for (bool on : active) {
    if (on) {
      ++active_count;
    }
  }
  if (active_count == 0) {
    return false;
  }

  if (active_count == kWheelCount) {
    const double w0 = wheel_rad_s[0];
    const double w1 = wheel_rad_s[1];
    const double w2 = wheel_rad_s[2];
    const double w3 = wheel_rad_s[3];
    out[0] = 0.25 * (w0 + w1 + w2 + w3);
    out[1] = 0.25 * (w1 - w0 + w2 - w3);
    out[2] = (-w0 + w1 - w2 + w3) / (4.0 * k);
    return true;
  }

  double ata[3][3] = {};
  double atb[3] = {};
  for (int i = 0; i < kWheelCount; ++i) {
    if (!active[i]) {
      continue;
    }
    double row[3];
    wheel_jacobian_row(i, k, row);
    const double b = wheel_rad_s[i];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        ata[r][c] += row[r] * row[c];
      }
      atb[r] += row[r] * b;
    }
  }
  return solve_normal_equations(ata, atb, out);
}

} // namespace

class QuadOdomNode : public rclcpp::Node {
public:
  QuadOdomNode() : Node("quad_odom_node") {
    wheel_base_x_ = declare_parameter("wheel_base_x", 0.30);
    wheel_base_y_ = declare_parameter("wheel_base_y", 0.20);
    odom_frame_ = declare_parameter("odom_frame_id", "odom");
    base_frame_ = declare_parameter("base_frame_id", "base_link");
    publish_tf_ = declare_parameter("publish_tf", true);
    velocity_ema_alpha_ = declare_parameter("velocity_ema_alpha", 0.35);
    stale_timeout_ms_ = declare_parameter("stale_timeout_ms", 500);
    min_dt_s_ = declare_parameter("min_dt_s", 0.001);
    max_dt_s_ = declare_parameter("max_dt_s", 0.25);
    open_loop_on_stale_ = declare_parameter("open_loop_on_stale", false);
    publish_path_ = declare_parameter("publish_path", true);
    path_max_points_ = declare_parameter("path_max_points", 2000);
    path_min_dist_m_ = declare_parameter("path_min_dist_m", 0.02);

    x_ = declare_parameter("initial_x", 0.0);
    y_ = declare_parameter("initial_y", 0.0);
    yaw_ = declare_parameter("initial_yaw", 0.0);

    pose_covariance_ =
        declare_parameter("pose_covariance_diagonal",
                          std::vector<double>{0.02, 0.02, 1e6, 1e6, 1e6, 0.05});
    twist_covariance_ =
        declare_parameter("twist_covariance_diagonal",
                          std::vector<double>{0.02, 0.02, 1e6, 1e6, 1e6, 0.03});

    std::vector<int64_t> active_wheels =
        declare_parameter("active_wheels", std::vector<int64_t>{1, 2, 3, 4});
    auto wheel_sign_param =
        declare_parameter("wheel_sign", std::vector<double>{1, 1, 1, 1});

    wheel_active_.fill(false);
    for (int64_t id : active_wheels) {
      if (id >= 1 && id <= 4) {
        wheel_active_[static_cast<size_t>(id - 1)] = true;
      }
    }
    for (int i = 0; i < kWheelCount; ++i) {
      if (static_cast<size_t>(i) < wheel_sign_param.size()) {
        wheel_sign_[i] = wheel_sign_param[i];
        if (std::abs(wheel_sign_[i]) < 1e-6) {
          wheel_sign_[i] = 1.0;
        }
      }
    }

    const double lx = wheel_base_x_ * 0.5;
    const double ly = wheel_base_y_ * 0.5;
    geom_k_ = lx + ly;

    auto feedback_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    wheel_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/quad/wheel_states", feedback_qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          on_wheel_states(msg);
        });

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    if (publish_path_) {
      path_pub_ = create_publisher<nav_msgs::msg::Path>("/odom/path", 10);
    }
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    reset_srv_ = create_service<std_srvs::srv::Empty>(
        "/quad/odom/reset",
        [this](const std_srvs::srv::Empty::Request::SharedPtr,
               std_srvs::srv::Empty::Response::SharedPtr) {
          reset_odom();
          return;
        });

    int active_count = 0;
    std::string active_list;
    for (int i = 0; i < kWheelCount; ++i) {
      if (!wheel_active_[i]) {
        continue;
      }
      ++active_count;
      if (!active_list.empty()) {
        active_list += ",";
      }
      active_list += "leg" + std::to_string(i + 1);
    }

    RCLCPP_INFO(get_logger(),
                "quad_odom | wheel_states → /odom | k=%.3f active=%d [%s] tf=%d",
                geom_k_, active_count, active_list.c_str(),
                static_cast<int>(publish_tf_));
    if (active_count < 4) {
      RCLCPP_WARN(get_logger(),
                  "仅 %d 轮在线：横移/转角为最小二乘估计，精度低于四轮全配",
                  active_count);
    }
  }

private:
  int wheel_index_from_name(const std::string &name) const {
    if (name.size() < 9 || name.rfind("_wheel") == std::string::npos) {
      return -1;
    }
    if (name.compare(0, 3, "leg") != 0) {
      return -1;
    }
    const int leg = name[3] - '0';
    if (leg < 1 || leg > 4) {
      return -1;
    }
    return leg - 1;
  }

  void on_wheel_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::array<double, kWheelCount> wheel_rad_s{};
    std::array<bool, kWheelCount> got{{false, false, false, false}};

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const int idx = wheel_index_from_name(msg->name[i]);
      if (idx < 0 || !wheel_active_[idx]) {
        continue;
      }
      if (i >= msg->velocity.size()) {
        continue;
      }
      const double vel = msg->velocity[i];
      if (!std::isfinite(vel)) {
        continue;
      }
      wheel_rad_s[idx] = vel * wheel_sign_[idx];
      got[idx] = true;
    }

    int got_count = 0;
    for (int i = 0; i < kWheelCount; ++i) {
      if (wheel_active_[i] && got[i]) {
        ++got_count;
      }
    }
    if (got_count == 0) {
      return;
    }

    double body[3] = {};
    if (!fk_mecanum(wheel_rad_s, wheel_active_, geom_k_, body)) {
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    if (last_stamp_.nanoseconds() > 0) {
      const double dt = (stamp - last_stamp_).seconds();
      if (dt >= min_dt_s_ && dt <= max_dt_s_) {
        integrate(body[0], body[1], body[2], dt);
      }
    } else {
      vx_ = body[0];
      vy_ = body[1];
      omega_ = body[2];
    }
    last_stamp_ = stamp;
    last_update_ = now();
    publish_odom(stamp);
  }

  void integrate(double raw_vx, double raw_vy, double raw_omega, double dt) {
    if (velocity_ema_alpha_ > 0.0 && velocity_ema_alpha_ < 1.0) {
      const double a = velocity_ema_alpha_;
      vx_ = a * raw_vx + (1.0 - a) * vx_;
      vy_ = a * raw_vy + (1.0 - a) * vy_;
      omega_ = a * raw_omega + (1.0 - a) * omega_;
    } else {
      vx_ = raw_vx;
      vy_ = raw_vy;
      omega_ = raw_omega;
    }

    const double cos_h = std::cos(yaw_);
    const double sin_h = std::sin(yaw_);
    x_ += (vx_ * cos_h - vy_ * sin_h) * dt;
    y_ += (vx_ * sin_h + vy_ * cos_h) * dt;
    yaw_ = normalize_angle(yaw_ + omega_ * dt);
  }

  void reset_odom() {
    x_ = y_ = yaw_ = vx_ = vy_ = omega_ = 0.0;
    last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    path_.poses.clear();
    last_path_x_ = last_path_y_ = 0.0;
    RCLCPP_INFO(get_logger(), "odom reset → (0, 0, 0)");
    publish_odom(now());
  }

  void maybe_append_path(const rclcpp::Time &stamp) {
    if (!publish_path_ || !path_pub_) {
      return;
    }
    if (!path_.poses.empty()) {
      const double dx = x_ - last_path_x_;
      const double dy = y_ - last_path_y_;
      if (std::hypot(dx, dy) < path_min_dist_m_) {
        return;
      }
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = odom_frame_;
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    pose.pose.orientation = tf2::toMsg(q);
    path_.poses.push_back(pose);
    last_path_x_ = x_;
    last_path_y_ = y_;
    while (static_cast<int>(path_.poses.size()) > path_max_points_) {
      path_.poses.erase(path_.poses.begin());
    }
    path_.header.stamp = stamp;
    path_.header.frame_id = odom_frame_;
    path_pub_->publish(path_);
  }

  void fill_covariance(std::array<double, 36> &cov,
                       const std::vector<double> &diag) const {
    cov.fill(0.0);
    for (size_t i = 0; i < diag.size() && i < 6; ++i) {
      cov[i * 6 + i] = diag[i];
    }
  }

  void publish_odom(const rclcpp::Time &stamp) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.angular.z = omega_;

    fill_covariance(odom.pose.covariance, pose_covariance_);
    fill_covariance(odom.twist.covariance, twist_covariance_);

    const int64_t age_ms =
        (now() - last_update_).nanoseconds() / 1000000;
    if (age_ms > stale_timeout_ms_ && !open_loop_on_stale_) {
      odom.twist.twist.linear.x = 0.0;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.angular.z = 0.0;
    }

    odom_pub_->publish(odom);
    maybe_append_path(stamp);

    if (!publish_tf_) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  double wheel_base_x_{0.30};
  double wheel_base_y_{0.20};
  double geom_k_{0.25};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  bool publish_tf_{true};
  double velocity_ema_alpha_{0.35};
  int stale_timeout_ms_{500};
  double min_dt_s_{0.001};
  double max_dt_s_{0.25};
  bool open_loop_on_stale_{false};
  bool publish_path_{true};
  int path_max_points_{2000};
  double path_min_dist_m_{0.02};
  std::vector<double> pose_covariance_;
  std::vector<double> twist_covariance_;

  std::array<bool, kWheelCount> wheel_active_{{true, true, true, true}};
  std::array<double, kWheelCount> wheel_sign_{{1, 1, 1, 1}};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double vx_{0.0};
  double vy_{0.0};
  double omega_{0.0};

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};

  nav_msgs::msg::Path path_;
  double last_path_x_{0.0};
  double last_path_y_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadOdomNode>());
  rclcpp::shutdown();
  return 0;
}
