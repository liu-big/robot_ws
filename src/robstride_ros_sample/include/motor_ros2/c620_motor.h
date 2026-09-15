#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <linux/can.h>
#include <net/if.h>
#include <optional>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

/// 大疆 RoboMaster C620 + M3508 — 手册 V1.01 标准帧 CAN
/// 参考 RoboMaster 开源队控 / rm_motors_can 协议实现
class C620Motor {
public:
  static constexpr int kControlId_1_4 = 0x200;
  static constexpr int kControlId_5_8 = 0x1FF;
  static constexpr int kCurrentMaxRaw = 16384;
  static constexpr float kCurrentMaxA = 20.0f;
  static constexpr int kAngleRawMax = 8191;

  struct Feedback {
    uint16_t angle_raw{0};   ///< 转子机械角 0~8191 → 0~360°
    int16_t speed_rpm{0};    ///< 转子转速 rpm（手册 DATA[2:3]）
    int16_t current_raw{0};  ///< 实际转矩电流 raw
    float current_a{0.f};
    uint8_t temperature_c{0};
    bool valid{false};
  };

  C620Motor(const std::string &iface, uint8_t motor_id);
  ~C620Motor();

  /// 设置本电机目标电流 (A)，写入待发控制帧
  void set_current_a(float amps);
  /// 发送控制帧（建议 200~500Hz，过高会塞满 gs_usb 队列）
  /// force=true 时即使 payload 未变也重发（CAN 恢复后保活）
  bool transmit(bool force = false);
  /// 关闭并重开 CAN 套接字（TX 堵塞时调用）
  bool recover_socket();
  /// 排空 RX 缓冲（TX 队列满时先读反馈再发）
  void drain_rx(int max_frames = 64);
  /// 读尽 RX 缓冲，返回本电机最新反馈
  Feedback poll_feedback();
  bool is_ready() const { return fd_ >= 0; }
  uint32_t tx_fail_streak() const { return tx_fail_streak_; }
  uint32_t tx_ok_count() const { return tx_ok_count_; }
  float pending_current_a() const { return pending_amps_; }
  int fb_age_ms() const;

  static int16_t amps_to_raw(float amps);
  static float raw_to_amps(int16_t raw);

private:
  bool open_socket();
  canid_t control_can_id() const;
  int payload_offset() const;
  std::optional<Feedback> parse_frame(const struct can_frame &frame) const;

  std::string iface_;
  uint8_t motor_id_;
  int fd_{-1};
  std::array<uint8_t, 8> tx_payload_{};
  float pending_amps_{0.f};
  Feedback latest_{};
  uint32_t tx_fail_streak_{0};
  uint32_t tx_ok_count_{0};
  std::array<uint8_t, 8> last_sent_payload_{};
  bool has_last_sent_{false};
  bool has_fb_{false};
  std::chrono::steady_clock::time_point last_fb_tp_{};
};

/// 单 socket 控 4×C620（一帧 0x200），供 c620_quad_ros2 使用
class C620Bus {
public:
  explicit C620Bus(const std::string &iface);
  ~C620Bus();

  void set_current_a(uint8_t motor_id, float amps);
  void zero_all();
  bool transmit(bool force = false);
  bool recover_socket();
  void drain_rx(int max_frames = 64);
  C620Motor::Feedback feedback(uint8_t motor_id) const;
  int fb_age_ms(uint8_t motor_id) const;
  bool is_ready() const { return fd_ >= 0; }
  uint32_t tx_fail_streak() const { return tx_fail_streak_; }

private:
  bool open_socket();
  static int payload_offset(uint8_t motor_id);
  static std::optional<C620Motor::Feedback>
  parse_frame(const struct can_frame &frame);

  std::string iface_;
  int fd_{-1};
  std::array<uint8_t, 8> tx_payload_{};
  std::array<uint8_t, 8> last_sent_payload_{};
  bool has_last_sent_{false};
  uint32_t tx_fail_streak_{0};
  std::array<C620Motor::Feedback, 4> latest_{};
  std::array<bool, 4> has_fb_{};
  std::array<std::chrono::steady_clock::time_point, 4> last_fb_tp_{};
};
