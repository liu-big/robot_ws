#include "motor_ros2/c620_motor.h"

#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <unistd.h>

C620Motor::C620Motor(const std::string &iface, uint8_t motor_id)
    : iface_(iface), motor_id_(motor_id) {
  if (motor_id_ < 1 || motor_id_ > 8) {
    motor_id_ = 1;
  }
  tx_payload_.fill(0);
  open_socket();
}

C620Motor::~C620Motor() {
  if (fd_ >= 0) {
    set_current_a(0.f);
    for (int i = 0; i < 5; ++i) {
      if (transmit()) {
        break;
      }
      usleep(1000);
    }
    close(fd_);
    fd_ = -1;
  }
}

int16_t C620Motor::amps_to_raw(float amps) {
  amps = std::max(-kCurrentMaxA, std::min(kCurrentMaxA, amps));
  return static_cast<int16_t>(amps / kCurrentMaxA * kCurrentMaxRaw);
}

float C620Motor::raw_to_amps(int16_t raw) {
  return static_cast<float>(raw) / kCurrentMaxRaw * kCurrentMaxA;
}

canid_t C620Motor::control_can_id() const {
  return static_cast<canid_t>(motor_id_ <= 4 ? kControlId_1_4 : kControlId_5_8);
}

int C620Motor::payload_offset() const {
  return (motor_id_ <= 4) ? (motor_id_ - 1) * 2 : (motor_id_ - 5) * 2;
}

bool C620Motor::open_socket() {
  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);

  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    perror("c620 socket");
    return false;
  }

  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    perror("c620 ioctl");
    close(fd_);
    fd_ = -1;
    return false;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    perror("c620 bind");
    close(fd_);
    fd_ = -1;
    return false;
  }

  const int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  int sndbuf = 1 << 16;
  setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
  return true;
}

void C620Motor::set_current_a(float amps) {
  pending_amps_ = amps;
  const int16_t raw = amps_to_raw(amps);
  const int off = payload_offset();
  tx_payload_[off] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  tx_payload_[off + 1] = static_cast<uint8_t>(raw & 0xFF);
}

bool C620Motor::recover_socket() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  usleep(50000);
  tx_fail_streak_ = 0;
  has_last_sent_ = false;
  return open_socket();
}

void C620Motor::drain_rx(int max_frames) {
  if (fd_ < 0 || max_frames <= 0) {
    return;
  }
  struct can_frame frame{};
  for (int i = 0; i < max_frames; ++i) {
    const ssize_t n = read(fd_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
      break;
    }
    if (auto fb = parse_frame(frame)) {
      latest_ = *fb;
      last_fb_tp_ = std::chrono::steady_clock::now();
      has_fb_ = true;
    }
  }
}

bool C620Motor::transmit(bool force) {
  if (fd_ < 0) {
    return false;
  }

  if (!force && has_last_sent_ && tx_payload_ == last_sent_payload_ &&
      tx_fail_streak_ == 0) {
    return true;
  }

  if (tx_fail_streak_ > 0) {
    drain_rx(32);
  }

  struct can_frame frame{};
  frame.can_id = control_can_id();
  frame.can_dlc = 8;
  std::memcpy(frame.data, tx_payload_.data(), 8);

  for (int attempt = 0; attempt < 6; ++attempt) {
    const ssize_t n = write(fd_, &frame, sizeof(frame));
    if (n == static_cast<ssize_t>(sizeof(frame))) {
      tx_fail_streak_ = 0;
      ++tx_ok_count_;
      last_sent_payload_ = tx_payload_;
      has_last_sent_ = true;
      return true;
    }
    const int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK) {
      drain_rx(16);
      usleep(1000 * (attempt + 1));
      continue;
    }
    ++tx_fail_streak_;
    return false;
  }
  ++tx_fail_streak_;
  return false;
}

std::optional<C620Motor::Feedback>
C620Motor::parse_frame(const struct can_frame &frame) const {
  const canid_t expected =
      static_cast<canid_t>(kControlId_1_4 + motor_id_);
  if ((frame.can_id & CAN_SFF_MASK) != expected) {
    return std::nullopt;
  }

  Feedback fb{};
  fb.angle_raw =
      static_cast<uint16_t>((frame.data[0] << 8) | frame.data[1]);
  fb.speed_rpm =
      static_cast<int16_t>((frame.data[2] << 8) | frame.data[3]);
  fb.current_raw =
      static_cast<int16_t>((frame.data[4] << 8) | frame.data[5]);
  fb.current_a = raw_to_amps(fb.current_raw);
  fb.temperature_c = frame.data[6];
  fb.valid = true;
  return fb;
}

int C620Motor::fb_age_ms() const {
  if (!has_fb_) {
    return -1;
  }
  const auto age = std::chrono::steady_clock::now() - last_fb_tp_;
  return static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(age).count());
}

C620Motor::Feedback C620Motor::poll_feedback() {
  if (fd_ < 0) {
    return latest_;
  }

  struct can_frame frame{};
  while (read(fd_, &frame, sizeof(frame)) ==
         static_cast<ssize_t>(sizeof(frame))) {
    if (auto fb = parse_frame(frame)) {
      latest_ = *fb;
      last_fb_tp_ = std::chrono::steady_clock::now();
      has_fb_ = true;
    }
  }
  return latest_;
}

// ── C620Bus ─────────────────────────────────────────────────────────

C620Bus::C620Bus(const std::string &iface) : iface_(iface) {
  tx_payload_.fill(0);
  has_fb_.fill(false);
  open_socket();
}

C620Bus::~C620Bus() {
  if (fd_ >= 0) {
    zero_all();
    for (int i = 0; i < 5; ++i) {
      if (transmit(true)) {
        break;
      }
      usleep(1000);
    }
    close(fd_);
    fd_ = -1;
  }
}

bool C620Bus::open_socket() {
  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);

  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    perror("c620_bus socket");
    return false;
  }

  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    perror("c620_bus ioctl");
    close(fd_);
    fd_ = -1;
    return false;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    perror("c620_bus bind");
    close(fd_);
    fd_ = -1;
    return false;
  }

  const int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  int sndbuf = 1 << 16;
  setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
  return true;
}

int C620Bus::payload_offset(uint8_t motor_id) {
  if (motor_id >= 1 && motor_id <= 4) {
    return (motor_id - 1) * 2;
  }
  return -1;
}

void C620Bus::set_current_a(uint8_t motor_id, float amps) {
  const int off = payload_offset(motor_id);
  if (off < 0) {
    return;
  }
  const int16_t raw = C620Motor::amps_to_raw(amps);
  tx_payload_[off] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  tx_payload_[off + 1] = static_cast<uint8_t>(raw & 0xFF);
}

void C620Bus::zero_all() {
  tx_payload_.fill(0);
}

bool C620Bus::recover_socket() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  usleep(50000);
  tx_fail_streak_ = 0;
  has_last_sent_ = false;
  return open_socket();
}

void C620Bus::drain_rx(int max_frames) {
  if (fd_ < 0 || max_frames <= 0) {
    return;
  }
  struct can_frame frame{};
  for (int i = 0; i < max_frames; ++i) {
    const ssize_t n = read(fd_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
      break;
    }
    if (auto fb = parse_frame(frame)) {
      const uint8_t mid = static_cast<uint8_t>((frame.can_id & CAN_SFF_MASK) -
                                               C620Motor::kControlId_1_4);
      if (mid >= 1 && mid <= 4) {
        const size_t idx = mid - 1;
        latest_[idx] = *fb;
        has_fb_[idx] = true;
        last_fb_tp_[idx] = std::chrono::steady_clock::now();
      }
    }
  }
}

bool C620Bus::transmit(bool force) {
  if (fd_ < 0) {
    return false;
  }

  if (!force && has_last_sent_ && tx_payload_ == last_sent_payload_ &&
      tx_fail_streak_ == 0) {
    return true;
  }

  if (tx_fail_streak_ > 0) {
    drain_rx(32);
  }

  struct can_frame frame{};
  frame.can_id = C620Motor::kControlId_1_4;
  frame.can_dlc = 8;
  std::memcpy(frame.data, tx_payload_.data(), 8);

  for (int attempt = 0; attempt < 6; ++attempt) {
    const ssize_t n = write(fd_, &frame, sizeof(frame));
    if (n == static_cast<ssize_t>(sizeof(frame))) {
      tx_fail_streak_ = 0;
      last_sent_payload_ = tx_payload_;
      has_last_sent_ = true;
      return true;
    }
    const int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK) {
      drain_rx(16);
      usleep(1000 * (attempt + 1));
      continue;
    }
    ++tx_fail_streak_;
    return false;
  }
  ++tx_fail_streak_;
  return false;
}

std::optional<C620Motor::Feedback>
C620Bus::parse_frame(const struct can_frame &frame) {
  const canid_t id = frame.can_id & CAN_SFF_MASK;
  if (id < C620Motor::kControlId_1_4 + 1 ||
      id > C620Motor::kControlId_1_4 + 4) {
    return std::nullopt;
  }

  C620Motor::Feedback fb{};
  fb.angle_raw =
      static_cast<uint16_t>((frame.data[0] << 8) | frame.data[1]);
  fb.speed_rpm =
      static_cast<int16_t>((frame.data[2] << 8) | frame.data[3]);
  fb.current_raw =
      static_cast<int16_t>((frame.data[4] << 8) | frame.data[5]);
  fb.current_a = C620Motor::raw_to_amps(fb.current_raw);
  fb.temperature_c = frame.data[6];
  fb.valid = true;
  return fb;
}

C620Motor::Feedback C620Bus::feedback(uint8_t motor_id) const {
  if (motor_id >= 1 && motor_id <= 4) {
    return latest_[motor_id - 1];
  }
  return {};
}

int C620Bus::fb_age_ms(uint8_t motor_id) const {
  if (motor_id < 1 || motor_id > 4 || !has_fb_[motor_id - 1]) {
    return -1;
  }
  const auto age =
      std::chrono::steady_clock::now() - last_fb_tp_[motor_id - 1];
  return static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(age).count());
}
