#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "navigation_robot_base_driver/protocol.hpp"

using namespace std::chrono_literals;

namespace navigation_robot_base_driver
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
constexpr uint8_t kLeftMotorId = 1;
constexpr uint8_t kRightMotorId = 2;
constexpr uint8_t kSpeedMode = 2;
constexpr uint16_t kFeedbackOfflineMs = 500;
constexpr std::chrono::milliseconds kHandshakeRetryPeriod{300};
constexpr int kHandshakeMaxRetries = 8;

int64_t steady_time_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}
}  // namespace

class BaseDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit BaseDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("base_driver", options)
  {
    declare_parameter("transport", "serial");
    declare_parameter("serial_port", "/dev/navigation_base");
    declare_parameter("baud_rate", 115200);
    declare_parameter("tcp_host", "navigation-robot.local");
    declare_parameter("tcp_port", 3333);
    declare_parameter("wheel_radius_m", 0.0);
    declare_parameter("wheel_separation_m", 0.0);
    declare_parameter("max_rpm", 125.0);
    declare_parameter("max_linear_velocity", 0.6545);
    declare_parameter("max_angular_velocity", 5.2360);
    declare_parameter("motion_enabled", false);
    declare_parameter("command_timeout_sec", 0.2);
    declare_parameter("keepalive_period_sec", 0.1);
    declare_parameter("odom_frame_id", "odom");
    declare_parameter("base_frame_id", "base_footprint");
    declare_parameter("imu_frame_id", "imu_link");
    declare_parameter("imu_gyro_bias_calibration_enabled", true);
    declare_parameter("imu_gyro_bias_calibration_samples", 500);
    declare_parameter("imu_gyro_stationary_threshold_rad_s", 0.05);
    declare_parameter("imu_accel_norm_tolerance_m_s2", 1.5);
    declare_parameter("imu_wheel_stationary_threshold_rpm", 1.0);
  }

  ~BaseDriverNode() override
  {
    active_.store(false);
    running_.store(false);
    best_effort_stop();
    disconnect_transport();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    transport_type_ = get_parameter("transport").as_string();
    serial_port_ = get_parameter("serial_port").as_string();
    baud_rate_ = get_parameter("baud_rate").as_int();
    tcp_host_ = get_parameter("tcp_host").as_string();
    tcp_port_ = get_parameter("tcp_port").as_int();
    wheel_radius_m_ = get_parameter("wheel_radius_m").as_double();
    wheel_separation_m_ = get_parameter("wheel_separation_m").as_double();
    max_rpm_ = get_parameter("max_rpm").as_double();
    max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
    motion_enabled_ = get_parameter("motion_enabled").as_bool();
    command_timeout_ = std::chrono::duration<double>(
      get_parameter("command_timeout_sec").as_double());
    keepalive_period_ = std::chrono::duration<double>(
      get_parameter("keepalive_period_sec").as_double());
    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    imu_frame_id_ = get_parameter("imu_frame_id").as_string();
    imu_gyro_bias_calibration_enabled_ =
      get_parameter("imu_gyro_bias_calibration_enabled").as_bool();
    imu_gyro_bias_calibration_samples_ =
      get_parameter("imu_gyro_bias_calibration_samples").as_int();
    imu_gyro_stationary_threshold_rad_s_ =
      get_parameter("imu_gyro_stationary_threshold_rad_s").as_double();
    imu_accel_norm_tolerance_m_s2_ =
      get_parameter("imu_accel_norm_tolerance_m_s2").as_double();
    imu_wheel_stationary_threshold_rpm_ =
      get_parameter("imu_wheel_stationary_threshold_rpm").as_double();

    if (transport_type_ != "serial" && transport_type_ != "tcp") {
      RCLCPP_ERROR(get_logger(), "transport must be 'serial' or 'tcp'");
      return CallbackReturn::FAILURE;
    }
    if (baud_rate_ != 115200) {
      RCLCPP_ERROR(get_logger(), "ESP32 host protocol requires 115200 baud");
      return CallbackReturn::FAILURE;
    }
    if (imu_gyro_bias_calibration_samples_ < 50 ||
      !std::isfinite(imu_gyro_stationary_threshold_rad_s_) ||
      !std::isfinite(imu_accel_norm_tolerance_m_s2_) ||
      !std::isfinite(imu_wheel_stationary_threshold_rpm_) ||
      imu_gyro_stationary_threshold_rad_s_ <= 0.0 ||
      imu_accel_norm_tolerance_m_s2_ <= 0.0 ||
      imu_wheel_stationary_threshold_rpm_ < 0.0)
    {
      RCLCPP_ERROR(get_logger(), "invalid IMU gyro bias calibration parameters");
      return CallbackReturn::FAILURE;
    }

    wheel_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry", 20);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data_raw", rclcpp::SensorDataQoS());
    imu_corrected_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS());
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 20);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&BaseDriverNode::cmd_vel_callback, this, std::placeholders::_1));
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", std::bind(
        &BaseDriverNode::stop_callback, this, std::placeholders::_1, std::placeholders::_2));
    reconnect_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reconnect", std::bind(
        &BaseDriverNode::reconnect_callback, this, std::placeholders::_1, std::placeholders::_2));

    control_timer_ = create_wall_timer(20ms, std::bind(&BaseDriverNode::control_timer, this));
    diagnostics_timer_ = create_wall_timer(1s, std::bind(&BaseDriverNode::publish_diagnostics, this));
    reset_runtime_state();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    wheel_odom_pub_->on_activate();
    imu_pub_->on_activate();
    imu_corrected_pub_->on_activate();
    joint_pub_->on_activate();
    diagnostics_pub_->on_activate();
    active_.store(true);
    running_.store(true);
    worker_ = std::thread(&BaseDriverNode::transport_loop, this);
    if (!geometry_valid()) {
      RCLCPP_WARN(
        get_logger(),
        "wheel_radius_m/wheel_separation_m are not measured; motion is locked and odometry is disabled");
    }
    if (!motion_enabled_) {
      RCLCPP_INFO(get_logger(), "motion_enabled=false: sensor-only safety lock is active");
    }
    if (imu_gyro_bias_calibration_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "keep robot stationary: collecting %ld samples for IMU gyro bias calibration",
        static_cast<long>(imu_gyro_bias_calibration_samples_));
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_.store(false);
    best_effort_stop();
    running_.store(false);
    disconnect_transport();
    if (worker_.joinable()) {
      worker_.join();
    }
    wheel_odom_pub_->on_deactivate();
    imu_pub_->on_deactivate();
    imu_corrected_pub_->on_deactivate();
    joint_pub_->on_deactivate();
    diagnostics_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    cmd_sub_.reset();
    stop_service_.reset();
    reconnect_service_.reset();
    control_timer_.reset();
    diagnostics_timer_.reset();
    wheel_odom_pub_.reset();
    imu_pub_.reset();
    imu_corrected_pub_.reset();
    joint_pub_.reset();
    diagnostics_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  enum class LinkState {
    kDisconnected, kWaitHello, kWaitLeftQuery, kWaitLeftMode, kWaitRightQuery, kWaitRightMode,
    kReady
  };

  struct PendingAck
  {
    uint8_t request_type;
    std::chrono::steady_clock::time_point sent_at;
  };

  void clear_imu_bias_candidate()
  {
    imu_gyro_sum_x_ = 0.0;
    imu_gyro_sum_y_ = 0.0;
    imu_gyro_sum_z_ = 0.0;
    imu_gyro_bias_sample_count_.store(0);
  }

  void reset_imu_bias_calibration()
  {
    clear_imu_bias_candidate();
    imu_gyro_bias_x_.store(0.0);
    imu_gyro_bias_y_.store(0.0);
    imu_gyro_bias_z_.store(0.0);
    imu_gyro_bias_calibrated_.store(!imu_gyro_bias_calibration_enabled_);
  }

  bool update_imu_bias_calibration(
    double accel_x, double accel_y, double accel_z,
    double gyro_x, double gyro_y, double gyro_z)
  {
    if (imu_gyro_bias_calibrated_.load()) {
      return true;
    }

    const int64_t chassis_age_ns = steady_time_ns() - last_chassis_time_ns_.load();
    const double accel_norm = std::sqrt(
      accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    const double gyro_norm = std::sqrt(
      gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);
    const bool chassis_fresh = last_chassis_time_ns_.load() != 0 &&
      chassis_age_ns >= 0 && chassis_age_ns < 500000000LL;
    const bool feedback_fresh = left_feedback_age_ms_.load() <= kFeedbackOfflineMs &&
      right_feedback_age_ms_.load() <= kFeedbackOfflineMs;
    const bool wheels_stationary =
      std::abs(left_feedback_rpm_.load()) <= imu_wheel_stationary_threshold_rpm_ &&
      std::abs(right_feedback_rpm_.load()) <= imu_wheel_stationary_threshold_rpm_;
    const bool accel_stationary = std::isfinite(accel_norm) &&
      std::abs(accel_norm - 9.80665) <= imu_accel_norm_tolerance_m_s2_;
    const bool gyro_stationary = std::isfinite(gyro_norm) &&
      gyro_norm <= imu_gyro_stationary_threshold_rad_s_;

    if (!chassis_fresh || !feedback_fresh || !wheels_stationary ||
      !accel_stationary || !gyro_stationary)
    {
      if (imu_gyro_bias_sample_count_.load() > 0) {
        clear_imu_bias_candidate();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "IMU gyro bias calibration restarted because the robot was not stationary");
      }
      return false;
    }

    imu_gyro_sum_x_ += gyro_x;
    imu_gyro_sum_y_ += gyro_y;
    imu_gyro_sum_z_ += gyro_z;
    const int64_t sample_count = imu_gyro_bias_sample_count_.fetch_add(1) + 1;
    if (sample_count < imu_gyro_bias_calibration_samples_) {
      return false;
    }

    const double divisor = static_cast<double>(sample_count);
    imu_gyro_bias_x_.store(imu_gyro_sum_x_ / divisor);
    imu_gyro_bias_y_.store(imu_gyro_sum_y_ / divisor);
    imu_gyro_bias_z_.store(imu_gyro_sum_z_ / divisor);
    imu_gyro_bias_calibrated_.store(true);
    RCLCPP_INFO(
      get_logger(), "IMU gyro bias calibrated: x=%+.8f y=%+.8f z=%+.8f rad/s (%ld samples)",
      imu_gyro_bias_x_.load(), imu_gyro_bias_y_.load(), imu_gyro_bias_z_.load(),
      static_cast<long>(sample_count));
    return true;
  }

  void reset_runtime_state()
  {
    parser_.reset();
    link_state_.store(LinkState::kDisconnected);
    connected_.store(false);
    control_active_.store(false);
    last_command_active_.store(false);
    sequence_.store(0);
    target_left_rpm_ = 0;
    target_right_rpm_ = 0;
    command_dirty_ = false;
    last_cmd_time_ = std::chrono::steady_clock::time_point{};
    last_keepalive_time_ = std::chrono::steady_clock::time_point{};
    last_chassis_time_ns_.store(0);
    last_imu_time_ns_.store(0);
    left_feedback_rpm_.store(0.0);
    right_feedback_rpm_.store(0.0);
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    left_joint_position_ = 0.0;
    right_joint_position_ = 0.0;
    last_mcu_uptime_ms_ = 0;
    imu_time_initialized_ = false;
    reset_imu_bias_calibration();
  }

  bool geometry_valid() const
  {
    return std::isfinite(wheel_radius_m_) && std::isfinite(wheel_separation_m_) &&
           wheel_radius_m_ > 0.0 && wheel_separation_m_ > 0.0;
  }

  bool connect_transport()
  {
    int new_fd = -1;
    if (transport_type_ == "serial") {
      new_fd = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (new_fd >= 0) {
        termios tty{};
        if (tcgetattr(new_fd, &tty) != 0) {
          ::close(new_fd);
          new_fd = -1;
        } else {
          cfmakeraw(&tty);
          cfsetispeed(&tty, B115200);
          cfsetospeed(&tty, B115200);
          tty.c_cflag |= CLOCAL | CREAD;
          tty.c_cflag &= ~CSTOPB;
          tty.c_cflag &= ~CRTSCTS;
          if (tcsetattr(new_fd, TCSANOW, &tty) != 0) {
            ::close(new_fd);
            new_fd = -1;
          } else {
            tcflush(new_fd, TCIOFLUSH);
          }
        }
      }
    } else {
      addrinfo hints{};
      hints.ai_family = AF_UNSPEC;
      hints.ai_socktype = SOCK_STREAM;
      addrinfo * result = nullptr;
      const std::string port = std::to_string(tcp_port_);
      if (getaddrinfo(tcp_host_.c_str(), port.c_str(), &hints, &result) == 0) {
        for (addrinfo * entry = result; entry != nullptr && new_fd < 0; entry = entry->ai_next) {
          const int candidate = socket(entry->ai_family, entry->ai_socktype, entry->ai_protocol);
          if (candidate >= 0 && ::connect(candidate, entry->ai_addr, entry->ai_addrlen) == 0) {
            new_fd = candidate;
            fcntl(new_fd, F_SETFL, fcntl(new_fd, F_GETFL, 0) | O_NONBLOCK);
          } else if (candidate >= 0) {
            ::close(candidate);
          }
        }
        freeaddrinfo(result);
      }
    }

    if (new_fd < 0) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(transport_mutex_);
      fd_ = new_fd;
    }
    parser_.reset();
    sequence_.store(0);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      target_left_rpm_ = 0;
      target_right_rpm_ = 0;
      command_dirty_ = false;
      last_cmd_time_ = std::chrono::steady_clock::time_point{};
    }
    last_keepalive_time_ = std::chrono::steady_clock::time_point{};
    last_mcu_uptime_ms_ = 0;
    imu_time_initialized_ = false;
    connected_.store(true);
    link_state_.store(LinkState::kWaitHello);
    handshake_retries_ = 0;
    send_handshake_step();
    RCLCPP_INFO(get_logger(), "connected through %s", transport_type_.c_str());
    return true;
  }

  void disconnect_transport()
  {
    std::lock_guard<std::mutex> lock(transport_mutex_);
    if (fd_ >= 0) {
      ::shutdown(fd_, SHUT_RDWR);
      ::close(fd_);
      fd_ = -1;
    }
    connected_.store(false);
    control_active_.store(false);
    last_command_active_.store(false);
    link_state_.store(LinkState::kDisconnected);
    std::lock_guard<std::mutex> pending_lock(pending_mutex_);
    pending_acks_.clear();
  }

  bool write_bytes(const std::vector<uint8_t> & bytes)
  {
    std::lock_guard<std::mutex> lock(transport_mutex_);
    if (fd_ < 0) {
      return false;
    }
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const ssize_t written = transport_type_ == "tcp" ?
        ::send(fd_, bytes.data() + offset, bytes.size() - offset, MSG_NOSIGNAL) :
        ::write(fd_, bytes.data() + offset, bytes.size() - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        pollfd descriptor{fd_, POLLOUT, 0};
        if (::poll(&descriptor, 1, 100) > 0) {
          continue;
        }
      }
      return false;
    }
    return true;
  }

  uint8_t send_frame(uint8_t type, const std::vector<uint8_t> & payload, bool ack_required)
  {
    const uint8_t sequence = sequence_.fetch_add(1);
    Frame frame{type, sequence, ack_required ? kAckRequired : static_cast<uint8_t>(0), payload};
    const auto bytes = encode_frame(frame);
    if (ack_required) {
      // Register before writing: the serial worker can receive a fast ACK
      // immediately after write(), before this caller is scheduled again.
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_acks_[sequence] = PendingAck{type, std::chrono::steady_clock::now()};
    }
    if (bytes.empty() || !write_bytes(bytes)) {
      if (ack_required) {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_acks_.erase(sequence);
      }
      return sequence;
    }
    return sequence;
  }

  void send_handshake_step()
  {
    switch (link_state_.load()) {
      case LinkState::kWaitHello:
        send_frame(static_cast<uint8_t>(MessageType::kHello), {}, true);
        break;
      case LinkState::kWaitLeftQuery:
        send_frame(static_cast<uint8_t>(MessageType::kQuery), {kLeftMotorId}, true);
        break;
      case LinkState::kWaitLeftMode:
        send_frame(
          static_cast<uint8_t>(MessageType::kSetMode), {kLeftMotorId, kSpeedMode}, true);
        break;
      case LinkState::kWaitRightQuery:
        send_frame(static_cast<uint8_t>(MessageType::kQuery), {kRightMotorId}, true);
        break;
      case LinkState::kWaitRightMode:
        send_frame(
          static_cast<uint8_t>(MessageType::kSetMode), {kRightMotorId, kSpeedMode}, true);
        break;
      default:
        break;
    }
    handshake_sent_at_ = std::chrono::steady_clock::now();
  }

  void advance_handshake(LinkState next)
  {
    link_state_.store(next);
    handshake_retries_ = 0;
    send_handshake_step();
  }

  void handshake_watchdog()
  {
    const LinkState state = link_state_.load();
    if (!connected_.load() || state == LinkState::kDisconnected || state == LinkState::kReady) {
      return;
    }
    if (std::chrono::steady_clock::now() - handshake_sent_at_ < kHandshakeRetryPeriod) {
      return;
    }
    if (++handshake_retries_ > kHandshakeMaxRetries) {
      RCLCPP_WARN(get_logger(), "lower controller handshake timed out; reconnecting");
      disconnect_transport();
      return;
    }
    send_handshake_step();
  }

  void transport_loop()
  {
    std::vector<uint8_t> input(4096);
    while (running_.load()) {
      if (!connected_.load()) {
        if (!connect_transport()) {
          std::this_thread::sleep_for(1s);
          continue;
        }
      }
      int current_fd = -1;
      {
        std::lock_guard<std::mutex> lock(transport_mutex_);
        current_fd = fd_;
      }
      if (current_fd < 0) {
        continue;
      }
      pollfd descriptor{current_fd, POLLIN | POLLERR | POLLHUP, 0};
      const int poll_result = ::poll(&descriptor, 1, 100);
      if (poll_result < 0 && errno != EINTR) {
        disconnect_transport();
        continue;
      }
      if (poll_result <= 0) {
        continue;
      }
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        disconnect_transport();
        continue;
      }
      const ssize_t count = ::read(current_fd, input.data(), input.size());
      if (count > 0) {
        const auto frames = parser_.feed(input.data(), static_cast<std::size_t>(count));
        for (const auto & frame : frames) {
          handle_frame(frame);
        }
      } else if (count == 0 && transport_type_ == "tcp") {
        disconnect_transport();
      } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        disconnect_transport();
      }
    }
  }

  void handle_frame(const Frame & frame)
  {
    if (frame.type == static_cast<uint8_t>(MessageType::kAck)) {
      handle_ack(frame);
    } else if (frame.type == static_cast<uint8_t>(MessageType::kChassisTelemetry) &&
      frame.payload.size() == 56)
    {
      handle_chassis_telemetry(frame.payload);
    } else if (frame.type == static_cast<uint8_t>(MessageType::kImuTelemetry) &&
      frame.payload.size() == 44)
    {
      handle_imu_telemetry(frame.payload);
    }
  }

  void handle_ack(const Frame & frame)
  {
    if (frame.payload.size() != 4) {
      ++ack_errors_;
      return;
    }
    const uint8_t request_type = frame.payload[0];
    const uint8_t status = frame.payload[1];
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      const auto pending = pending_acks_.find(frame.sequence);
      if (pending == pending_acks_.end() || pending->second.request_type != request_type) {
        ++ack_errors_;
      } else {
        pending_acks_.erase(pending);
      }
    }
    const LinkState state = link_state_.load();
    if (status != 0) {
      ++ack_errors_;
      RCLCPP_WARN(get_logger(), "request 0x%02x rejected with status %u", request_type, status);
      if (request_type == static_cast<uint8_t>(MessageType::kSetDualRpm)) {
        control_active_.store(false);
        last_command_active_.store(false);
      }
      // The firmware rejects SET_MODE (precondition) until the motor has been
      // selected and confirmed by a fresh query; step back to the query phase
      // and let the handshake watchdog resend promptly.
      if (request_type == static_cast<uint8_t>(MessageType::kSetMode) &&
        state == LinkState::kWaitLeftMode)
      {
        link_state_.store(LinkState::kWaitLeftQuery);
      } else if (request_type == static_cast<uint8_t>(MessageType::kSetMode) &&
        state == LinkState::kWaitRightMode)
      {
        link_state_.store(LinkState::kWaitRightQuery);
      }
      handshake_sent_at_ = std::chrono::steady_clock::time_point{};
      return;
    }

    if (request_type == static_cast<uint8_t>(MessageType::kHello) &&
      state == LinkState::kWaitHello)
    {
      advance_handshake(LinkState::kWaitLeftQuery);
    } else if (request_type == static_cast<uint8_t>(MessageType::kQuery) &&
      state == LinkState::kWaitLeftQuery)
    {
      advance_handshake(LinkState::kWaitLeftMode);
    } else if (request_type == static_cast<uint8_t>(MessageType::kSetMode) &&
      state == LinkState::kWaitLeftMode)
    {
      advance_handshake(LinkState::kWaitRightQuery);
    } else if (request_type == static_cast<uint8_t>(MessageType::kQuery) &&
      state == LinkState::kWaitRightQuery)
    {
      advance_handshake(LinkState::kWaitRightMode);
    } else if (request_type == static_cast<uint8_t>(MessageType::kSetMode) &&
      state == LinkState::kWaitRightMode)
    {
      link_state_.store(LinkState::kReady);
      handshake_retries_ = 0;
      RCLCPP_INFO(get_logger(), "lower controller handshake complete");
    } else if (request_type == static_cast<uint8_t>(MessageType::kSetDualRpm)) {
      control_active_.store(last_command_active_.load());
    } else if (request_type == static_cast<uint8_t>(MessageType::kStopDual)) {
      control_active_.store(false);
    }
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!active_.load() || !motion_enabled_ || !geometry_valid() ||
      !imu_gyro_bias_calibrated_.load())
    {
      return;
    }
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "discarding non-finite cmd_vel");
      return;
    }
    const double linear = std::clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
    const double angular = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
    double left_rpm = ((linear - angular * wheel_separation_m_ / 2.0) / wheel_radius_m_) *
      60.0 / kTwoPi;
    double right_rpm = ((linear + angular * wheel_separation_m_ / 2.0) / wheel_radius_m_) *
      60.0 / kTwoPi;
    const double scale = std::max({1.0, std::abs(left_rpm) / max_rpm_,
      std::abs(right_rpm) / max_rpm_});
    left_rpm /= scale;
    right_rpm /= scale;
    const int16_t new_left_rpm = static_cast<int16_t>(std::lround(left_rpm));
    const int16_t new_right_rpm = static_cast<int16_t>(std::lround(right_rpm));
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      const bool target_changed =
        new_left_rpm != target_left_rpm_ || new_right_rpm != target_right_rpm_;
      const bool moving_target = new_left_rpm != 0 || new_right_rpm != 0;
      const bool needs_control_retry = moving_target &&
        !control_active_.load() && !last_command_active_.load();
      target_left_rpm_ = new_left_rpm;
      target_right_rpm_ = new_right_rpm;
      if (target_changed || needs_control_retry) {
        command_dirty_ = true;
      }
      last_cmd_time_ = std::chrono::steady_clock::now();
    }
  }

  void control_timer()
  {
    if (!active_.load()) {
      return;
    }
    handshake_watchdog();
    if (link_state_.load() != LinkState::kReady) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    bool send_command = false;
    int16_t left = 0;
    int16_t right = 0;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (last_cmd_time_ != std::chrono::steady_clock::time_point{} &&
        now - last_cmd_time_ > command_timeout_)
      {
        command_dirty_ = false;
        last_cmd_time_ = std::chrono::steady_clock::time_point{};
        if (control_active_.load()) {
          send_stop();
        }
        return;
      }
      send_command = command_dirty_;
      if (send_command) {
        left = target_left_rpm_;
        right = target_right_rpm_;
        command_dirty_ = false;
      }
    }
    if (send_command) {
      std::vector<uint8_t> payload{kLeftMotorId, kRightMotorId};
      append_i16_le(payload, left);
      append_i16_le(payload, right);
      payload.push_back(0);
      payload.push_back(0);
      send_frame(static_cast<uint8_t>(MessageType::kSetDualRpm), payload, true);
      // The firmware only considers dual control active (and only accepts
      // keepalives) when at least one target RPM is non-zero.
      last_command_active_.store(left != 0 || right != 0);
      last_keepalive_time_ = now;
    } else if (control_active_.load() && now - last_keepalive_time_ >= keepalive_period_) {
      send_frame(
        static_cast<uint8_t>(MessageType::kDualKeepalive), {kLeftMotorId, kRightMotorId}, false);
      last_keepalive_time_ = now;
    }
  }

  void send_stop()
  {
    send_frame(
      static_cast<uint8_t>(MessageType::kStopDual),
      {kLeftMotorId, kRightMotorId, 0xFF}, true);
    control_active_.store(false);
    last_command_active_.store(false);
  }

  void best_effort_stop()
  {
    if (connected_.load()) {
      send_stop();
    }
  }

  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      target_left_rpm_ = 0;
      target_right_rpm_ = 0;
      command_dirty_ = false;
      last_cmd_time_ = std::chrono::steady_clock::time_point{};
    }
    best_effort_stop();
    response->success = connected_.load();
    response->message = connected_.load() ? "braking stop sent" : "link is disconnected";
  }

  void reconnect_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    best_effort_stop();
    disconnect_transport();
    response->success = true;
    response->message = "reconnect requested";
  }

  void handle_chassis_telemetry(const std::vector<uint8_t> & payload)
  {
    last_chassis_time_ns_.store(steady_time_ns());
    const uint32_t uptime_ms = read_u32_le(payload.data());
    owner_.store(payload[4]);
    chassis_flags_.store(payload[5]);
    watchdog_stops_.store(read_u16_le(payload.data() + 6));
    const uint16_t left_age_ms = read_u16_le(payload.data() + 22);
    const uint16_t right_age_ms = read_u16_le(payload.data() + 46);
    left_feedback_age_ms_.store(left_age_ms);
    right_feedback_age_ms_.store(right_age_ms);
    if (left_age_ms > kFeedbackOfflineMs || right_age_ms > kFeedbackOfflineMs) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "wheel feedback stale (left %u ms, right %u ms); wheel odometry paused",
        left_age_ms, right_age_ms);
      return;
    }
    const double left_rpm = read_i16_le(payload.data() + 14);
    const double right_rpm = read_i16_le(payload.data() + 38);
    left_feedback_rpm_.store(left_rpm);
    right_feedback_rpm_.store(right_rpm);
    const double left_rad_s = left_rpm * kTwoPi / 60.0;
    const double right_rad_s = right_rpm * kTwoPi / 60.0;

    double dt = 0.0;
    if (last_mcu_uptime_ms_ != 0 && uptime_ms >= last_mcu_uptime_ms_) {
      dt = static_cast<double>(uptime_ms - last_mcu_uptime_ms_) / 1000.0;
    }
    last_mcu_uptime_ms_ = uptime_ms;

    if (geometry_valid() && dt > 0.0 && dt < 1.0) {
      const double left_velocity = left_rad_s * wheel_radius_m_;
      const double right_velocity = right_rad_s * wheel_radius_m_;
      const double linear = (left_velocity + right_velocity) / 2.0;
      const double angular = (right_velocity - left_velocity) / wheel_separation_m_;
      const double heading_mid = yaw_ + angular * dt / 2.0;
      x_ += linear * std::cos(heading_mid) * dt;
      y_ += linear * std::sin(heading_mid) * dt;
      yaw_ = std::atan2(std::sin(yaw_ + angular * dt), std::cos(yaw_ + angular * dt));
      left_joint_position_ += left_rad_s * dt;
      right_joint_position_ += right_rad_s * dt;
      publish_wheel_odom(linear, angular);
    }
    publish_joint_state(left_rad_s, right_rad_s);
  }

  void publish_wheel_odom(double linear, double angular)
  {
    if (!wheel_odom_pub_->is_activated()) {
      return;
    }
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = odom_frame_id_;
    msg.child_frame_id = base_frame_id_;
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
    msg.twist.twist.linear.x = linear;
    msg.twist.twist.angular.z = angular;
    msg.pose.covariance[0] = 0.05;
    msg.pose.covariance[7] = 0.05;
    msg.pose.covariance[35] = 0.10;
    msg.twist.covariance[0] = 0.02;
    msg.twist.covariance[35] = 0.05;
    wheel_odom_pub_->publish(msg);
  }

  void publish_joint_state(double left_rad_s, double right_rad_s)
  {
    if (!joint_pub_->is_activated()) {
      return;
    }
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = {"left_wheel_joint", "right_wheel_joint"};
    msg.position = {left_joint_position_, right_joint_position_};
    msg.velocity = {left_rad_s, right_rad_s};
    joint_pub_->publish(msg);
  }

  void handle_imu_telemetry(const std::vector<uint8_t> & payload)
  {
    last_imu_time_ns_.store(steady_time_ns());
    const uint8_t flags = payload[8];
    imu_flags_.store(flags);
    if ((flags & 0x06) != 0x06 || !imu_pub_->is_activated()) {
      return;
    }
    const uint64_t mcu_time_us = read_u64_le(payload.data());
    const rclcpp::Time host_now = now();
    if (!imu_time_initialized_ || mcu_time_us < last_imu_mcu_time_us_) {
      imu_time_offset_ns_ = host_now.nanoseconds() - static_cast<int64_t>(mcu_time_us * 1000ULL);
      imu_time_initialized_ = true;
    }
    last_imu_mcu_time_us_ = mcu_time_us;

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = rclcpp::Time(
      imu_time_offset_ns_ + static_cast<int64_t>(mcu_time_us * 1000ULL), get_clock()->get_clock_type());
    msg.header.frame_id = imu_frame_id_;
    msg.orientation_covariance[0] = -1.0;
    msg.linear_acceleration.x = read_f32_le(payload.data() + 12);
    msg.linear_acceleration.y = read_f32_le(payload.data() + 16);
    msg.linear_acceleration.z = read_f32_le(payload.data() + 20);
    msg.angular_velocity.x = read_f32_le(payload.data() + 24);
    msg.angular_velocity.y = read_f32_le(payload.data() + 28);
    msg.angular_velocity.z = read_f32_le(payload.data() + 32);
    msg.linear_acceleration_covariance[0] = 0.10;
    msg.linear_acceleration_covariance[4] = 0.10;
    msg.linear_acceleration_covariance[8] = 0.10;
    msg.angular_velocity_covariance[0] = 0.02;
    msg.angular_velocity_covariance[4] = 0.02;
    msg.angular_velocity_covariance[8] = 0.02;
    imu_pub_->publish(msg);

    if (!update_imu_bias_calibration(
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
    {
      return;
    }

    sensor_msgs::msg::Imu corrected = msg;
    corrected.angular_velocity.x -= imu_gyro_bias_x_.load();
    corrected.angular_velocity.y -= imu_gyro_bias_y_.load();
    corrected.angular_velocity.z -= imu_gyro_bias_z_.load();
    imu_corrected_pub_->publish(corrected);
  }

  void publish_diagnostics()
  {
    if (!diagnostics_pub_ || !diagnostics_pub_->is_activated()) {
      return;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "navigation_robot/base_driver";
    status.hardware_id = "esp32-s3-m0601-bmi088";
    const int64_t now_steady_ns = steady_time_ns();
    const int64_t last_chassis_ns = last_chassis_time_ns_.load();
    const bool chassis_fresh = last_chassis_ns != 0 &&
      now_steady_ns - last_chassis_ns < 500000000LL;
    if (!connected_.load()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "transport disconnected";
    } else if (link_state_.load() != LinkState::kReady || !chassis_fresh) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "link not ready or chassis telemetry stale";
    } else if (!imu_gyro_bias_calibrated_.load()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "imu gyro calibration in progress; keep robot stationary";
    } else if (!geometry_valid() || !motion_enabled_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "sensor-only motion lock";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "ready";
    }
    status.values = {
      kv("transport", transport_type_),
      kv("connected", connected_.load() ? "true" : "false"),
      kv("motion_enabled", motion_enabled_ ? "true" : "false"),
      kv("control_owner", std::to_string(owner_.load())),
      kv("chassis_flags", std::to_string(chassis_flags_.load())),
      kv("imu_flags", std::to_string(imu_flags_.load())),
      kv("imu_gyro_bias_calibrated", imu_gyro_bias_calibrated_.load() ? "true" : "false"),
      kv("imu_gyro_bias_samples", std::to_string(imu_gyro_bias_sample_count_.load())),
      kv("imu_gyro_bias_x_rad_s", std::to_string(imu_gyro_bias_x_.load())),
      kv("imu_gyro_bias_y_rad_s", std::to_string(imu_gyro_bias_y_.load())),
      kv("imu_gyro_bias_z_rad_s", std::to_string(imu_gyro_bias_z_.load())),
      kv("left_feedback_age_ms", std::to_string(left_feedback_age_ms_.load())),
      kv("right_feedback_age_ms", std::to_string(right_feedback_age_ms_.load())),
      kv("watchdog_stops", std::to_string(watchdog_stops_.load())),
      kv("host_crc_errors", std::to_string(parser_.crc_errors())),
      kv("host_format_errors", std::to_string(parser_.format_errors())),
      kv("ack_errors", std::to_string(ack_errors_.load()))};
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);

    std::lock_guard<std::mutex> lock(pending_mutex_);
    for (auto it = pending_acks_.begin(); it != pending_acks_.end();) {
      if (std::chrono::steady_clock::now() - it->second.sent_at > 500ms) {
        ++ack_errors_;
        it = pending_acks_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::string transport_type_;
  std::string serial_port_;
  int64_t baud_rate_{115200};
  std::string tcp_host_;
  int64_t tcp_port_{3333};
  double wheel_radius_m_{0.0};
  double wheel_separation_m_{0.0};
  double max_rpm_{125.0};
  double max_linear_velocity_{0.6545};
  double max_angular_velocity_{5.2360};
  bool motion_enabled_{false};
  bool imu_gyro_bias_calibration_enabled_{true};
  int64_t imu_gyro_bias_calibration_samples_{500};
  double imu_gyro_stationary_threshold_rad_s_{0.05};
  double imu_accel_norm_tolerance_m_s2_{1.5};
  double imu_wheel_stationary_threshold_rpm_{1.0};
  std::chrono::duration<double> command_timeout_{0.2};
  std::chrono::duration<double> keepalive_period_{0.1};
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string imu_frame_id_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_corrected_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnect_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  std::atomic<bool> active_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> control_active_{false};
  std::atomic<bool> last_command_active_{false};
  std::atomic<LinkState> link_state_{LinkState::kDisconnected};
  std::chrono::steady_clock::time_point handshake_sent_at_;
  int handshake_retries_{0};
  std::atomic<uint8_t> sequence_{0};
  std::atomic<uint64_t> ack_errors_{0};
  std::thread worker_;
  int fd_{-1};
  std::mutex transport_mutex_;
  StreamParser parser_;

  std::mutex pending_mutex_;
  std::map<uint8_t, PendingAck> pending_acks_;
  std::mutex command_mutex_;
  int16_t target_left_rpm_{0};
  int16_t target_right_rpm_{0};
  bool command_dirty_{false};
  std::chrono::steady_clock::time_point last_cmd_time_;
  std::chrono::steady_clock::time_point last_keepalive_time_;
  std::atomic<int64_t> last_chassis_time_ns_{0};
  std::atomic<int64_t> last_imu_time_ns_{0};
  std::atomic<double> left_feedback_rpm_{0.0};
  std::atomic<double> right_feedback_rpm_{0.0};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double left_joint_position_{0.0};
  double right_joint_position_{0.0};
  uint32_t last_mcu_uptime_ms_{0};
  bool imu_time_initialized_{false};
  uint64_t last_imu_mcu_time_us_{0};
  int64_t imu_time_offset_ns_{0};
  std::atomic<uint8_t> owner_{0};
  std::atomic<uint8_t> chassis_flags_{0};
  std::atomic<uint8_t> imu_flags_{0};
  std::atomic<bool> imu_gyro_bias_calibrated_{false};
  std::atomic<int64_t> imu_gyro_bias_sample_count_{0};
  std::atomic<double> imu_gyro_bias_x_{0.0};
  std::atomic<double> imu_gyro_bias_y_{0.0};
  std::atomic<double> imu_gyro_bias_z_{0.0};
  double imu_gyro_sum_x_{0.0};
  double imu_gyro_sum_y_{0.0};
  double imu_gyro_sum_z_{0.0};
  std::atomic<uint16_t> watchdog_stops_{0};
  std::atomic<uint16_t> left_feedback_age_ms_{0};
  std::atomic<uint16_t> right_feedback_age_ms_{0};
};

}  // namespace navigation_robot_base_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<navigation_robot_base_driver::BaseDriverNode>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
