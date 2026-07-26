#include <asm/termbits.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

namespace navigation_robot_lidar_driver
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

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

class LidarNode : public rclcpp::Node
{
public:
  LidarNode()
  : Node("lidar_node")
  {
    port_name_ = declare_parameter("port_name", "/dev/navigation_lidar");
    baud_rate_ = declare_parameter("baud_rate", 150000);
    frame_id_ = declare_parameter("frame_id", "lidar_link");
    scan_topic_ = declare_parameter("scan_topic", "/scan");
    range_min_ = declare_parameter("range_min", 0.05);
    range_max_ = declare_parameter("range_max", 8.0);
    inverted_ = declare_parameter("inverted", true);
    scan_size_ = declare_parameter("scan_size", 720);
    reconnect_interval_ = std::chrono::duration<double>(
      declare_parameter("reconnect_interval_sec", 1.0));

    if (baud_rate_ <= 0 || scan_size_ < 90 || range_min_ <= 0.0 || range_max_ <= range_min_) {
      throw std::runtime_error("invalid lidar parameters");
    }
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    diagnostics_timer_ = create_wall_timer(1s, std::bind(&LidarNode::publish_diagnostics, this));
    points_.reserve(1200);
    running_.store(true);
    worker_ = std::thread(&LidarNode::read_loop, this);
  }

  ~LidarNode() override
  {
    running_.store(false);
    close_serial();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct Point
  {
    double angle;
    double distance;
  };

  enum class ParseState {kHeader1, kHeader2, kMeta, kPayload};

  bool open_serial()
  {
    const int new_fd = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (new_fd < 0) {
      return false;
    }
    termios2 settings{};
    if (ioctl(new_fd, TCGETS2, &settings) != 0) {
      ::close(new_fd);
      return false;
    }
    settings.c_cflag &= ~CBAUD;
    settings.c_cflag |= BOTHER | CLOCAL | CREAD;
    settings.c_ispeed = static_cast<speed_t>(baud_rate_);
    settings.c_ospeed = static_cast<speed_t>(baud_rate_);
    settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    settings.c_cflag |= CS8;
    settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    settings.c_iflag &= ~(IXON | IXOFF | IXANY);
    settings.c_oflag &= ~OPOST;
    if (ioctl(new_fd, TCSETS2, &settings) != 0) {
      ::close(new_fd);
      return false;
    }
    ioctl(new_fd, TCFLSH, TCIOFLUSH);
    {
      std::lock_guard<std::mutex> lock(fd_mutex_);
      fd_ = new_fd;
    }
    const uint8_t start[] = {0xA5, 0x60};
    const ssize_t start_written = ::write(new_fd, start, sizeof(start));
    if (start_written != static_cast<ssize_t>(sizeof(start))) {
      ++write_errors_;
    }
    connected_.store(true);
    reset_parser();
    RCLCPP_INFO(get_logger(), "lidar connected on %s at %d baud", port_name_.c_str(), baud_rate_);
    return true;
  }

  void close_serial()
  {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ >= 0) {
      const uint8_t stop[] = {0xA5, 0x00, 0xA5, 0x65, 0xA5, 0x65};
      const ssize_t stop_written = ::write(fd_, stop, sizeof(stop));
      if (stop_written != static_cast<ssize_t>(sizeof(stop))) {
        ++write_errors_;
      }
      ::close(fd_);
      fd_ = -1;
    }
    connected_.store(false);
  }

  void reset_parser()
  {
    state_ = ParseState::kHeader1;
    packet_.clear();
    target_packet_size_ = 0;
    points_.clear();
    last_angle_ = 0.0;
    have_complete_scan_ = false;
    scan_start_ = now();
  }

  void read_loop()
  {
    std::vector<uint8_t> input(2048);
    while (running_.load()) {
      if (!connected_.load()) {
        if (!open_serial()) {
          std::this_thread::sleep_for(reconnect_interval_);
          continue;
        }
      }
      int current_fd = -1;
      {
        std::lock_guard<std::mutex> lock(fd_mutex_);
        current_fd = fd_;
      }
      pollfd descriptor{current_fd, POLLIN | POLLERR | POLLHUP, 0};
      const int result = ::poll(&descriptor, 1, 100);
      if (result < 0 && errno != EINTR) {
        close_serial();
        continue;
      }
      if (result <= 0) {
        continue;
      }
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        close_serial();
        continue;
      }
      const ssize_t count = ::read(current_fd, input.data(), input.size());
      if (count > 0) {
        for (ssize_t i = 0; i < count; ++i) {
          process_byte(input[static_cast<std::size_t>(i)]);
        }
      } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        close_serial();
      }
    }
  }

  void process_byte(uint8_t byte)
  {
    switch (state_) {
      case ParseState::kHeader1:
        if (byte == 0xAA) {
          packet_ = {byte};
          state_ = ParseState::kHeader2;
        }
        break;
      case ParseState::kHeader2:
        if (byte == 0x55) {
          packet_.push_back(byte);
          state_ = ParseState::kMeta;
        } else if (byte == 0xAA) {
          packet_ = {byte};
        } else {
          packet_.clear();
          state_ = ParseState::kHeader1;
        }
        break;
      case ParseState::kMeta:
        packet_.push_back(byte);
        if (packet_.size() == 4) {
          const uint8_t sample_count = packet_[3];
          if (sample_count == 0 || sample_count > 120) {
            ++invalid_packets_;
            packet_.clear();
            state_ = ParseState::kHeader1;
          } else {
            target_packet_size_ = 8 + static_cast<std::size_t>(sample_count) * 3;
            state_ = ParseState::kPayload;
          }
        }
        break;
      case ParseState::kPayload:
        packet_.push_back(byte);
        if (packet_.size() == target_packet_size_) {
          parse_packet();
          packet_.clear();
          state_ = ParseState::kHeader1;
        }
        break;
    }
  }

  static uint16_t read_u16(const std::vector<uint8_t> & bytes, std::size_t offset)
  {
    return static_cast<uint16_t>(bytes[offset]) |
           static_cast<uint16_t>(static_cast<uint16_t>(bytes[offset + 1]) << 8);
  }

  void parse_packet()
  {
    const uint8_t count = packet_[3];
    const double start_angle = static_cast<double>(read_u16(packet_, 4) >> 1) / 64.0;
    const double end_angle = static_cast<double>(read_u16(packet_, 6) >> 1) / 64.0;
    double delta = end_angle - start_angle;
    if (delta < 0.0) {
      delta += 360.0;
    }
    for (uint8_t index = 0; index < count; ++index) {
      const std::size_t offset = 8 + static_cast<std::size_t>(index) * 3;
      const double distance_mm = static_cast<double>(read_u16(packet_, offset)) / 4.0;
      if (distance_mm <= 0.0) {
        continue;
      }
      double angle_deg = start_angle;
      if (count > 1) {
        angle_deg += delta * static_cast<double>(index) / static_cast<double>(count - 1);
      }
      const double correction = std::atan(
        21.8 * (155.3 - distance_mm) / (155.3 * distance_mm)) * 180.0 / M_PI;
      angle_deg = std::fmod(angle_deg + correction + 360.0, 360.0);
      const double angle = angle_deg * M_PI / 180.0;
      if (angle < last_angle_ - M_PI) {
        if (have_complete_scan_ && !points_.empty()) {
          publish_scan();
        }
        points_.clear();
        scan_start_ = now();
        have_complete_scan_ = true;
      }
      points_.push_back(Point{angle, distance_mm / 1000.0});
      last_angle_ = angle;
    }
  }

  void publish_scan()
  {
    const rclcpp::Time end = now();
    double scan_time = (end - scan_start_).seconds();
    if (!std::isfinite(scan_time) || scan_time <= 0.0 || scan_time > 1.0) {
      scan_time = 1.0 / 7.0;
    }
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = scan_start_;
    scan.header.frame_id = frame_id_;
    scan.angle_min = 0.0F;
    scan.angle_increment = static_cast<float>(kTwoPi / static_cast<double>(scan_size_));
    scan.angle_max = static_cast<float>(kTwoPi - scan.angle_increment);
    scan.scan_time = static_cast<float>(scan_time);
    scan.time_increment = static_cast<float>(scan_time / static_cast<double>(scan_size_));
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges.assign(static_cast<std::size_t>(scan_size_),
      std::numeric_limits<float>::infinity());
    for (const auto & point : points_) {
      if (point.distance < range_min_ || point.distance > range_max_) {
        continue;
      }
      double angle = inverted_ ? kTwoPi - point.angle : point.angle;
      angle = std::fmod(angle + kTwoPi, kTwoPi);
      const auto bin = static_cast<std::size_t>(angle / scan.angle_increment) %
        static_cast<std::size_t>(scan_size_);
      scan.ranges[bin] = std::min(scan.ranges[bin], static_cast<float>(point.distance));
    }
    scan_pub_->publish(scan);
    ++published_scans_;
    last_scan_time_ns_.store(steady_time_ns());
    last_scan_period_sec_.store(scan_time);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "navigation_robot/lidar";
    status.hardware_id = "usb-34bf:ff0a";
    const int64_t steady_now_ns = steady_time_ns();
    const int64_t last_scan_ns = last_scan_time_ns_.load();
    const bool scan_fresh = last_scan_ns != 0 && steady_now_ns - last_scan_ns < 1000000000LL;
    if (!connected_.load()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "serial disconnected";
    } else if (!scan_fresh) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "no recent complete scan";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "streaming";
    }
    status.values = {
      kv("port", port_name_),
      kv("baud_rate", std::to_string(baud_rate_)),
      kv("published_scans", std::to_string(published_scans_.load())),
      kv("invalid_packets", std::to_string(invalid_packets_.load())),
      kv("write_errors", std::to_string(write_errors_.load())),
      kv("last_scan_period_sec", std::to_string(last_scan_period_sec_.load()))};
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  std::string port_name_;
  int baud_rate_{150000};
  std::string frame_id_;
  std::string scan_topic_;
  double range_min_{0.05};
  double range_max_{8.0};
  bool inverted_{true};
  int scan_size_{720};
  std::chrono::duration<double> reconnect_interval_{1.0};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::thread worker_;
  std::mutex fd_mutex_;
  int fd_{-1};

  ParseState state_{ParseState::kHeader1};
  std::vector<uint8_t> packet_;
  std::size_t target_packet_size_{0};
  std::vector<Point> points_;
  double last_angle_{0.0};
  bool have_complete_scan_{false};
  rclcpp::Time scan_start_{0, 0, RCL_ROS_TIME};
  std::atomic<int64_t> last_scan_time_ns_{0};
  std::atomic<uint64_t> published_scans_{0};
  std::atomic<uint64_t> invalid_packets_{0};
  std::atomic<uint64_t> write_errors_{0};
  std::atomic<double> last_scan_period_sec_{0.0};
};

}  // namespace navigation_robot_lidar_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation_robot_lidar_driver::LidarNode>());
  rclcpp::shutdown();
  return 0;
}
