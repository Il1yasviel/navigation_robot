#ifndef NAVIGATION_ROBOT_BASE_DRIVER__PROTOCOL_HPP_
#define NAVIGATION_ROBOT_BASE_DRIVER__PROTOCOL_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace navigation_robot_base_driver
{

constexpr uint8_t kProtocolVersion = 0x01;
constexpr uint8_t kAckRequired = 0x01;
constexpr std::size_t kMaxPayload = 128;

enum class MessageType : uint8_t
{
  kHello = 0x01,
  kQuery = 0x13,
  kSetMode = 0x16,
  kSetDualRpm = 0x1A,
  kDualKeepalive = 0x1B,
  kStopDual = 0x1C,
  kAck = 0x80,
  kChassisTelemetry = 0x92,
  kImuTelemetry = 0x93,
};

struct Frame
{
  uint8_t type{0};
  uint8_t sequence{0};
  uint8_t flags{0};
  std::vector<uint8_t> payload;
};

uint16_t crc16_ccitt_false(const uint8_t * data, std::size_t size);
std::vector<uint8_t> encode_frame(const Frame & frame);

uint16_t read_u16_le(const uint8_t * data);
int16_t read_i16_le(const uint8_t * data);
uint32_t read_u32_le(const uint8_t * data);
uint64_t read_u64_le(const uint8_t * data);
float read_f32_le(const uint8_t * data);
void append_u16_le(std::vector<uint8_t> & output, uint16_t value);
void append_i16_le(std::vector<uint8_t> & output, int16_t value);

class StreamParser
{
public:
  std::vector<Frame> feed(const uint8_t * data, std::size_t size);
  void reset();

  uint64_t crc_errors() const {return crc_errors_.load();}
  uint64_t format_errors() const {return format_errors_.load();}

private:
  std::vector<uint8_t> buffer_;
  std::atomic<uint64_t> crc_errors_{0};
  std::atomic<uint64_t> format_errors_{0};
};

}  // namespace navigation_robot_base_driver

#endif  // NAVIGATION_ROBOT_BASE_DRIVER__PROTOCOL_HPP_
