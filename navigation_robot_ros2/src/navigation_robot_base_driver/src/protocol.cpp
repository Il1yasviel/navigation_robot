#include "navigation_robot_base_driver/protocol.hpp"

#include <algorithm>
#include <cstring>

namespace navigation_robot_base_driver
{

uint16_t crc16_ccitt_false(const uint8_t * data, std::size_t size)
{
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) != 0 ? static_cast<uint16_t>((crc << 1) ^ 0x1021) :
        static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void append_u16_le(std::vector<uint8_t> & output, uint16_t value)
{
  output.push_back(static_cast<uint8_t>(value & 0xFF));
  output.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void append_i16_le(std::vector<uint8_t> & output, int16_t value)
{
  append_u16_le(output, static_cast<uint16_t>(value));
}

uint16_t read_u16_le(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
}

int16_t read_i16_le(const uint8_t * data)
{
  return static_cast<int16_t>(read_u16_le(data));
}

uint32_t read_u32_le(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

uint64_t read_u64_le(const uint8_t * data)
{
  return static_cast<uint64_t>(read_u32_le(data)) |
         (static_cast<uint64_t>(read_u32_le(data + 4)) << 32);
}

float read_f32_le(const uint8_t * data)
{
  const uint32_t raw = read_u32_le(data);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

std::vector<uint8_t> encode_frame(const Frame & frame)
{
  if (frame.payload.size() > kMaxPayload) {
    return {};
  }
  std::vector<uint8_t> output;
  output.reserve(10 + frame.payload.size());
  output.push_back(0xAA);
  output.push_back(0x55);
  output.push_back(kProtocolVersion);
  output.push_back(frame.type);
  output.push_back(frame.sequence);
  output.push_back(frame.flags);
  append_u16_le(output, static_cast<uint16_t>(frame.payload.size()));
  output.insert(output.end(), frame.payload.begin(), frame.payload.end());
  const uint16_t crc = crc16_ccitt_false(output.data() + 2, output.size() - 2);
  append_u16_le(output, crc);
  return output;
}

std::vector<Frame> StreamParser::feed(const uint8_t * data, std::size_t size)
{
  buffer_.insert(buffer_.end(), data, data + size);
  std::vector<Frame> frames;

  while (true) {
    static constexpr uint8_t kHeader[] = {0xAA, 0x55};
    const auto header = std::search(
      buffer_.begin(), buffer_.end(), std::begin(kHeader), std::end(kHeader));
    if (header == buffer_.end()) {
      const bool keep_aa = !buffer_.empty() && buffer_.back() == 0xAA;
      buffer_.clear();
      if (keep_aa) {
        buffer_.push_back(0xAA);
      }
      break;
    }
    buffer_.erase(buffer_.begin(), header);
    if (buffer_.size() < 8) {
      break;
    }
    if (buffer_[2] != kProtocolVersion) {
      ++format_errors_;
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t payload_size = read_u16_le(buffer_.data() + 6);
    if (payload_size > kMaxPayload) {
      ++format_errors_;
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t total_size = 10 + payload_size;
    if (buffer_.size() < total_size) {
      break;
    }
    const uint16_t expected = read_u16_le(buffer_.data() + 8 + payload_size);
    const uint16_t actual = crc16_ccitt_false(buffer_.data() + 2, 6 + payload_size);
    if (expected != actual) {
      ++crc_errors_;
      buffer_.erase(buffer_.begin());
      continue;
    }
    Frame frame;
    frame.type = buffer_[3];
    frame.sequence = buffer_[4];
    frame.flags = buffer_[5];
    frame.payload.assign(buffer_.begin() + 8, buffer_.begin() + 8 + payload_size);
    frames.push_back(std::move(frame));
    buffer_.erase(buffer_.begin(), buffer_.begin() + total_size);
  }
  return frames;
}

void StreamParser::reset()
{
  buffer_.clear();
}

}  // namespace navigation_robot_base_driver
