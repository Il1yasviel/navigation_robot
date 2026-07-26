#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "navigation_robot_base_driver/protocol.hpp"

using navigation_robot_base_driver::Frame;
using navigation_robot_base_driver::MessageType;
using navigation_robot_base_driver::StreamParser;
using navigation_robot_base_driver::encode_frame;

TEST(Protocol, EncodesGoldenHello)
{
  Frame frame;
  frame.type = static_cast<uint8_t>(MessageType::kHello);
  frame.sequence = 0x2A;
  const std::vector<uint8_t> expected = {
    0xAA, 0x55, 0x01, 0x01, 0x2A, 0x00, 0x00, 0x00, 0x04, 0xBE};
  EXPECT_EQ(encode_frame(frame), expected);
}

TEST(Protocol, ParsesFragmentedAndConcatenatedFrames)
{
  Frame hello{static_cast<uint8_t>(MessageType::kHello), 1, 0, {}};
  Frame keepalive{static_cast<uint8_t>(MessageType::kDualKeepalive), 2, 0, {1, 2}};
  auto bytes = encode_frame(hello);
  const auto second = encode_frame(keepalive);
  bytes.insert(bytes.end(), second.begin(), second.end());

  StreamParser parser;
  auto frames = parser.feed(bytes.data(), 5);
  EXPECT_TRUE(frames.empty());
  frames = parser.feed(bytes.data() + 5, bytes.size() - 5);
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].type, static_cast<uint8_t>(MessageType::kHello));
  EXPECT_EQ(frames[1].payload, (std::vector<uint8_t>{1, 2}));
}

TEST(Protocol, RejectsBadCrcAndResynchronizes)
{
  Frame hello{static_cast<uint8_t>(MessageType::kHello), 1, 0, {}};
  auto bad = encode_frame(hello);
  bad.back() ^= 0xFF;
  const auto good = encode_frame(hello);
  bad.insert(bad.end(), good.begin(), good.end());

  StreamParser parser;
  const auto frames = parser.feed(bad.data(), bad.size());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(parser.crc_errors(), 1U);
}
