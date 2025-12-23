#include <gtest/gtest.h>
#include "serial_pkg/packet_handler.hpp"

using namespace auto_serial_bridge;

namespace
{
  // --- 1. 定义一个测试用的结构体 ---
  struct TestData
  {
    float x;
    float y;
    int32_t count;
  };

  // --- 2. 测试：完整的打包与解包流程 ---
  TEST(PacketHandlerTest, FullPackAndParse)
  {
    PacketHandler handler;

    // 准备原始数据
    TestData sent_data = {1.23f, -4.56f, 100};
    PacketID id = kCmdVel;

    // 执行打包
    std::vector<uint8_t> buffer = handler.pack(id, sent_data);

    // 检查打包后的长度是否正确
    // Header(3) + Data(12) + Checksum(1) + Tail(1) = 17 bytes
    EXPECT_EQ(buffer.size(), 3 + sizeof(TestData) + 2);

    // 模拟投喂数据
    handler.feed_data(buffer);

    // 执行解析
    Packet out_packet;
    bool success = handler.parse_packet(out_packet);

    // 验证结果
    ASSERT_TRUE(success);
    EXPECT_EQ(out_packet.id, id);

    // 把 buffer 里的字节还原回结构体进行比对
    TestData received_data;
    std::memcpy(&received_data, out_packet.data_buffer.data(), sizeof(TestData));

    EXPECT_FLOAT_EQ(received_data.x, sent_data.x);
    EXPECT_FLOAT_EQ(received_data.y, sent_data.y);
    EXPECT_EQ(received_data.count, sent_data.count);
  }

  // --- 3. 测试：数据包被切断的情况 (碎片化测试) ---
  TEST(PacketHandlerTest, FragmentedData)
  {
    PacketHandler handler;
    TestData data = {1.0f, 2.0f, 3};
    auto buffer = handler.pack(kCmdVel, data);

    // 先投喂前半段数据
    std::vector<uint8_t> part1(buffer.begin(), buffer.begin() + 5);
    handler.feed_data(part1);

    Packet out;
    EXPECT_FALSE(handler.parse_packet(out)); // 此时应该解析失败，因为数据不全

    // 再投喂后半段
    std::vector<uint8_t> part2(buffer.begin() + 5, buffer.end());
    handler.feed_data(part2);

    EXPECT_TRUE(handler.parse_packet(out)); // 此时应该成功
  }

  // --- 4. 测试：校验和错误的情况 ---
  TEST(PacketHandlerTest, ChecksumError)
  {
    PacketHandler handler;
    TestData data = {1.0f, 1.0f, 1};
    auto buffer = handler.pack(kCmdVel, data);

    // 故意破坏数据（改动中间的一个字节）
    buffer[5] ^= 0xFF;

    handler.feed_data(buffer);
    Packet out;
    EXPECT_FALSE(handler.parse_packet(out)); // 校验和不对，应该返回 false
  }
} // namespace