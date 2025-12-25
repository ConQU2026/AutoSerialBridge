#include <gtest/gtest.h>
#include <vector>
#include <cstring>
#include "serial_pkg/packet_handler.hpp"
#include "protocol.h" // For IDs and Hash

using namespace auto_serial_bridge;

namespace {

// 用于测试的 PACKET_ID_CMDVEL 匹配结构定义
struct TestCmdVel {
    float linear_x;
    float angular_z;
};

class PacketHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // ...
    }
};

TEST_F(PacketHandlerTest, FullPackAndParse) {
    PacketHandler handler(1024);
    
    TestCmdVel data_in = {1.5f, -0.5f};
    std::vector<uint8_t> bytes = handler.pack(PACKET_ID_CMDVEL, data_in);
    
    // Header(2) + ID(1) + Len(1) + Payload(8) + CRC(1) = 13 字节
    EXPECT_EQ(bytes.size(), 13);
    EXPECT_EQ(bytes[0], FRAME_HEADER1);
    EXPECT_EQ(bytes[1], FRAME_HEADER2);
    
    handler.feed_data(bytes);
    
    Packet pkt;
    ASSERT_TRUE(handler.parse_packet(pkt));
    EXPECT_EQ(pkt.id, PACKET_ID_CMDVEL);
    EXPECT_EQ(pkt.payload.size(), sizeof(TestCmdVel));
    
    TestCmdVel data_out = pkt.as<TestCmdVel>();
    EXPECT_FLOAT_EQ(data_out.linear_x, 1.5f);
    EXPECT_FLOAT_EQ(data_out.angular_z, -0.5f);
}

TEST_F(PacketHandlerTest, FragmentedData) {
    PacketHandler handler(1024);
    TestCmdVel data_in = {1.0f, 2.0f};
    std::vector<uint8_t> bytes = handler.pack(PACKET_ID_CMDVEL, data_in);
    
    // 投喂前半部分
    size_t split = 5;
    handler.feed_data(bytes.data(), split);
    
    Packet pkt;
    EXPECT_FALSE(handler.parse_packet(pkt));
    
    // 投喂剩余部分
    handler.feed_data(bytes.data() + split, bytes.size() - split);
    
    ASSERT_TRUE(handler.parse_packet(pkt));
    EXPECT_EQ(pkt.id, PACKET_ID_CMDVEL);
}

TEST_F(PacketHandlerTest, StickyPackets) {
    PacketHandler handler(1024);
    TestCmdVel data1 = {1.0f, 1.0f};
    TestCmdVel data2 = {2.0f, 2.0f};
    
    std::vector<uint8_t> bytes1 = handler.pack(PACKET_ID_CMDVEL, data1);
    std::vector<uint8_t> bytes2 = handler.pack(PACKET_ID_CMDVEL, data2);
    
    std::vector<uint8_t> all_bytes = bytes1;
    all_bytes.insert(all_bytes.end(), bytes2.begin(), bytes2.end());
    
    handler.feed_data(all_bytes);
    
    Packet pkt;
    // 第一个包
    ASSERT_TRUE(handler.parse_packet(pkt));
    auto d1 = pkt.as<TestCmdVel>();
    EXPECT_FLOAT_EQ(d1.linear_x, 1.0f);
    
    // 第二个包
    ASSERT_TRUE(handler.parse_packet(pkt));
    auto d2 = pkt.as<TestCmdVel>();
    EXPECT_FLOAT_EQ(d2.linear_x, 2.0f);
    
    // 没有更多了
    EXPECT_FALSE(handler.parse_packet(pkt));
}

TEST_F(PacketHandlerTest, WrapAround) {
    // 小缓冲区以强制回绕
    // 每个包 13 字节。缓冲区 20 字节。
    PacketHandler handler(20); 
    
    TestCmdVel data = {1.0f, 1.0f};
    std::vector<uint8_t> bytes = handler.pack(PACKET_ID_CMDVEL, data); // 13 bytes
    
    // 1. 填充 13 字节
    handler.feed_data(bytes);
    Packet pkt;
    ASSERT_TRUE(handler.parse_packet(pkt)); // 消耗 13 字节。Tail 在 13。
    
    // 2. 再次投喂。13 字节。
    // 缓冲区容量 21 (20+1)。Head 13。
    // 投喂 13 字节: 13->20 (8 字节), 回绕 0->5 (5 字节)。
    handler.feed_data(bytes);
    
    ASSERT_TRUE(handler.parse_packet(pkt));
    TestCmdVel out = pkt.as<TestCmdVel>();
    EXPECT_FLOAT_EQ(out.linear_x, 1.0f);
}

TEST_F(PacketHandlerTest, ChecksumError) {
    PacketHandler handler(1024);
    TestCmdVel data = {1.0f, 1.0f};
    std::vector<uint8_t> bytes = handler.pack(PACKET_ID_CMDVEL, data);
    
    // 损坏数据 (最后一个字节是 CRC)
    bytes.back() ^= 0xFF;
    
    handler.feed_data(bytes);
    Packet pkt;
    EXPECT_FALSE(handler.parse_packet(pkt));
}

TEST_F(PacketHandlerTest, NoiseBeforeHeader) {
    PacketHandler handler(1024);
    TestCmdVel data = {1.0f, 1.0f};
    std::vector<uint8_t> bytes = handler.pack(PACKET_ID_CMDVEL, data);
    
    std::vector<uint8_t> noise = {0x00, 0x11, 0x5A, 0x00, 0xFF}; // 0x5A 但后面不是 0xA5
    
    handler.feed_data(noise);
    handler.feed_data(bytes);
    
    Packet pkt;
    ASSERT_TRUE(handler.parse_packet(pkt)); // 应跳过噪音并找到有效数据包
    EXPECT_EQ(pkt.id, PACKET_ID_CMDVEL);
}

}