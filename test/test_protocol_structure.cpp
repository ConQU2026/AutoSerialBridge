#include <gtest/gtest.h>
#include "protocol.h"

// 验证生成的 C 头文件是否有效并符合预期

TEST(ProtocolStructureTest, StructPacking) {
    // Handshake: u32 (4 字节)
    EXPECT_EQ(sizeof(Packet_Handshake), 4);
    
    // Heartbeat: u8 (1 字节)
    EXPECT_EQ(sizeof(Packet_Heartbeat), 1);
    
    // CmdVel: f32 + f32 (8 字节)
    EXPECT_EQ(sizeof(Packet_CmdVel), 8);
    
    // ChassisStatus: f32 + f32 (8 字节)
    EXPECT_EQ(sizeof(Packet_ChassisStatus), 8);
}

TEST(ProtocolStructureTest, Constants) {
    // 帧头
    EXPECT_EQ(FRAME_HEADER1, 0x5A);
    EXPECT_EQ(FRAME_HEADER2, 0xA5);
    
    // 哈希应该已定义
    EXPECT_NE(PROTOCOL_HASH, 0);
}

TEST(ProtocolStructureTest, CRCTableCheck) {
    // 抽查标准 CRC8-MAXIM 多项式 (0x31) 的几个值
    // 0 -> 0x00
    EXPECT_EQ(CRC8_TABLE[0], 0x00);
    // 1 -> 0x31
    EXPECT_EQ(CRC8_TABLE[1], 0x31);
    // 255 -> 0xAC
    EXPECT_EQ(CRC8_TABLE[255], 0xAC); 
}
