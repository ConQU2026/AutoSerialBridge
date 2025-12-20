#pragma once
#include <cstdint>
#include <vector>
#include <cstring>

// 1. 帧头帧尾定义
const uint8_t HEAD_BYTE = 0xAA;
const uint8_t TAIL_BYTE = 0x55;

// 2. 功能码 ID 定义
enum PacketID : uint8_t
{
  ID_CMD_VEL = 0x02, // 发送：控制指令
};

// 3. 基础帧结构
struct __attribute__((packed)) FrameHeader
{
  uint8_t head;
  uint8_t id;
  uint8_t length; // 数据长度
};

struct __attribute__((packed)) FrameTail
{
  uint8_t checksum;
  uint8_t tail;
};

//
struct __attribute__((packed)) CmdVelData
{
  float linear_x;
  float angular_z;
  // float linear_y; // 麦轮解算使用
};

// 缓存接收数据结构体
struct Packet
{
  PacketID id;
  std::vector<uint8_t> data_buffer; 

  // 辅助函数：将二进制数据转换为具体结构体
  template <typename T>
  T as() const
  {
    if (data_buffer.size() != sizeof(T))
    {
      return T();
    }
    T t;
    std::memcpy(&t, data_buffer.data(), sizeof(T));
    return t;
  }
};
