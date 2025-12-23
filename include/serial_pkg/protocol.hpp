#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

// 1. 帧头帧尾定义
const uint8_t kHeadByte = 0xAA;
const uint8_t kTailByte = 0x55;

// 2. 功能码 ID 定义
enum PacketID : uint8_t {
  kCmdVel = 0x02,  // 发送：控制指令
};

// 3. 基础帧结构
/**
 * @brief 串口通信帧头结构体
 *
 * 定义了数据包的头部信息，包含帧头字节、功能码和数据长度。
 */
struct __attribute__((packed)) FrameHeader {
  uint8_t head;    // 帧头字节 (kHeadByte)
  uint8_t id;      // 功能码 (PacketID)
  uint8_t length;  // 数据长度
};

/**
 * @brief 串口通信帧尾结构体
 *
 * 定义了数据包的尾部信息，包含校验和和帧尾字节。
 */
struct __attribute__((packed)) FrameTail {
  uint8_t checksum;  // 校验和
  uint8_t tail;      // 帧尾字节 (kTailByte)
};

/**
 * @brief 速度控制数据结构体
 *
 * 包含机器人的线速度和角速度信息。
 */
struct __attribute__((packed)) CmdVelData {
  float linear_x;   // 线速度 x (m/s)
  float angular_z;  // 角速度 z (rad/s)
  // float linear_y; // 麦轮解算使用 (预留)
};

// 缓存接收数据结构体
/**
 * @brief通用数据包结构体
 *
 * 用于存储接收到的原始数据包，并提供类型转换辅助函数。
 */
struct Packet {
  PacketID id;                     // 功能码
  std::vector<uint8_t> data_buffer;  // 数据载荷

  /**
   * @brief 将二进制数据转换为具体结构体
   *
   * 将 data_buffer 中的数据拷贝到目标类型的结构体中。
   *
   * @tparam T 目标结构体类型
   * @return T 转换后的结构体实例。如果数据长度不匹配，返回默认构造的实例。
   */
  template <typename T>
  T as() const {
    if (data_buffer.size() != sizeof(T)) {
      return T();
    }
    T t;
    std::memcpy(&t, data_buffer.data(), sizeof(T));
    return t;
  }
};
