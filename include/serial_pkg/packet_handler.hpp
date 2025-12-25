#pragma once

#include "serial_pkg/protocol.hpp"
#include <vector>
#include <iostream>
#include <algorithm>

namespace auto_serial_bridge
{

  /**
   * @brief 数据包处理类
   *
   * 负责数据的校验、打包和解包。使用环形缓冲区实现零拷贝和高性能解析。
   */
  class PacketHandler
  {
  private:
    std::vector<uint8_t> ring_buffer_;
    size_t head_ = 0; // Write index
    size_t tail_ = 0; // Read index
    size_t capacity_;
    
    // 最小包长: Header(2) + ID(1) + Len(1) + CRC(1) = 5 字节 (假设载荷为0)
    static constexpr size_t MIN_PACKET_SIZE = 5; 

  public:
    explicit PacketHandler(size_t buffer_size) : capacity_(buffer_size + 1)
    {
       ring_buffer_.resize(capacity_);
    }

    /**
     * @brief 计算校验和 (查表法)
     */
    static uint8_t calculate_checksum(const uint8_t* data, size_t len)
    {
      uint8_t crc = 0;
      for (size_t i = 0; i < len; ++i)
      {
        crc = CRC8_TABLE[crc ^ data[i]];
      }
      return crc;
    }

    /**
     * @brief 打包数据 (ROS -> MCU)
     */
    template <typename T>
    std::vector<uint8_t> pack(PacketID id, const T &data) const
    {
      static_assert(sizeof(T) <= 255, "数据大小不能超过255字节");
      // 双帧头 + ID + 长度 + 数据 + CRC = 2 + 1 + 1 + N + 1 = 5 + N
      const size_t packet_size = 5 + sizeof(T);
      std::vector<uint8_t> packet;
      packet.reserve(packet_size);

      packet.push_back(FRAME_HEADER1);
      packet.push_back(FRAME_HEADER2);
      packet.push_back(static_cast<uint8_t>(id));
      packet.push_back(static_cast<uint8_t>(sizeof(T)));
      
      const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&data);
      packet.insert(packet.end(), ptr, ptr + sizeof(T));
      
      // 校验和覆盖范围: ID, 长度, 数据
      // packet[2] 是 ID.
      uint8_t checksum = calculate_checksum(packet.data() + 2, packet.size() - 2);
      packet.push_back(checksum);

      return packet;
    }

    /**
     * @brief 接收数据投喂口
     */
    void feed_data(const uint8_t* data, size_t len)
    {
        // 简单实现：循环写入
        for (size_t i = 0; i < len; ++i) {
            size_t next_head = (head_ + 1) % capacity_;
            if (next_head != tail_) { // 未满
                ring_buffer_[head_] = data[i];
                head_ = next_head;
            } else {
                // 缓冲区溢出，丢弃数据或处理错误
                // 目前简单丢弃新数据
                break; 
            }
        }
    }
    
    // std::vector 的兼容包装器
    void feed_data(const std::vector<uint8_t>& data) {
        feed_data(data.data(), data.size());
    }

    /**
     * @brief 解析数据包
     */
    bool parse_packet(Packet &out_packet)
    {
        while (data_available() >= MIN_PACKET_SIZE) 
        {
            // 1. 快速搜寻帧头: 寻找 0x5A, 0xA5
            // 检查 tail 和 (tail+1)%cap
            uint8_t b1 = ring_buffer_[tail_];
            uint8_t b2 = ring_buffer_[(tail_ + 1) % capacity_];
            
            if (b1 != FRAME_HEADER1 || b2 != FRAME_HEADER2) {
                // 不是帧头，滑动窗口
                tail_ = (tail_ + 1) % capacity_;
                continue;
            }
            
            // 发现帧头在 tail_, tail_+1
            // 需要检查是否有足够的数据获取长度信息
            // Header(2) + ID(1) + Len(1) = 需要4个字节来知道长度
            
            if (data_available() < 4) return false; // 等待更多数据
            
            uint8_t id_byte = ring_buffer_[(tail_ + 2) % capacity_];
            uint8_t len_byte = ring_buffer_[(tail_ + 3) % capacity_];
            
            size_t total_len = 2 + 1 + 1 + len_byte + 1; // Header(2) + ID(1) + Len(1) + Payload(N) + CRC(1)
            
            if (data_available() < total_len) return false; // 等待完整数据包
            
            // 检查 CRC
            // 我们需要对 ID, Len, Payload 进行 CRC校验。
            // ID 在偏移 2, Len 在 3, Payload 在 4...
            // 直接在环形缓冲区上计算更好，但需要处理回绕。
            
            uint8_t calc_crc = 0;
            // 迭代范围: ID (index 2) 到 payload 结束
            // 范围: ID, Len, Payload. 
            // 起始索引: (tail_ + 2) % capacity_
            // 计数: 1 + 1 + len_byte = 2 + len_byte
            
            size_t crc_start_idx = (tail_ + 2) % capacity_;
            size_t crc_count = 2 + len_byte;
            
            for (size_t i = 0; i < crc_count; ++i) {
                uint8_t byte = ring_buffer_[(crc_start_idx + i) % capacity_];
                calc_crc = CRC8_TABLE[calc_crc ^ byte];
            }
            
            uint8_t recv_crc = ring_buffer_[(tail_ + total_len - 1) % capacity_];
            
            if (calc_crc == recv_crc) {
                 // 有效数据包
                 out_packet.id = static_cast<PacketID>(id_byte);
                 out_packet.payload.resize(len_byte);
                 
                 // 复制载荷
                 size_t payload_start = (tail_ + 4) % capacity_;
                 for (size_t i = 0; i < len_byte; ++i) {
                     out_packet.payload[i] = ring_buffer_[(payload_start + i) % capacity_];
                 }
                 
                 // 消耗数据包
                 tail_ = (tail_ + total_len) % capacity_;
                 return true;
            } else {
                 // CRC 无效，仅丢弃帧头的第一个字节以重新搜索 (也许 0xA5 是下一个头的开始?)
                 // 实际上严格来说如果我们匹配了 0x5A 0xA5 但CRC失败，这可能是垃圾数据。
                 // 但安全起见滑动 1 字节。
                 tail_ = (tail_ + 1) % capacity_;
            }
            
        }
        return false;
    }
    
    size_t data_available() const {
        if (head_ >= tail_) return head_ - tail_;
        return capacity_ - tail_ + head_;
    }
    
    
  };

}
