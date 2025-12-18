#pragma once
#include "serial_pkg/protocol.hpp"
#include <vector>
#include <cstring>
#include <iostream>

namespace serial_pkg
{

  class PacketHandler
  {
  private:
    std::vector<uint8_t> rx_buffer_;

  public:
    uint8_t calculate_checksum(const uint8_t *data, size_t len)
    {
      uint8_t sum = 0;
      for (size_t i = 0; i < len; i++)
      {
        sum += data[i];
      }
      return sum;
    }

    // 2. 打包函数 (ROS -> MCU)
    template <typename T>
    std::vector<uint8_t> pack(PacketID id, const T &data)
    {
      std::vector<uint8_t> packet;
      // 1. 压入帧头
      packet.push_back(HEAD_BYTE);
      // 2. 压入 ID (强制转换成 uint8_t)
      packet.push_back(static_cast<uint8_t>(id));
      // 3. 压入数据长度
      packet.push_back(sizeof(T));

      // 4. 压入数据
      const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&data);
      for (size_t i = 0; i < sizeof(T); i++)
      {
        packet.push_back(ptr[i]);
      }

      // 5. 压入校验
      // 校验范围：从功能码 (Index 1) 开始，到数据段结束
      uint8_t checksum = calculate_checksum(packet.data() + 1, packet.size() - 1);
      packet.push_back(checksum);

      // 6. 压入帧尾
      packet.push_back(TAIL_BYTE);

      return packet;
    }
    // 3. 接收数据投喂口
    void feed_data(const std::vector<uint8_t> &raw_data)
    {
      rx_buffer_.insert(rx_buffer_.end(), raw_data.begin(), raw_data.end());
    }

    // 通用解析函数，返回 Packet 结构
    bool parse_packet(Packet &out_packet)
    {
      while (rx_buffer_.size() >= sizeof(FrameHeader) + sizeof(FrameTail))
      {
        if (rx_buffer_[0] != HEAD_BYTE)
        {
          rx_buffer_.erase(rx_buffer_.begin());
          continue;
        }

        // FrameHeader: head(0), id(1), length(2)
        uint8_t id_byte = rx_buffer_[1];
        uint8_t data_len = rx_buffer_[2];
        size_t total_len = sizeof(FrameHeader) + data_len + sizeof(FrameTail);

        if (rx_buffer_.size() < total_len)
        {
          return false; // 数据不够，等待下一次
        }

        if (rx_buffer_[total_len - 1] != TAIL_BYTE)
        {
          rx_buffer_.erase(rx_buffer_.begin());
          continue;
        }

        // 检查校验和
        uint8_t calc_sum = calculate_checksum(&rx_buffer_[1], data_len + 2);
        uint8_t recv_sum = rx_buffer_[total_len - 2];

        if (calc_sum != recv_sum)
        {
          rx_buffer_.erase(rx_buffer_.begin());
          continue;
        }

        // 提取数据
        out_packet.id = static_cast<PacketID>(id_byte);
        out_packet.data_buffer.assign(
            rx_buffer_.begin() + sizeof(FrameHeader),
            rx_buffer_.begin() + sizeof(FrameHeader) + data_len);

        // 移除已处理的数据
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_len);
        return true;
      }
      return false;
    }
  };

}
