#pragma once
#include "serial_pkg/protocol.hpp"
#include <vector>
#include <deque>
#include <cstring>
#include <iostream>

namespace auto_serial_bridge
{

  /**
   * @brief 数据包处理类
   *
   * 负责数据的校验、打包和解包。维护一个接收缓冲区，并处理粘包和断包问题。
   */
  class PacketHandler
  {
  private:
    std::deque<uint8_t> rx_buffer_;

  public:
    /**
     * @brief 计算校验和
     *
     * 计算给定数据段的累加和。
     *
     * @tparam Iterator 迭代器类型
     * @param start 数据起始迭代器
     * @param len 数据长度
     * @return uint8_t 校验和结果
     */
    template <typename Iterator>
    static uint8_t calculate_checksum(Iterator start, size_t len)
    {
      uint8_t sum = 0;
      for (size_t i = 0; i < len; i++)
      {
        sum += *start;
        ++start;
      }
      return sum;
    }

    /**
     * @brief 打包数据 (ROS -> MCU)
     *
     * 将数据结构体打包成串口通信协议格式的字节流。
     *
     * @tparam T 数据结构体类型
     * @param id 功能码
     * @param data 数据对象
     * @return std::vector<uint8_t> 打包后的字节流
     */
    template <typename T>
    std::vector<uint8_t> pack(PacketID id, const T &data) const
    {
      static_assert(sizeof(T) <= 255, "Data size exceeds 255 bytes");
      const size_t packet_size = sizeof(FrameHeader) + sizeof(T) + sizeof(FrameTail);
      std::vector<uint8_t> packet;
      packet.reserve(packet_size);

      // 1. 压入帧头
      packet.push_back(kHeadByte);
      // 2. 压入 ID (强制转换成 uint8_t)
      packet.push_back(static_cast<uint8_t>(id));
      // 3. 压入数据长度
      packet.push_back(sizeof(T));

      // 4. 压入数据 (使用批量插入优化)
      const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&data);
      packet.insert(packet.end(), ptr, ptr + sizeof(T));

      // 5. 压入校验
      // 校验范围：从功能码 (Index 1) 开始，到数据段结束
      uint8_t checksum = calculate_checksum(packet.data() + 1, packet.size() - 1);
      packet.push_back(checksum);

      // 6. 压入帧尾
      packet.push_back(kTailByte);

      return packet;
    }

    /**
     * @brief 接收数据投喂口
     *
     * 将从串口接收到的原始数据放入内部缓冲区。
     *
     * @param raw_data 原始数据字节流
     */
    void feed_data(const std::vector<uint8_t> &raw_data)
    {
      // Bulk insert is efficient for deque (amortized O(1) at both ends)
      rx_buffer_.insert(rx_buffer_.end(), raw_data.begin(), raw_data.end());
    }

    /**
     * @brief 解析数据包
     *
     * 从缓冲区中提取完整的数据包。如果缓冲区中有完整且校验通过的数据包，则返回 true 并填充 out_packet。
     *
     * @param out_packet [out] 解析成功的数据包
     * @return true 解析成功
     * @return false 缓冲区数据不足或未找到有效数据包
     */
    bool parse_packet(Packet &out_packet)
    {
      while (rx_buffer_.size() >= sizeof(FrameHeader) + sizeof(FrameTail))
      {
        if (rx_buffer_[0] != kHeadByte)
        {
          rx_buffer_.pop_front();
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

        if (rx_buffer_[total_len - 1] != kTailByte)
        {
          rx_buffer_.pop_front();
          continue;
        }

        // 检查校验和
        uint8_t calc_sum = calculate_checksum(rx_buffer_.begin() + 1, data_len + 2);
        uint8_t recv_sum = rx_buffer_[total_len - 2];

        if (calc_sum != recv_sum)
        {
          rx_buffer_.pop_front();
          continue;
        }

        // 提取数据 (preserve capacity to avoid reallocation)
        out_packet.id = static_cast<PacketID>(id_byte);
        out_packet.data_buffer.clear();
        // Only reserve if current capacity is insufficient
        if (out_packet.data_buffer.capacity() < data_len) {
          out_packet.data_buffer.reserve(data_len);
        }
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
