#pragma once

#include <cstdint>
#include <vector>
#include <cstring>
#include "protocol.h" // Generated MCU header

namespace auto_serial_bridge {

/**
 * @brief General Packet structure for ROS usage
 */
struct Packet {
  PacketID id;
  std::vector<uint8_t> payload;

  /**
   * @brief Convert payload to specific struct
   */
  template <typename T>
  T as() const {
    if (payload.size() != sizeof(T)) {
      return T();
    }
    T t;
    std::memcpy(&t, payload.data(), sizeof(T));
    return t;
  }
};

} // namespace auto_serial_bridge
