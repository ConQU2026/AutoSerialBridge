#pragma once

namespace auto_serial_bridge {
namespace config {

    constexpr uint32_t DEFAULT_BAUDRATE = 921600;
    constexpr size_t BUFFER_SIZE = 4096;
    constexpr uint8_t CFG_FRAME_HEADER1 = 90;
    constexpr uint8_t CFG_FRAME_HEADER2 = 165;
    // Checksum Algorithm: CRC8

}
}
