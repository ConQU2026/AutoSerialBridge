#pragma once

namespace auto_serial_bridge {
namespace config {

    constexpr uint32_t DEFAULT_BAUDRATE = 115200;
    constexpr size_t BUFFER_SIZE = 256;
    constexpr uint8_t CFG_FRAME_HEADER1 = 90;
    constexpr uint8_t CFG_FRAME_HEADER2 = 165;
    // Checksum Algorithm: CRC8

}
}
