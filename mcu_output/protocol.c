#include "protocol.h"
#include <string.h>


// 解析器状态定义
typedef enum {
    STATE_WAIT_HEADER1,
    STATE_WAIT_HEADER2,
    STATE_WAIT_ID,
    STATE_WAIT_LEN,
    STATE_WAIT_DATA,
    STATE_WAIT_CRC
} State;

static State rx_state = STATE_WAIT_HEADER1;
static uint8_t rx_buffer[256]; // 最大包长
static uint8_t rx_cnt = 0;
static uint8_t rx_data_len = 0;
static uint8_t rx_id = 0;
static uint8_t rx_crc = 0;

// CRC8 计算函数 (查表法)
uint8_t calculate_crc8(const uint8_t* data, uint8_t len, uint8_t initial_crc) {
    uint8_t crc = initial_crc;
    for (uint8_t i = 0; i < len; i++) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    return crc;
}

// 用户需要实现的回调函数 (弱定义或外部声明)
void on_receive_Handshake(const Packet_Handshake* pkt);
void on_receive_Heartbeat(const Packet_Heartbeat* pkt);
void on_receive_CmdVel(const Packet_CmdVel* pkt);

/**
 * @brief 协议解析状态机，在串口中断或轮询中调用此函数处理每个接收到的字节
 * @param byte 接收到的单个字节
 */
void protocol_fsm_feed(uint8_t byte) {
    switch (rx_state) {
        case STATE_WAIT_HEADER1:
            if (byte == FRAME_HEADER1) {
                rx_state = STATE_WAIT_HEADER2;
                rx_crc = 0; // CRC 重置 (假设CRC不包含Header1，根据你的定义调整)
            }
            break;
            
        case STATE_WAIT_HEADER2:
            if (byte == FRAME_HEADER2) {
                rx_state = STATE_WAIT_ID;
            } else {
                rx_state = STATE_WAIT_HEADER1; // 重置
            }
            break;
            
        case STATE_WAIT_ID:
            rx_id = byte;
            rx_crc = CRC8_TABLE[0 ^ rx_id]; // 开始计算 CRC (假设包含 ID)
            rx_state = STATE_WAIT_LEN;
            break;
            
        case STATE_WAIT_LEN:
            rx_data_len = byte;
            rx_crc = CRC8_TABLE[rx_crc ^ rx_data_len]; // CRC 包含 Len
            rx_cnt = 0;
            if (rx_data_len > 0) {
                rx_state = STATE_WAIT_DATA;
            } else {
                rx_state = STATE_WAIT_CRC; // 数据长度为0的情况
            }
            break;
            
        case STATE_WAIT_DATA:
            rx_buffer[rx_cnt++] = byte;
            rx_crc = CRC8_TABLE[rx_crc ^ byte]; // CRC 包含 Data
            if (rx_cnt >= rx_data_len) {
                rx_state = STATE_WAIT_CRC;
            }
            break;
            
        case STATE_WAIT_CRC:
            if (byte == rx_crc) {
                // 校验通过，分发数据
                switch (rx_id) {
                    case PACKET_ID_HANDSHAKE:
                        if (rx_data_len == sizeof(Packet_Handshake)) {
                            on_receive_Handshake((Packet_Handshake*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_HEARTBEAT:
                        if (rx_data_len == sizeof(Packet_Heartbeat)) {
                            on_receive_Heartbeat((Packet_Heartbeat*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_CMDVEL:
                        if (rx_data_len == sizeof(Packet_CmdVel)) {
                            on_receive_CmdVel((Packet_CmdVel*)rx_buffer);
                        }
                        break;

                    default:
                        break;
                }
            }
            // 无论校验成功与否，都重置状态
            rx_state = STATE_WAIT_HEADER1;
            break;
            
        default:
            rx_state = STATE_WAIT_HEADER1;
            break;
    }
}

// --- 发送函数 ---
// 外部依赖：用户必须实现 void serial_write_byte(uint8_t byte);
extern void serial_write_byte(uint8_t byte);

void send_Handshake(const Packet_Handshake* pkt) {
    uint8_t header[4] = {FRAME_HEADER1, FRAME_HEADER2, PACKET_ID_HANDSHAKE, sizeof(Packet_Handshake)};
    uint8_t crc = 0;
    // Send Header
    for(int i=0; i<4; i++) { serial_write_byte(header[i]); }
    
    // Calc CRC part 1
    crc = CRC8_TABLE[crc ^ header[2]]; // ID
    crc = CRC8_TABLE[crc ^ header[3]]; // Len
    
    // Send Data & Calc CRC
    const uint8_t* data = (const uint8_t*)pkt;
    for(int i=0; i<sizeof(Packet_Handshake); i++) {
        serial_write_byte(data[i]);
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    
    // Send CRC
    serial_write_byte(crc);
}
void send_Heartbeat(const Packet_Heartbeat* pkt) {
    uint8_t header[4] = {FRAME_HEADER1, FRAME_HEADER2, PACKET_ID_HEARTBEAT, sizeof(Packet_Heartbeat)};
    uint8_t crc = 0;
    // Send Header
    for(int i=0; i<4; i++) { serial_write_byte(header[i]); }
    
    // Calc CRC part 1
    crc = CRC8_TABLE[crc ^ header[2]]; // ID
    crc = CRC8_TABLE[crc ^ header[3]]; // Len
    
    // Send Data & Calc CRC
    const uint8_t* data = (const uint8_t*)pkt;
    for(int i=0; i<sizeof(Packet_Heartbeat); i++) {
        serial_write_byte(data[i]);
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    
    // Send CRC
    serial_write_byte(crc);
}
void send_ChassisStatus(const Packet_ChassisStatus* pkt) {
    uint8_t header[4] = {FRAME_HEADER1, FRAME_HEADER2, PACKET_ID_CHASSISSTATUS, sizeof(Packet_ChassisStatus)};
    uint8_t crc = 0;
    // Send Header
    for(int i=0; i<4; i++) { serial_write_byte(header[i]); }
    
    // Calc CRC part 1
    crc = CRC8_TABLE[crc ^ header[2]]; // ID
    crc = CRC8_TABLE[crc ^ header[3]]; // Len
    
    // Send Data & Calc CRC
    const uint8_t* data = (const uint8_t*)pkt;
    for(int i=0; i<sizeof(Packet_ChassisStatus); i++) {
        serial_write_byte(data[i]);
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    
    // Send CRC
    serial_write_byte(crc);
}
