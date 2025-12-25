
import yaml
import os
import sys
import hashlib
import argparse

def generate_crc8_table():
    """生成CRC8查找表。

    使用多项式0x31计算256个可能值的CRC8校验码。

    Returns:
        包含256个uint8_t值的列表。
    """
    polynomial = 0x31
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ polynomial
            else:
                crc = crc << 1
        table.append(crc & 0xFF)
    return table

def calculate_protocol_hash(yaml_content):
    """计算协议哈希值。

    基于YAML内容计算MD5哈希，用于校验MCU和ROS端的协议一致性。

    Args:
        yaml_content: 协议文件的原始内容字符串。

    Returns:
        32位整数哈希值。
    """
    return int(hashlib.md5(yaml_content.encode('utf-8')).hexdigest()[:8], 16)

def get_c_type(yaml_type, type_mappings):
    """获取对应的C语言类型。

    Args:
        yaml_type: YAML中定义的类型名称。
        type_mappings: 类型映射字典。

    Returns:
        对应的C语言类型字符串，如果没有映射则返回原值。
    """
    if yaml_type in type_mappings:
        return type_mappings[yaml_type]
    return yaml_type  # 兜底返回

def generate_mcu_header(config, messages, type_mappings, protocol_hash, output_path):
    """生成MCU端使用的C语言头文件。

    Args:
        config: 配置字典。
        messages: 消息定义列表。
        type_mappings: 类型映射字典。
        protocol_hash: 协议哈希值。
        output_path: 输出文件路径。
    """
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    with open(output_path, 'w') as f:
        f.write("#pragma once\n")
        f.write("#include <stdint.h>\n")
        f.write("\n")
        f.write("// 协议哈希校验码\n")
        f.write(f"#define PROTOCOL_HASH 0x{protocol_hash:08X}\n")
        f.write("\n")
        
        f.write("// 帧头定义\n")
        f.write(f"#define FRAME_HEADER1 {config['head_byte_1']}\n")
        f.write(f"#define FRAME_HEADER2 {config['head_byte_2']}\n")
        f.write("\n")

        f.write("// 数据包ID定义\n")
        f.write("typedef enum {\n")
        for msg in messages:
            f.write(f"    PACKET_ID_{msg['name'].upper()} = {msg['id']},\n")
        f.write("} PacketID;\n")
        f.write("\n")

        f.write("#pragma pack(1)\n")
        for msg in messages:
            f.write(f"typedef struct {{\n")
            for field in msg['fields']:
                 c_type = get_c_type(field['type'], type_mappings)
                 f.write(f"    {c_type} {field['proto']};\n")
            f.write(f"}} Packet_{msg['name']};\n")
            f.write("\n")
        f.write("#pragma pack()\n")
        f.write("\n")
        
        # 生成CRC8表
        table = generate_crc8_table()
        f.write("// CRC8查找表\n")
        f.write("static const uint8_t CRC8_TABLE[256] = {\n")
        for i in range(0, 256, 16):
            line = ", ".join(f"0x{x:02X}" for x in table[i:i+16])
            f.write(f"    {line},\n")
        f.write("};\n")

def generate_mcu_source(messages, output_path):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    with open(output_path, 'w') as f:
        f.write("#include \"protocol.h\"\n")
        f.write("#include <string.h>\n\n")
        
        # --- 1. 定义解析器状态机 ---
        f.write("""
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
""")

        # --- 2. 生成回调函数原型 ---
        f.write("\n// 用户需要实现的回调函数 (弱定义或外部声明)\n")
        for msg in messages:
            if msg['direction'] == 'tx' or msg['direction'] == 'both':
                # MCU 接收方向 (ROS -> MCU)
                f.write(f"void on_receive_{msg['name']}(const Packet_{msg['name']}* pkt);\n")
        
        # --- 3. 核心状态机函数 ---
        f.write("""
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
""")
        # --- 4. 自动生成分发逻辑 ---
        for msg in messages:
             if msg['direction'] == 'tx' or msg['direction'] == 'both':
                f.write(f"                    case PACKET_ID_{msg['name'].upper()}:\n")
                f.write(f"                        if (rx_data_len == sizeof(Packet_{msg['name']})) {{\n")
                f.write(f"                            on_receive_{msg['name']}((Packet_{msg['name']}*)rx_buffer);\n")
                f.write(f"                        }}\n")
                f.write(f"                        break;\n")

        f.write("""
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
""")

        # --- 5. 生成发送辅助函数 ---
        f.write("\n// --- 发送函数 ---\n")
        f.write("// 外部依赖：用户必须实现 void serial_write_byte(uint8_t byte);\n")
        f.write("extern void serial_write_byte(uint8_t byte);\n\n")
        
        for msg in messages:
            if msg['direction'] == 'rx' or msg['direction'] == 'both':
                # MCU 发送方向 (MCU -> ROS)
                f.write(f"void send_{msg['name']}(const Packet_{msg['name']}* pkt) {{\n")
                f.write(f"    uint8_t header[4] = {{FRAME_HEADER1, FRAME_HEADER2, PACKET_ID_{msg['name'].upper()}, sizeof(Packet_{msg['name']})}};\n")
                f.write(f"    uint8_t crc = 0;\n")
                f.write(f"    // Send Header\n")
                f.write(f"    for(int i=0; i<4; i++) {{ serial_write_byte(header[i]); }}\n")
                f.write(f"    \n")
                f.write(f"    // Calc CRC part 1\n")
                f.write(f"    crc = CRC8_TABLE[crc ^ header[2]]; // ID\n")
                f.write(f"    crc = CRC8_TABLE[crc ^ header[3]]; // Len\n")
                f.write(f"    \n")
                f.write(f"    // Send Data & Calc CRC\n")
                f.write(f"    const uint8_t* data = (const uint8_t*)pkt;\n")
                f.write(f"    for(int i=0; i<sizeof(Packet_{msg['name']}); i++) {{\n")
                f.write(f"        serial_write_byte(data[i]);\n")
                f.write(f"        crc = CRC8_TABLE[crc ^ data[i]];\n")
                f.write(f"    }}\n")
                f.write(f"    \n")
                f.write(f"    // Send CRC\n")
                f.write(f"    serial_write_byte(crc);\n")
                f.write(f"}}\n")

def generate_ros_bindings(messages, type_mappings, output_path):
    """生成ROS端使用的C++绑定代码。

    包含自动订阅和发布逻辑。

    Args:
        messages: 消息定义列表。
        type_mappings: 类型映射字典。
        output_path: 输出文件路径。
    """
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    includes = set()
    for msg in messages:
        includes.add(msg['ros_msg'])
    
    with open(output_path, 'w') as f:
        f.write("#pragma once\n")
        f.write("#include <functional>\n")
        f.write("#include \"serial_pkg/serial_controller.hpp\"\n")
        for inc in includes:
             # inc 例如 "geometry_msgs/msg/Twist"
             parts = inc.split('/')
             pkg = parts[0]
             sub = parts[1] # msg
             typ = parts[2] # Twist
             
             import re
             # 驼峰转蛇形命名
             s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', typ)
             snake_typ = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()
             
             f.write(f"#include <{pkg}/{sub}/{snake_typ}.hpp>\n")
        
        f.write("#include \"../../mcu_output/protocol.h\"\n") # 包含生成的MCU结构体定义
        f.write("\n")
        
        f.write("namespace auto_serial_bridge {\n")
        f.write("namespace generated {\n")
        f.write("\n")
        
        f.write("template <typename T> void register_subscriber(SerialController* node, const std::string& topic, PacketID id);\n")
        f.write("\n")
        
        f.write("inline void register_all(SerialController* node) {\n")
        
        for msg in messages:
            if msg['direction'] == 'tx' or msg['direction'] == 'both':
                # 注册订阅者 (ROS -> MCU)
                # 需要将ROS消息转换为MCU结构体并发送
                # 使用lambda表达式进行转换和发送绑定
                
                # 如果没有明确指定topic，使用默认规则
                # 假设 protocol.yaml 中没有 topic 字段，我们根据消息名推断
                
                # Infer topic: /serial/{name} or /cmd_vel etc.
                # Use lambda to bind.
                
                topic = msg.get('sub_topic')
                if not topic:
                    raise ValueError(f"Message {msg['name']} missing 'sub_topic'")
                
                parts = msg['ros_msg'].split('/')
                ros_type_cpp = f"{parts[0]}::{parts[1]}::{parts[2]}" 
                
                f.write(f"    // {msg['name']} (ROS -> MCU)\n")
                f.write(f"    node->add_subscription(node->create_subscription<{ros_type_cpp}>(\n")
                f.write(f"        \"{topic}\", 10,\n")
                f.write(f"        [node](const {ros_type_cpp}::SharedPtr msg) {{\n")
                f.write(f"            Packet_{msg['name']} pkt;\n")
                # Assignment logic
                for field in msg['fields']:
                    # field['ros'] 例如 "linear.x" -> msg->linear.x
                    ros_acc = field['ros'].replace('.', '.') 
                    f.write(f"            pkt.{field['proto']} = msg->{ros_acc};\n")
                
                f.write(f"            node->send_packet(PACKET_ID_{msg['name'].upper()}, pkt);\n")
                f.write(f"        }}));\n")
                f.write("\n")

        for msg in messages:
             if msg['direction'] == 'rx' or msg['direction'] == 'both':
                # 注册发布者逻辑 (MCU -> ROS)
                # 具体的发布逻辑在 dispatch_packet 中处理，这里暂不需要预注册除了 map 之外的内容
                pass
        
        f.write("}\n\n")

        # 消息分发函数
        f.write("inline void dispatch_packet(SerialController* node, uint8_t id, const std::vector<uint8_t>& data) {\n")
        f.write("    switch(id) {\n")
        for msg in messages:
            if msg['direction'] == 'rx' or msg['direction'] == 'both':
                f.write(f"        case PACKET_ID_{msg['name'].upper()}: {{\n")
                f.write(f"            if (data.size() != sizeof(Packet_{msg['name']})) break;\n")
                f.write(f"            const Packet_{msg['name']}* pkt = reinterpret_cast<const Packet_{msg['name']}*>(data.data());\n")
                
                parts = msg['ros_msg'].split('/')
                ros_type_cpp = f"{parts[0]}::{parts[1]}::{parts[2]}"
                f.write(f"            auto msg = {ros_type_cpp}();\n")
                for field in msg['fields']:
                     # 映射关系: ros = proto
                     # 例如: msg.data = pkt->count
                     ros_acc = field['ros']
                     f.write(f"            msg.{ros_acc} = pkt->{field['proto']};\n")
                
                snake_name = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', msg['name'])
                snake_name = re.sub('([a-z0-9])([A-Z])', r'\1_\2', snake_name).lower()
                topic = msg.get('pub_topic')
                if not topic:
                    raise ValueError(f"Message {msg['name']} missing 'pub_topic'")
                
                # 优化: 实际代码中应预先创建publisher
                # 这里假设节点有动态创建或获取publisher的方法
                # node->publish_message<MsgType>("topic", msg);
                f.write(f"            node->publish_message<{ros_type_cpp}>(\"{topic}\", msg);\n")
                f.write(f"            break;\n")
                f.write(f"        }}\n")
        f.write("    }\n")
        f.write("}\n")

        f.write("}\n") # namespace
        f.write("}\n") # namespace

def main():
    if len(sys.argv) < 3:
        print("Usage: codegen.py <protocol_yaml> <output_dir>")
        sys.exit(1)
        
    yaml_file = sys.argv[1]
    output_dir = sys.argv[2] # 输出目录（基准目录）
    
    # 示例调用: codegen.py config/protocol.yaml .
    # 生成 ./mcu_output/protocol.h 和 ./include/serial_pkg/generated_bindings.hpp
    
    with open(yaml_file, 'r') as f:
        content = f.read()
        config_data = yaml.safe_load(content)
        
    phash = calculate_protocol_hash(content)
    
    generate_mcu_header(config_data['config'], config_data['messages'], config_data['type_mappings'], phash, 
                        os.path.join(output_dir, 'mcu_output', 'protocol.h'))
    
    generate_mcu_source(config_data['messages'], 
                        os.path.join(output_dir, 'mcu_output', 'protocol.c'))
                        
    generate_ros_bindings(config_data['messages'], config_data['type_mappings'], 
                          os.path.join(output_dir, 'include', 'serial_pkg', 'generated_bindings.hpp'))

if __name__ == "__main__":
    main()
