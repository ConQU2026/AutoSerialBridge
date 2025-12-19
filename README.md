# Serial Package

## 项目说明

这是一个通用的 ROS 2 串口通信包，旨在解决以下问题：
- **解耦**: 视觉组与电控组解耦和
- **可拓展性强**: 提供简单的接口来绑定 ROS 话题和串口数据包, 后续增减传输数据非常简单
- **高效**: 基于 `serial_driver` 和 `asio` 实现异步 I/O。


## 功能特性

- **通用协议定义**: 在 `protocol.hpp` 中集中管理通信协议（帧头、帧尾、数据结构）。
- **自动封包/解包**: `PacketHandler` 自动处理帧头、帧尾和校验和。
- **双向绑定**:
  - `bind_topic_to_serial`: 订阅 ROS 话题 -> 自动打包 -> 发送串口数据。
  - `bind_serial_to_topic`: 接收串口数据 -> 自动解包 -> 发布 ROS 话题。
- **参数配置**: 支持通过 ROS 参数配置串口端口和波特率。

## 快速开始

### 1. 定义协议 (`include/serial_pkg/protocol.hpp`)

在 `protocol.hpp` 中定义你的数据帧格式、ID 和数据结构。

```cpp
// 1. 帧头帧尾定义
const uint8_t HEAD_BYTE = 0xAA;
const uint8_t TAIL_BYTE = 0x55;

// 2. 功能码 ID 定义
enum PacketID : uint8_t
{
  ID_CMD_VEL = 0x01, // 示例：控制指令
  ID_IMU_DATA = 0x02 // 示例：IMU 数据
};

// 3. 数据结构定义 (必须是 packed)
struct __attribute__((packed)) CmdVelData
{
  float linear_x;
  float angular_z;
};
```

### 2. 绑定话题 (`src/serial_controller.cpp`)

在 `SerialController` 的 `register_tx_handlers` (发送) 和 `register_rx_handlers` (接收) 函数中进行绑定。

**发送 (ROS -> Serial):**

```cpp
void SerialController::register_tx_handlers()
{
  // 绑定 cmd_vel 话题到 ID_CMD_VEL 数据包
  bind_topic_to_serial<geometry_msgs::msg::Twist, CmdVelData>(
      "cmd_vel",
      ID_CMD_VEL,
      [](const geometry_msgs::msg::Twist &msg) {
        CmdVelData data;
        data.linear_x = static_cast<float>(msg.linear.x);
        data.angular_z = static_cast<float>(msg.angular.z);
        return data;
      });
}
```

**接收 (Serial -> ROS):**

```cpp
void SerialController::register_rx_handlers()
{
  // 绑定 ID_CMD_VEL 数据包到 cmd_vel_feedback 话题
  bind_serial_to_topic<geometry_msgs::msg::Twist, CmdVelData>(
      "cmd_vel_feedback",
      ID_CMD_VEL,
      [](const CmdVelData &data) {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = data.linear_x;
        msg.angular.z = data.angular_z;
        return msg;
      });
}
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `port` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `baudrate` | int | `115200` | 波特率 |
| `timeout` | double | `0.1` | 超时时间 (秒) |
| `serial_frequency` | double | `100.0` | 串口频率 (Hz) |

## 代码结构

- **`include/serial_pkg/protocol.hpp`**: 协议定义（ID、结构体）。
- **`include/serial_pkg/serial_controller.hpp`**: 节点类定义，包含绑定模板函数。
- **`include/serial_pkg/packet_handler.hpp`**: 封包与解包逻辑。
- **`src/serial_controller.cpp`**: 节点实现，在此处注册话题绑定。
- **`src/serial_node.cpp`**: 程序入口。

## TEST

```bash
colcon test --event-handlers console_cohesion+ --packages-select serial_pkg 
```

## TODO

- [ ] 自动生成电控对应 C 代码模块。
- [ ] udev script 自动配置串口权限。
- [ ] 支持自定义接口

