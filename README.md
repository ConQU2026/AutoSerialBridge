## 1. 重构目标 (Goals)
- **极致解耦**: ROS 业务逻辑与底层通信完全分离，`SerialController` 不再包含具体的业务代码，只负责搬运字节。
- **全自动化**: 仅需维护唯一的 `protocol.yaml`，通过脚本自动生成 MCU C 代码 (`protocol.h`) 和 ROS C++ 绑定代码 (`generated_bindings.hpp`)，彻底消除手动同步代码的风险。
- **高性能**: 引入环形缓冲区 (Ring Buffer) 和零拷贝设计，彻底解决高频通信下的内存拷贝和动态分配瓶颈。
- **安全稳定**: 引入协议哈希校验 (Protocol Hash) 和 握手状态机，防止版本不匹配或启动时的无效数据冲击。

## 2. 核心架构设计 (Architecture)

### 2.1 唯一的真理：`protocol.yaml`
所有配置（波特率、缓冲区大小、消息定义、ROS 映射）均在此文件中定义。

#### 核心 YAML 配置示例

```yaml
# 1. 全局配置
config:
  baudrate: 921600
  buffer_size: 4096
  head_byte_1: 0x5A      # 双帧头 1
  head_byte_2: 0xA5      # 双帧头 2
  checksum: "CRC8"       # 校验算法

# 2. 类型映射
type_mappings:
  f32: "float"
  i32: "int32_t"
  u8:  "uint8_t"
  u16: "uint16_t"

# 3. 消息列表 (核心配置)
messages:
  # === 握手与心跳 (系统级) ===
  - name: "Handshake"
    id: 0x00
    direction: "bidirectional"
    ros_msg: "std_msgs/msg/UInt32" # 用于调试或记录
    fields:
      - { proto: "protocol_hash", type: "u32", ros: "data" } # 协议文件的 Hash 值

  - name: "Heartbeat"
    id: 0x01
    direction: "bidirectional"
    ros_msg: "std_msgs/msg/UInt8"
    fields:
      - { proto: "count", type: "u8", ros: "data" }

  # === 业务消息 ===
  # ROS -> MCU: 速度控制
  - name: "CmdVel"
    id: 0x03
    direction: "tx"
    ros_msg: "geometry_msgs/msg/Twist" # 指定 ROS 消息类型
    fields:
      # proto: MCU 结构体成员名 | type: MCU 类型 | ros: ROS 消息成员路径
      - { proto: "linear_x",  type: "f32", ros: "linear.x" }
      - { proto: "angular_z", type: "f32", ros: "angular.z" }

  # MCU -> ROS: 里程计反馈
  - name: "ChassisStatus"
    id: 0x04
    direction: "rx"
    ros_msg: "nav_msgs/msg/Odometry"
    fields:
      - { proto: "pos_x", type: "f32", ros: "pose.pose.position.x" }
      - { proto: "pos_y", type: "f32", ros: "pose.pose.position.y" }

```
### 2.2 代码生成脚本 (`scripts/codegen.py`)

这是构建流程的核心，编译前自动运行。

* **输入**: `protocol.yaml`
* **输出**:
1. **MCU 头文件 (`mcu_output/protocol.h`)**:
* 定义 `enum PacketID`。
* 定义所有 `struct` (使用 `#pragma pack(1)` 对齐)。
* 生成 CRC8 查找表（可选）和协议 Hash 值宏定义。


2. **ROS 绑定头文件 (`include/generated/generated_bindings.hpp`)**:
* 包含所有必要的 ROS 消息头文件。
* 生成 `bind_tx` (ROS->Struct) 和 `bind_rx` (Struct->ROS) 的 Lambda 转换函数。
* 生成 `register_all(SerialController* node)` 函数，实现一键注册。





## 3. 底层性能优化 (Performance)

### 3.1 环形缓冲区 (Ring Buffer)

* **方案**: 废弃 `std::deque`，改用 `std::vector<uint8_t>` 配合 `head_`/`tail_` 指针模拟环形缓冲区。
* **优势**:
* **内存零分配**: 初始化后不再动态申请内存。
* **缓存友好**: 连续内存布局，极大提高 CPU 缓存命中率。



### 3.2 零拷贝写入

* **接口**: 修改 `feed_data` 接口，直接接受原始指针。
```cpp
// 旧: void feed_data(const std::vector<uint8_t> &data); // 发生一次拷贝
// 新: void feed_data(const uint8_t* data, size_t len);   // 直接写入 RingBuffer

```


* **流程**: 驱动读取 -> `feed_data` (一次拷贝进 RingBuffer) -> 解析 (直接在 RingBuffer 上操作或读出 Payload)。

### 3.3 快速帧头搜索

* **逻辑**: 不再单纯查找 `0x5A`，而是使用双字节匹配查找 `[0x5A, 0xA5]` 序列。
* **实现**:
```cpp
// 伪代码思路
while (size >= MIN_PACKET_SIZE) {
    if (buffer[tail] == 0x5A && buffer[(tail+1)%N] == 0xA5) {
        // 命中双帧头，开始解析...
    } else {
        tail = (tail + 1) % N; // 滑动窗口
        size--;
    }
}

```


* **优势**: 大幅减少因数据段中出现 `0x5A` 导致的误判和 CRC 计算浪费。

## 4. 功能实现细节 (Implementation)

### 4.1 协议帧结构

* **Header (2 Bytes)**: `0x5A`, `0xA5` (双字节抗干扰)
* **ID (1 Byte)**: 功能码
* **Length (1 Byte)**: 数据长度
* **Data (N Bytes)**: 载荷
* **CRC (1 Byte)**: CRC8 (查表法，表由脚本生成或静态定义)
* **(去掉帧尾)**: 依靠 Length 和 CRC 即可定界，帧尾非必须。

### 4.2 握手与状态机 (State Machine)

`SerialController` 内部维护简单的状态流转：

1. **State: WAITING_HANDSHAKE**:
* 启动后默认进入此状态。
* **行为**: 周期性发送 `Handshake` 请求 (包含 Hash)。
* **限制**: 此时**丢弃**所有接收到的非握手包，且**不发送**任何控制指令 (CmdVel)。


2. **State: RUNNING**:
* **条件**: 收到 MCU 的 `Handshake` 回应且 Hash 匹配。
* **行为**: 开启正常的 `CmdVel` 转发和 `Odometry` 接收。
* **异常**: 若心跳超时，回退到 `WAITING_HANDSHAKE`。



### 4.3 代码融合步骤

1. **准备环境**:
* 编写 `scripts/codegen.py`。
* 更新 `CMakeLists.txt` 添加 `add_custom_command` 以在编译前运行脚本。


2. **重写底层**:
* 重写 `PacketHandler.hpp` 为 RingBuffer 版本。


3. **精简上层**:
* 清空 `SerialController.cpp` 中手写的绑定代码。
* 在构造函数中调用 `auto_serial_bridge::generated::register_all(this);`。



## 5. 验证计划 (Verification)

1. **生成测试**: 运行 `python3 scripts/codegen.py`，检查生成的 `protocol.h` 和 `generated_bindings.hpp` 语法是否正确。
2. **单元测试**: 针对新的 `PacketHandler` (RingBuffer) 编写 GTest，重点测试：
* 跨越缓冲区末尾的写入/读取 (Wrap around)。
* 粘包、断包处理。
* 错误帧头过滤。


3. **集成测试**: 使用虚拟串口 (socat) 验证 ROS 话题与二进制流的转换是否符合预期，特别是握手流程。
