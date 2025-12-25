# Auto Serial Bridge

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)
![License](https://img.shields.io/badge/License-Apache%202.0-green)

## 项目简介

**Auto Serial Bridge** 是一个致力于提升 ROS2 与嵌入式设备之间串口通信开发效率的工具。

鉴于 microROS 在某些场景下的配置繁琐与不稳定，本项目旨在提供一个**轻量级、自动化、可配置**的替代方案。只需在 `protocol.yaml` 配置文件中定义通信协议，即可自动生成 ROS2 端的 C++ 解析代码以及嵌入式端的 C 代码。

**核心优势：**
*   **高效开发**：无需手动编写繁琐的序列化/反序列化代码。
*   **配置即代码**：通过 YAML 文件集中管理协议，修改协议只需改配置并重新编译（无需花时间在电控组和视觉组之间调试通信）。
*   **无缝对接**：自动生成 ROS2 消息与嵌入式结构体，消除协议不一致带来的隐患。
*   **高性能**：基于 C++ 实现的高效串口通信与数据解包。


## 项目结构

```text
auto_serial_bridge/
├── config/              # 配置文件目录 (protocol.yaml)
├── include/             # 头文件 (含自动生成的协议头文件)
├── launch/              # ROS2 Launch 启动文件
├── mcu_output/          # 自动生成的 MCU 端 C 代码 (直接拷贝给电控组)
├── scripts/             # 代码生成与辅助脚本
├── src/                 # 核心源代码
└── test/                # 单元测试
```

## 快速开始

### 1. 环境依赖
*   Ubuntu 22.04
*   ROS2 Humble
*   Python 3

```bash
sudo apt install ros-humble-serial-driver libasio-dev ros-humble-rclcpp-components ros-humble-io-context python3-serial socat
```

### 2. 编译项目

在你的 ROS2 工作空间根目录下：

```bash
colcon build --packages-select auto_serial_bridge
source install/setup.bash
```

### 3. 配置串口权限

使用本项目提供的脚本自动设置 udev 规则（只需运行一次）：

```bash
cd src/auto_serial_bridge/scripts
sudo ./auto_udev.sh
# 重新插拔串口设备后生效
```

### 4. 运行节点

```bash
ros2 launch auto_serial_bridge launch_serial.py
```

## 配置说明 (protocol.yaml)

核心配置文件位于 `config/protocol.yaml`。可通过修改此文件来增删改数据协议，**修改后重新编译即可生效**。

### 全局配置
```yaml
config:
  baudrate: 921600       # 串口波特率
  buffer_size: 4096      # 缓冲区大小
  head_byte_1: 0x5A      # 帧头1
  head_byte_2: 0xA5      # 帧头2
  checksum: "CRC8"       # 校验算法
```

### 消息定义
可以在 `messages` 列表中定义多个消息类型：

```yaml
messages:
  - name: "CmdVel"                 # 消息名称（生成类名）
    id: 0x04                       # 消息ID (唯一)
    direction: "tx"                # 传输方向: tx (ROS->MCU), rx (MCU->ROS), both(双向)
    sub_topic: "/cmd_vel"          # 订阅话题 (仅 tx/both 有效)
    ros_msg: "geometry_msgs/msg/Twist" # 对应的 ROS 消息类型
    fields:
      # proto: 协议字段名 | type: 数据类型 | ros: ROS消息字段映射路径
      - { proto: "linear_x",  type: "f32", ros: "linear.x" }
      - { proto: "angular_z", type: "f32", ros: "angular.z" }
```

## MCU 端集成

当你在 ROS2 端运行 `colcon build` 后，`mcu_output` 目录下会自动更新生成的 C 代码：
*   `protocol.c`: 协议打包与解包实现。
*   `protocol.h`: 协议结构体与函数声明。

**电控开发人员集成步骤：**
1.  将 `mcu_output` 文件夹内容复制到单片机工程中。
2.  实现 `serial_write` 等底层发送接口。
3.  调用 `protocol.h` 中的 `pack_*` 和 `unpack_*` 接口进行数据收发。

## 测试

运行单元测试：
```bash
colcon test --packages-select auto_serial_bridge --event-handlers console_direct+
```

## 更新日志

*   **v1.0**: 实现了基于 `protocol.yaml` 的全自动代码生成。
    *   仅需在 `protocol.yaml` 中添加需要的协议, 即可自动生成对应的 ROS2 和电控的代码。
*   **Beta**: 初始验证版本，需在代码中手动添加相应的数据解析逻辑。