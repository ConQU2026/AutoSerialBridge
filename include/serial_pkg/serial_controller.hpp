#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <mutex>
#include <atomic>

#include "rcutils/logging.h"
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "io_context/io_context.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "serial_pkg/packet_handler.hpp"
#include "serial_pkg/protocol.hpp"

namespace auto_serial_bridge
{
  /**
   * @brief 串口控制节点
   *
   * 负责与下位机进行串口通信，实现 ROS Topic 和串口数据包之间的双向转发。
   * 包含自动重连、数据校验和分发机制。
   */
  class SerialController : public rclcpp::Node
  {
  public:
    /**
     * @brief 构造函数：初始化节点并设置串口
     *
     * @param options 节点选项
     */
    explicit SerialController(const rclcpp::NodeOptions &options);

    /**
     * @brief 析构函数：确保程序关闭时，串口也能优雅地关闭
     */
    ~SerialController() override;

  private:
    /**
     * @brief 获取参数
     *
     * 从参数服务器加载串口配置（端口号、波特率等）。
     */
    void get_parameters();

    /**
     * @brief 开始异步接收
     *
     * 启动串口的异步读取操作。
     */
    void start_receive();

    /**
     * @brief 异步发送数据
     *
     * @param packet_bytes 要发送的数据字节流
     */
    void async_send(const std::vector<uint8_t> &packet_bytes);

    // 连接状态与重连相关
    /**
     * @brief 检查连接状态
     *
     * 定时器回调函数，用于监控串口连接状态并尝试重连。
     */
    void check_connection(); 

    /**
     * @brief 重置串口
     *
     * 关闭串口并清理相关资源。
     */
    void reset_serial();     

    /**
     * @brief 尝试打开串口
     *
     * @return true 打开成功
     * @return false 打开失败
     */
    bool try_open_serial();  

    // 注册函数
    /**
     * @brief 注册接收处理函数
     *
     * 绑定具体的协议 ID 到 ROS Topic 发布者。
     */
    void register_rx_handlers();

    /**
     * @brief 注册发送处理函数
     *
     * 绑定 ROS Topic 订阅者到具体的协议 ID 发送逻辑。
     */
    void register_tx_handlers(); 

    // IoContext 处理数据
    std::shared_ptr<drivers::common::IoContext> ctx_;

    // SerialDriver 执行具体的读写动作
    std::unique_ptr<drivers::serial_driver::SerialDriver> driver_;

    // Config 记录了波特率、端口名等
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;

    PacketHandler packet_handler_;
    std::mutex rx_mutex_; // 接收专用锁

    // 1. 接收处理映射表 (Rx: Serial -> ROS)
    using RxHandlerFunc = std::function<void(const Packet &)>;
    std::map<PacketID, RxHandlerFunc> rx_handlers_;

    // 2. 发送订阅列表 (Tx: ROS -> Serial)
    // 使用 vector 保存所有 subscription 以维持生命周期
    std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;

    // 3. 接收发布列表 (Rx: Serial -> ROS)
    // 使用 vector 保存所有 publisher 以维持生命周期
    std::vector<std::shared_ptr<rclcpp::PublisherBase>> publishers_;

    /**
     * @brief 绑定 ROS Topic 到串口发送
     *
     * @tparam MsgT ROS 消息类型 (如 geometry_msgs::msg::Twist)
     * @tparam DataT 串口协议数据结构 (protocol.hpp 中定义的结构体)
     * @param topic_name 订阅的话题名
     * @param id 发送的数据包 ID
     * @param converter 将 ROS 消息转换为协议数据的转换函数
     */
    template <typename MsgT, typename DataT>
    void bind_topic_to_serial(
        const std::string &topic_name,
        PacketID id,
        std::function<DataT(const MsgT &)> converter)
    {
      auto sub = this->create_subscription<MsgT>(
          topic_name, 10,
          [this, id, converter](const typename MsgT::SharedPtr msg)
          {
            // 1. 转换数据
            DataT data = converter(*msg);

            // 2. 打包
            auto packet_bytes = this->packet_handler_.pack<DataT>(id, data);

            // 3. 发送 (异步)
            this->async_send(packet_bytes);
          });

      RCLCPP_DEBUG(this->get_logger(), "Subscribed to topic '%s' for packet ID 0x%02X", topic_name.c_str(), id);
      subscriptions_.push_back(sub);
    }

    /**
     * @brief 绑定串口接收数据到 ROS Topic 发布
     *
     * @tparam MsgT ROS 消息类型 (如 geometry_msgs::msg::Twist)
     * @tparam DataT 串口协议数据结构 (如 在protocol.hpp 中 定义的 CmdVelData)
     * @param topic_name 发布的话题名
     * @param id 接收的数据包 ID
     * @param converter 将协议数据转换为 ROS 消息的转换函数
     */
    template <typename MsgT, typename DataT>
    void bind_serial_to_topic(
        const std::string &topic_name,
        PacketID id,
        std::function<MsgT(const DataT &)> converter)
    {
      // 1. 创建 Publisher
      auto pub = this->create_publisher<MsgT>(topic_name, 10);
      publishers_.push_back(pub);

      // 2. 注册接收回调
      rx_handlers_[id] = [this, pub, converter](const Packet &pkt)
      {
        DataT data = pkt.as<DataT>();
        MsgT msg = converter(data);

        pub->publish(msg);

        RCLCPP_DEBUG(this->get_logger(), "Published msg from packet ID 0x%02X", pkt.id);
      };
    }

    // Publishers

    Packet current_packet_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<bool> is_connected_{false};

    // 参数变量
    std::string port_;
    uint32_t baudrate_;
    double timeout_;
    double serial_frequency_;
  };
} // namespace auto_serial_bridge