#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "io_context/io_context.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "serial_pkg/packet_handler.hpp"
#include "serial_pkg/protocol.hpp"

namespace serial_pkg
{
  class SerialController : public rclcpp::Node
  {
  public:
    /**
     * @brief 构造函数：初始化节点并设置串口
     */
    explicit SerialController(const rclcpp::NodeOptions &options);

    /**
     * @brief 析构函数：确保程序关掉时，串口也能优雅地关闭
     */
    ~SerialController() override;

  private:
    void get_parameters();                                     // 像查字典一样获取波特率等参数
    void setup_serial();                                       // 像插插头一样初始化串口硬件
    void start_receive();                                      // 开启异步接收
    void async_send(const std::vector<uint8_t> &packet_bytes); // 异步发送

    // 注册函数
    void register_rx_handlers(); // 注册接收处理 (Serial -> ROS)
    void register_tx_handlers(); // 注册发送处理 (ROS -> Serial)

    // IoContext 处理数据的流动
    std::shared_ptr<drivers::common::IoContext> ctx_;

    // SerialDriver 执行具体的读写动作
    std::unique_ptr<drivers::serial_driver::SerialDriver> driver_;

    // Config 记录了波特率、端口名等
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;

    PacketHandler packet_handler_;

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
     * @tparam MsgT ROS 消息类型 (如 geometry_msgs::msg::Twist)
     * @tparam DataT 串口协议数据结构 (protocol.hpp 中定义的结构体)
     * @param topic_name 订阅的话题名
     * @param id 发送的数据包 ID
     * @param converter 将 ROS 消息转换为协议数据的 lambda 函数
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
      subscriptions_.push_back(sub);
    }

    /**
     * @brief 绑定串口接收数据到 ROS Topic 发布
     * @tparam MsgT ROS 消息类型 (如 geometry_msgs::msg::Twist)
     * @tparam DataT 串口协议数据结构 (如 CmdVelData)
     * @param topic_name 发布的话题名
     * @param id 接收的数据包 ID
     * @param converter 将协议数据转换为 ROS 消息的 lambda 函数
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
        // a. 转换数据结构
        DataT data = pkt.as<DataT>();

        // b. 转换为 ROS 消息
        MsgT msg = converter(data);

        // c. 发布
        pub->publish(msg);

        // 可选：打印调试信息
        // RCLCPP_DEBUG(this->get_logger(), "Published msg from packet ID 0x%02X", pkt.id);
      };
    }

    // Publishers

    Packet current_packet;
    rclcpp::TimerBase::SharedPtr timer_;

    // 参数变量
    std::string port_;
    uint32_t baudrate_;
    double timeout_;
    double serial_frequency_;

    std::thread io_thread_;
  };
} // namespace serial_pkg