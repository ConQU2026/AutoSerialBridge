#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <mutex>
#include <atomic>
#include <map>

#include "rcutils/logging.h"
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "io_context/io_context.hpp"

#include "auto_serial_bridge/packet_handler.hpp"
#include "auto_serial_bridge/protocol.hpp"

namespace auto_serial_bridge
{
  /**
   * @brief 串口控制节点
   */
  class SerialController : public rclcpp::Node
  {
  public:
    explicit SerialController(const rclcpp::NodeOptions &options);
    ~SerialController() override;

    // 发送数据包的模板方法 (供生成的绑定代码使用)
    template <typename T>
    void send_packet(PacketID id, const T& data) {
         auto bytes = packet_handler_.pack(id, data);
         async_send(bytes);
    }

    // 发布消息的模板方法 (供生成的绑定代码使用)
    template <typename MsgT>
    void publish_message(const std::string& topic, const MsgT& msg) {
        std::string key = topic;
        // 优化: 先检查是否存在 (如果从多个线程动态添加publisher，这里不是线程安全的，但在当前上下文中是可接受的)
        if (publishers_map_.find(key) == publishers_map_.end()) {
             // 如果不存在则创建发布者
             publishers_map_[key] = this->create_publisher<MsgT>(topic, 10);
        }
        
        // 动态转换为具体类型的发布者
        auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<MsgT>>(publishers_map_[key]);
        if (pub) {
            pub->publish(msg);
        }
    }

    // 保持订阅对象存活的辅助函数
    void add_subscription(std::shared_ptr<rclcpp::SubscriptionBase> sub) {
        subscriptions_.push_back(sub);
    }

  private:
    void get_parameters();
    void start_receive();
    void async_send(const std::vector<uint8_t> &packet_bytes);
    void check_connection();
    void reset_serial();
    bool try_open_serial();
    
    // 状态机
    enum class State {
        WAITING_HANDSHAKE, // 等待握手
        RUNNING            // 运行中
    };
    State state_;
    void process_handshake(const Packet& pkt);
    
    // IoContext 和 驱动
    std::shared_ptr<drivers::common::IoContext> ctx_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> driver_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::mutex rx_mutex_; // 接收缓冲区互斥锁 

    PacketHandler packet_handler_;
    
    // 通用存储，用于保持Publisher存活
    std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>> publishers_map_;
    std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;
    
    // 定时器和状态
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    std::atomic<bool> is_connected_{false}; // 连接状态
    
    rclcpp::Time last_handshake_time_;
    rclcpp::Time last_heartbeat_time_;

    // 参数
    std::string port_;
    uint32_t baudrate_;
    double timeout_;

    // 存储生成的 ProtocolPublishers 结构体 (使用 void* 避免循环依赖)
    std::shared_ptr<void> protocol_impl_;
  };
} // namespace auto_serial_bridge