#include <chrono>
#include <sstream>
#include <iomanip>

#include "serial_pkg/serial_controller.hpp"
#include "serial_pkg/generated_bindings.hpp" // 生成的绑定代码
#include "serial_pkg/generated_config.hpp"   // 生成的配置常数
#include "rclcpp_components/register_node_macro.hpp"

namespace auto_serial_bridge
{
  SerialController::SerialController(const rclcpp::NodeOptions &options)
      : Node("serial_controller", options),
        state_(State::WAITING_HANDSHAKE),
        ctx_(std::make_shared<drivers::common::IoContext>(2)),
        packet_handler_(auto_serial_bridge::config::BUFFER_SIZE)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing SerialController...");

    get_parameters();
    
    // 注册所有 ROS 订阅者
    auto_serial_bridge::generated::register_all(this);

    // 设备配置
    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
        baudrate_,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);

    // 连接检查定时器
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&SerialController::check_connection, this));
        
    // 心跳/握手定时器 (1Hz)
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        [this]() {
            if (!is_connected_) return;
            
            if (state_ == State::WAITING_HANDSHAKE) {
                // 发送握手请求
                Packet_Handshake pkt;
                pkt.protocol_hash = PROTOCOL_HASH;
                send_packet(PACKET_ID_HANDSHAKE, pkt);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Handshake (Hash: 0x%08X)...", PROTOCOL_HASH);
            } else {
                // 发送心跳包
                // Packet_Heartbeat pkt;
                // pkt.count++; 
                // send_packet(PACKET_ID_HEARTBEAT, pkt);
            }
        });
  }

  SerialController::~SerialController()
  {
    reset_serial();
  }

  void SerialController::get_parameters()
  {
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", auto_serial_bridge::config::DEFAULT_BAUDRATE);
    this->declare_parameter<double>("timeout", 0.1);

    this->get_parameter("port", port_);
    int baudrate_temp = auto_serial_bridge::config::DEFAULT_BAUDRATE;
    this->get_parameter("baudrate", baudrate_temp);
    baudrate_ = static_cast<uint32_t>(baudrate_temp);
    this->get_parameter("timeout", timeout_);

    RCLCPP_INFO(this->get_logger(), "Port: %s, Baudrate: %u", port_.c_str(), baudrate_);
  }

  bool SerialController::try_open_serial()
  {
    try
    {
      reset_serial();
      driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*ctx_);
      driver_->init_port(port_, *device_config_);
      driver_->port()->open();
      return driver_->port()->is_open();
    }
    catch (const std::exception &e)
    {
      return false;
    }
  }

  void SerialController::reset_serial()
  {
    if (driver_)
    {
      if (driver_->port()->is_open())
      {
        driver_->port()->close();
      }
      driver_.reset();
    }
    state_ = State::WAITING_HANDSHAKE;
  }

  void SerialController::check_connection()
  {
    if (is_connected_) return;

    if (try_open_serial())
    {
      RCLCPP_INFO(this->get_logger(), "Serial connected. Waiting for handshake...");
      is_connected_ = true;
      state_ = State::WAITING_HANDSHAKE;
      start_receive();
    }
    else
    {
      // Silent retry or log only occasionally
    }
  }
  
  void SerialController::process_handshake(const Packet& pkt) {
      if (pkt.payload.size() != sizeof(Packet_Handshake)) return;
      
      const Packet_Handshake* data = reinterpret_cast<const Packet_Handshake*>(pkt.payload.data());
      if (data->protocol_hash == PROTOCOL_HASH) {
          state_ = State::RUNNING;
          RCLCPP_INFO(this->get_logger(), "Handshake SUCCESS! Protocol Hash Matched. Entering RUNNING state.");
      } else {
          RCLCPP_ERROR(this->get_logger(), "Handshake FAILED! Hash mismatch. Local: 0x%08X, Remote: 0x%08X", PROTOCOL_HASH, data->protocol_hash);
      }
  }

  void SerialController::start_receive()
  {
    if (!is_connected_ || !driver_ || !driver_->port()->is_open()) return;

    driver_->port()->async_receive(
        [this](const std::vector<uint8_t> &buffer, const size_t bytes_read)
        {
          if (bytes_read > 0)
          {
             {
                 std::lock_guard<std::mutex> lock(rx_mutex_);
                 packet_handler_.feed_data(buffer.data(), bytes_read); //零拷贝意图（虽然 buffer 是 vector）
             }
             
             Packet pkt;
             while (packet_handler_.parse_packet(pkt)) {
                 if (pkt.id == PACKET_ID_HANDSHAKE) {
                     process_handshake(pkt);
                     // 允许将握手包也转发给 ROS? 生成的代码确实暴露了它。
                     auto_serial_bridge::generated::dispatch_packet(this, static_cast<uint8_t>(pkt.id), pkt.payload);
                 } else if (state_ == State::RUNNING) {
                     auto_serial_bridge::generated::dispatch_packet(this, static_cast<uint8_t>(pkt.id), pkt.payload);
                 } else {
                     // 如果未握手成功，丢弃数据包
                 }
             }
             
             this->start_receive();
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "Read error/close.");
            is_connected_ = false;
            reset_serial();
          }
        });
  }

  void SerialController::async_send(const std::vector<uint8_t> &packet_bytes)
  {
    if (!is_connected_ || !driver_ || !driver_->port()->is_open()) return;
    
    // 在 WAITING_HANDSHAKE 状态下, 只允许发送握手数据包?
    // 如果我们需要严格的输出控制，可以在这里过滤。
    // 实现:
    // 数据包类型在第二个字节? 我们不想在这里解析。
    // 依赖其他地方的逻辑。
    
    try
    {
        driver_->port()->async_send(packet_bytes);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Send error: %s", e.what());
        is_connected_ = false;
        reset_serial();
    }
  }

} // namespace auto_serial_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(auto_serial_bridge::SerialController)