#include <chrono>
#include <sstream>
#include <iomanip>

#include "serial_pkg/serial_controller.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace auto_serial_bridge
{
  SerialController::SerialController(const rclcpp::NodeOptions &options)
      : Node("serial_controller", options),
        ctx_(std::make_shared<drivers::common::IoContext>(2)) // 启用2个线程
  {
    RCLCPP_INFO(this->get_logger(), "Initializing SerialController node...");

    packet_handler_ = PacketHandler();

    get_parameters();
    setup_serial();

    // 注册所有的处理逻辑
    register_rx_handlers();
    register_tx_handlers();

    // 启动接收
    start_receive();
  }

  // 析构函数
  SerialController::~SerialController()
  {
    RCLCPP_INFO(this->get_logger(), "正在关闭串口节点...");

    if (driver_)
    {
      driver_->port()->close();
    }
  }

  // 获取参数
  void SerialController::get_parameters()
  {
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<double>("timeout", 0.1);
    // this->declare_parameter<double>("serial_frequency", 100.0);

    this->get_parameter("port", port_);
    int baudrate_temp = 115200;
    this->get_parameter("baudrate", baudrate_temp);
    baudrate_ = static_cast<uint32_t>(baudrate_temp);
    this->get_parameter("timeout", timeout_);
    // this->get_parameter("serial_frequency", serial_frequency_);

    RCLCPP_INFO(this->get_logger(), "Port: %s", port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baudrate: %u", baudrate_);
    RCLCPP_INFO(this->get_logger(), "Timeout: %.2f", timeout_);
    // RCLCPP_INFO(this->get_logger(), "Serial Frequency: %.2f", serial_frequency_);
  }

  void SerialController::setup_serial()
  {
    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
        baudrate_,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);

    driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*ctx_);
    driver_->init_port(port_, *device_config_);

    try
    {
      driver_->port()->open();
      RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
    }
  }

  void SerialController::register_rx_handlers()
  {
    // 注册接收处理函数 (Serial -> ROS)
    // 示例：将 ID_CMD_VEL 数据包绑定到 /cmd_vel_feedback 话题
    bind_serial_to_topic<geometry_msgs::msg::Twist, CmdVelData>(
        "cmd_vel_feedback",
        ID_CMD_VEL,
        [](const CmdVelData &data)
        {
          geometry_msgs::msg::Twist msg;
          msg.linear.x = data.linear_x;
          msg.angular.z = data.angular_z;
          return msg;
        });
  }

  void SerialController::register_tx_handlers()
  {
    // 注册发送处理函数 (ROS -> Serial)
    // 绑定 cmd_vel 话题到 ID_CMD_VEL 数据包
    bind_topic_to_serial<geometry_msgs::msg::Twist, CmdVelData>(
        "/cmd_vel",
        ID_CMD_VEL,
        [](const geometry_msgs::msg::Twist &msg)
        {
          CmdVelData data;
          data.linear_x = static_cast<float>(msg.linear.x);
          data.angular_z = static_cast<float>(msg.angular.z);
          return data;
        });
  }

  void SerialController::start_receive()
  {
    if (!driver_ || !driver_->port()->is_open())
      return;

    driver_->port()->async_receive(
        [this](const std::vector<uint8_t> &buffer,
               const size_t bytes_read)
        {
          RCLCPP_DEBUG(this->get_logger(), "havn't come in async_receive callback");
          if (bytes_read > 0)
          {
            // 用于debug接收数据部分
            RCLCPP_DEBUG(this->get_logger(), "Received %zu bytes from serial port.", bytes_read);

            std::vector<uint8_t> actual_data(
                buffer.begin(), buffer.begin() + bytes_read);
            std::lock_guard<std::mutex> lock(rx_mutex_);
            packet_handler_.feed_data(actual_data);

            Packet pkt;
            while (packet_handler_.parse_packet(pkt))
            {
              if (rx_handlers_.count(pkt.id))
              {
                rx_handlers_[pkt.id](pkt);
              }
            }

            // 只有成功读取数据后才继续接收
            this->start_receive();
          }
          else
          {
            RCLCPP_WARN(this->get_logger(),
                        "串口断开或读取为空，停止接收循环。");
          }
        });
  }

  void SerialController::async_send(const std::vector<uint8_t> &packet_bytes)
  {
    if (driver_ && driver_->port()->is_open())
    {
      try
      {
        driver_->port()->async_send(packet_bytes);

        RCLCPP_DEBUG(this->get_logger(), "Sent packet asynchronously, size: %zu", packet_bytes.size());

        if (!rcutils_logging_logger_is_enabled_for(
                this->get_logger().get_name(),
                RCUTILS_LOG_SEVERITY_DEBUG))
        {
          return;
        }

        // 1. 创建一个字符串流
        std::stringstream ss;
        for (auto byte : packet_bytes)
        {
          ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        // 输出发送的16进制packet数据包
        RCLCPP_DEBUG(this->get_logger(), "Packet bytes: %s", ss.str().c_str());
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Async send failed: %s", e.what());
      }
    }
  }
} // namespace auto_serial_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(auto_serial_bridge::SerialController)