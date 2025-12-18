#include "serial_pkg/serial_controller.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace serial_pkg
{
  SerialController::SerialController(const rclcpp::NodeOptions &options)
      : Node("serial_controller", options),
        ctx_(std::make_shared<drivers::common::IoContext>())
  {
    RCLCPP_INFO(this->get_logger(), "Initializing SerialController node...");

    packet_handler_ = PacketHandler();

    get_parameters();
    setup_serial();

    // 注册所有的处理逻辑
    register_rx_handlers();
    register_tx_handlers();
  }

  // 析构函数
  SerialController::~SerialController()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down SerialController node...");
    if (driver_)
    {
      driver_->port()->close();
    }
  }

  // 获取参数
  void SerialController::get_parameters()
  {
    this->declare_parameter<std::string>("device_name", device_name_);
    this->declare_parameter<int>("baud_rate", static_cast<int>(baud_rate_));

    this->get_parameter("device_name", device_name_);
    int baud_rate_int;
    this->get_parameter("baud_rate", baud_rate_int);
    baud_rate_ = static_cast<uint32_t>(baud_rate_int);

    RCLCPP_INFO(this->get_logger(), "Device Name: %s", device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud Rate: %u", baud_rate_);
  }

  void SerialController::setup_serial()
  {
    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
        baud_rate_,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);

    driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*ctx_);
    driver_->init_port(device_name_, *device_config_);

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
        "cmd_vel",
        ID_CMD_VEL,
        [](const geometry_msgs::msg::Twist &msg)
        {
          CmdVelData data;
          data.linear_x = static_cast<float>(msg.linear.x);
          data.angular_z = static_cast<float>(msg.angular.z);
          return data;
        });
  }

  void SerialController::receive_callback()
  {
    if (!driver_ || !driver_->port()->is_open())
    {
      RCLCPP_WARN(this->get_logger(), "Serial port is not open.");
      return;
    }

    try
    {
      // 读取所有可用数据
      std::vector<uint8_t> buffer(1024);
      size_t bytes_read = driver_->port()->receive(buffer);

      if (bytes_read > 0)
      {
        buffer.resize(bytes_read);
        packet_handler_.feed_data(buffer);

        // 循环解析所有完整的数据包
        Packet pkt;
        while (packet_handler_.parse_packet(pkt))
        {
          // 查找并执行对应的处理函数
          if (rx_handlers_.count(pkt.id))
          {
            rx_handlers_[pkt.id](pkt);
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Unknown packet ID: 0x%02X", pkt.id);
          }
        }
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", e.what());
    }
  }
} // namespace serial_pkg

RCLCPP_COMPONENTS_REGISTER_NODE(serial_pkg::SerialController)