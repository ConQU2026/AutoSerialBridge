#include <chrono>
#include <sstream>
#include <iomanip>

#include "serial_pkg/serial_controller.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace auto_serial_bridge
{
  SerialController::SerialController(const rclcpp::NodeOptions &options)
      : Node("serial_controller", options),
        ctx_(std::make_shared<drivers::common::IoContext>(2))
  {
    RCLCPP_INFO(this->get_logger(), "Initializing SerialController node...");

    packet_handler_ = PacketHandler();

    get_parameters();

    // 注册所有的处理逻辑
    register_rx_handlers();
    register_tx_handlers();

    // 初始化串口配置，但不立即打开串口
    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
        baudrate_,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);

    // 创建定时器：每 1 秒检查一次连接并尝试重连
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&SerialController::check_connection, this));
  }

  // 析构函数
  SerialController::~SerialController()
  {
    RCLCPP_INFO(this->get_logger(), "正在关闭串口节点...");
    reset_serial();
  }

  // 获取参数
  void SerialController::get_parameters()
  {
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<double>("timeout", 0.1);

    this->get_parameter("port", port_);
    int baudrate_temp = 115200;
    this->get_parameter("baudrate", baudrate_temp);
    baudrate_ = static_cast<uint32_t>(baudrate_temp);
    this->get_parameter("timeout", timeout_);

    RCLCPP_INFO(this->get_logger(), "Port: %s", port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baudrate: %u", baudrate_);
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
    catch (const std::exception &)
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
  }

  void SerialController::check_connection()
  {
    if (is_connected_)
    {
      return;
    }

    RCLCPP_WARN(this->get_logger(),
                "串口未连接，正在尝试连接设备: %s ...",
                port_.c_str());

    if (try_open_serial())
    {
      RCLCPP_INFO(this->get_logger(), ">>> 串口重连成功！恢复通信 <<<");
      is_connected_ = true;
      start_receive();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),
                   "连接失败: 找不到设备或权限不足 (将在1秒后重试)");
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
    if (!is_connected_ || !driver_ || !driver_->port()->is_open())
    {
      return;
    }

    driver_->port()->async_receive(
        [this](const std::vector<uint8_t> &buffer,
               const size_t bytes_read)
        {
          if (bytes_read > 0)
          {
            RCLCPP_DEBUG(this->get_logger(),
                         "Received %zu bytes", bytes_read);

            // Avoid unnecessary copy by using buffer directly when fully consumed
            {
              std::lock_guard<std::mutex> lock(rx_mutex_);
              // Feed only the valid portion of the buffer
              if (bytes_read > 0 && bytes_read == buffer.size())
              {
                packet_handler_.feed_data(buffer);
              }
              else if (bytes_read > 0)
              {
                std::vector<uint8_t> actual_data(
                    buffer.begin(), buffer.begin() + bytes_read);
                packet_handler_.feed_data(actual_data);
              }
            }

            Packet pkt;
            while (packet_handler_.parse_packet(pkt))
            {
              auto it = rx_handlers_.find(pkt.id);
              if (it != rx_handlers_.end())
              {
                it->second(pkt);
              }
            }

            this->start_receive();
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(),
                         "!!! 检测到串口断开或读取异常 !!!");
            is_connected_ = false;
            reset_serial();
          }
        });
  }

  void SerialController::async_send(const std::vector<uint8_t> &packet_bytes)
  {
    if (!is_connected_)
    {
      return;
    }

    if (driver_ && driver_->port()->is_open())
    {
      try
      {
        driver_->port()->async_send(packet_bytes);

        RCLCPP_DEBUG(this->get_logger(),
                     "Sent packet asynchronously, size: %zu",
                     packet_bytes.size());

        if (!rcutils_logging_logger_is_enabled_for(
                this->get_logger().get_name(),
                RCUTILS_LOG_SEVERITY_DEBUG))
        {
          return;
        }

        std::stringstream ss;
        for (auto byte : packet_bytes)
        {
          ss << std::hex << std::uppercase << std::setw(2)
             << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        RCLCPP_DEBUG(this->get_logger(), "Packet bytes: %s",
                     ss.str().c_str());
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "发送失败 (设备可能已拔出): %s", e.what());
        is_connected_ = false;
        reset_serial();
      }
    }
  }
} // namespace auto_serial_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(auto_serial_bridge::SerialController)