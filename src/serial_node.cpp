#include "rclcpp/rclcpp.hpp"
#include "serial_pkg/serial_controller.hpp"

/**
 * @brief 主函数
 *
 * 初始化 ROS 2 节点并运行 SerialController。
 *
 * @param argc 参数个数
 * @param argv 参数列表
 * @return int 程序退出码
 */
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<auto_serial_bridge::SerialController>(
      rclcpp::NodeOptions());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}