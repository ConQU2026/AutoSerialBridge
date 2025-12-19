#include "rclcpp/rclcpp.hpp"
#include "serial_pkg/serial_controller.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<serial_pkg::SerialController>(rclcpp::NodeOptions());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}