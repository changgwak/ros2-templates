#include "rclcpp/rclcpp.hpp"
#include "logging_example.cpp"  // 또는 헤더 분리 가능

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoggingExampleNode>());
  rclcpp::shutdown();
  return 0;
}
