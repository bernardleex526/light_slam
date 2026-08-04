// Copyright 2026 admin
#include "leg_wheel_odom/leg_wheel_odom_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegWheelOdomNode>());
  rclcpp::shutdown();
  return 0;
}
