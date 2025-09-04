/**
 * @file      main.cpp
 * @author    Jaehong Lee (leejae0720@gmail.com)
 * @brief     main.cpp file implementation
 * @date      2025-09-04
 */

#include "teleop_ramp_keyboard.hpp"

int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopRampKeyboard>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}