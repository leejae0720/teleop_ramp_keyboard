/**
 * @file      teleop_ramp_keyboard.cpp
 * @author    Jaehong Lee (leejae0720@gmail.com)
 * @brief     teleop ramp ROS2 keyboard implementation
 * @date      2025-09-17
 * @note      This code is based on teleop_twist_keyboard.
 */

#include "teleop_ramp_keyboard.hpp"

TeleopRampKeyboard::TeleopRampKeyboard() : Node("teleop_ramp_keyboard") {

  declare_parameter("linear_accel", 0.2f);
  declare_parameter("angular_accel", 0.1f);
  declare_parameter("control_period", 200);
  get_parameter("linear_accel", linear_accel_);
  get_parameter("angular_accel", angular_accel_);
  get_parameter("control_period", control_period_); 
  pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  spdlog::info("@copyright Copyright (c) Jaehong Lee(leejae0720@gmail.com) All rights reserved.");
  spdlog::info("{}", msg);
  spdlog::info("Now max speed is {0} and turn is {1}", speed_, turn_);

  spdlog::info("linear acceleration: {0} m/s^2", linear_accel_);
  spdlog::info("angular acceleration: {0} rad/s^2", angular_accel_);
  spdlog::info("control period: {0} hz", control_period_);

  running_ = true;
  last_key_time_ = std::chrono::steady_clock::now();
  worker_ = std::thread([this]() {
    char key;
    geometry_msgs::msg::Twist current_twist;
    geometry_msgs::msg::Twist target_twist;

    rclcpp::Rate loop_rate(control_period_);

    while (rclcpp::ok() && running_) {
      key = getch();

      if (key != 0) {
        last_key_time_ = std::chrono::steady_clock::now();
      }

      if (key == 'A' || key == 'B') {
        float x = l_vel(key, target_twist.linear.x / speed_);
        target_twist.linear.x = x * speed_;
      }
      else if (key == 'C' || key == 'D') {
        float th = a_vel(key, target_twist.angular.z / turn_);
        target_twist.angular.z = th * turn_;
      }
      else if (moveBindings.count(key) == 1) {
        target_twist.linear.x = moveBindings[key][0] * speed_;
        target_twist.linear.y = moveBindings[key][1] * speed_;
        target_twist.linear.z = moveBindings[key][2] * speed_;
        target_twist.angular.z = moveBindings[key][3] * turn_;
      }
      else if (speedBindings.count(key) == 1) {
        speed_ *= speedBindings[key][0];
        turn_  *= speedBindings[key][1];
        spdlog::info("Now top speed is {0} and turn is {1} Current speed might be affected", speed_, turn_);
      }
      else if (key == 's' || key == 'S') {
        target_twist.linear.x = 0;
        target_twist.linear.y = 0;
        target_twist.linear.z = 0;
        target_twist.angular.z = 0;

        current_twist.linear.x = 0;
        current_twist.linear.y = 0;
        current_twist.linear.z = 0;
        current_twist.angular.x = 0;
        current_twist.angular.y = 0;
        current_twist.angular.z = 0;

        pub_->publish(current_twist);
        spdlog::warn("Robot Stopped..!!");
        continue;
      }
      else {
        if (key != 0) {
          spdlog::warn("Current: speed {0} turn {1} | Invalid command! {2}", speed_, turn_, key);
        }
      }

      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_key_time_).count() > 500) {
        target_twist = geometry_msgs::msg::Twist();
      }

      current_twist.linear.x  = ramped_vel(current_twist.linear.x,  target_twist.linear.x,  linear_accel_);
      current_twist.linear.y  = ramped_vel(current_twist.linear.y,  target_twist.linear.y,  linear_accel_);
      current_twist.linear.z  = ramped_vel(current_twist.linear.z,  target_twist.linear.z,  linear_accel_);
      current_twist.angular.z = ramped_vel(current_twist.angular.z, target_twist.angular.z, angular_accel_);

      pub_->publish(current_twist);
      loop_rate.sleep();
    }
  });
}

TeleopRampKeyboard::~TeleopRampKeyboard() {
  running_ = false;
  if (worker_.joinable()) worker_.join();
}

int TeleopRampKeyboard::getch(void)
{
  int ch = 0;
  int fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);
  if (fd < 0) return 0;

  struct termios oldt{}, newt{};
  tcgetattr(fd, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &newt);

  unsigned char c;
  if (read(fd, &c, 1) == 1) {
    ch = c;
    if (ch == 0x1B) {
      unsigned char buf[2];
      if (read(fd, buf, 1) == 1 && buf[0] == '[') {
        if (read(fd, buf+1, 1) == 1) ch = buf[1];
      }
    }
  }

  tcsetattr(fd, TCSANOW, &oldt);
  close(fd);
  return ch;
}

float TeleopRampKeyboard::ramped_vel(float current, float target, float step) {
  if (std::fabs(target - current) < step)
    return target;
  return current + std::copysign(step, target - current);
}

float TeleopRampKeyboard::vel_check(float curr, bool decrease = false){
  if (decrease)
    curr = (curr >= -0.95) ? curr - 0.05 : -1;
  else
    curr = (curr <= 0.95) ? curr + 0.05 : 1;
  return curr;
}

float TeleopRampKeyboard::l_vel(char key, float x){
  if(key=='A') return vel_check(x,false);
  if(key=='B') return vel_check(x,true);
  return 0;
}

float TeleopRampKeyboard::a_vel(char key, float th){
  if(key=='C') return vel_check(th,true);
  if(key=='D') return vel_check(th,false);
  return 0;
}