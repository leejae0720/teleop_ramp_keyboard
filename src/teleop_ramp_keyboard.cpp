/**
 * @file      teleop_ramp_keyboard.cpp
 * @author    Jaehong Lee (leejae0720@gmail.com)
 * @brief     teleop ramp ROS2 keyboard implementation
 * @date      2025-04-01
 * @note      
 */

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>
#include <cmath>

#include "spdlog/spdlog.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono;

std::map<char, std::vector<float>> moveBindings
{
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'O', {1, -1, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'J', {0, 1, 0, 0}},
  {'L', {0, -1, 0, 0}},
  {'U', {1, 1, 0, 0}},
  {'<', {-1, 0, 0, 0}},
  {'>', {-1, -1, 0, 0}},
  {'M', {-1, 1, 0, 0}},
  {'t', {0, 0, 1, 0}},
  {'b', {0, 0, -1, 0}},
  {'k', {0, 0, 0, 0}},
  {'K', {0, 0, 0, 0}}
};

std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};

const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
---------------------------
t : up (+z)
b : down (-z)
s/S : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
)";

int getch(void)
{
  int ch;
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

float ramped_vel(float current, float target, float step) {
  if (std::fabs(target - current) < step)
    return target;
  return current + std::copysign(step, target - current);
}

float vel_check(float curr, bool decrease = false){
  if (decrease)
    curr = (curr >= -0.95) ? curr - 0.05 : -1;
  else
    curr = (curr <= 0.95) ? curr + 0.05 : 1;
  return curr;
}

float Lvel(char key, float x){
  if(key=='A') return vel_check(x,false);
  if(key=='B') return vel_check(x,true);
  return 0;
}

float Avel(char key, float th){
  if(key=='C') return vel_check(th,true);
  if(key=='D') return vel_check(th,false);
  return 0;
}

int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("teleop_ramp_keyboard");
  auto _pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  float speed = 0.5;
  float turn = 1.0;
  char key;

  geometry_msgs::msg::Twist current_twist;
  geometry_msgs::msg::Twist target_twist;

  float linear_accel = 0.1;   // m/s per loop
  float angular_accel = 0.25;  // rad/s per loop

  spdlog::info("@copyright Copyright (c) Jaehong Lee(leejae0720@gmail.com) All rights reserved.");
  spdlog::info("{}", msg);
  spdlog::info("Now top speed is {0} and turn is {1}", speed, turn);

  rclcpp::Rate loop_rate(20); // 20Hz = 50ms 주기

  while (rclcpp::ok()) {
    key = getch();

    if (key == 'A' || key == 'B') {
      float x = Lvel(key, target_twist.linear.x / speed);
      target_twist.linear.x = x * speed;
    }
    else if (key == 'C' || key == 'D') {
      float th = Avel(key, target_twist.angular.z / turn);
      target_twist.angular.z = th * turn;
    }
    else if (moveBindings.count(key) == 1) {
      target_twist.linear.x = moveBindings[key][0] * speed;
      target_twist.linear.y = moveBindings[key][1] * speed;
      target_twist.linear.z = moveBindings[key][2] * speed;
      target_twist.angular.z = moveBindings[key][3] * turn;

    }
    else if (speedBindings.count(key) == 1) {
      speed *= speedBindings[key][0];
      turn *= speedBindings[key][1];
      spdlog::info("Now top speed is {0} and turn is {1} Current speed might be affected", speed, turn);
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

      _pub->publish(current_twist); 
      spdlog::warn("Robot Stopped..!!");
      continue;
    }
    
    else {
      spdlog::warn("Current: speed {0} turn {1} | Invalid command! {2}", speed, turn, key);
    }

    current_twist.linear.x  = ramped_vel(current_twist.linear.x,  target_twist.linear.x,  linear_accel);
    current_twist.linear.y  = ramped_vel(current_twist.linear.y,  target_twist.linear.y,  linear_accel);
    current_twist.linear.z  = ramped_vel(current_twist.linear.z,  target_twist.linear.z,  linear_accel);
    current_twist.angular.z = ramped_vel(current_twist.angular.z, target_twist.angular.z, angular_accel);

    _pub->publish(current_twist);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
