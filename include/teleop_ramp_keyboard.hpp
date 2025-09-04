/**
 * @file      teleop_ramp_keyboard.hpp
 * @author    Jaehong Lee (leejae0720@gmail.com)
 * @brief     teleop ramp ROS2 keyboard header file
 * @date      2025-09-04
 * @note      This code is based on teleop_twist_keyboard.
 */

#ifndef TELEOP_RAMP_KEYBOARD_
#define TELEOP_RAMP_KEYBOARD_

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>
#include <cmath>
#include <fcntl.h>

#include "spdlog/spdlog.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono;

class TeleopRampKeyboard : public rclcpp::Node {
  public:
    TeleopRampKeyboard();
    ~TeleopRampKeyboard();

    int getch(void);
    float ramped_vel(float current, float target, float step);
    float vel_check(float curr, bool decrease);
    float l_vel(char key, float x);
    float a_vel(char key, float th);
  private:

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    std::thread worker_;
    std::atomic_bool running_{false};

    float speed_ = 0.5f;
    float turn_  = 1.0f;
    float linear_accel_  = 0.2f;  // linear_acceleration[m/s^2]
    float angular_accel_ = 0.1f;  // angular_acceleration [rad/s^2]
    int control_period_ = 200;    // control period [hz]

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
};

#endif // TELEOP_RAMP_KEYBOARD_

