# ramp_teleop_twist_keyboard
C++ Implementation of the Generic Keyboard Teleop for ROS2: https://github.com/ros-teleop/teleop_twist_keyboard

## Features

The original teleop_twist_keyboard package, which provided step commands, has been extended to support ramp commands. By adjusting acceleration and deceleration values and setting the control loop rate, this implementation enables ramp-based motion testing of the robot.

## Prerequisites
 - OS: Ubuntu OS or Window (Author using Ubuntu 22.04 vesion)
 - ROS2 (Author using humble version)

## Getting Started

```bash
git clone https://github.com/leejae0720/rampinput_teleop_twist_keyboard.git
cd ..
colcon build --packages-select teleop_ramp_keyboard

source install/setup.bash
```

## Run

```bash
source the ros2 worksapce and

# In terminal, run
ros2 run teleop_ramp_keyboard teleop_ramp_keyboard

# If you want to see the outputs, check the /cmd_vel topic
ros2 topic echo /cmd_vel
```

## Usage

Same as the original + some addons

```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
---------------------------
Simple Teleoperation with arrow keys
          ⇧
        ⇦   ⇨
          ⇩

          A
        D   C
          B
This increases/decreases speed linearly.
---------------------------
t : up (+z)
b : down (-z)
s/S : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
NOTE: Increasing or Decreasing will take affect live on the moving robot.
      Consider Stopping the robot before changing it.
      
CTRL-C to quit
```



------

## Contributor
 - Name: Jaehong Lee (이재홍)
 - Email: leejae0720@gmail.com
