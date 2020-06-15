# bear_car_launch

This package collects various system relevant launch files for both platform 6WD and 4WD.

## Remote-controlled (RC) driving

Depending on the used platform use either

`roslaunch bear_car_launch direct_rc_fourwd.launch` 

or

`roslaunch bear_car_launch direct_rc_sixwd.launch`

## Autonomous Navigation

To let the car autonomously drive to provided target positions (e.g. rviz) use depending on the car either

`roslaunch bear_car_launch autonomous_navigation_fourwd.launch`

or

`roslaunch bear_car_launch autonomous_navigation_sixwd.launch`


## rplidar_a3.launch

Launch file for starting the lidar.

## low_level_starter.launch

The launch file to easily start six wheeled platform.

It is a launch file that starts the nodes:

- low_level_speed_controller/six_wheel_low_level_controller
- low_level_steering_controller/six_wheel_low_level_steering_controller
- serial_6w/serial_communicator_6w

## rs_imu

Launch file to start imu filter and sensor

rs_imu.launch is launched by odometry_agent. IMU filtered data is being fused under ekf for acceleration after filtering.

## morse_telop

Running nodes for tele-operation of simulated morse robots.

Nodes are publishing twist messages to the topic `/robot/basic_actuate`

Keyboard control:
`roslaunch bear_car_launch morse_teleop.launch use_keyboard:=true` 

Gamepad control:
`roslaunch bear_car_launch morse_teleop.launch use_keyboard:=false`

Without the parameter `use_keyboard` default will be keyboard.

## Morse RC teleoperation

You can also plug the RC to your machine and run following launch file for controlling
either 6WD or 4WD platform

`roslaunch ackermann_rc rc_driver_morse.launch` 
