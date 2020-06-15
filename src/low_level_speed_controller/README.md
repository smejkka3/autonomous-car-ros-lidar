# low_level_speed_controller ROS Package

This is low level linear speed controller currently used in both car platforms. Its main purpose is basically receive commanded velocities from topics, receive odometry information and generate necessary motor commands. Basic documentation is included on the source code but main features are:

- Supports sending commands periodically to the motor controllers. Also commands still could be sent manually
- Limit checking in multiple layers. Can limit speed in terms of m/s and also in terms of motor input. Acceleration limits are also considered.
- Although steering controller which is another package in this repo does not support going to a goal point, in this package a basic PID controller is added for goal point following behaviour.
- Timestamps of the commands are checked.
- Abstract Interfaces are included. Adapting a new system should be fast.
- Supports Twist, Ackermann and Ackermann stamped message types for control and can be changed dynamically.
- Retrieves odometry data and extracts position and speed data for goal point behaviour and acceleration check. Position and speed can also be received from seperate interfaces.

**TODO: different control modes and parameters under /param folder**
