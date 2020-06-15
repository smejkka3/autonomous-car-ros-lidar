Arduino Code To Receive and publish steering and throttle commands from RC controller.
Please make sure to setup rosserial_arduino package correctly.

http://wiki.ros.org/rosserial

This link describes the steps for setting rosserial in general. After that please refer to wiki http://wiki.ros.org/rosserial_arduino/Tutorials. Make sure to follow tutorial about adding custom packages.

Usage:
After rosserial is setup, run Arduino code then restart Arduino.
Run:
    rosrun rosserial_python serial_node.py /dev/ttyUSB0
with correct port number. Code requires the output of RC controller to be on a interrupt capable pin. Output of the RC controller can be directly connected.


To add Custom Messages to Arduino,please refer to http://wiki.ros.org/rosserial_arduino/Tutorials/Adding%20Custom%20Messages

rc_msgs package is a ROS package. Please make sure to copy to catkin workspace and catkin_make. This is a custom package we have created for easy interfacing from arduino. It is located under the catkin workspace repository.

