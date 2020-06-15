#! /usr/bin/env python

import rospy
from geometry_msgs.msg import *
import math
    
force_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
DEBUG = True

#Fetch cmd_vel msgs and set it as target velocities
def cmd_vel_callback(msg):
    global input
    input = msg
    #if DEBUG:
        #rospy.loginfo("Received a /cmd_vel message! V(x),V(y): [%f, %f] Theta(z):[%f]"%(msg.linear.x, msg.linear.y, msg.angular.z))
    
def loop(input, max_steering_angle, min_speed, max_speed, min_input_speed, speed_factor):
    #Calculate Steering angle
    angle = input.angular.z
    
    # Clamp angle to [-max_steering, max_steering]
    angle = max(min(angle, max_steering_angle), -max_steering_angle)

    speed = 0.0
    if input.linear.x == 0:  # 0.05 is set as minimum in the local planner
        speed = 0.0
    else:
        #speed = input.linear.x * speed_factor
        #speed = 2.0 - abs(angle)
        e = 2.718
        speed = pow(e, -abs(angle)) * max_speed
    
    # Clamp speed to 0 or [min, max]
    if speed > 0.0:
        speed = max(min(speed, max_speed), min_speed)

    if DEBUG:
        rospy.loginfo("Speed: [%f], Theta: [%f]"%(speed, angle))

    #Publish Steerforce Command    
    final_control = Twist()
    final_control.linear.x = speed
    final_control.angular.z = angle
    force_pub.publish(final_control)
    
if __name__ == '__main__':
    global input
    input = Twist()

    max_steering_angle = rospy.get_param("/steerforce_converter/max_steer_ang")
    min_speed = rospy.get_param("/steerforce_converter/min_speed")
    max_speed = rospy.get_param("/steerforce_converter/max_speed")
    min_input_speed = rospy.get_param("/steerforce_converter/min_input_speed")
    speed_factor = rospy.get_param("/steerforce_converter/speed_factor")

    rospy.init_node('steerforce_converter')
    rospy.Subscriber("/cmd_vel_planner", Twist, cmd_vel_callback)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        loop(input, max_steering_angle, min_speed, max_speed, min_input_speed, speed_factor)
        rate.sleep()


   
