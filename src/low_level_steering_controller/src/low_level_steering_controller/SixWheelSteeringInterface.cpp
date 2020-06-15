#include "low_level_steering_controller/SixWheelSteeringInterface.h"

SixWheelSteeringInterface::SixWheelSteeringInterface() : SteeringCommandInterfaceBase()
{
}

SixWheelSteeringInterface::SixWheelSteeringInterface(ros::NodeHandle &n)
{
    node_handle = n;

    if (!node_handle.getParam("steer_topic_name", steer_topic_name))
    {
        ROS_WARN("Did not receive steer_topic_name param. Using '/motor_controller/motor_commands'");
        steer_topic_name = "/motor_controller/motor_commands";
    }

    steer_data_publisher = node_handle.advertise<sixwd_msgs::SixWheelCommand>(steer_topic_name, 10, false);
}

bool SixWheelSteeringInterface::send_command(SteerRequest com, bool manual)
{
    sixwd_msgs::SixWheelCommand msg;
    msg.controltype = 0;
	com.value=-com.value;

    double left_speed_req = ((output_speed * 2) + (com.value * 0.27)) / 2;
    double right_speed_req = ((output_speed * 2) - (com.value * 0.27)) / 2;

    double sign_left = (left_speed_req > 0) ? 1.0 : -1.0;
    double sign_right = (right_speed_req > 0) ? 1.0 : -1.0;

    msg.left_speed = (411.6445 * left_speed_req) - (88.65495 * sign_left);
    msg.right_speed = (411.6445 * right_speed_req) - (88.65495 * sign_right);
    if (abs(left_speed_req * 10000) < (0.2740 * 10000))
    {
        msg.left_speed = 0;
  	 if (abs(left_speed_req * 10000) > (0.10 * 10000))
   	 {
        msg.left_speed = 22*sign_left;
    	}
}

    if (abs(right_speed_req * 10000) < (0.2740 * 10000))
    {
        msg.right_speed = 0;
      if (abs(right_speed_req * 10000) > (0.10 * 10000))
       {
        msg.right_speed = 22*sign_right;
        } 
    }

    if (abs(left_speed_req * 10000) > (0.82 * 10000))
    {
        msg.left_speed = 255 * sign_left;
    }

    if (abs(right_speed_req * 10000) > (0.82 * 10000))
    {
        msg.right_speed = 255 * sign_left;
    }


    steer_data_publisher.publish(msg);

    return true;
}
