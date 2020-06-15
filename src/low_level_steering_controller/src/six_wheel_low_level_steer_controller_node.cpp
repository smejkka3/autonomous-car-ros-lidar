#include "ros/ros.h"

#include "low_level_steering_controller/SixWheelSteeringInterface.h"
#include "geometry_msgs/Twist.h"

//#define SteeringGain (180.0 * 45.0) / (3.1415 * 3000)
//#define SteeringOffset 0.45

//#define WheelBase 0.28

SixWheelSteeringInterface *interface;
ros::Subscriber speed_sub;
ros::Subscriber angle_sub;

void angle_callback(const geometry_msgs::Twist::ConstPtr &msg){
    SixWheelSteeringInterface::SteerRequest req;

    double comm_val;
    req.value = msg->angular.z;
    req.req_time = ros::Time::now();
    interface->queue_command(req);
    interface->issue_write_command();
}

void speed_callback(const std_msgs::Float64::ConstPtr &msg){
    interface->set_speed(msg->data);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "steer_controller_node");
    ros::NodeHandle n;

    //ROS_INFO("Inside Node");
    interface = new SixWheelSteeringInterface(n);
    interface->set_operation_type(SixWheelSteeringInterface::OperationType::AUTOMATIC);
    interface->set_steering_limits(100, -100);
    interface->set_command_timeout(ros::Duration(1.0));
    interface->set_execution_rate(ros::Duration(1.0/50.0));
    interface->set_steering_gain(1.0).set_steering_offset(0.0);
    speed_sub = n.subscribe("/speed_output", 10, speed_callback);
    angle_sub = n.subscribe("cmd_vel", 10, angle_callback);
    //ROS_INFO("Sub to Topic");
    
    ros::spin();
    return 0;
}