#include "ros/ros.h"

#include "low_level_steering_controller/VescSteeringInterface.h"
#include "geometry_msgs/Twist.h"

//#define SteeringGain (180.0 * 45.0) / (3.1415 * 3000)
//#define SteeringOffset 0.45

#define WheelBase 0.28

VescSteeringInterface *interface;
ros::Subscriber speed_pub;

void speed_callback(const geometry_msgs::Twist::ConstPtr &msg){
    VescSteeringInterface::SteerRequest req;
    double comm_val;

    //if(msg->linear.x == 0.0 || msg->angular.z == 0.0){
    //    comm_val = 0.0;
    //}else{
    //    double radius = (msg->linear.x) / (msg->angular.z);
    //    comm_val = std::atan(WheelBase / radius);
    //}
    
    req.value = -msg->angular.z;
    req.req_time = ros::Time::now();
    interface->queue_command(req);
    interface->issue_write_command();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "steer_controller_node");
    ros::NodeHandle n;

    //ROS_INFO("Inside Node");
    interface = new VescSteeringInterface(n);
    interface->set_operation_type(VescSteeringInterface::OperationType::MANUAL);
    interface->set_steering_limits(0.9, 0.0);
    interface->set_command_timeout(ros::Duration(1.0));
    
    interface->set_steering_gain(0.85).set_steering_offset(0.45);

    speed_pub = n.subscribe("cmd_vel", 10, speed_callback);
    //ROS_INFO("Sub to Topic");
    
    ros::spin();
    return 0;
}