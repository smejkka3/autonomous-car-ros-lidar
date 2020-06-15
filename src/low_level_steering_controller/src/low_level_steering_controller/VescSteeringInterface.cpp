#include "low_level_steering_controller/VescSteeringInterface.h"

VescSteeringInterface::VescSteeringInterface() : SteeringCommandInterfaceBase(){

}

VescSteeringInterface::VescSteeringInterface(ros::NodeHandle &n){
    node_handle = n;

    if(!node_handle.getParam("steer_topic_name",steer_topic_name)){
        ROS_WARN("Did not receive steer_topic_name param. Using /commands/servo/position");
        steer_topic_name = "/commands/servo/position";
    }

    
    steer_data_publisher = node_handle.advertise<std_msgs::Float64>(steer_topic_name, 10, false);
}


bool VescSteeringInterface::send_command(SteerRequest com, bool manual){
    std_msgs::Float64 msg;
    msg.data = (com.value > 0.0) ? (com.value) : (0.0);

    steer_data_publisher.publish(msg);

    return true;
}