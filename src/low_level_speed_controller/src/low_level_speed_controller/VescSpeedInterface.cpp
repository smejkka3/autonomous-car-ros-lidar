#include "low_level_speed_controller/VescSpeedInterface.h"
#include "std_msgs/Float64.h"

VescSpeedInterface::VescSpeedInterface() : SpeedCommandInterfaceBase(){
    takeoff_topic_name = "/commands/motor/current";
    data_publisher = node_handle.advertise<std_msgs::Float64>(topic_name,10,false);
}

VescSpeedInterface::VescSpeedInterface(ros::NodeHandle &n) : SpeedCommandInterfaceBase(n){
    if(!node_handle.getParam("topic_name",topic_name)){
        ROS_WARN_NAMED("Speed Interface","Did not receive topic_name param. Using /commands/motor/speed");
        topic_name = "/commands/motor/speed";
    }

    if(!node_handle.getParam("takeoff_topic_name",takeoff_topic_name)){
        ROS_WARN_NAMED("Speed Interface","Did not receive takeoff_topic_name param. Using /commands/motor/current");
        takeoff_topic_name = "/commands/motor/current";
    }

    if(!node_handle.getParam("takeoff_current", takeoff_current)){
        ROS_WARN_NAMED("Speed Interface","Did not receive takeoff_current param. Using 20.0");
        takeoff_current = 20.0;
    }

    takeoff_publisher = node_handle.advertise<std_msgs::Float64>(takeoff_topic_name, 10, false);
    data_publisher = node_handle.advertise<std_msgs::Float64>(topic_name,10,false);
}


bool VescSpeedInterface::send_command(CommandRequest &comm, bool manual = false){
    std_msgs::Float64 msg;

    switch(comm.req_type){
        case CommandRequest::RequestType::SPEED:
        case CommandRequest::RequestType::ACCELERATION:
            msg.data = comm.value;
            data_publisher.publish(msg);
        break;

        case CommandRequest::RequestType::TAKEOFF:
            msg.data = takeoff_current * sign(comm.value);
            takeoff_publisher.publish(msg);
        break;
    }

    return true;
}