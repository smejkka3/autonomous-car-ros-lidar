#include "low_level_speed_controller/SpeedCommandGeneratorBase.h"
#include "ros/ros.h"

SpeedCommandGeneratorBase::SpeedCommandGeneratorBase() : node_handle("~"){
    ROS_INFO("Speed Command Generator Initialized");
}

SpeedCommandGeneratorBase::SpeedCommandGeneratorBase(ros::NodeHandle &n) : SpeedCommandGeneratorBase(){
    node_handle = n;
}

SpeedCommandGeneratorBase &SpeedCommandGeneratorBase::set_limits(double m_acc,double max_speed,double min_speed){
    this->max_acc_limit = m_acc;
    this->max_linear_speed = max_speed;
    this->min_linear_speed = min_speed;

    return *this;
}
