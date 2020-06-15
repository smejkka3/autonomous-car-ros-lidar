#include "low_level_steering_controller/SteeringCommandInterfaceBase.h"

SteeringCommandInterfaceBase::SteeringCommandInterfaceBase() : auto_execute_rate(1.0 / 50.0), node_handle("~"), command_timeout(1.0){
    ROS_INFO("Speed Interface Initialized");
    this->op_type = OperationType::MANUAL;
    this->max_angle = 0;
    this->min_angle = 0;
    this->steering_gain = 1.0;
    this->steering_offset = 0.0;
}

SteeringCommandInterfaceBase::SteeringCommandInterfaceBase(ros::NodeHandle &n) : SteeringCommandInterfaceBase(){
    node_handle = n;
    //callback_timer = node_handle.createTimer(ros::Rate(10.0), &SteeringCommandInterfaceBase::automatic_callback,this);
}

SteeringCommandInterfaceBase &SteeringCommandInterfaceBase::set_steering_gain(double gain){
    this->steering_gain = gain;

    return *this;
}

SteeringCommandInterfaceBase &SteeringCommandInterfaceBase::set_steering_offset(double offset){
    this->steering_offset = offset;

    return *this;
}

SteeringCommandInterfaceBase &SteeringCommandInterfaceBase::set_steering_limits(double max_ang, double min_ang){
    this->max_angle = max_ang;
    this->min_angle = min_ang;

    return *this;
}

SteeringCommandInterfaceBase &SteeringCommandInterfaceBase::set_operation_type(SteeringCommandInterfaceBase::OperationType t){
    this->op_type = t;

    if(t == SteeringCommandInterfaceBase::OperationType::AUTOMATIC){
        callback_timer = node_handle.createTimer(auto_execute_rate, &SteeringCommandInterfaceBase::automatic_callback,this);
    }else{
        callback_timer.stop();
    }

    return *this;
}

SteeringCommandInterfaceBase& SteeringCommandInterfaceBase::set_command_timeout(ros::Duration new_dur){
    this->command_timeout = new_dur;

    return *this;
}

SteeringCommandInterfaceBase& SteeringCommandInterfaceBase::set_execution_rate(ros::Duration new_rate){
    this->auto_execute_rate = new_rate;

    callback_timer.setPeriod(this->auto_execute_rate);
    
    return *this;
}