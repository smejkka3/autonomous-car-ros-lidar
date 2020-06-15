#include "low_level_speed_controller/VescSpeedGenerator.h"



CommandRequest VescSpeedGenerator::createSpeedCommand(double input_setpoint){
    CommandRequest command;

    //ROS_INFO("Set Point received for speed: %f", input_setpoint);
    
    
    switch(control_type){
        case SpeedCommandGeneratorBase::ControlType::Acceleration:
        {
            input_setpoint = (input_setpoint > 0.0) ? 
                (clamp_value(input_setpoint, max_acc_limit, 0.0)) : 
                    (-clamp_value(-input_setpoint, max_acc_limit, 0.0));

            double setpoint = input_setpoint + current_speed;
            setpoint = (setpoint > 0.0) ? 
                (clamp_value(setpoint, max_linear_speed, min_linear_speed)) : 
                    (-clamp_value(-setpoint, max_linear_speed, min_linear_speed));

            command.value = setpoint;
            command.req_type = CommandRequest::RequestType::SPEED;
            break;
        }
            
        case SpeedCommandGeneratorBase::ControlType::Speed:
        {
            ROS_INFO("Speed Input Received: %f", input_setpoint);
            if(input_setpoint != 0.0){
                input_setpoint = (input_setpoint > 0.0) ? 
                    (clamp_value(input_setpoint, max_linear_speed, min_linear_speed)) : 
                        (-clamp_value(-input_setpoint, max_linear_speed, min_linear_speed));
                if(abs(current_speed) < takeoff_speed_limit && abs(input_setpoint) > takeoff_speed_limit && !has_took_off){
                    //ROS_INFO("Current Speed %f", current_speed);
                    still_executing_takeoff = true;
                    ROS_INFO("Takeoff Issued. Current Speed: %f", current_speed);
                }else{
                    command.value = input_setpoint;
                    command.req_type = CommandRequest::RequestType::SPEED;
                    still_executing_takeoff = false;
                    has_took_off = true;
                }
            
            }else{ // zero speed request
                command.req_type = CommandRequest::RequestType::SPEED;
                command.value = 0.0;
                still_executing_takeoff = false; // stop take-off if zero speed is requested
            }
            break;
        }
            
    }

    if(still_executing_takeoff){
        command.req_type = CommandRequest::RequestType::TAKEOFF;
        command.value = input_setpoint;
    }
    
    return command;
}



CommandRequest VescSpeedGenerator::createSpeedCommand(geometry_msgs::Point &goal){
    goal_point = goal;

    return last_request;
}

void VescSpeedGenerator::on_goal_initialize(){
    std_msgs::Bool msg;
    msg.data = true;
    enable_pub.publish(msg);
    ROS_INFO("Goal Point Received for Planning");
    state_update_timer = node_handle.createTimer(state_update_rate, &VescSpeedGenerator::update_states, this);
}

void VescSpeedGenerator::update_states(const ros::TimerEvent &e){
    std_msgs::Float64 msg;

    msg.data = distance_between_points(last_position, goal_point);

    state_pub.publish(msg);

    ROS_INFO("Distance to Goal is %f", msg.data);

    std_msgs::Float64 setpoint;
    setpoint.data = 0;
    setpoint_pub.publish(setpoint);
}


void VescSpeedGenerator::on_goal_reached(){
    std_msgs::Bool msg;
    msg.data = false;
    enable_pub.publish(msg);
    ROS_INFO("Linear Position Goal Has Been Reached");
}

VescSpeedGenerator::VescSpeedGenerator() : SpeedCommandGeneratorBase(){
    control_effort_topic = "/control_effort";
    setpoint_topic = "/setpoint";
    state_topic = "/state";
    enable_topic = "/pid_enable";

    state_update_rate = ros::Duration(0.05);
    state_update_timer.setPeriod(state_update_rate);
}


VescSpeedGenerator::VescSpeedGenerator(ros::NodeHandle &n) : SpeedCommandGeneratorBase(n){
    control_effort_topic = "/control_effort";
    setpoint_topic = "/setpoint";
    state_topic = "/state";
    enable_topic = "/pid_enable";

    state_update_rate = ros::Duration(0.05);
    state_update_timer.setPeriod(state_update_rate);

    takeoff_speed_limit = 0.2524;

    has_took_off = false;
    still_executing_takeoff = false;
    

    if (!node_handle.getParam("control_effort_topic", control_effort_topic))
    {
        ROS_WARN("COULDNT FIND PARAM control_effort_topic. USING /control_effort");
    }
    
    if (!node_handle.getParam("state_topic", state_topic))
    {
        ROS_WARN("COULDNT FIND PARAM state_topic. USING /state");
    }
    
    if (!node_handle.getParam("setpoint_topic", setpoint_topic))
    {
        ROS_WARN("COULDNT FIND PARAM setpoint_topic. USING /setpoint");
    }

    if (!node_handle.getParam("enable_topic", enable_topic))
    {
        ROS_WARN("COULDNT FIND PARAM enable_topic. USING /pid_enable");
    }

    if (!node_handle.getParam("takeoff_speed_limit", takeoff_speed_limit))
    {
        ROS_WARN("COULDNT FIND PARAM takeoff_speed_limit. USING 0.2524");
    }
    
    control_effort_sub = node_handle.subscribe(control_effort_topic, 100, &VescSpeedGenerator::control_effort_callback, this);
    state_pub = node_handle.advertise<std_msgs::Float64>(state_topic, 10, false);
    setpoint_pub = node_handle.advertise<std_msgs::Float64>(setpoint_topic, 10, false);
    enable_pub = node_handle.advertise<std_msgs::Bool>(enable_topic, 10, false);
}

void VescSpeedGenerator::control_effort_callback(const std_msgs::Float64::ConstPtr &msg){
    last_request.value = msg->data;
}
