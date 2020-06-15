#include "low_level_speed_controller/SixWheelSpeedGenerator.h"


CommandRequest SixWheelSpeedGenerator::createSpeedCommand(double input_setpoint)
{
    CommandRequest command;

    ROS_INFO("Set Point received for speed: %f", input_setpoint);

    input_setpoint = ((input_setpoint > 0.0) ? 
        (clamp_value(input_setpoint, max_linear_speed, min_linear_speed)) : 
            (-clamp_value(-input_setpoint, max_linear_speed, min_linear_speed))) * (input_setpoint != 0.0);

    command.value = input_setpoint;
    command.req_type = CommandRequest::RequestType::SPEED;

    return command;
}

CommandRequest SixWheelSpeedGenerator::createSpeedCommand(geometry_msgs::Point &goal) {}
void SixWheelSpeedGenerator::on_goal_initialize() {}
void SixWheelSpeedGenerator::on_goal_reached() {}
SixWheelSpeedGenerator::SixWheelSpeedGenerator() : SpeedCommandGeneratorBase() {}
SixWheelSpeedGenerator::SixWheelSpeedGenerator(ros::NodeHandle &n) : SpeedCommandGeneratorBase(n) {}
void SixWheelSpeedGenerator::control_effort_callback(const std_msgs::Float64::ConstPtr &msg) {}
