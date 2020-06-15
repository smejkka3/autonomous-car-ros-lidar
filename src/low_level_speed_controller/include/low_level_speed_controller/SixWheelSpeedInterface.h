#pragma once

#include "low_level_speed_controller/SpeedCommandInterfaceBase.h"
#include "std_msgs/Float64.h"
#include "sixwd_msgs/SixWheelCommand.h"
#include "sixwd_msgs/SixWheelInfo.h"


typedef SpeedCommandInterfaceBase::CommandRequest CommandRequest;

class SixWheelSpeedInterface : public SpeedCommandInterfaceBase{
    public:
        SixWheelSpeedInterface(ros::NodeHandle &n);


    private:
        SixWheelSpeedInterface();
        bool send_command(CommandRequest &cmd,bool manual);
        std::string topic_name;
        ros::Publisher data_publisher;
        ros::Publisher takeoff_publisher;
        std::string takeoff_topic_name;

        double takeoff_current;
};