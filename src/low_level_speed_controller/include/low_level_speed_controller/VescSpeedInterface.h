#pragma once

#include "low_level_speed_controller/SpeedCommandInterfaceBase.h"

typedef SpeedCommandInterfaceBase::CommandRequest CommandRequest;

class VescSpeedInterface : public SpeedCommandInterfaceBase{
    public:
        VescSpeedInterface();
        VescSpeedInterface(ros::NodeHandle &n);


    private:
        bool send_command(CommandRequest &cmd,bool manual);
        std::string topic_name;
        ros::Publisher data_publisher;

        ros::Publisher takeoff_publisher;
        std::string takeoff_topic_name;

        double takeoff_current;
};