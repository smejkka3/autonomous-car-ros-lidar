#pragma once

#include "low_level_steering_controller/SteeringCommandInterfaceBase.h"
#include "std_msgs/Float64.h"

class VescSteeringInterface : public SteeringCommandInterfaceBase{
    public:
        
        VescSteeringInterface(ros::NodeHandle &n);

        bool send_command(SteerRequest com, bool manual = false);

    private:
        VescSteeringInterface();
        ros::Publisher steer_data_publisher;
        std::string steer_topic_name;
};
