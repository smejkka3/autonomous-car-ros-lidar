#pragma once

#include "low_level_steering_controller/SteeringCommandInterfaceBase.h"
#include "std_msgs/Float64.h"
#include "sixwd_msgs/SixWheelCommand.h"
#include "sixwd_msgs/SixWheelInfo.h"

class SixWheelSteeringInterface : public SteeringCommandInterfaceBase{
    public:
        
        SixWheelSteeringInterface(ros::NodeHandle &n);

        bool send_command(SteerRequest com, bool manual = false);
        void set_speed(double speed){
            output_speed = speed;
        }

    private:
        SixWheelSteeringInterface();
        ros::Publisher steer_data_publisher;
        std::string steer_topic_name;
        double output_speed;
};
