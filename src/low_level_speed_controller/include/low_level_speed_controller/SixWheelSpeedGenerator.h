#pragma once 


#include "low_level_speed_controller/SpeedCommandGeneratorBase.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"



class SixWheelSpeedGenerator : public SpeedCommandGeneratorBase{
    public:
        SixWheelSpeedGenerator(ros::NodeHandle &n);
        CommandRequest createSpeedCommand(double speed_setpoint);
        CommandRequest createSpeedCommand(geometry_msgs::Point &nav_goal);
        void on_goal_reached();
        void on_goal_initialize();
        void update_states(const ros::TimerEvent &e);
        ~SixWheelSpeedGenerator(){}

    private:
        SixWheelSpeedGenerator();
        void control_effort_callback(const std_msgs::Float64::ConstPtr &msg);

        ros::Subscriber control_effort_sub;
        ros::Publisher state_pub;
        ros::Publisher setpoint_pub;
        ros::Publisher enable_pub;

        std::string control_effort_topic;
        std::string state_topic;
        std::string setpoint_topic;
        std::string enable_topic;

        ros::Timer state_update_timer;
        ros::Duration state_update_rate;

        double takeoff_speed_limit;
};