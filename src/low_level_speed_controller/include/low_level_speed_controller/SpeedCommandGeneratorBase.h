#pragma once

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"

#include "low_level_speed_controller/SpeedCommandInterfaceBase.h"
#include "math_utils.h"

/* This is abstract class definition for speed generation. It receives current speed and input setpoint via
* LowLevelSpeedController class and then generates an CommandRequest for Speed Interface.
* It can generate a speed command for a commanded velocity and it also has an interface for tracking a goal point
* linearly. User should override constructors and four functions that can be seen below.
*/

typedef SpeedCommandInterfaceBase::CommandRequest CommandRequest;

class SpeedCommandGeneratorBase{
    public:
        SpeedCommandGeneratorBase(ros::NodeHandle &n);

        virtual ~SpeedCommandGeneratorBase(){}

        // Select the control type.
        enum ControlType{Speed, Acceleration};

        // Setters for internal data
        SpeedCommandGeneratorBase &set_limits(double acc, double max, double min);

        SpeedCommandGeneratorBase &set_control_type(ControlType type){
            control_type = type;
            
            return *this;
        }

        SpeedCommandGeneratorBase &set_odom_data(nav_msgs::Odometry &msg){
            last_odom_msg = msg;
            last_position = last_odom_msg.pose.pose.position;

            return *this;
        }

        SpeedCommandGeneratorBase &set_last_point(geometry_msgs::Point &data){
            this->last_position = data;

            return *this;
        }

        SpeedCommandGeneratorBase &set_current_speed(double speed){
            this->current_speed = speed;
            //ROS_INFO("Received speed %f",speed);

            return *this;
        }

        
        //Getters for internal data
        double get_max_acceleration() const { return max_acc_limit; }
        double get_min_speed() const { return min_linear_speed; }
        double get_max_linear_speed() const { return max_linear_speed; }
        double get_speed() const { return current_speed; }

        // Function signatures to override. This is  the part that user should implement.
        
        // This function is called when a goal is sent to LowLevelSpeedController. It should set up everything
        // needed to control the car to the goal point.
        virtual void on_goal_initialize() = 0;

        // This function is called after LowLevelSpeedController decides that it has reached the goal. Serves as a 
        // cleanup function after goal has been reached.
        virtual void on_goal_reached() = 0;

        // This function is called from LowLevelSpeedController in a periodic manner to receive commands.
        // Output should contain the commands to move the robot to goal. 
        // It should handle takeoff and braking behaviour if needed
        virtual CommandRequest createSpeedCommand(geometry_msgs::Point &goal_point) = 0;
        
        // This function is used when speed has been requested without a goal point. This is basically for
        // translating setpoint value to motor command. It should handle takeoff and braking behaviour if needed
        virtual CommandRequest createSpeedCommand(double speed_setpoint) = 0;

    private:
    protected:
        SpeedCommandGeneratorBase();

        ros::NodeHandle node_handle;

        CommandRequest last_request;
        
        ControlType control_type;

        std::string odom_topic_name;
        
        double max_acc_limit;
        double max_linear_speed;
        double min_linear_speed;
        
        double current_speed;

        nav_msgs::Odometry last_odom_msg;
        geometry_msgs::Point last_position;

        geometry_msgs::Point goal_point;
};