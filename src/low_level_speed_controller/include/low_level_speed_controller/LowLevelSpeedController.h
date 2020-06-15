#pragma once

#include <type_traits>
#include <memory>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include "low_level_speed_controller/SpeedCommandInterfaceBase.h"
#include "low_level_speed_controller/SpeedCommandGeneratorBase.h"

#include "math_utils.h"

/** This is a class for coordinating linear speed commands and motor commands output.
 * It has one object for each SpeedCommandInterfaceBase and SpeedCommandGeneratorBase derived classes.
 * Those classes do not listen to any topic. Only way to pass information to them is through this class.
 * Utilities are also included to modify underlying objects properties.

**/

class SpeedCommandInterfaceBase;
class SpeedCommandGeneratorBase;


class LowLevelSpeedController{
    public:
        
        //Type of the control message to the system. Only one control message type is active at a time.
        enum ControlMsgType { Twist, Ackermann, AckermannStamped };

        // Type of control that should be applied when in FeedForward speed control.
        // Acceleration interprets incoming control message as speed change.
        // Speed Control is simple basic speed control. 
        enum ControlType { Speed, Acceleration };
        //enum OperationMode { Manual, Automatic };

           
        

        // Constructor with a node handle. Node handle will be used for every timer, subscriber or publisher.
        LowLevelSpeedController(ros::NodeHandle &n);

        // Constructor which sets the internal class handles.
        LowLevelSpeedController(SpeedCommandGeneratorBase *com_gen, SpeedCommandInterfaceBase *speed_interface);
        
        // Switch between different Control Message Types.
        // Only one Control type will be listened. Before a call to this function,
        // No message will be processed.
        LowLevelSpeedController &set_control_msg_type(ControlMsgType ctrl_type);

        // Set Speed Generator Handler
        LowLevelSpeedController &set_speed_generator(SpeedCommandGeneratorBase *gen);
        
        // Set Speed Interface Handler
        LowLevelSpeedController &set_speed_interface(SpeedCommandInterfaceBase *interface);
        
        // Set acceleration max and min limits of speed generator and speed interface at the same time.
        LowLevelSpeedController &set_max_limits(double acc, double max_speed, double min_speed);

        // Switch between Speed or Acceleration Control.
        LowLevelSpeedController &set_control_type(ControlType type);

        // If this is true, last generated speed command continously gets sent to speed  Interface.
        LowLevelSpeedController &set_continously_send_msg(bool send);

        // Period of the message sending rate to the speed interface.
        LowLevelSpeedController &set_message_send_rate(ros::Duration duration);

        // Issue direct speed command value to the system.
        LowLevelSpeedController &send_direct_speed(double speed_value);

        // This is the rate of control command generation when going towards a goal point.
        LowLevelSpeedController &set_goal_point_control_rate(ros::Duration new_rate);

        // Set the point to go. 
        // When received a goal from this interface, direct speed commands are discarded.
        // Speed Generation happens at the function generateCommand(geometry_msgs::Point)
        // of SpeedGenerator Class.
        LowLevelSpeedController &set_goal_point(geometry_msgs::Point &goal_point);

        // If this is set, current position data will be fetched from incoming odom messages.
        LowLevelSpeedController &set_use_odom_for_position(bool use);

        // Set Current Position directly. Has no effect if set_use_odom_for_position(true) is called.
        LowLevelSpeedController &set_current_position(geometry_msgs::Point &pos);
        
        // Set if odom message will be used for speed measurement
        LowLevelSpeedController &set_use_odom_for_speed(bool use){
            use_odom_for_current_speed = use;

            return *this;
        }
        
        // Set the current linear speed directly. No effect if set_use_odom_for_speed(true) is called.
        LowLevelSpeedController &set_current_speed(double speed){
            if(use_odom_for_current_speed)
                return *this;

            current_speed = speed;
            speedInterface->set_current_speed(speed);
            commandGenerator->set_current_speed(speed);

            return *this;
        }

    protected:
        // Periodic event to send motor commands with a rate.
        void send_msg(const ros::TimerEvent &e);

        // Callback for Position Callback
        void point_cb(const geometry_msgs::Point::ConstPtr &msg);

        // This is the callback that starts the goal following behaviour.
        void goal_point_callback(const ros::TimerEvent &e);

        void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){ 
            last_odom_data = *msg;
            //ROS_INFO("Odom Received %f", msg->twist.twist.linear.x);
            
            if(use_odom_for_current_speed){
                current_speed = last_odom_data.twist.twist.linear.x;
                commandGenerator->set_current_speed(current_speed);
                speedInterface->set_current_speed(current_speed);
                commandGenerator->set_odom_data(last_odom_data);
            }

            if(use_odom_for_position){
                current_position = msg->pose.pose.position;
                commandGenerator->set_last_point(current_position);
                commandGenerator->set_odom_data(last_odom_data);
            }

        }
        ros::NodeHandle node_handle;
        ros::Subscriber command_sub;
        ros::Subscriber point_sub;
        ros::Subscriber odom_sub;

        ControlMsgType control_msg_type;
        ControlType control_type;
        
        SpeedCommandGeneratorBase *commandGenerator;
        SpeedCommandInterfaceBase *speedInterface;
        
        double current_speed;
        double calculated_command;
        
        SpeedCommandInterfaceBase::CommandRequest motor_cmd;
        
        ros::Timer message_timer;

        ros::Timer goal_point_callback_timer;
        ros::Duration goal_point_control_rate;

        std::string odom_topic_name;
        std::string point_topic_name;
        std::string control_topic_name;

        ros::Duration message_rate;
        bool keepSendingCommands;

        bool use_odom_for_current_speed;
        nav_msgs::Odometry last_odom_data;

        geometry_msgs::Twist last_msg_twist;
        ackermann_msgs::AckermannDrive last_msg_ackermann;
        ackermann_msgs::AckermannDriveStamped last_msg_ackermann_stamped;

        bool use_odom_for_position;
        geometry_msgs::Point current_position;

        geometry_msgs::Point goal_point;
        bool goal_still_active;

    private:
        LowLevelSpeedController();
        void twist_callback(const geometry_msgs::Twist::ConstPtr &msg);
        void ackermann_callback(const ackermann_msgs::AckermannDrive::ConstPtr &msg);
        void ackermann_stamped_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg);
        
};
