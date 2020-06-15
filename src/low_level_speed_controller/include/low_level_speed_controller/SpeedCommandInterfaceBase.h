#pragma once
#include <queue>
#include <ros/ros.h>
#include "math_utils.h"

/* This is the abstract class definition for speed interface of the linear control system. It is responsible for
* taking a generated speed Command as struct CommandRequest and translates it into corresponding motor commands.
* Speed Limit testing, timestamp check and acceleration check is integrated. User should derive the class and only
* override Constructors and virtual void sendCommand(). This function takes a command that has been checked and class
* should translate the value into motor commands.
*/

class SpeedCommandInterfaceBase{
    public:
        
        SpeedCommandInterfaceBase(ros::NodeHandle &n);

        virtual ~SpeedCommandInterfaceBase(){}

        // Selecting operation Mode. Automatic means commands stored in a queue is popped periodically and executed.
        // Any manual execute request by issue_write_command is ignored. 
        enum OperationType{ AUTOMATIC, MANUAL };

        // struct for command requests
        struct CommandRequest{
            enum RequestType{ SPEED, BRAKE, TAKEOFF, ACCELERATION };
            double value;
            RequestType req_type;
            ros::Time req_time;
        };

        // Internally every command sent is stored in a queue. Insert a command into queue.
        // Value is transformed by value = value * gain + offset; 
        void queue_command(struct CommandRequest command){ 
            //ROS_INFO("COMMAND RECEIVED");
            double com_sign = sign(command.value);
            command.value = ((command.value * speed_gain) + (speed_offset * com_sign)) * (command.value != 0.0); 
            command_queue.push(command); 
        }

        // Get how many commands thats left on queue.
        int return_remaining_command_count() const{ return command_queue.size(); }

        // Setters for internal data.
        SpeedCommandInterfaceBase &set_max_limits(double acc,double max_speed,double min_speed);
        SpeedCommandInterfaceBase &set_operation_type(OperationType);
        SpeedCommandInterfaceBase &set_command_timeout(ros::Duration); // If a time this much is passed after the issue of the command, ignore it.
        SpeedCommandInterfaceBase &set_execution_rate(ros::Duration);
        SpeedCommandInterfaceBase &set_current_speed(double speed);
        SpeedCommandInterfaceBase &set_speed_gain(double gain);
        SpeedCommandInterfaceBase &set_speed_offset(double offset);

        void issue_write_command(){
            if(op_type == OperationType::AUTOMATIC){
                return;
            }

            if(command_queue.size()){
                CommandRequest comm = command_queue.front();
                
                if(!check_command_integrity(comm)){
                    command_queue.pop();
                    return;
                }

                send_command(comm, true);
                command_queue.pop();
                //ROS_INFO("COMMAND PROCESSED");
            }else{
                ROS_WARN("NO COMMAND LEFT TO SEND");
            }
        }

        // Getters for internal data
        const OperationType &get_operation_type() const { return op_type; }
        const ros::Duration &get_command_timeout() const { return command_timeout; }
        double get_max_acceleration() const { return max_acceleration; }
        double get_max_speed() const { return max_speed; }
        double get_min_speed() const { return min_speed; }
        double get_speed_offset() const { return speed_offset; }
        double get_speed_gain() const { return speed_gain; }

    private:
    protected:
        SpeedCommandInterfaceBase();
        void automatic_callback(const ros::TimerEvent &event){
             if(command_queue.size()){
                CommandRequest comm = command_queue.front();

                if(!check_command_integrity(comm)){
                    command_queue.pop();
                    return;
                }

                send_command(comm, false);
                command_queue.pop();
                //ROS_INFO("COMMAND PROCESSED");
            }else{
                ROS_WARN("NO COMMAND LEFT TO SEND");
            }
        }
        virtual bool send_command(struct CommandRequest &command, bool manual = false) = 0;

        double max_acceleration;
        double max_speed;
        double min_speed;
        double current_speed;
        
        ros::Duration command_timeout;
        ros::Duration auto_execute_rate;
        std::queue<struct CommandRequest> command_queue;
        
        OperationType op_type;
        ros::NodeHandle node_handle;
        ros::Timer callback_timer;

        double speed_gain;
        double speed_offset;

        virtual bool check_command_integrity(CommandRequest &command){
            if(abs(current_speed) > max_speed && command.req_type != CommandRequest::BRAKE){
                //ROS_ERROR("REJECTING COMMANDS BECAUSE SPEED IS TOO HIGH");
                return false;
            }


            if(abs(command.value) > max_speed){
                //ROS_ERROR("SPEED COMMAND IS LARGER THAN MAXIMUM VALUE %f", command.value);
                return false;
            }

            if(abs(command.value) < min_speed){
                //ROS_ERROR("SPEED COMMAND IS SMALLER THAN MINIMUM VALUE %f", command.value);
                return false;
            }

            if(abs(command.value - current_speed) > max_acceleration){
                //ROS_ERROR("SPEED DIFFERENCE IS LARGER THAN MAX ACCELERATION");
                return false;
            }

            if(!check_command_timestamp(command)){
                //ROS_ERROR("COMMAND HAS TIMED OUT");
                return false;
            }


            return true;
        }

        inline bool check_command_timestamp(CommandRequest &x){
            return (ros::Time::now() - x.req_time) < command_timeout;
        }
};
