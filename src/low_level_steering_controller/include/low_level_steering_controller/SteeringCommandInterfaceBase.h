#pragma once

#include "ros/ros.h"

#include <queue>

class SteeringCommandInterfaceBase{
    public:
        SteeringCommandInterfaceBase(ros::NodeHandle &n);

        enum OperationType{ AUTOMATIC, MANUAL };

        struct SteerRequest {
            ros::Time req_time;
            double value;
        };

        void queue_command(struct SteerRequest command){ 
            command.value = ((command.value * steering_gain) + steering_offset);
            ROS_INFO("COMMAND RECEIVED %f", command.value); 
            command_queue.push(command); 
        }

        int return_remaining_command_count() const{ return command_queue.size(); }

        SteeringCommandInterfaceBase &set_steering_limits(double max_angle, double min_angle);

        SteeringCommandInterfaceBase &set_operation_type(OperationType type);
        SteeringCommandInterfaceBase &set_command_timeout(ros::Duration timeout_duration);
        SteeringCommandInterfaceBase &set_execution_rate(ros::Duration exec_rate);
        SteeringCommandInterfaceBase &set_steering_gain(double gain);
        SteeringCommandInterfaceBase &set_steering_offset(double offset);


        const OperationType &get_operation_type() const { return op_type; }
        const ros::Duration &get_command_timeout() const { return command_timeout; }
        double get_max_angle() const { return max_angle; }
        double get_min_angle() const { return min_angle; }
        double get_steering_gain() const { return steering_gain; }
        double get_steering_offset() const { return steering_offset; }
        
        void issue_write_command(){
            if(op_type == OperationType::AUTOMATIC){
                return;
            }

            if(command_queue.size()){
                SteerRequest comm = command_queue.front();
                
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

        virtual ~SteeringCommandInterfaceBase(){
            
        }

    private:
    protected:
        SteeringCommandInterfaceBase();
        virtual bool send_command(SteerRequest comm, bool manual = false) = 0;

        void automatic_callback(const ros::TimerEvent &event){
             if(command_queue.size()){
                SteerRequest comm = command_queue.front();

                if(!check_command_integrity(comm)){
                    command_queue.pop();
                    return;
                }

                send_command(comm, false);
                command_queue.pop();
                //ROS_INFO("COMMAND PROCESSED");
            }else{
             //   ROS_WARN("NO COMMAND LEFT TO SEND");
            }
        }

        virtual bool check_command_integrity(SteerRequest &command){
            if(abs(command.value) > max_angle){
                ROS_ERROR("STEER COMMAND IS LARGER THAN MAXIMUM VALUE %f", command.value);
                return false;
            }

            if(abs(command.value) < min_angle){
                ROS_ERROR("STEER COMMAND IS SMALLER THAN MINIMUM VALUE %f", command.value);
                return false;
            }

            if(!check_command_timestamp(command)){
                ROS_ERROR("COMMAND HAS TIMED OUT");
                return false;
            }


            return true;
        }

        inline bool check_command_timestamp(SteerRequest &x){
            return (ros::Time::now() - x.req_time) < command_timeout;
        }

        ros::NodeHandle node_handle;
        
        double max_angle;
        double min_angle;

        OperationType op_type;

        ros::Timer callback_timer;
        ros::Duration auto_execute_rate;

        ros::Duration command_timeout;

        std::queue<SteerRequest> command_queue;

        double steering_gain;
        double steering_offset;
};