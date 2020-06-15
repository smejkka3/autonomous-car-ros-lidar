/****
* ROS Node to receive rc_msgs from Arduino and then transform it to AckermannDriveStamped for publishing
*
****/

#include<ros/ros.h>
#include<rc_msgs/RCControlMsg.h>
#include<ackermann_msgs/AckermannDriveStamped.h>
#include<ackermann_msgs/AckermannDrive.h>
#include<sixwd_msgs/SixWheelCommand.h>
#include<string>

// #define DEBUG 1




inline bool near(float a, float b,float distance){
    return (abs(a - b) < distance);
}

inline float sgn(float a){
    if(a > 0.0f){
        return 1.0f;
    }

    return -1.0f;
}

class RC_Driver{
public:

    RC_Driver(){
        n = ros::NodeHandle("~");

        if(n.getParam("max_throttle_output",max_throttle_output) && n.getParam("max_steering_output",max_steering_output) 
        && n.getParam("min_throttle_output",min_throttle_output) && n.getParam("min_steering_output",min_steering_output) 
        && n.getParam("middle_throttle_output",middle_throttle_output) && n.getParam("middle_steering_output",middle_steering_output) 
        && n.getParam("max_car_linear_speed",max_car_linear_speed) && n.getParam("max_car_angle",max_car_angle)
        && n.getParam("min_car_linear_speed",min_car_linear_speed) && n.getParam("min_car_angle",min_car_angle)){
            
        }else{
            ROS_ERROR("THERE ARE MISSING PARAMETERS PLEASE SUPPLY REQUIRED PARAMETERS. EXITING...");
            return;
        }

        
        rc_command_topic = std::string("/rc_command");
        drive_topic = std::string("/rc_to_6w");


        
        n.getParam("rc_topic",rc_command_topic);
        n.getParam("drive_topic",drive_topic);


        rc_command_sub = n.subscribe(rc_command_topic,10,&RC_Driver::rc_command_callback_unique_command,this);
        rc_to_six_wheel_pub = n.advertise<sixwd_msgs::SixWheelCommand>(drive_topic,10);
        

        ROS_INFO("INITIALIZATION COMPLETE. STARTING MESSAGE TRANSFORMING");
    }


    void rc_command_callback_unique_command(const rc_msgs::RCControlMsg::ConstPtr &msg){
        //Messages are only zero if RC is closed. Discard Values;
        if(msg->steering_cmd == 0 || msg->throttle_cmd == 0){
            ROS_WARN("RC IS CLOSED. OPEN RC TO CONTROL");
            return;
        }

        //MIT Racecar Lightweight 2-D Simulator Updates poses on regular intervals.
        //If same command is being published from RC, then there is no need to publish this since
        //Pose still get updated.

        //Check if the old command is the same with new command
        //ROS_INFO("RC command Unique callback is received");

        

        float throttle_percentage = 0.0f;
        float steer_percentage = 0.0f;

        if(msg->steering_cmd > middle_steering_output){
            steer_percentage = (msg->steering_cmd - middle_steering_output)/(max_steering_output - middle_steering_output);
        }else{
            steer_percentage = -(middle_steering_output - msg->steering_cmd)/(middle_steering_output - min_steering_output);
        }

        if(msg->throttle_cmd > middle_throttle_output){
            throttle_percentage = (msg->throttle_cmd - middle_throttle_output)/(max_throttle_output - middle_throttle_output);
        }else{
            throttle_percentage = -(middle_throttle_output - msg->throttle_cmd)/(middle_throttle_output - min_throttle_output);
        }

            
       
            //ROS_INFO("Calculating Output Throttle Percentage:%f Steering Percentage:%f",throttle_percentage,steer_percentage);
            

        float throttle_interval = max_car_linear_speed - min_car_linear_speed;
        float steering_interval = max_car_angle - min_car_angle;

        float speed_output = sgn(throttle_percentage) * (min_car_linear_speed + (sgn(throttle_percentage) * throttle_interval * throttle_percentage));
        float steer_output = sgn(steer_percentage) * (min_car_angle + (sgn(steer_percentage) * steering_interval) * steer_percentage);

        #ifdef DEBUG
            ROS_INFO("Outputs To Motors Linear:%f Angular %f",speed_output,steer_output);
        #endif
            //ROS_INFO("Speed Outputs: %f %f",speed_output,steer_output);

            /*if(near(steer_output * sign(steer_output),0.01,0.05)){
                steer_output = 0;
            }

            if(near(speed_output * sign(speed_output),0.01,0.05)){
                speed_output = 0;
            }*/

        if(sgn(steer_percentage) * steer_percentage < 0.05){
            steer_output = 0;
        }

        if(sgn(throttle_percentage) * throttle_percentage< 0.05){
            speed_output = 0;
        }


        drive_output.controltype = true;
        drive_output.linearspeed = speed_output;
        drive_output.angle= steer_output;
        
        rc_to_six_wheel_pub.publish(drive_output);
    }

private:
    ros::Subscriber rc_command_sub;
    ros::Publisher rc_to_six_wheel_pub;
    ros::NodeHandle n;

    //Store old Command Value
    sixwd_msgs::SixWheelCommand drive_output;
    rc_msgs::RCControlMsg command_value;

    //Store Max and Min RC Values and the middle point
    float max_throttle_output;
    float max_steering_output;

    float min_throttle_output;
    float min_steering_output;

    float middle_throttle_output;
    float middle_steering_output;

    //Store Max and Min Values of the Car.
    //Output will be based on percentage value and these values is
    //necessary to output correct values;

    float max_car_linear_speed;
    float max_car_angle;

    float min_car_linear_speed;
    float min_car_angle;

    std::string drive_topic;
    std::string rc_command_topic;
};


int main(int argc,char **argv)
{
    ros::init(argc,argv,"/rc_to_six_wheel");
    RC_Driver driver;

    ros::spin();
    return 0;
}
