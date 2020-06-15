/****
* ROS Node to receive rc_msgs from Arduino and then transform it to AckermannDriveStamped for publishing
*
****/

#include<ros/ros.h>
#include<rc_msgs/RCControlMsg.h>
#include<ackermann_msgs/AckermannDriveStamped.h>
#include<ackermann_msgs/AckermannDrive.h>
#include<string>


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

        drive_topic = std::string("/drive");
        rc_command_topic = std::string("/rc_command");

        unique_command = n.param("unique_command",true);
        stamped_output = n.param("stamped_output",true);

        n.getParam("drive_topic",drive_topic);
        n.getParam("rc_topic",rc_command_topic);

        seq_count = 0;

        if(unique_command){
            rc_command_sub = n.subscribe(rc_command_topic,10,&RC_Driver::rc_command_callback_unique_command,this);
        }
        
        //TODO Implement non unique_command
        if(stamped_output){
            ackermann_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic,10);
        }else{
            ackermann_pub = n.advertise<ackermann_msgs::AckermannDrive>(drive_topic,10);
        }

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


        if(stamped_output){
            drive_output.drive.speed = speed_output;
            drive_output.drive.steering_angle = steer_output;
            drive_output.header.seq = seq_count++;
            drive_output.header.stamp = ros::Time::now();
            ackermann_pub.publish(drive_output);
        }else{
            drive_output_unstamped.speed = speed_output;
            drive_output_unstamped.steering_angle = steer_output;
            ackermann_pub.publish(drive_output_unstamped);
        }

        command_value.throttle_cmd = msg->throttle_cmd;
        command_value.steering_cmd = msg->steering_cmd;
        
    }

private:
    ros::Subscriber rc_command_sub;
    ros::Publisher ackermann_pub;
    ros::NodeHandle n;
    //seq_count for drive message stamp
    uint32_t seq_count;

    //Store old Command Value
    rc_msgs::RCControlMsg command_value;
    ackermann_msgs::AckermannDriveStamped drive_output;
    ackermann_msgs::AckermannDrive drive_output_unstamped;

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

    //If the simulation or other systems update their values on message callback
    //And not with regular intervals, then publish a message every time a new command arrives
    //If update has regular intervals, then only output message when there is a unique message
    bool unique_command;


    //Choose between Stamped or Unstamped Message
    bool stamped_output;


};










int main(int argc,char **argv){
    ros::init(argc,argv,"/rc_drive");
    RC_Driver driver;

    ros::spin();
    return 0;
}
