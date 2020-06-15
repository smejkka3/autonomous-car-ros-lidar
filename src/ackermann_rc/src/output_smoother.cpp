#include "ros/ros.h"
#include <string>
#include "std_msgs/Float64.h"


#define ALPHA_MOTOR 0.7
#define ALPHA_SERVO 0.7



ros::Subscriber unsmoothed_subs_motor;
ros::Publisher smoothed_pub_motor;

ros::Subscriber unsmoothed_subs_servo;
ros::Publisher smoothed_pub_servo;

double accumulated_values_motor;
double accumulated_values_servo;

std::string subs_topic_motor("/unsmoothed_motor_acc");
std::string pub_topic_motor("/smoothed_motor_acc");

std::string subs_topic_servo("/unsmoothed_servo_pos");
std::string pub_topic_servo("/smoothed_servo_pos");


void command_receive_callback_motor(const std_msgs::Float64::ConstPtr &msg){
    std_msgs::Float64 return_msg;

    accumulated_values_motor = (ALPHA_MOTOR * msg->data) + ((1 - ALPHA_MOTOR) * accumulated_values_motor);
    return_msg.data = accumulated_values_motor;

    smoothed_pub_motor.publish(return_msg);
}

void command_receive_callback_servo(const std_msgs::Float64::ConstPtr &msg){
    std_msgs::Float64 return_msg;

    accumulated_values_servo = (ALPHA_SERVO * msg->data) + ((1 - ALPHA_SERVO) * accumulated_values_servo);
    return_msg.data = accumulated_values_servo;

    smoothed_pub_motor.publish(return_msg);
}

int main(int argc,char **argv){
    ros::init(argc,argv,"smoother_node");
    ros::NodeHandle n;

    accumulated_values_motor = 0.0;
    accumulated_values_servo = 0.0;

    unsmoothed_subs_motor = n.subscribe(subs_topic_motor,10,command_receive_callback_motor);
    smoothed_pub_motor = n.advertise<std_msgs::Float64>(pub_topic_motor,10);

    unsmoothed_subs_motor = n.subscribe(subs_topic_servo,10,command_receive_callback_servo);
    smoothed_pub_motor = n.advertise<std_msgs::Float64>(pub_topic_servo,10);

    ros::spin();

    return 0;
}




