/* 
 * Publishes RC Commands as A custom package.
 * Requires rc_msgs package to be installed with rosserial_client
 * Please Refer to README for installation.
 */

#include<ros.h>
#include<std_msgs/Int32.h>
#include<rc_msgs/RCControlMsg.h>

#define THROTTLE_INT_PIN 3
#define STEERING_INT_PIN 2


volatile int pwm_value_throttle = 0;
volatile int prev_time_throttle = 0;

volatile int pwm_value_steering = 0;
volatile int prev_time_steering = 0;

//String throttle_string = "Throttle Duty Cycle:";
//String steering_string = "Steering Duty Cycle:";

ros::NodeHandle nh;
//std_msgs::Int32 throttle_value;
//std_msgs::Int32 steering_value;
rc_msgs::RCControlMsg combined_msg;

//ros::Publisher throttle_publisher("/throttle_publisher",&throttle_value);
//ros::Publisher steering_publisher("/steering_publisher",&steering_value);
ros::Publisher combined_command_publisher("/rc_command",&combined_msg);

//ros::Subscriber<geometry_msgs::PoseArray> s("poses",messageCb);

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  //nh.subscribe(s);
  //nh.advertise(throttle_publisher);
  //nh.advertise(steering_publisher);
  nh.advertise(combined_command_publisher);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_INT_PIN), rising_throttle, RISING);
  attachInterrupt(digitalPinToInterrupt(STEERING_INT_PIN),rising_steering, RISING);
}

void loop()
{  
  //throttle_value.data = pwm_value_throttle;
  //throttle_publisher.publish(&throttle_value);

  //steering_value.data = pwm_value_steering;
  //steering_publisher.publish(&steering_value);
  combined_msg.steering_cmd = pwm_value_steering;
  combined_msg.throttle_cmd = pwm_value_throttle;

  combined_command_publisher.publish(&combined_msg);

  nh.spinOnce();
  delay(10);
}


void rising_throttle() {
  attachInterrupt(digitalPinToInterrupt(THROTTLE_INT_PIN), falling_throttle, FALLING);
  prev_time_throttle = micros();
}
 
void falling_throttle() {
  attachInterrupt(digitalPinToInterrupt(THROTTLE_INT_PIN), rising_throttle, RISING);
  pwm_value_throttle = micros()- prev_time_throttle;
  //Serial.println(pwm_value);
}


void rising_steering(){
  attachInterrupt(digitalPinToInterrupt(STEERING_INT_PIN), falling_steering, FALLING);
  prev_time_steering = micros();
}

void falling_steering(){
  attachInterrupt(digitalPinToInterrupt(STEERING_INT_PIN), rising_steering, RISING);
  pwm_value_steering = micros() - prev_time_steering;
}