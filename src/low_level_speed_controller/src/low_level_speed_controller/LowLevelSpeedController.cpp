#include "low_level_speed_controller/LowLevelSpeedController.h"
#include "low_level_speed_controller/VescSpeedGenerator.h"
#include "low_level_speed_controller/VescSpeedInterface.h"

LowLevelSpeedController ::LowLevelSpeedController() : node_handle("~"), current_speed(0), message_rate(0.1), goal_point_control_rate(0.1)
{
    ROS_INFO("Low Level Speed Controller Initialized");
}

LowLevelSpeedController ::LowLevelSpeedController(ros::NodeHandle &n) : current_speed(0), message_rate(0.1), goal_point_control_rate(0.1)
{
    node_handle = n;
    
    odom_topic_name = "/odometry/filtered";
    point_topic_name = "/current_position";
    control_msg_type = ControlMsgType::Twist;
    control_type = ControlType::Speed;

    use_odom_for_position = true;

    motor_cmd.req_type = SpeedCommandInterfaceBase::CommandRequest::RequestType::SPEED;
    motor_cmd.value = 0.0;
    goal_still_active = false;
    keepSendingCommands = false;

    if (!node_handle.getParam("control_topic_name", control_topic_name))
    {
        ROS_WARN("COULDNT FIND PARAM control_topic_name. USING /cmd_vel");
        control_topic_name = "/cmd_vel";
    }

     if (!node_handle.getParam("odom_topic_name", odom_topic_name))
    {
        ROS_WARN("COULDNT FIND PARAM odom_topic_name. USING /odometry/filtered");
    }

     if (!node_handle.getParam("point_topic_name", point_topic_name))
    {
        ROS_WARN("COULDNT FIND PARAM control_topic_name. USING /current_position");
    }

    odom_sub = node_handle.subscribe(odom_topic_name, 10, &LowLevelSpeedController::odom_callback, this);
}

LowLevelSpeedController ::
    LowLevelSpeedController(SpeedCommandGeneratorBase *com_gen, SpeedCommandInterfaceBase *s_int)
    : LowLevelSpeedController()
{
    commandGenerator = com_gen;
    speedInterface = s_int;
}

LowLevelSpeedController &LowLevelSpeedController::set_control_msg_type(ControlMsgType ctrl_type)
{

    this->control_msg_type = ctrl_type;

    switch (ctrl_type)
    {
        case ControlMsgType::Twist:
        {
            command_sub = node_handle.subscribe(control_topic_name, 10,
                                            &LowLevelSpeedController ::twist_callback, this);
            break;
        }
        

        case ControlMsgType::Ackermann:
        {
            command_sub = node_handle.subscribe(control_topic_name, 10,
                                            &LowLevelSpeedController ::ackermann_callback, this);
            break;
        }
        

        case ControlMsgType::AckermannStamped:
        {
            command_sub = node_handle.subscribe(control_topic_name, 10,
                                            &LowLevelSpeedController ::ackermann_stamped_callback, this);
            break;
        }
    }

    return *this;
}

LowLevelSpeedController &LowLevelSpeedController ::set_control_type(ControlType ctrl_type)
{

    this->control_type = ctrl_type;

    switch (ctrl_type)
    {
    case ControlType::Acceleration:
        commandGenerator->set_control_type(SpeedCommandGeneratorBase::ControlType::Acceleration);
        break;

    case ControlType::Speed:
        commandGenerator->set_control_type(SpeedCommandGeneratorBase::ControlType::Speed);
        break;
    }

    return *this;
}


void LowLevelSpeedController::send_msg(const ros::TimerEvent &e)
{
    //motor_cmd.req_time = ros::Time::now(); // Uncommenting causes last generated command to be latched.
    speedInterface->queue_command(motor_cmd);
    speedInterface->issue_write_command();
}

LowLevelSpeedController &LowLevelSpeedController::set_speed_generator(SpeedCommandGeneratorBase *gen)
{
    commandGenerator = gen;

    return *this;
}

LowLevelSpeedController &LowLevelSpeedController::set_message_send_rate(ros::Duration dur)
{
    message_rate = dur;
    message_timer.setPeriod(dur);

    return *this;
}

LowLevelSpeedController &LowLevelSpeedController::set_speed_interface(SpeedCommandInterfaceBase *base)
{
    speedInterface = base;

    return *this;
}

LowLevelSpeedController &LowLevelSpeedController ::set_continously_send_msg(bool send)
{
    keepSendingCommands = send;

    switch (send)
    {
    case true:
        message_timer = node_handle.createTimer(message_rate,
                                                &LowLevelSpeedController::send_msg, this);
        break;

    case false:
        message_timer.stop();
        break;
    }

    return *this;
}

void LowLevelSpeedController::twist_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    last_msg_twist = *msg;

    if(goal_still_active){
        ROS_WARN_NAMED("Low Level Controller","ALREADY GOING TO A GOAL POINT. DISCARDING SPEED REQUEST");
        return;
    }

    motor_cmd = commandGenerator->createSpeedCommand(last_msg_twist.linear.x);
    motor_cmd.req_time = ros::Time::now();

    //ROS_WARN("Speed Read from command generator: %f",commandGenerator->get_speed());

    if (!keepSendingCommands)
    {
        motor_cmd.req_time = ros::Time::now();
        speedInterface->queue_command(motor_cmd);
        speedInterface->issue_write_command();
    }
}

void LowLevelSpeedController::ackermann_callback(const ackermann_msgs::AckermannDrive::ConstPtr &msg){
    last_msg_ackermann = *msg;

    if(goal_still_active){
        ROS_WARN_NAMED("Low Level Controller","ALREADY GOING TO A GOAL POINT. DISCARDING SPEED REQUEST");
        return;
    }

    motor_cmd = commandGenerator->createSpeedCommand(last_msg_ackermann.speed);

    if(!keepSendingCommands){
        motor_cmd.req_time = ros::Time::now();
        speedInterface->queue_command(motor_cmd);
        speedInterface->issue_write_command();
    }

}

void LowLevelSpeedController::
    ackermann_stamped_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg){
    
    last_msg_ackermann_stamped = *msg;

    if(goal_still_active){
        ROS_WARN_NAMED("Low Level Controller","ALREADY GOING TO A GOAL POINT. DISCARDING SPEED REQUEST");
        return;
    }

    motor_cmd = commandGenerator->createSpeedCommand(last_msg_ackermann.speed);

    if(!keepSendingCommands){
        motor_cmd.req_time = last_msg_ackermann_stamped.header.stamp;
        speedInterface->queue_command(motor_cmd);
        speedInterface->issue_write_command();
    }
}

LowLevelSpeedController &LowLevelSpeedController::set_max_limits(double acc, double max_sp, double min_sp){
    speedInterface->set_max_limits(acc, max_sp, min_sp);
    commandGenerator->set_limits(acc, max_sp,min_sp);

    return *this;
}

LowLevelSpeedController &LowLevelSpeedController::set_use_odom_for_position(bool use){
    use_odom_for_position = use;

    switch(use){
        case true:
            point_sub.shutdown();
        break;

        case false:
            point_sub = node_handle.subscribe(point_topic_name,10, &LowLevelSpeedController::point_cb, this);
        break;
    }

    return *this;
}

void LowLevelSpeedController::point_cb(const geometry_msgs::Point::ConstPtr &msg){
    current_position = *msg;
    commandGenerator->set_last_point(current_position);
}

void LowLevelSpeedController::goal_point_callback(const ros::TimerEvent &e){
    if(distance_between_points(current_position,goal_point) < 0.05){
        goal_point_callback_timer.stop();
        commandGenerator->on_goal_reached();
        goal_still_active = false;
    }

    motor_cmd = commandGenerator->createSpeedCommand(goal_point);

    if(!keepSendingCommands){
        motor_cmd.req_time = ros::Time::now();
        speedInterface->queue_command(motor_cmd);
        speedInterface->issue_write_command();
    }
}

LowLevelSpeedController &LowLevelSpeedController::set_goal_point(geometry_msgs::Point &goal){
    commandGenerator->on_goal_initialize();

    goal_point = goal;
    goal_still_active = true;
    
    goal_point_callback_timer = node_handle.createTimer(goal_point_control_rate, 
        &LowLevelSpeedController::goal_point_callback, this);

    return *this;
}

LowLevelSpeedController &LowLevelSpeedController::set_goal_point_control_rate(ros::Duration new_rate){
    goal_point_control_rate = new_rate;
    goal_point_callback_timer.setPeriod(new_rate);

    return *this;
}

LowLevelSpeedController &LowLevelSpeedController::set_current_position(geometry_msgs::Point &pos){
    if(use_odom_for_position)
        return *this;

    current_position = pos;
    commandGenerator->set_last_point(pos);

    return *this;
}
