#include "low_level_speed_controller/SixWheelSpeedInterface.h"

SixWheelSpeedInterface::SixWheelSpeedInterface(ros::NodeHandle &n) : SpeedCommandInterfaceBase(n)
{

    if (!node_handle.getParam("topic_name", topic_name))
    {
        ROS_WARN_NAMED("Speed Interface", "Did not receive topic_name param. Using /serial_communicator/motor_commands");
        topic_name = "/serial_communicator/motor_commands";
    }
    data_publisher = node_handle.advertise<sixwd_msgs::SixWheelCommand>(topic_name, 10, false);
    //takeoff_publisher = node_handle.advertise<std_msgs::Float64>(takeoff_topic_name, 10, false);
}

bool SixWheelSpeedInterface::send_command(CommandRequest &comm, bool manual)
{
    sixwd_msgs::SixWheelCommand msg;
    ROS_INFO("Msg Value %f", comm.value);

    switch (comm.req_type)
    {
    case CommandRequest::RequestType::SPEED:
    case CommandRequest::RequestType::ACCELERATION:
        msg.controltype = true;
        msg.linearspeed = comm.value;
        data_publisher.publish(msg);
        break;

    case CommandRequest::RequestType::TAKEOFF:
        //msg.data = takeoff_current * sign(comm.value);
        //takeoff_publisher.publish(msg);
        break;
    }
}
