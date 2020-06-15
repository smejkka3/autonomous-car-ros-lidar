#include "ros/ros.h"

#include "low_level_speed_controller/LowLevelSpeedController.h"
#include "low_level_speed_controller/VescSpeedInterface.h"
#include "low_level_speed_controller/VescSpeedGenerator.h"



LowLevelSpeedController *test;




int main(int argc,char **argv){
    ros::init(argc, argv, "low_level_controller");
    ros::NodeHandle n("~");
    
    VescSpeedInterface vesc(n);
    VescSpeedGenerator vesc_speed_gen(n);
    vesc.set_command_timeout(ros::Duration(1.0));
    vesc.set_operation_type(SpeedCommandInterfaceBase::AUTOMATIC);
    vesc.set_execution_rate(ros::Duration(1.0 / 50.0));


    vesc_speed_gen.set_limits(1000.0, 1.0, 0.42);
    vesc.set_max_limits(10000.0, 3000.0, 0.0);
    vesc.set_speed_offset(16.6);
    vesc.set_speed_gain(3485.3528);
    
    
    test = new LowLevelSpeedController(n);

    test->set_speed_interface(&vesc).set_speed_generator(&vesc_speed_gen)
                .set_control_msg_type(LowLevelSpeedController::ControlMsgType::Twist)
                    .set_control_type(LowLevelSpeedController::ControlType::Speed)
                        .set_continously_send_msg(true)
                            .set_message_send_rate(ros::Duration(1.0 / 50.0))
                                .set_use_odom_for_speed(true).set_use_odom_for_position(true);

    ros::spin();
    return 0;
}
