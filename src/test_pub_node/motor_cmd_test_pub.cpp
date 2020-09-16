#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "AKROS_bridge/motor_cmd.h"

ros::Publisher cmd_pub;
ros::Publisher enter_motor_control_mode_pub;
ros::Publisher exit_motor_control_mode_pub;
ros::Publisher reset_motor_position_pub;

std_msgs::UInt8 id;
AKROS_bridge::motor_cmd cmd;

int main(int argc, char** argv){
    ros::init(argc, argv, "motor_cmd_publisher");
    ros::NodeHandle nh;
    
    ros::Rate loop_rate(1); // 1Hz
    
    
    
    enter_motor_control_mode_pub = nh.advertise<std_msgs::UInt8>("enter_motor_control_mode", 1);
    exit_motor_control_mode_pub = nh.advertise<std_msgs::UInt8>("exit_motor_control_mode", 1);
    reset_motor_position_pub = nh.advertise<std_msgs::UInt8>("reset_motor_position", 1);

    cmd_pub = nh.advertise<AKROS_bridge::motor_cmd>("motor_cmd", 10);
    

    id.data = 1;

    // 若干のdelayを入れないとCANメッセージが届かない！！！
    // 初回だけcan_motor_cmdが二回送信される...
    sleep(3);
    enter_motor_control_mode_pub.publish(id);
    sleep(1);
    //exit_motor_control_mode_pub.publish(id);
    //sleep(1);
    //reset_motor_position_pub.publish(id);

    ROS_INFO("Motor Initialized");
    //ros::spinOnce();

    //sleep(1);
    cmd.id = 1;
    cmd.position = -M_PI;
    cmd.velocity = 0.0;
    cmd.Kp = 450.0;
    cmd.Kd = 4.8;
    cmd.torque = 0.0;
    

    while(ros::ok()){ 
        cmd_pub.publish(cmd);
        //ros::spinOnce();
        loop_rate.sleep();
    }
}