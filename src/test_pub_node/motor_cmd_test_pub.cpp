#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "AKROS_bridge/motor_cmd.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>

ros::Publisher cmd_pub;
ros::Publisher enter_motor_control_mode_pub;
ros::Publisher exit_motor_control_mode_pub;
ros::Publisher reset_motor_position_pub;
ros::Time t_start;
std_msgs::UInt8 id;
AKROS_bridge::motor_cmd cmd;

static const double Amp = M_PI;
static const double Freq = 1.0;

// タイマ割り込み関数
void timer_callback(const ros::TimerEvent& e){
    cmd.position = Amp * sin(2 * M_PI * Freq * (ros::Time::now() - t_start).toSec());
    cmd_pub.publish(cmd);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "motor_cmd_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

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
    cmd.position = 0.0;
    cmd.velocity = 0.0;
    cmd.Kp = 450.0;
    cmd.Kd = 4.8;
    cmd.torque = 0.0;
    
    t_start = ros::Time::now();

    ros::Timer timer = nh.createTimer(ros::Duration(0.01), timer_callback);
    ros::spin();
    return 0;
}