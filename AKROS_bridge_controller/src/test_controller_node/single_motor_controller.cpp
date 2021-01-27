// controller2: 正弦波状の位置指令を与える．

#include <iostream>
#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_cmd.h>


static const double endTime = 15.0; // [s]
static const double wave_frequency = 1.0;   // [Hz]
static const double control_frequency = 50.0;  // [Hz]
static const double amplitude = M_PI / 2;   //[rad]

ros::Publisher cmd_pub;
AKROS_bridge_msgs::motor_cmd cmd;


int main(int argc, char** argv){
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(control_frequency);

    cmd_pub = nh.advertise<AKROS_bridge_msgs::motor_cmd>("motor_cmd", 2);
    
    // 若干のdelayを入れないとCANメッセージが届かない！！！
    // 初回だけcan_motor_cmdが二回送信される...
    sleep(0.01);
    ROS_INFO("Motor Initialized");

    float q_init = 0;

    cmd.motor.resize(1);
    cmd.motor[0].CAN_ID = 2;
    cmd.motor[0].Kp = 1.0;
    cmd.motor[0].Kd = 3.0;
    cmd.motor[0].effort = 0;
    cmd.motor[0].velocity = 0;

    float omega = 2*M_PI*wave_frequency;

    ros::Time t_start = ros::Time::now();
    
    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec(); // 現在時刻

        cmd.motor[0].position = 2*M_PI;

        // 時間が来たらpublishをやめる
        if(current_time <= endTime){
            cmd_pub.publish(cmd);
        }else{
            ROS_INFO("control finished !");
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}