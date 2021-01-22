// 複数モータ制御
#include <iostream>
#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_msgs/currentState.h>

#define MOTOR_NUM  3    // rosparamで読み取れるようにしたい

ros::Publisher cmd_pub; // 指令値のPublisher
ros::ServiceClient current_state_client;
AKROS_bridge_msgs::motor_cmd cmd;    // Publish_msg
AKROS_bridge_msgs::currentState current_state_srv;  // モータ状態確認用のservice client

int main(int argc, char** argv){
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    cmd_pub = nh.advertise<AKROS_bridge_msgs::motor_cmd>("motor_cmd", 1);
    cmd.motor.resize(MOTOR_NUM);

    // CAN_IDs
    cmd.motor[0].CAN_ID = 1;    // Hip
    cmd.motor[1].CAN_ID = 2;    // Knee
    cmd.motor[2].CAN_ID = 3;    // Wheel

    cmd.motor[0].Kp = 5.0;
    cmd.motor[1].Kp = 20.0;
    cmd.motor[2].Kp = 20.0;

    cmd.motor[0].Kd = 0.5;
    cmd.motor[1].Kd = 0.5;
    cmd.motor[2].Kd = 0.5;

    cmd.motor[0].velocity = 0.0;
    cmd.motor[1].velocity = 0.0;
    cmd.motor[2].velocity = 0.0;

    cmd.motor[0].effort = 0.0;
    cmd.motor[1].effort = 0.0;
    cmd.motor[2].effort = 0.0;


    ROS_INFO("controller start!");
    ros::Time t_start = ros::Time::now();
    
    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec(); // 現在時刻

        for(uint8_t i=0; i<MOTOR_NUM; i++){
            cmd.motor[i].position = 0.0;
        }

        // 時間が来たらpublishをやめる
        if(current_time <= 1.0){
            cmd_pub.publish(cmd);
        }else{
            ROS_INFO("control finished !");
            break;
        }
           
        ros::spinOnce();
    }
    return 0;
}