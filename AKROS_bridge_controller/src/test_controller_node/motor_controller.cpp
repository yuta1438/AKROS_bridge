// 複数モータ制御

#include <iostream>
#include <ros/ros.h>

#include <AKROS_bridge_msgs/motor_cmd.h>

#include <AKROS_bridge_msgs/currentState.h>
#define MOTOR_NUM  2    // rosparamで読み取れるようにしたい

static const double endTime = 15.0; // [s]
static const double control_frequency = 50.0;  // [Hz]

static const double wave_frequency[2] = {1.0, 0.25};   // [Hz]
static const double amplitude[2] = {M_PI/4, M_PI/4};   //[rad]

double q_init[MOTOR_NUM];
double q_old[MOTOR_NUM];

ros::Publisher cmd_pub; // 指令値のPublisher
ros::ServiceClient current_state_client;


AKROS_bridge_msgs::motor_cmd cmd;    // Publish_msg
AKROS_bridge_msgs::currentState current_state_srv;  // モータ状態確認用のservice client


int main(int argc, char** argv){
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(control_frequency);

    current_state_client = nh.serviceClient<AKROS_bridge_msgs::currentState>("current_state");
    cmd_pub = nh.advertise<AKROS_bridge_msgs::motor_cmd>("motor_cmd", 1);
    cmd.motor.resize(MOTOR_NUM);


    ROS_INFO("Loading current position...");
    // 動かす前に現在位置を取得

    for(int id=1; id<=MOTOR_NUM; id++){
        current_state_srv.request.CAN_ID = id;  // CAN_IDなので
        if(current_state_client.call(current_state_srv)){
            if(current_state_srv.response.success){
                q_init[id] = current_state_srv.response.reply.position;
            }
        }
    }

    ROS_INFO("Finish loading current position...");
    float omega[2];
    omega[0] = 2*M_PI*wave_frequency[0];
    omega[1] = 2*M_PI*wave_frequency[1];

    // CAN_IDs
    cmd.motor[0].CAN_ID = 1;
    cmd.motor[1].CAN_ID = 2;

    cmd.motor[0].Kp = 5.0;
    cmd.motor[1].Kp = 20.0;
    cmd.motor[0].Kd = 0.5;
    cmd.motor[1].Kd = 0.5;
    cmd.motor[0].effort = 0.0;
    cmd.motor[1].effort = 0.0;

    ROS_INFO("controller start!");
    ros::Time t_start = ros::Time::now();
    
    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec(); // 現在時刻

        cmd.motor[0].position = (amplitude[0] + q_init[0]) + amplitude[0] * cos(omega[0]*current_time - M_PI);
        cmd.motor[1].position = (amplitude[1] + q_init[1]) + amplitude[1] * cos(omega[1]*current_time - M_PI);
        
        cmd.motor[0].velocity = -omega[0]*amplitude[0]*sin(omega[0]*current_time-M_PI);
        cmd.motor[1].velocity = -omega[1]*amplitude[1]*sin(omega[1]*current_time-M_PI);


        // 時間が来たらpublishをやめる
        if(current_time <= endTime){
            cmd_pub.publish(cmd);
        }else{
            ROS_INFO("control finished !");
        }
           
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}