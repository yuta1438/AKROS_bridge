// controller2: 正弦波状の位置指令を与える．

#include <iostream>
#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_cmd_single.h>
#include <AKROS_bridge_msgs/motor_reply_single.h>
#include <AKROS_bridge_msgs/Initialize_single.h>


static const double endTime = 15.0; // [s]
static const double wave_frequency = 1.0;   // [Hz]
static const double control_frequency = 50.0;  // [Hz]
static const double amplitude = M_PI / 2;   //[rad]

ros::Publisher cmd_pub;
ros::ServiceClient initialize_client, finalize_client;

AKROS_bridge_msgs::motor_cmd_single cmd;
AKROS_bridge_msgs::Initialize_single initialize_srv, finalize_srv;


int main(int argc, char** argv){
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(control_frequency);

    cmd_pub = nh.advertise<AKROS_bridge_msgs::motor_cmd_single>("motor_cmd", 2);
    
    // 若干のdelayを入れないとCANメッセージが届かない！！！
    // 初回だけcan_motor_cmdが二回送信される...
    sleep(0.01);
    initialize_client.call(initialize_srv);
    ROS_INFO("Motor Initialized");
    
    //float q_init = initialize_srv.response.q;
    float q_init = 0;
    ROS_INFO("Initial position is %f", q_init);

    cmd.id = 0;
    cmd.Kp = 0.3;
    cmd.Kd = 0.1;
    cmd.torque = 0;
    //cmd.velocity = 0;

    float omega = 2*M_PI*wave_frequency;

    ros::Time t_start = ros::Time::now();
    
    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec(); // 現在時刻

        cmd.position = (amplitude + q_init) + amplitude * cos(omega*current_time - M_PI);
        cmd.velocity = -omega*amplitude*sin(omega*current_time-M_PI);

        // 時間が来たらpublishをやめる
        if(current_time <= endTime){
            cmd_pub.publish(cmd);
        }else{
            ROS_INFO("control finished !");
            finalize_client.call(finalize_srv);
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}