// controller2: 正弦波状の位置指令を与える．

#include <iostream>
#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_cmd.h>

static const double marginTime = 3.0;       // [s]
static const double accelTime = 1.0;        // [s]
static const double constantTime = 10.0;    // [s]
static const double decelTime = 1.0;        // [s]

static const double max_dq = 2*M_PI; // [rad/s]
static const double control_frequency = 100;    // [Hz]

bool control_flag = true;

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

    float dq_init = 0.0;
    float dq_ref = 0.0;


    cmd.motor.resize(1);
    cmd.motor[0].CAN_ID = 1;
    cmd.motor[0].Kp = 0.0;
    cmd.motor[0].Kd = 3.0;
    cmd.motor[0].position = 0;
    cmd.motor[0].effort = 0;

    int phase = 0;

    ros::Time t_start = ros::Time::now();
    
    while(ros::ok() && control_flag){
        double current_time = (ros::Time::now() - t_start).toSec() - marginTime; // 現在時刻

        // Margin Time
        if(phase == 0){
            if(current_time >= 0){
                phase = 1;
                ROS_INFO("Accelerating ...");
            }
        }
        // acceleration
        else if(phase == 1){
            dq_ref = max_dq * current_time / accelTime;

            if(current_time >= accelTime){
                phase = 2;
                ROS_INFO("Keep constant speed ...");
            }
        }

        // steady
        else if(phase == 2){
            dq_ref = max_dq;

            if(current_time >= accelTime+constantTime){
                phase = 3;
                ROS_INFO("Decelerating ...");
            }
        }

        // Deceleration
        else if(phase == 3){
            dq_ref = max_dq - (max_dq*(current_time - (accelTime+constantTime)/decelTime));
            
            if(current_time >= accelTime + constantTime + decelTime){
                ROS_INFO("Control Finish !");
                control_flag = false;
            }
        }
        else{
            dq_ref = 0.0;
        }

        cmd.motor[0].velocity = dq_ref;
        cmd_pub.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}