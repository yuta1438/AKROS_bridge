#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <Interpolator.h>
#include <std_srvs/Empty.h>
#include <AKROS_bridge/motor_cmd_single.h>
#include <AKROS_bridge/motor_reply_single.h>
#include <AKROS_bridge/Initialize_single.h>

using namespace std;
using namespace Eigen;
using namespace cnoid;

static const double endTime = 3.0;
static const double extraTime = 5.0;
static const double control_Hz = 100.0;

ros::Publisher cmd_pub;
ros::ServiceClient initialize_client, finalize_client;
AKROS_bridge::motor_cmd_single cmd;
AKROS_bridge::Initialize_single initialize_srv, finalize_srv;
Interpolator<Vector2d> interpolator;


int main(int argc, char** argv){
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(control_Hz);

    initialize_client = nh.serviceClient<AKROS_bridge::Initialize_single>("enter_control_mode");
    finalize_client = nh.serviceClient<AKROS_bridge::Initialize_single>("exit_control_mode");
    cmd_pub = nh.advertise<AKROS_bridge::motor_cmd_single>("motor_cmd", 2);
    
    // 若干のdelayを入れないとCANメッセージが届かない！！！
    // 初回だけcan_motor_cmdが二回送信される...
    sleep(0.1);
    initialize_client.call(initialize_srv);
    ROS_INFO("Motor Initialized");
    
    float q_init = M_PI_2;
    float q_end = 0;

    ROS_INFO("Initial position is %f", initialize_srv.response.q);
    
    interpolator.clear();
    interpolator.appendSample(0, q_init*Vector2d::Ones());
    interpolator.appendSample(endTime, q_end*Vector2d::Ones());
    interpolator.update();
    
    cmd.id = 0;
    cmd.Kp = 1.0;
    cmd.Kd = 0.5;
    cmd.velocity = 0;
    cmd.torque = 0;
    
    float dq_ref = 0;
    float q_old = 0;

    Vector2d temp;
    ros::Time t_start = ros::Time::now();
    
    
    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec(); // 現在時刻のチェック

        temp = interpolator.interpolate(current_time);  // 現在時刻での目標角度を計算
        float q_target = temp[0];

        // 値を計算
        // 位置
        cmd.position = q_target;
        // 速度
        //cmd.velocity = (q_target - q_old)/(1.0 / control_Hz);
        cmd.velocity = 0;
        // 時間が来たらpublishをやめる
        if(current_time <= interpolator.domainUpper() + extraTime){
            cmd_pub.publish(cmd);
            // ROS_INFO("current target: %f");
        }else{
            ROS_INFO("control finished !");
            finalize_client.call(finalize_srv);
            break;
        }
        q_old = q_target;
        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}