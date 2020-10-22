#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <Interpolator.h>
#include <AKROS_bridge/motor_cmd_single.h>
#include <AKROS_bridge/motor_reply_single.h>
#include <AKROS_bridge/Initialize_single.h>

using namespace std;
using namespace Eigen;
using namespace cnoid;

static const double endTime = 3.0;

ros::Publisher cmd_pub;
ros::ServiceClient client;
AKROS_bridge::motor_cmd_single cmd;
AKROS_bridge::Initialize_single srv;
Interpolator<Vector2d> interpolator;


int main(int argc, char** argv){
    ros::init(argc, argv, "motor_cmd_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    client = nh.serviceClient<AKROS_bridge::Initialize_single>("enter_control_mode");
    cmd_pub = nh.advertise<AKROS_bridge::motor_cmd_single>("motor_cmd", 10);
    
    // 若干のdelayを入れないとCANメッセージが届かない！！！
    // 初回だけcan_motor_cmdが二回送信される...
    sleep(1);
    client.call(srv);
    ROS_INFO("Motor Initialized");
    
    float q_init = srv.response.q;
    float q_end = q_init + 2*M_PI;

    ROS_INFO("Initial position is %f", q_init);

    interpolator.clear();
    interpolator.appendSample(0, q_init*Vector2d::Ones());
    interpolator.appendSample(endTime, q_end*Vector2d::Ones());
    interpolator.update();

    cmd.id = 0;
    cmd.Kp = 3.0;
    cmd.Kd = 1.0;
    cmd.velocity = 0;
    cmd.torque = 0;

    ROS_INFO("Ready...");
    sleep(3);
    char a;
    cin >> a;
    ROS_INFO("Go !");


    Vector2d temp;
    ros::Time t_start = ros::Time::now();
    
    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec(); // 現在時刻のチェック

        temp = interpolator.interpolate(current_time);  // 現在時刻での目標角度を計算
        float q_target = temp[0];

        cmd.position = q_target;    // 値を格納

        // 時間が来たらpublishをやめる
        if(current_time <= interpolator.domainUpper()){
            cmd_pub.publish(cmd);
            // ROS_INFO("current target: %f");
        }else{
            ROS_INFO("control finished !");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}