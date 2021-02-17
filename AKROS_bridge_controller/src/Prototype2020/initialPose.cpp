// 単脚用プログラム
// q = [15.0, -30.0]T[deg] (initialPose) にするコントローラ
// 何かしらの動作をさせる前にこのコントローラでinitialPoseにすること！

#include <iostream>
#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_controller/Prototype2020.h>
#include <AKROS_bridge_controller/Interpolator.h>   // choreonoidの補間ライブラリ
#include <AKROS_bridge_msgs/currentState.h>
#include <std_msgs/Float32.h>

#define JOINT_NUM   3

static const double control_frequency = 100.0;  // 制御周期[Hz]
static const double marginTime = 1.0;
static const double settingTime = 3.0;

static const double q_target_deg[JOINT_NUM] = {15.0f, -30.0f, 0.0f};   // 基準ポーズ


ros::Publisher cmd_pub;
AKROS_bridge_msgs::motor_cmd cmd;

ros::ServiceClient currentState_client;
AKROS_bridge_msgs::currentState currentState_srv;

int motor_num;
bool initializeFlag = false;
Eigen::VectorXd qref, qref_old;

cnoid::Interpolator<Eigen::VectorXd> q_trajectory; // 関節空間での補間器


int main(int argc, char** argv){
    ros::init(argc, argv, "flexion_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(control_frequency);

    cmd_pub = nh.advertise<AKROS_bridge_msgs::motor_cmd>("motor_cmd", 1);
    currentState_client = nh.serviceClient<AKROS_bridge_msgs::currentState>("current_state");

    // rosparamからCAN_ID，Kp, Kdを読み込む
    XmlRpc::XmlRpcValue rosparams;
    nh.getParam("motor_list", rosparams);
    motor_num = rosparams.size();

    // モータ個数が違う場合は中止
    if(motor_num != JOINT_NUM){
        ROS_ERROR("the number of motor is not the same as controller");
        ROS_BREAK();
    }

    cmd.motor.resize(motor_num);

    int phase = 0;
    int counter = 0;
    
    qref = Eigen::VectorXd::Zero(JOINT_NUM);
    qref_old = Eigen::VectorXd::Zero(JOINT_NUM);

    // 初期状態を取得
    Eigen::VectorXd q_init(JOINT_NUM);
    q_init = Eigen::VectorXd::Zero(JOINT_NUM);

    Eigen::VectorXd q_target(JOINT_NUM);

    if(currentState_client.call(currentState_srv)){
        for(int i=0; i<JOINT_NUM; i++){
            q_target[i] = deg2rad(q_target_deg[i]);
            q_init[i] = currentState_srv.response.reply.motor[i].position;
            // std::cout << "q_init[" << i << "] : " << q_init[i] << std::endl; // debug
        }
        qref_old = q_init;
    }else{
        ROS_ERROR("Failed to get current state !");
    }


    for(auto param_itr=rosparams.begin(); param_itr!=rosparams.end(); ++param_itr){
        cmd.motor[counter].CAN_ID = static_cast<int>(param_itr->second["can_id"]);
        cmd.motor[counter].Kp     = static_cast<double>(param_itr->second["Kp"]);
        cmd.motor[counter].Kd     = static_cast<double>(param_itr->second["Kd"]);
        cmd.motor[counter].effort = 0.0;
        counter++;
    }

    ROS_INFO("Transition to basic pose ...");
    ros::Time t_start = ros::Time::now();

    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec() - marginTime; // 現在時刻

        // 待機
        if(phase == 0){
            qref = q_init;
            if(current_time > 0.0){
                initializeFlag = false;
                phase = 1;
            }
        }

        // 初期位置から基準Poseへ遷移
        else if(phase == 1){
            if(initializeFlag == false){
                q_trajectory.clear();
                q_trajectory.appendSample(current_time, q_init);
                q_trajectory.appendSample(current_time+settingTime, q_target);
                q_trajectory.update();
                initializeFlag = true;
            }

            qref = q_trajectory.interpolate(current_time);

            if(current_time > q_trajectory.domainUpper()){
                initializeFlag = false;
                break;
            }
        }
        for(int i=0; i<JOINT_NUM; i++){
            cmd.motor[i].position = qref[i];
            cmd.motor[i].velocity = (qref[i] / qref_old[i]) / control_frequency;
        }
        qref_old = qref;
        cmd_pub.publish(cmd);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}