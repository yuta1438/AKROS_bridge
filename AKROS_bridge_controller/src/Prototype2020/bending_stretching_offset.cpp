// 単脚用プログラム
// 矢状平面内でx軸方向にオフセットをもたせた屈伸運動(Bending & Stretching)

#include <iostream>
#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_controller/Prototype2020.h>  // ロボットに関するヘッダファイル
#include <AKROS_bridge_controller/Interpolator.h>   // choreonoidの補間ライブラリ
#include <AKROS_bridge_msgs/currentState.h>
#include <std_msgs/Float32.h>

#define JOINT_NUM   3

static const double control_frequency = 100.0;  // 制御周期[Hz]

// 屈伸開始時の脚先位置
static const double offset_x = 0.2;
static const double offset_z = -0.3;

static const double marginTime = 2.0;
static const double settingTime = 3.0;
static const double movingTime = 30.0;

static const double wave_frequency = 0.5;       // 脚先正弦波指令の周波数[Hz]
static const double amplitude = 0.05;           // 正弦波振幅[m]
static const double omega = 2*M_PI*wave_frequency;

Eigen::Vector2d pref, p_offset, p_init;
Eigen::VectorXd qref, qref_old;

ros::Publisher cmd_pub;
AKROS_bridge_msgs::motor_cmd cmd;

ros::Publisher z_pub;
std_msgs::Float32 z;

ros::ServiceClient currentState_client;
AKROS_bridge_msgs::currentState currentState_srv;


int motor_num;
bool initializeFlag = false;

cnoid::Interpolator<Eigen::Vector2d> joint_trajectory; // 関節空間での補間器


int main(int argc, char** argv){
    ros::init(argc, argv, "flexion_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(control_frequency);

    cmd_pub = nh.advertise<AKROS_bridge_msgs::motor_cmd>("motor_cmd", 1);
    z_pub = nh.advertise<std_msgs::Float32>("z_value", 1);  // for debug
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
    qref.resize(motor_num);
    qref_old.resize(motor_num);
    int phase = 0;
    int counter = 0;

    // 初期状態を取得
    Eigen::VectorXd q_init(motor_num);
    q_init = Eigen::VectorXd::Zero(motor_num);
    if(currentState_client.call(currentState_srv)){
        for(int i=0; i<3; i++){
            q_init[i] = currentState_srv.response.reply.motor[i].position;
            // std::cout << "q_init[" << i << "] : " << q_init[i] << std::endl; // debug
        }
    }else{
        ROS_ERROR("Failed to get current state !");
    }

    // 各種計算
    p_offset << offset_x, offset_z;   // 振動中心点
    Eigen::Vector2d q_initialize(2);
    bool isIKsucceeded;
    q_initialize = solve_sagittal_IK(p_offset, isIKsucceeded);
    ROS_ASSERT(!isIKsucceeded);
    if(isIKsucceeded)   ROS_ERROR("IK out of range");

    

    for(auto param_itr=rosparams.begin(); param_itr!=rosparams.end(); ++param_itr){
        cmd.motor[counter].CAN_ID = static_cast<int>(param_itr->second["can_id"]);
        cmd.motor[counter].Kp     = static_cast<double>(param_itr->second["Kp"]);
        cmd.motor[counter].Kd     = static_cast<double>(param_itr->second["Kd"]);
        cmd.motor[counter].effort = 0.0;
        counter++;
    }

    ROS_INFO("flexion controller start !");
    ros::Time t_start = ros::Time::now();

    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec() - marginTime; // 現在時刻

        // 待機
        if(phase == 0){
            qref = q_init;  // 
            if(current_time > 0.0){
                initializeFlag = false;
                phase = 1;
            }
        }

        // 初期位置からスタート地点へ移動
        else if(phase == 1){
            if(initializeFlag == false){
                ROS_INFO("phase 1");
                joint_trajectory.clear();
                joint_trajectory.appendSample(current_time, q_init.head<2>());
                joint_trajectory.appendSample(current_time+settingTime, q_initialize);
                joint_trajectory.update();
                initializeFlag = true;
            }

            qref = joint_trajectory.interpolate(current_time);

            if(current_time > joint_trajectory.domainUpper()){
                initializeFlag = false;
                phase = 2;
                break;
            }
        }

        for(int i=0; i<2; i++){
            cmd.motor[i].position = qref[i];
            cmd.motor[i].velocity = (qref[i] / qref_old[i]) / control_frequency;
            qref_old[i] = qref[i];
        }
        cmd_pub.publish(cmd);
        Eigen::Vector2d p_buff = solve_sagittal_FK(qref.head<2>());
        
        z.data = p_buff[1];
        z_pub.publish(z);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // キー入力待ち
    ROS_INFO("Enter 's' to start Bending & Stretching ...");
    char buf;
    std::cin >> buf;

    t_start = ros::Time::now(); // リスタート
    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec(); // 現在時刻
        // 屈伸
        if(phase == 2){
            if(initializeFlag == false){    // 各phaseの最初の一回だけ実行
                ROS_INFO("phase 2 : start flexion");
                pref[0] = p_offset[0];
                initializeFlag = true;
            }
            pref[1] = (p_offset[1] + amplitude) - amplitude * cos(omega * (current_time));
            z.data = pref[1];
            qref.head<2>() = solve_sagittal_IK(pref);

            if(current_time > movingTime){
                initializeFlag = false;
                ROS_INFO("controller finished !");
                break;
            }
        }

    

        for(int i=0; i<2; i++){
            cmd.motor[i].position = qref[i];
            cmd.motor[i].velocity = (qref[i] / qref_old[i]) / control_frequency;
            qref_old[i] = qref[i];
        }
        cmd.motor[2].position = -cmd.motor[0].position; // 符号に気をつける

        cmd_pub.publish(cmd);

        Eigen::Vector2d p_buff = solve_sagittal_FK(qref.head<2>());
        z.data = p_buff[1];
        z_pub.publish(z);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}