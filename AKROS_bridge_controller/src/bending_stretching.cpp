// 単脚用プログラム
// 矢状平面内での屈伸運動(Bending & Stretching)

#include <iostream>
#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_controller/Prototype2020.h>
#include <AKROS_bridge_controller/Interpolator.h>   // choreonoidの補間ライブラリ
#include <AKROS_bridge_msgs/currentState.h>
#include <std_msgs/Float32.h>

static const double control_frequency = 100.0;  // 制御周期[Hz]

static const double marginTime = 2.0;
static const double settingTime = 3.0;
static const double movingTime = 30.0;

static const double wave_frequency = 1.5;       // 脚先正弦波指令の周波数[Hz]
// static const double amplitude = 0.1;           // 正弦波振幅[m]
static const double omega = 2*M_PI*wave_frequency;

static const double q_extention_deg[2] = {15.0f, -30.0f};   // 一番Kneeを伸ばすポーズ
static const double q_flexion_deg[2] = {60.0f, -120.0f};    // 一番Kneeを曲げるポーズ
Eigen::Vector2d pref, p_offset;
Eigen::Vector2d qref, qref_old;

ros::Publisher cmd_pub;
AKROS_bridge_msgs::motor_cmd cmd;

ros::Publisher z_pub;
std_msgs::Float32 z;

ros::ServiceClient currentState_client;
AKROS_bridge_msgs::currentState currentState_srv;


int motor_num;
bool initializeFlag = false;

cnoid::Interpolator<Eigen::Vector2d> q_trajectory; // 関節空間での補間器


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
    cmd.motor.resize(motor_num);

    int phase = 0;
    int counter = 0;
    qref = Eigen::Vector2d::Zero();
    qref_old = Eigen::Vector2d::Zero();

    // 初期状態を取得
    Eigen::VectorXd q_init(3);
    q_init = Eigen::VectorXd::Zero(3);
    if(currentState_client.call(currentState_srv)){
        for(int i=0; i<3; i++){
            q_init[i] = -1 * currentState_srv.response.reply.motor[i].position;
            std::cout << "q_init[" << i << "] : " << q_init[i] << std::endl;
        }
    }else{
        ROS_ERROR("Failed to get current state !");
    }

    // 各種計算
    Eigen::Vector2d q_extension(deg2rad(q_extention_deg[0]), deg2rad(q_extention_deg[1]));
    Eigen::Vector2d q_flexion(deg2rad(q_flexion_deg[0]), deg2rad(q_flexion_deg[1]));
    Eigen::Vector2d p_extension = solve_sagittal_FK(q_extension);
    Eigen::Vector2d p_flexion = solve_sagittal_FK(q_flexion);

    // std::cout << "p_extention is \n" << p_extension << std::endl;
    // std::cout << "p_flexion is \n" << p_flexion << std::endl;

    p_offset = (p_extension + p_flexion) / 2.0;
    double amplitude = abs((p_extension[1] - p_flexion[1]) / 2.0);

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
                q_trajectory.clear();
                q_trajectory.appendSample(current_time, q_init.head<2>());
                q_trajectory.appendSample(current_time+settingTime, q_extension);
                q_trajectory.update();
                initializeFlag = true;
            }

            qref = q_trajectory.interpolate(current_time);

            if(current_time > q_trajectory.domainUpper()){
                initializeFlag = false;
                phase = 2;
                break;
            }
        }
        for(int i=0; i<2; i++){
            cmd.motor[i].position = -qref[i];
            cmd.motor[i].velocity = -(qref[i] / qref_old[i]) / control_frequency;
            qref_old[i] = qref[i];
        }
        cmd_pub.publish(cmd);

        Eigen::Vector2d p_buff = solve_sagittal_FK(qref);
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
        double current_time = (ros::Time::now() - t_start).toSec() - marginTime; // 現在時刻
        // 屈伸
        if(phase == 2){
            if(initializeFlag == false){    // 各phaseの最初の一回だけ実行
                pref[0] = p_offset[0];
                initializeFlag = true;
            }
            pref[1] = p_offset[1] - amplitude * cos(omega * (current_time));
            z.data = pref[1];
            qref = solve_sagittal_IK(pref);

            

            if(current_time > movingTime){
                initializeFlag = false;
                phase = 3;
            }
        }

        // 終了
        else if(phase == 3){
            if(initializeFlag == false){
                Eigen::Vector2d q_last(qref[0], qref[1]);
                q_trajectory.clear();
                q_trajectory.appendSample(current_time, q_last);
                q_trajectory.appendSample(current_time + 3.0, q_extension);
                q_trajectory.update();
                initializeFlag = true;
            }
            
            qref = q_trajectory.interpolate(current_time);

            if(current_time > q_trajectory.domainUpper()){
                ROS_INFO("controller finished !");
                break;
            }
        }

        for(int i=0; i<2; i++){
            cmd.motor[i].position = -qref[i];
            cmd.motor[i].velocity = -(qref[i] / qref_old[i]) / control_frequency;
            qref_old[i] = qref[i];
        }
        cmd.motor[2].position = -cmd.motor[0].position;
        cmd_pub.publish(cmd);

        Eigen::Vector2d p_buff = solve_sagittal_FK(qref);
        z.data = p_buff[1];
        z_pub.publish(z);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}