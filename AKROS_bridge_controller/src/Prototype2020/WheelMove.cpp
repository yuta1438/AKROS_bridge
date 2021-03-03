// 単脚用プログラム
// 車輪による移動

#include <iostream>
#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_controller/Prototype2020.h>
#include <AKROS_bridge_controller/Interpolator.h>   // choreonoidの補間ライブラリ
#include <AKROS_bridge_msgs/currentState.h>
#include <std_msgs/Float32.h>

#define JOINT_NUM   3

static const double control_frequency = 100.0;  // 制御周期[Hz]

static const double marginTime = 1.0;       // 待機時間[s]
static const double settingTime = 2.0;      // initialPose遷移時間[s]

static const double movingTime = 1.0;       // 移動時間[s]
static const double movingDistance = 1.0;   // 目標移動距離[m]

ros::Publisher cmd_pub;
AKROS_bridge_msgs::motor_cmd cmd;


ros::ServiceClient currentState_client;
AKROS_bridge_msgs::currentState currentState_srv;

int motor_num;
bool initializeFlag = false;
Eigen::VectorXd qref, qref_old;


cnoid::Interpolator<Eigen::VectorXd> joint_trajectory;  // 関節空間での補間器
cnoid::Interpolator<Eigen::Vector2d> wheel_trajectory;


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
    
    qref.resize(JOINT_NUM);
    qref_old.resize(JOINT_NUM);
    qref = Eigen::VectorXd::Zero(JOINT_NUM);
    qref_old = Eigen::VectorXd::Zero(JOINT_NUM);

    // 初期状態を取得
    Eigen::VectorXd q_init(JOINT_NUM);
    q_init = Eigen::VectorXd::Zero(JOINT_NUM);

    Eigen::VectorXd q_target(JOINT_NUM);

    if(currentState_client.call(currentState_srv)){
        for(int i=0; i<JOINT_NUM; i++){
            q_init[i] = currentState_srv.response.reply.motor[i].position;
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

    cmd.motor[WHEEL].Kp = 0.0;

    // 各種計算
    Eigen::VectorXd q_initial(2);
    q_initial << deg2rad(initialPose[0]), deg2rad(initialPose[1]);
    
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

        // initialPoseからまずq = [10.0, -20.0][deg]の状態へ
        else if(phase == 1){
            if(!initializeFlag){
                joint_trajectory.clear();
                joint_trajectory.appendSample(current_time, q_init.head<2>());
                joint_trajectory.appendSample(current_time+settingTime, q_initial);
                joint_trajectory.update();
                initializeFlag = true;
            }

            qref.head<2>() = joint_trajectory.interpolate(current_time);

            if(current_time > joint_trajectory.domainUpper()){
                initializeFlag = false;
                phase = 2;
                break;
            }
        }

        for(int i=0; i<JOINT_NUM; i++){
            cmd.motor[i].position = qref[i];
            cmd.motor[i].velocity = (qref[i] - qref_old[i]) * control_frequency;
        }
        qref_old = qref;
        cmd_pub.publish(cmd);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("send s to start move ...");
    char buff;
    std::cin >> buff;

    ROS_INFO("start moving ...");


    // 車輪移動に関する計算
    t_start = ros::Time::now();

    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec(); // 現在時刻
        
        if(phase == 2){
            if(initializeFlag == false){
                wheel_trajectory.clear();
                wheel_trajectory.appendSample(current_time, Eigen::Vector2d::Zero());
                wheel_trajectory.appendSample(current_time+movingTime, (2*movingDistance/wheel_D)*Eigen::Vector2d::Ones());
                wheel_trajectory.update();
                initializeFlag = true;
            }

            Eigen::Vector2d q_wheel_buf = wheel_trajectory.interpolate(current_time);
            qref[WHEEL] = q_wheel_buf[0];
            
            if(current_time > wheel_trajectory.domainUpper()){
                initializeFlag = false;
                qref = qref_old;
                cmd.motor[WHEEL].velocity = 0.0;
                cmd_pub.publish(cmd);
                phase = 3;
                break;
            }
        }

        // 車輪は速度制御のみ，他は位置，速度を指令値として与える
        for(int i=0; i<JOINT_NUM; i++){
            cmd.motor[i].position = qref[i];
            cmd.motor[i].velocity = (qref[i] - qref_old[i]) * control_frequency;
        }
        cmd.motor[WHEEL].position = 0.0;
        cmd.motor[WHEEL].Kp = 0.0;  // 車輪は速度制御
        qref_old = qref;
        cmd_pub.publish(cmd);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}