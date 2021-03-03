// 複数モータ制御
// モックアップ制御用
// 継承クラスを作って最初の初期化部分をテンプレート化したい(毎回書くのが面倒なので...)

#include <iostream>
#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_cmd.h>

#define MOTOR_NUM  2

static const double endTime = 10.0; // [s]
static const double control_frequency = 100.0;  // [Hz]

static const double wave_frequency[2] = {1.0, 0.25};   // [Hz]
static const double amplitude[2] = {M_PI/4, M_PI/4};   //[rad]

double q_init[MOTOR_NUM];
double q_old[MOTOR_NUM];

ros::Publisher cmd_pub; // 指令値のPublisher
AKROS_bridge_msgs::motor_cmd cmd;    // Publish_msg

int main(int argc, char** argv){
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(control_frequency);

    cmd_pub = nh.advertise<AKROS_bridge_msgs::motor_cmd>("motor_cmd", 1);

    for(int i=0; i<MOTOR_NUM; i++){
        q_init[i] = 0;
        q_old[i] = 0;
    }

    float omega[2];
    omega[0] = 2*M_PI*wave_frequency[0];
    omega[1] = 2*M_PI*wave_frequency[1];

    // rosparamからCAN_ID，Kp, Kdを読み込む
    XmlRpc::XmlRpcValue params;
    nh.getParam("motor_list", params);

    cmd.motor.resize(params.size());
    int counter = 0;
    for(auto params_itr = params.begin(); params_itr != params.end(); ++params_itr){
        cmd.motor[counter].CAN_ID = static_cast<int>(params_itr->second["can_id"]);
        cmd.motor[counter].Kp     = static_cast<double>(params_itr->second["Kp"]);
        cmd.motor[counter].Kd     = static_cast<double>(params_itr->second["Kd"]);
        cmd.motor[counter].effort = 0.0;
        counter++;
    }

    ROS_INFO("controller start!");
    ros::Time t_start = ros::Time::now();
    
    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec(); // 現在時刻

        for(uint8_t i=0; i<MOTOR_NUM; i++){
            cmd.motor[i].position = (amplitude[i] + q_init[i]) + amplitude[i] * cos(omega[i]*current_time - M_PI);
            cmd.motor[i].velocity = (cmd.motor[i].position - q_old[i]) * control_frequency;
            q_old[i] = cmd.motor[i].position;
        }

        // 時間が来たらpublishをやめる
        if(current_time <= endTime){
            cmd_pub.publish(cmd);
        }else{
            ROS_INFO("control finished !");
            break;
        }
           
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}