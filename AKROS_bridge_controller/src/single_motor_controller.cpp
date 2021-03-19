// 単体モータの位置制御（P制御）
// その場で固まる指令
#include <ros/ros.h>
#include <AKROS_bridge_msgs/currentState.h>
#include <AKROS_bridge_msgs/motor_cmd.h>

#define JOINTNUM 1
static const double controller_frequency = 100.0;

ros::Publisher pub;
ros::ServiceClient currentState_client;

AKROS_bridge_msgs::motor_cmd cmd;
AKROS_bridge_msgs::currentState currentState_srv;

double qref, qref_old;
double q_init;


// 一定周期で呼び出される関数
void send(const ros::TimerEvent& e){
    for(int i=0; i<JOINTNUM; i++){
        cmd.motor[i].position = qref;
        cmd.motor[i].velocity = (qref - qref_old) * controller_frequency;
    }
    qref_old = qref;    // qrefベクトルの更新
    pub.publish(cmd); // 指令値をpublish
}


int main(int argc, char** argv){
    ros::init(argc, argv, "single_motor_controller");
    ros::NodeHandle nh;
    
    pub = nh.advertise<AKROS_bridge_msgs::motor_cmd>("motor_cmd", 1);
    currentState_client = nh.serviceClient<AKROS_bridge_msgs::currentState>("current_state");
    
    // rosparamの読み取り
    XmlRpc::XmlRpcValue rosparams;
    if(!nh.getParam("motor_list", rosparams)){
        ROS_ERROR("Failed to load rosparams");
    }
    
    cmd.motor.resize(JOINTNUM);
    int counter = 0;
    for(auto param_itr=rosparams.begin(); param_itr!=rosparams.end(); ++param_itr){
        ROS_INFO("1");
        cmd.motor[counter].CAN_ID = static_cast<int>(param_itr->second["can_id"]);
        ROS_INFO("2");
        cmd.motor[counter].Kp     = static_cast<double>(param_itr->second["Kp"]);
        ROS_INFO("3");
        cmd.motor[counter].Kd     = static_cast<double>(param_itr->second["Kd"]);
        ROS_INFO("4");
        cmd.motor[counter].effort = 0.0;
        counter++;
    }

    
    // 現在状態の読み取り
    if(currentState_client.call(currentState_srv)){
        for(int i=0; i<JOINTNUM; i++){
            q_init = currentState_srv.response.reply.motor[i].position;
        }
    }else{
        ROS_ERROR("Failed to get current state !");
    }


    // 現在の角度を目標値にセット
    // tweak_Controlで好きな位置に戻す
    qref = q_init;
    qref_old = q_init;

    ros::Timer timer = nh.createTimer(ros::Duration(1.0/controller_frequency), &send);

    ros::spin();
    return 0;
}