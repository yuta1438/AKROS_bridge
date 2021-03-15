// 単体モータの位置制御（P制御）

#include <ros/ros.h>
#include <AKROS_bridge_msgs/currentState.h>
#include <AKROS_bridge_msgs/motor_cmd.h>

#define JOINTNUM 1

ros::Publisher pub;
ros::ServiceClient currentState_client;

AKROS_bridge_msgs::motor_cmd cmd;
AKROS_bridge_msgs::currentState currentState_srv;

int main(int argc, char** argv){
    ros::init(argc, argv, "single_motor_controller");
    ros::NodeHandle nh;

    // rosparamの読み取り
    XmlRpc::XmlRpcValue rosparams;
    if(!nh.getParam("motor_list", rosparams)){
        ROS_ERROR("Failed to load rosparams");
    }

    int counter = 0;
    for(auto param_itr=rosparams.begin(); param_itr!=rosparams.end(); ++param_itr){
        cmd.motor[counter].CAN_ID = static_cast<int>(param_itr->second["can_id"]);
        cmd.motor[counter].Kp     = static_cast<double>(param_itr->second["Kp"]);
        cmd.motor[counter].Kd     = static_cast<double>(param_itr->second["Kd"]);
        cmd.motor[counter].effort = 0.0;
        counter++;
    }


    if(currentState_client.call(currentState_srv)){
        for(int i=0; i<JOINTNUM; i++){
            q_[i] = currentState_srv.response.reply.motor[i].position;
        }
    }else{
        ROS_ERROR("Failed to get current state !");
    }


}