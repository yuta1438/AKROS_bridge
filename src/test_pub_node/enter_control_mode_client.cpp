#include <ros/ros.h>
#include <AKROS_bridge/Initialize.h>

static const uint8_t motor_num = 1;

int main(int argc, char** argv){
    ros::init(argc, argv, "enter_control_mode_client");

    ros::NodeHandle nh;
    ros::ServiceClient client1 = nh.serviceClient<AKROS_bridge::Initialize>("/cmd/enter_control_mode");
    //ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>("exit_control_mode");

    AKROS_bridge::Initialize srv;
    srv.request.joint_num = motor_num;  // モータ個数を指定

    // Initialize

    ROS_INFO("client called");
    client1.call(srv);
    ROS_INFO("client finish");

    // モータの初期状態を表示
    /*
    for(uint8_t i=0; i<motor_num; i++){
        ROS_INFO("position %d : %f", i, srv.response.)
    }
    */
    return 0;
}