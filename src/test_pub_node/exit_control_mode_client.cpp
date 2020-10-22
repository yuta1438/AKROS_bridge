#include <ros/ros.h>
#include <AKROS_bridge/Initialize_single.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "exit_control_mode_client");

    ros::NodeHandle nh;
    ros::ServiceClient client1 = nh.serviceClient<AKROS_bridge::Initialize_single>("exit_control_mode");
    //ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>("exit_control_mode");

    AKROS_bridge::Initialize_single srv;


    // Initialize
    ROS_INFO("client called");
    client1.call(srv);
    ROS_INFO("client finish");

    ROS_INFO("current pos: %f", srv.response.q);
    ROS_INFO("current vel: %f", srv.response.dq);
    ROS_INFO("current tau: %f", srv.response.tau);

    return 0;
}