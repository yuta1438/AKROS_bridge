#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "exit_control_mode_client");

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("exit_control_mode");
    
    std_srvs::Empty srv;

    ROS_INFO("client called");
    client.call(srv);
    ROS_INFO("client finish");


    return 0;
}