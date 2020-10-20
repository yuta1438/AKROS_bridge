#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "set_position_zero_client");

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("set_position_zero");
    
    std_srvs::Empty srv;

    ROS_INFO("client called");
    client.call(srv);
    ROS_INFO("client finish");


    return 0;
}