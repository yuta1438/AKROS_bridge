#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "test_client");

    ros::NodeHandle nh;
    ros::ServiceClient client1 = nh.serviceClient<std_srvs::Empty>("enter_control_mode");
    //ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>("exit_control_mode");

    std_srvs::Empty srv;

    ROS_INFO("client 1 called");
    client1.call(srv);
    ROS_INFO("client 1 finish");

    
    //ROS_INFO("client 2 called");
    //client2.call(srv);
    //ROS_INFO("client 2 finish");
    

    return 0;
}