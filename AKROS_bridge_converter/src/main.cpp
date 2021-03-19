// デジタル値とアナログ値をそれぞれ変換するノード

#include <ros/ros.h>
#include <AKROS_bridge_converter/AKROS_bridge_converter.h>

#define ROS_Hz      100


int main(int argc, char** argv){
    ros::init(argc, argv, "motor_can_converter");
    ros::NodeHandle nh;
    ros::Rate loop_rate(ROS_Hz);

    AKROS_bridge_converter converter(&nh);

    ROS_INFO("Converter Ready !");

    while(ros::ok()){
        converter.publish_can_cmd();
        converter.publish_reply();
        loop_rate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}