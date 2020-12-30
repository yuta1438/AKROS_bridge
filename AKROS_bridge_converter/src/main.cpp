// デジタル値とアナログ値をそれぞれ変換するノード

#include <ros/ros.h>
#include <AKROS_bridge_converter/converter.h>

#define ROS_Hz      100


int main(int argc, char** argv){
    ros::init(argc, argv, "motor_can_converter");
    ros::NodeHandle nh;
    ros::Rate loop_rate(ROS_Hz);

    AKROS_bridge_converter converter(nh);

    

    ROS_INFO("Converter Ready !");
    spinner.start();

    while(ros::ok()){
        loop_rate.sleep();
    }
    
    spinner.stop();
    ros::waitForShutdown();
    return 0;
}