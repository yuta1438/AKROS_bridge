/* F303K8にデータを流す */
#include <ros/ros.h>
#include "std_msgs/UInt8MultiArray.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "can_motor_reply_pub");
    ros::NodeHandle nh;
    ros::Publisher can_reply_pub = nh.advertise<std_msgs::UInt8MultiArray>("can_motor_reply", 10);
    ros::Rate loop_rate(1); // 1Hz
    
    std_msgs::UInt8MultiArray can_reply;
    can_reply.data.resize(6);
    
    while(ros::ok()){ 
        can_reply.data[0] = 0x77;
        can_reply.data[1] = 0x88;
        can_reply.data[2] = 0x99;
        can_reply.data[3] = 0xaa;
        can_reply.data[4] = 0xbb;
        can_reply.data[5] = 0xcc;
        

        can_reply_pub.publish(can_reply);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}