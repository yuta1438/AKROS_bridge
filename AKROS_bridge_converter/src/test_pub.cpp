#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_can.h>

#define MOTOR_NUM       2   // モータ個数
#define DATA_LENGTH     8   // データ個数

ros::Publisher pub;
AKROS_bridge_msgs::motor_can can_msgs;

int count = 0;
int main(int argc, char** argv){
    ros::init(argc, argv, "motor_can_pub");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2);

    pub = nh.advertise<AKROS_bridge_msgs::motor_can>("motor_can_msg", 1);

    can_msgs.motor.resize(2);

    for(uint8_t i=0; i<MOTOR_NUM; i++){
        can_msgs.motor[i].data.resize(DATA_LENGTH);
    }    

    while(ros::ok()){
        can_msgs.motor[0].data[0] = 0  + count;
        can_msgs.motor[0].data[1] = 1  + count;
        can_msgs.motor[0].data[2] = 2  + count;
        can_msgs.motor[0].data[3] = 3  + count;
        can_msgs.motor[0].data[4] = 4  + count;
        can_msgs.motor[0].data[5] = 5  + count;
        can_msgs.motor[0].data[6] = 6  + count;
        can_msgs.motor[0].data[7] = 7  + count;

        can_msgs.motor[1].data[0] = 8  + count;
        can_msgs.motor[1].data[1] = 9  + count;
        can_msgs.motor[1].data[2] = 10 + count;
        can_msgs.motor[1].data[3] = 11 + count;
        can_msgs.motor[1].data[4] = 12 + count;
        can_msgs.motor[1].data[5] = 13 + count;
        can_msgs.motor[1].data[6] = 14 + count;
        can_msgs.motor[1].data[7] = 15 + count;

        count++;
        
        pub.publish(can_msgs);
        loop_rate.sleep();
    }
}