/* ROS_mbed */
// PCからの情報(ROSMessage)をMotorに流す
// Motorからの返答をROSに流す

#include <AKROS_bridge.h>

int main(void){
    ros::NodeHandle nh;
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    AKROS_bridge akros(&nh);
    wait_ms(100);

    while(1){
        akros.publish();
        nh.spinOnce();
    }
}