/* ROS_mbed */
// PCからの情報(ROSMessage)をMotorに流す
// Motorからの返答をROSに流す
// 指令値，現在地は一旦マイコン内に格納することにする


// #include <mbed.h>
#include <AKROS_bridge.h>
AKROS_bridge *akros;

int main(void){
    ros::NodeHandle nh;
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    akros = new AKROS_bridge(&nh);    // インスタンス生成．NodeHandleのアドレスを渡す．
    wait_ms(100);

    while(1){
        // akros->loop();
        wait_ms(10);    // 早すぎるとダメ
        akros->publish();
        nh.spinOnce();
    }
}