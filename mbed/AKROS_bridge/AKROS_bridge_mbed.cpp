/* AKROS_bridge_CAN */
// PCからの情報(CANMessage)をMotorに流す
// Slaveからの返答をPCに流す
// NucleoではCANメッセージを扱う
#include "mbed.h"
#include "CAN.h"
#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>

#define CAN_RX_PIN  PA_11
#define CAN_TX_PIN  PA_12

#define CAN_HOST_ID 0x00
#define CAN_SLAVE_ID  0x01

#define CAN_TX_DATA_LENGTH  8
#define CAN_RX_DATA_LENGTH  6


// CAN通信
CAN can(CAN_RX_PIN, CAN_TX_PIN);
CANMessage Tx_msg;  // モータに送信するデータ
CANMessage Rx_msg;  // モータからのデータ


// ROS側から受け取ったメッセージをそのままモータに流す
void messageCb(const std_msgs::UInt8MultiArray& msg){
    // ROS_topicの内容をCANMessageにコピー
    // 最初の
    for(int i=0; i<msg.data_length; i++){
        if(i==0){
            Tx_msg.id = msg.data[i];
        }else{
            Tx_msg.data[i-1] = msg.data[i];
        }
    }
    can.write(Tx_msg);  // Slaveに送信
}


// ROS
ros::NodeHandle nh;
std_msgs::UInt8MultiArray ros_msg;
ros::Publisher pub("can_motor_reply", &ros_msg);    // Motor -> STM -> PC
ros::Subscriber<std_msgs::UInt8MultiArray> sub("motor_can_cmd", &messageCb);    // PC -> STM -> Motor



// Slaveからの返信をそのままROSへ流す
void CAN_receive(void){
    if(can.read(Rx_msg)){
        if(Rx_msg.id == CAN_HOST_ID){
            for(int i=0; i<Rx_msg.len; i++){
                ros_msg.data[i] = Rx_msg.data[i];
            }
        }
        pub.publish(&ros_msg);
    }
}


int main(void){
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub);
    nh.subscribe(sub);
    
    // CAN 
    can.frequency(1000000);
    //Tx_msg.id = CAN_SLAVE_ID;
    Tx_msg.len = CAN_TX_DATA_LENGTH;
    can.attach(&CAN_receive);
    
    // ROS
    ros_msg.data_length = CAN_RX_DATA_LENGTH;
    ros_msg.data = (uint8_t *)malloc(sizeof(uint8_t)*CAN_RX_DATA_LENGTH);
    
    while(1){
        nh.spinOnce();
        wait_ms(1);
    }
}