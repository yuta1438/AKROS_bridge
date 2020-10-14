/* AKROS_bridge_can_mbed_single */

// モータ単体用
// PCからの情報(CANMessage)をMotorに流す
// Slaveからの返答をPCに流す
// NucleoではCANメッセージを扱う

// #include <mbed.h>
#include <ros.h>
#include "CAN_controller/CAN_controller.h"
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt8.h>


// CAN通信
CAN can(CAN_RX_PIN, CAN_TX_PIN);
CANMessage Tx_msg;  // モータに送信するデータ
CANMessage Rx_msg;  // モータからのデータ


// ROS側から受け取ったメッセージをそのままモータに流す
void motor_can_Cb(const std_msgs::UInt8MultiArray& msg_);
void enter_control_mode_Cb(const std_msgs::UInt8& id_);
void exit_control_mode_Cb(const std_msgs::UInt8& id_);
void set_zero_pos_Cb(const std_msgs::UInt8& id_);


// ROS
ros::NodeHandle nh;
std_msgs::UInt8MultiArray ros_msg;
ros::Publisher pub("can_motor_reply", &ros_msg);    // Motor -> STM -> PC
ros::Subscriber<std_msgs::UInt8MultiArray> motor_can_cmd_sub("motor_can_cmd", &motor_can_Cb);    // PC -> STM -> Motor
ros::Subscriber<std_msgs::UInt8> enter_control_mode_sub("enter_control_mode", &enter_control_mode_Cb);
ros::Subscriber<std_msgs::UInt8> exit_control_mode_sub("exit_control_mode", &exit_control_mode_Cb);
ros::Subscriber<std_msgs::UInt8> set_zero_pos_sub("set_zero_pos", &set_zero_pos_Cb);



// Slaveからの返信をそのままROSへ流す
void CAN_receive(void){
    if(can.read(Rx_msg)){
        if(Rx_msg.id == CAN_HOST_ID){
            for(uint8_t i=0; i<Rx_msg.len; i++){
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
    nh.subscribe(motor_can_cmd_sub);
    nh.subscribe(enter_control_mode_sub);
    nh.subscribe(exit_control_mode_sub);
    nh.subscribe(set_zero_pos_sub);
    
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

// ROS_topicの内容をCANMessageにコピー
void motor_can_Cb(const std_msgs::UInt8MultiArray& msg_){
    for(uint8_t i=0; i<msg_.data_length; i++){
        // 先頭バイトはモータのID
        if(i==0){
            Tx_msg.id = msg_.data[i];
        }
        // それ以降はモータへの指令
        else{
            Tx_msg.data[i-1] = msg_.data[i];
        }
    }
    can.write(Tx_msg);  // Slaveに送信
}

void enter_control_mode_Cb(const std_msgs::UInt8& id_){
    CAN_controller::enter_control_mode(can, id_.data);
}

void exit_control_mode_Cb(const std_msgs::UInt8& id_){
    CAN_controller::exit_control_mode(can, id_.data);
}

void set_zero_pos_Cb(const std_msgs::UInt8& id_){
    CAN_controller::set_position_to_zero(can, id_.data);
}
