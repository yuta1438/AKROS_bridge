/* CAN_ROS_Slave_ReceiveOnly */
// MasterからのCANMessageを受信

// 内容が正しければLEDを点滅 + printfで内容表示
// cmdと同じpos, vel, torを返す．

#include "mbed.h"
#include "CAN.h"

// F303K8
#define CAN_RX_PIN  PA_11
#define CAN_TX_PIN  PA_12

#define CAN_HOST_ID 0x00
#define CAN_SLAVE_ID  0x01

#define CAN_TX_DATA_LENGTH  6

DigitalOut myled(LED1);

CAN can(CAN_RX_PIN, CAN_TX_PIN);    // CAN通信

Serial pc(USBTX, USBRX, 57600);
CANMessage Tx_msg;

void can_Cb(void){
    CANMessage Rx_msg;
    if(can.read(Rx_msg)){
        if(Rx_msg.id == CAN_SLAVE_ID){
            Tx_msg.data[0] = CAN_SLAVE_ID;
            Tx_msg.data[1] = Rx_msg.data[0];    // Position 8-H
            Tx_msg.data[2] = Rx_msg.data[1];    // Position 8-L
            Tx_msg.data[3] = Rx_msg.data[2];    // Velocity 8-H
            // Velocity 4-L & Torque 4-H
            Tx_msg.data[4] = (Rx_msg.data[3] & 0xF0) | (Rx_msg.data[6] & 0x0F); 
            Tx_msg.data[5] = Rx_msg.data[7];
            can.write(Tx_msg);
        }
    }
}


int main(void){
    Tx_msg.id = CAN_HOST_ID;
    Tx_msg.len = CAN_TX_DATA_LENGTH;
    
    can.frequency(1000000);
    myled = 0;
    
    // 受信割込み設定(メッセージを受け取ったらreplyを実行)
    can.attach(&can_Cb);
    
    while(1){
        wait_ms(1);
    }
}