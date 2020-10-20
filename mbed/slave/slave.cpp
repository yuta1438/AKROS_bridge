/* CAN_ROS_Slave_ReceiveOnly */
// MasterからのCANMessageを受信

// Masterに返信しない！
// 内容が正しければLEDを点滅 + printfで内容表示


#include "mbed.h"
#include "CAN.h"
#define CAN_RX_PIN  PA_11
#define CAN_TX_PIN  PA_12

#define CAN_HOST_ID 0x00
#define CAN_SLAVE_ID  0x01

#define CAN_TX_DATA_LENGTH  6

DigitalOut myled(LED1);

CAN can(CAN_RX_PIN, CAN_TX_PIN);    // CAN通信
CANMessage Tx_msg, Rx_msg;

Serial pc(USBTX, USBRX, 57600);


// 受信割込み
void reply(void){
    // 指定したIDの受信に成功したら
    if(can.read(Rx_msg)){
        myled = !myled;
        // IDが合致したら
        if(Rx_msg.id == CAN_SLAVE_ID){
            // 内容表示
            for(int i=0; i<Rx_msg.len; i++){
                pc.printf("%2x,", Rx_msg.data[i]);
            }
            pc.printf("\r\n");

            Tx_msg.data[0] = 0x03;
            Tx_msg.data[1] = 0xFF;
            Tx_msg.data[2] = 0x00;
            Tx_msg.data[3] = 0xF0;
            Tx_msg.data[4] = 0x0F;
            Tx_msg.data[5] = 0x00;
            can.write(Tx_msg);
        }
    }
}


int main(void){
    Tx_msg.id = CAN_HOST_ID;
    Tx_msg.len = CAN_TX_DATA_LENGTH;
    
    can.frequency(1000000);
    pc.printf("I am ready !\r\n");
    myled = 0;
    
    // 受信割込み設定(メッセージを受け取ったらreplyを実行)
    can.attach(&reply);
    
    while(1){
        wait_ms(1);
    }
}