/* AKROS_bridge_CAN */
// PCからの情報(CANMessage)をMotorに流す
// Slaveからの返答をPCに流す
// NucleoではCANメッセージを扱う

// #include <mbed.h>
#include <ros.h>
#include <mbed.h>
#include <vector>
#include <AKROS_bridge/Initialize_can.h>
#include <AKROS_bridge/can_msg.h>
#include <config.h>

// CAN Settings
#define CAN_HOST_ID     0
#define CAN_FREQ        1000000

#if TARGET_BOARD==NUCLEO_F446RE
    #define CAN_RX_PIN  PB_8
    #define CAN_TX_PIN  PB_9
#elif TARGET_BOARD==NUCLEO_F303K8
    #define CAN_RX_PIN  PA_11
    #define CAN_TX_PIN  PA_12
#else
#endif

#define CAN_TX_DATA_LENGTH  8
#define CAN_RX_DATA_LENGTH  6

// CAN通信
CAN can(CAN_RX_PIN, CAN_TX_PIN);

uint8_t motor_num = 0;
std::vector<CANMessage> can_motor_reply;
std::vector<CANMessage> can_motor_cmd;

AKROS_bridge::can_msg can_reply_msg;


// ROS側から受け取ったメッセージをそのままモータに流す
void motor_can_Cb(const AKROS_bridge::can_msg&);
void can_Cb(void);
void initialize_Cb(const AKROS_bridge::Initialize_can::Request&, AKROS_bridge::Initialize_can::Response&);


// ROS
ros::NodeHandle nh;
AKROS_bridge::can_msg can_reply;
ros::Publisher can_reply_pub("can_motor_reply", &can_reply);
ros::Subscriber<AKROS_bridge::can_msg> can_cmd_sub("can_motor_cmd", &motor_can_Cb);
ros::ServiceServer<AKROS_bridge::Initialize_can::Request, AKROS_bridge::Initialize_can::Response> initialize_srv("initialize", &initialize_Cb);


int main(void){
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    
    nh.advertise(can_reply_pub);
    nh.subscribe(can_cmd_sub);
    nh.advertiseService(initialize_srv);
    
    // CAN 
    can.frequency(1000000);
    can.attach(&can_Cb);
    
    // ROS
    // can_reply.data = (uint8_t *)malloc(sizeof(uint8_t)*CAN_RX_DATA_LENGTH);
    
    while(1){
        // can.write();
        can_reply_pub.publish(&can_reply_msg);
        nh.spinOnce();
        wait_ms(5);
    }
}


// ROS側からcmdを受け取ってモータに送信
void motor_can_Cb(const AKROS_bridge::can_msg& msg_){
    for(uint8_t i=0; i<msg_.can_cmd_length; i++){
        can_motor_cmd[i].id = msg_.can_cmd[i].id;
        for(uint8_t j=0; j<CAN_TX_DATA_LENGTH; j++){
            can_motor_cmd[i].data[j] = msg_.can_cmd[i].data[j];
        }
    }
}


// motor_replyを変数に保存
void can_Cb(void){
    CANMessage temp_msg;
    if(can.read(temp_msg)){
        if(temp_msg.id == CAN_HOST_ID){
            uint8_t id_ = temp_msg.data[0];
            for(uint8_t i=0; i<temp_msg.len; i++){
                can_motor_reply[id_-1].data[i] = temp_msg.data[i];
            }
        }
    }
}


// srvのコールバック関数
void initialize_Cb(const AKROS_bridge::Initialize_can::Request& req_, AKROS_bridge::Initialize_can::Response& res_){
    motor_num = req_.joint_num;
    can_motor_cmd.resize(motor_num);
    can_motor_reply.resize(motor_num);
}