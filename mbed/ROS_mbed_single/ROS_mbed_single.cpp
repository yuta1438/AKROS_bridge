/* ROS_mbed_single */
// モータ単体ver
// PCからの情報(ROSMessage)をMotorに流す
// Motorからの返答をROSに流す
// モータの情報をマイコン内に格納しておけば同期等が必要なくなるのでは？

#define MOTOR_ID    1
#define TARGET_BOARD NUCLEO_F446RE

// #include <mbed.h>
#include <vector>
#include <ros.h>
#include <std_srvs/Empty.h>
#include <motor_status/motor_status.h>
#include <AKROS_bridge/Initialize_single.h>
#include <AKROS_bridge/motor_cmd_single.h>
#include <AKROS_bridge/motor_reply_single.h>
#include <CAN_controller/CAN_controller.h>


// CAN通信
// MotorのCAN_IDは1から始まるので注意！
// F303K8
//#if TARGET_BOARD==NUCLEO_F303K8
// F446RE
//#define CAN_RX_PIN  PA_11
//#define CAN_TX_PIN  PA_12
//#elif TARGET_BOARD==NUCLEO_F446RE

// F446RE
//#define CAN_RX_PIN PB_8
//#define CAN_TX_PIN PB_9
//#endif
CAN can(CAN_RX_PIN, CAN_TX_PIN);
motor_status motor;

DigitalOut myled(LED1);

// Motorの返信をROS側に返す
void CAN_Cb(void);

// ROS側から受け取ったメッセージをそのままモータに流す
void motor_cmd_Cb(const AKROS_bridge::motor_cmd_single&);

// 初期化関係はservice通信で行う
void enter_control_mode_Cb(const AKROS_bridge::Initialize_single::Request&, AKROS_bridge::Initialize_single::Response&);
void exit_control_mode_Cb(const AKROS_bridge::Initialize_single::Request&, AKROS_bridge::Initialize_single::Response&);
void set_zero_pos_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);


// ROS
ros::NodeHandle nh;
AKROS_bridge::motor_reply_single motor_reply;

ros::Publisher motor_reply_pub("motor_reply", &motor_reply);
ros::Subscriber<AKROS_bridge::motor_cmd_single> motor_cmd_sub("motor_cmd", &motor_cmd_Cb);

ros::ServiceServer<AKROS_bridge::Initialize_single::Request, AKROS_bridge::Initialize_single::Response> enter_control_mode_srv("enter_control_mode", &enter_control_mode_Cb);
ros::ServiceServer<AKROS_bridge::Initialize_single::Request, AKROS_bridge::Initialize_single::Response> exit_control_mode_srv("exit_control_mode", &exit_control_mode_Cb);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> set_zero_pos_srv("set_position_zero", &set_zero_pos_Cb);


int main(void){
    myled = 0;
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    motor.initialize(1);
    nh.advertiseService(enter_control_mode_srv);
    nh.advertiseService(exit_control_mode_srv);
    nh.advertiseService(set_zero_pos_srv);

    nh.advertise(motor_reply_pub);
    nh.subscribe(motor_cmd_sub);

    // CAN
    can.frequency(1000000);
    
    // ROS
    while(1){
        nh.spinOnce();
        wait_ms(10);
    }

    return 0;
}


// ROS_topicの内容をCANMessageにコピー
void motor_cmd_Cb(const AKROS_bridge::motor_cmd_single& cmd_){
    CANMessage msg_;
    msg_.id = cmd_.id + 1;  // MotorのCAN_IDは1から始まるので注意！
    CAN_controller::pack_cmd(&msg_, cmd_.position, cmd_.velocity, cmd_.Kp, cmd_.Kd, cmd_.torque);
    can.write(msg_);  // Slaveに送信
}


// CANメッセージを受け取ったら
// 一旦変数に格納
void CAN_Cb(void){
    CANMessage msg_;
    if(can.read(msg_)){
        if(msg_.id == CAN_HOST_ID){
            uint8_t id_;
            float pos_, vel_, tt_f_;
            CAN_controller::unpack_reply(msg_, &id_, &pos_, &vel_, &tt_f_);

            motor_reply.id = id_ - 1;   // MotorのCAN_IDは1から始まるので注意！
            motor_reply.position = pos_;
            motor_reply.velocity = vel_;
            motor_reply.torque = tt_f_;
            motor_reply_pub.publish(&motor_reply);
        }
    }
}

// enter control mode of one motor
void enter_control_mode_Cb(const AKROS_bridge::Initialize_single::Request& req_, AKROS_bridge::Initialize_single::Response& res_){
    CANMessage msg_;
    CAN_controller::enter_control_mode(&can, MOTOR_ID);
    wait_ms(100);

    if(can.read(msg_)){
        if(msg_.id == CAN_HOST_ID){
            uint8_t id_;
            float pos_, vel_, tt_f_;
            CAN_controller::unpack_reply(msg_, &id_, &pos_, &vel_, &tt_f_);
            res_.q = pos_;
            res_.dq = vel_;
            res_.tau = tt_f_;
            
            motor.q[0] = pos_;
            motor.dq[0] = vel_;
            motor.tau[0] = tt_f_;
        }
    }

    can.attach(&CAN_Cb);
}

// exit control mode of motor-1
void exit_control_mode_Cb(const AKROS_bridge::Initialize_single::Request& req_, AKROS_bridge::Initialize_single::Response& res_){
    CANMessage msg_;
    CAN_controller::exit_control_mode(&can, MOTOR_ID);
    
    if(can.read(msg_)){
        if(msg_.id == CAN_HOST_ID){
            uint8_t id_;
            float pos_, vel_, tt_f_;
            CAN_controller::unpack_reply(msg_, &id_, &pos_, &vel_, &tt_f_);
            res_.q = pos_;
            res_.dq = vel_;
            res_.tau = tt_f_;
        }
    }

    //can.attach(&CAN_Cb);
}

// set the angle of motor-1 to zero
void set_zero_pos_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    CAN_controller::set_position_to_zero(&can, MOTOR_ID);
}
