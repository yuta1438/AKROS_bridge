/* ROS_mbed */
// PCからの情報(ROSMessage)をMotorに流す
// Motorからの返答をROSに流す
// 指令値，現在地は一旦マイコン内に格納することにする


// #include <mbed.h>
#include <ros.h>
#include <AKROS_bridge/Initialize.h>
#include <std_srvs/Empty.h>
#include <CAN_controller/CAN_controller.h>
#include <AKROS_bridge/motor_cmd.h>
#include <AKROS_bridge/motor_reply.h>

// motor
std::vector<motor_status> motor;
uint8_t motor_num;

// CAN通信
// MotorのCAN_IDは1から始まるので注意！
CAN can(CAN_RX_PIN, CAN_TX_PIN);

DigitalOut myled(LED1);


void can_Cb(void);
void motor_cmd_Cb(const AKROS_bridge::motor_cmd&);

// 初期化関係はservice通信で行う
void enter_control_mode_Cb(const AKROS_bridge::Initialize::Request&, AKROS_bridge::Initialize::Response&);
void exit_control_mode_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);
void set_zero_pos_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);


// ROS
ros::NodeHandle nh;
AKROS_bridge::motor_reply reply;

ros::Publisher motor_reply_pub("motor_reply", &reply);
ros::Subscriber<AKROS_bridge::motor_cmd> motor_cmd_sub("motor_cmd", &motor_cmd_Cb);

ros::ServiceServer<AKROS_bridge::Initialize::Request, AKROS_bridge::Initialize::Response> enter_control_mode_sub("enter_control_mode", &enter_control_mode_Cb);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> exit_control_mode_sub("exit_control_mode", &exit_control_mode_Cb);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> set_zero_pos_sub("set_zero_pos", &set_zero_pos_Cb);


int main(void){
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    myled = 0;
    
    // 
    nh.advertise(motor_reply_pub);  // stm->ROS
    nh.subscribe(motor_cmd_sub);    // ROS->stm

    // special can code
    nh.advertiseService(enter_control_mode_sub);
    nh.advertiseService(exit_control_mode_sub);
    nh.advertiseService(set_zero_pos_sub);
    

    // CAN 
    can.frequency(CAN_FREQ);
    can.attach(&can_Cb);
    
    // ROS
    
    while(1){
        nh.spinOnce();
        wait_ms(1);
    }
}


// 指令値をmotor_statusに格納
void motor_cmd_Cb(const AKROS_bridge::motor_cmd& cmd_){
    for(uint8_t i=0; i<motor_num; i++){
        motor[i].q_ref = cmd_.cmd.positions[i];
        motor[i].dq_ref = cmd_.cmd.velocities[i];
        motor[i].tau_ref = cmd_.cmd.effort[i];
    }
}


// Motorからのreplyをmotor_statusに格納
void can_Cb(void){
    CANMessage msg_;
    if(can.read(msg_)){
        if(msg_.id == CAN_HOST_ID){
            CAN_controller::unpack_reply(msg_, &motor); // &motorの引数をなくしたい
        }
    }
}


// enter control mode server
void enter_control_mode_Cb(const AKROS_bridge::Initialize::Request& req_, AKROS_bridge::Initialize::Response& res_){
    // 何個のモータを使用するか？
    motor_num = req_.joint_num;
    res_.jointstate.position_length = motor_num;
    res_.jointstate.velocity_length = motor_num;
    res_.jointstate.effort_length   = motor_num;

    res_.jointstate.position = (double *)malloc(sizeof(double) * motor_num);
    res_.jointstate.velocity = (double *)malloc(sizeof(double) * motor_num);
    res_.jointstate.effort   = (double *)malloc(sizeof(double) * motor_num);


    for(uint8_t i=0; i<motor_num; i++){
        CANMessage msg_;
        // モータのCAN_IDは1から始まる
        CAN_controller::enter_control_mode(&can, i+1);
        wait_ms(1);

        // 受信待ち
        if(can.read(msg_)){
            if(msg_.id == CAN_HOST_ID){
                CAN_controller::unpack_reply(msg_);

                // store in Response
                res_.jointstate.position[i] = motor[i].q;
                res_.jointstate.velocity[i] = motor[i].dq;
                res_.jointstate.effort[i]   = motor[i].tau;

            }
        }
    }
    myled = 1;
    can.attach(&can_Cb);
}

// exit control mode of motor-1
void exit_control_mode_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    for(int i=0; i<motor_num; i++){
        CAN_controller::exit_control_mode(&can, i+1);
        wait_ms(10);
    }
    myled = 0;
}

// set the angle of motor-1 to zero
void set_zero_pos_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    for(int i=0; i<motor_num; i++){
        CAN_controller::set_position_to_zero(&can, i+1);
        wait_ms(10);
    }
}
