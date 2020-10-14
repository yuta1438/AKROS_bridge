/* ROS_mbed_single */
// モータ単体ver
// PCからの情報(ROSMessage)をMotorに流す
// Motorからの返答をROSに流す

// #include <mbed.h>
#include <ros.h>
#include <std_srvs/Empty.h>
#include <AKROS_bridge/motor_cmd_single.h>
#include <AKROS_bridge/motor_reply_single.h>
#include <CAN_controller/CAN_controller.h>


#define MOTOR_ID    1

// CAN通信
CAN can(CAN_RX_PIN, CAN_TX_PIN);

DigitalOut myled(LED1);

// ROS側から受け取ったメッセージをそのままモータに流す
void motor_cmd_Cb(const AKROS_bridge::motor_cmd_single&);

// 初期化関係はservice通信で行う
void enter_control_mode_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);
void exit_control_mode_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);
void set_zero_pos_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);


// ROS
ros::NodeHandle nh;
AKROS_bridge::motor_reply_single motor_reply;

ros::Publisher motor_reply_pub("motor_reply", &motor_reply);
ros::Subscriber<AKROS_bridge::motor_cmd_single> motor_cmd_sub("motor_cmd", &motor_cmd_Cb);

ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> enter_control_mode_sub("enter_control_mode", &enter_control_mode_Cb);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> exit_control_mode_sub("exit_control_mode", &exit_control_mode_Cb);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> set_zero_pos_sub("set_zero_pos", &set_zero_pos_Cb);



// Slaveからの返信をそのままROSへ流す
void CAN_Cb(void){
    /*
    if(can.read(Rx_msg)){
        if(Rx_msg.id == CAN_HOST_ID){
            for(uint8_t i=0; i<Rx_msg.len; i++)
                ros_msg.data[i] = Rx_msg.data[i];
        }
        pub.publish(&ros_msg);
    }
    */
}


int main(void){
    myled = 0;
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    
    nh.advertise(motor_reply_pub);
    nh.subscribe(motor_cmd_sub);

    nh.advertiseService(enter_control_mode_sub);
    nh.advertiseService(exit_control_mode_sub);
    nh.advertiseService(set_zero_pos_sub);
    

    // CAN 
    can.frequency(1000000);
    
    
    // Initialize_srvを実行したあと！
    
    //
    
    can.attach(&CAN_Cb);
    
    // ROS
    while(1){
        nh.spinOnce();
        wait_ms(1);
    }
}

// ROS_topicの内容をCANMessageにコピー
void motor_cmd_Cb(const AKROS_bridge::motor_cmd_single& cmd_){
    CANMessage msg_;
    msg_.id = cmd_.id;
    CAN_controller::pack_cmd(&msg_, cmd_.position, cmd_.velocity, cmd_.Kp, cmd_.Kd, cmd_.torque);
    can.write(msg_);  // Slaveに送信
}

// enter control mode of one motor
void enter_control_mode_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    // 何個のモータを使用するか？

    CANMessage msg_;
    CAN_controller::enter_control_mode(can, MOTOR_ID);
    
    // 受信待ち
    while(!can.read(msg_)){
        if(msg_.id == CAN_HOST_ID){
            uint8_t id_;
            float pos_, vel_, tt_f_;
            CAN_controller::unpack_reply(msg_, &id_, &pos_, &vel_, &tt_f_);

            motor_reply.position = pos_;
            motor_reply.velocity = vel_;
            motor_reply.torque = tt_f_;
        }
    }

    motor_reply_pub.publish(&motor_reply);
}

// exit control mode of motor-1
void exit_control_mode_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    CAN_controller::exit_control_mode(can, MOTOR_ID);
}

// set the angle of motor-1 to zero
void set_zero_pos_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    CAN_controller::set_position_to_zero(can, MOTOR_ID);
}
