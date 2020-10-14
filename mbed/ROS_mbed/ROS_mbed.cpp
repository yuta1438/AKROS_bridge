/* ROS_mbed_single */
// モータ単体ver
// PCからの情報(ROSMessage)をMotorに流す
// Motorからの返答をROSに流す

// #include <mbed.h>
#include <ros.h>
#include <AKROS_bridge/Initialize.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <CAN_controller/CAN_controller.h>


// CAN通信
CAN can(CAN_RX_PIN, CAN_TX_PIN);

DigitalOut myled(LED1);

// ROS側から受け取ったメッセージをそのままモータに流す
void motor_cmd_Cb(const trajectory_msgs::JointTrajectoryPoint&);

// 初期化関係はservice通信で行う
void enter_control_mode_Cb(const AKROS_bridge::Initialize::Request&, AKROS_bridge::Initialize::Response&);
void exit_control_mode_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);
void set_zero_pos_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);


// ROS
ros::NodeHandle nh;
sensor_msgs::JointState js;

ros::Publisher motor_reply_pub("motor_reply", &js);
ros::Subscriber<trajectory_msgs::JointTrajectoryPoint> motor_cmd_sub("motor_cmd", &motor_cmd_Cb);

ros::ServiceServer<AKROS_bridge::Initialize::Request, AKROS_bridge::Initialize::Response> enter_control_mode_sub("enter_control_mode", &enter_control_mode_Cb);
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
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    
    nh.advertise(motor_reply_pub);
    nh.subscribe(motor_cmd_sub);

    nh.advertiseService(enter_control_mode_sub);
    nh.advertiseService(exit_control_mode_sub);
    nh.advertiseService(set_zero_pos_sub);
    

    // CAN 
    can.frequency(1000000);
    can.attach(&CAN_Cb);
    
    // ROS
    
    while(1){
        nh.spinOnce();
        wait_ms(1);
    }
}

// ROS_topicの内容をCANMessageにコピー
void motor_cmd_Cb(const trajectory_msgs::JointTrajectoryPoint& jt_){
    /*
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
    */
}

// enter control mode of motor-1
void enter_control_mode_Cb(const AKROS_bridge::Initialize::Request& req_, AKROS_bridge::Initialize::Response& res_){
    // 何個のモータを使用するか？
    js.position_length = req_.joint_num;
    js.velocity_length = req_.joint_num;
    js.effort_length = req_.joint_num;

    js.position = (double *)malloc(sizeof(double) * js.position_length);
    js.velocity = (double *)malloc(sizeof(double) * js.velocity_length);
    js.effort = (double *)malloc(sizeof(double) * js.effort_length);


    for(uint8_t i=0; i<req_.joint_num; i++){
        CANMessage msg_;
        CAN_controller::enter_control_mode(can, i);
        
        // 受信待ち
        while(!can.read(msg_)){
            if(msg_.id == CAN_HOST_ID){
                uint8_t id_;
                float pos_, vel_, tt_f_;
                CAN_controller::unpack_reply(msg_, &id_, &pos_, &vel_, &tt_f_);

                js.position[id_] = pos_;
                js.velocity[id_] = vel_;
                js.effort[id_] = tt_f_;
            }
        }
    }
    

}

// exit control mode of motor-1
void exit_control_mode_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    CAN_controller::exit_control_mode(can, MOTOR_ID);
}

// set the angle of motor-1 to zero
void set_zero_pos_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    CAN_controller::set_position_to_zero(can, MOTOR_ID);
}
