#ifndef AKROS_BRIDGE_H_
#define AKROS_BRIDGE_H_

#include <mbed.h>
#include <vector>
#include <ros.h>

#include <std_srvs/Empty.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_msgs/motor_reply.h>
#include <AKROS_bridge_msgs/Initialize.h>
#include <AKROS_bridge_msgs/finalize.h>
#include <AKROS_bridge_msgs/set_zero_pos.h>

#include <CAN_controller.h>


class AKROS_bridge{
private:
    // variables
<<<<<<< HEAD
    AKROS_bridge_msgs::motor_reply   motor_reply_msg;   // ROSにモータの状態を返すmsg

    // Objects
    DigitalOut  myled;  // Nucleo上のLED
    
    CAN_controller can_controller;  // CAN通信に関するクラス
=======
    AKROS_bridge_msgs::motor_reply   motor_reply_msg;
    uint8_t motor_num = 0;
    // Objects
    DigitalOut  myled;
    CAN_controller can_controller;
>>>>>>> parent of ac169de... 説明文を追加
    
    // Pubs,Subs,Srvs
    ros::NodeHandle *nh_priv;
    ros::Publisher motor_status_pub;
    ros::Subscriber<AKROS_bridge_msgs::motor_cmd, AKROS_bridge> motor_cmd_sub;
    ros::ServiceServer<AKROS_bridge_msgs::Initialize::Request, AKROS_bridge_msgs::Initialize::Response, AKROS_bridge> enter_control_mode_srv;
    ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response, AKROS_bridge> initialize_finish_srv;    // 初期化完了
    ros::ServiceServer<AKROS_bridge_msgs::finalize::Request, AKROS_bridge_msgs::finalize::Response, AKROS_bridge> exit_control_mode_srv;
    ros::ServiceServer<AKROS_bridge_msgs::set_zero_pos::Request, AKROS_bridge_msgs::set_zero_pos::Response, AKROS_bridge> set_zero_pos_srv;

    // callback functions
    void motor_cmd_Cb(const AKROS_bridge_msgs::motor_cmd&);
    void enter_control_mode_Cb(const AKROS_bridge_msgs::Initialize::Request&, AKROS_bridge_msgs::Initialize::Response&);
    void exit_control_mode_Cb(const AKROS_bridge_msgs::finalize::Request&, AKROS_bridge_msgs::finalize::Response&);
    void set_zero_pos_Cb(const AKROS_bridge_msgs::set_zero_pos::Request&, AKROS_bridge_msgs::set_zero_pos::Response&);
    void initialize_finish_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);

    void addMotor(uint8_t);
    uint8_t find_iterator(uint8_t);
    
public:
    AKROS_bridge(ros::NodeHandle*);
    ~AKROS_bridge(void){};
    void loop(void);
};
#endif