#ifndef AKROS_BRIDGE_H_
#define AKROS_BRIDGE_H_

#include <mbed.h>
#include <vector>
#include <ros.h>

#include <config.h>
#include <std_srvs/Empty.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_msgs/motor_reply.h>
#include <AKROS_bridge_msgs/Initialize.h>

#include <CAN_controller.h>


class AKROS_bridge{
private:
    // variables
    AKROS_bridge_msgs::motor_reply   motor_reply_msg;   // ROSにモータの状態を返すmsg
    uint8_t motor_num = 0;  // モータの個数

    // Objects
    DigitalOut  myled;  // Nucleo上のLED
    DigitalIn   tweak_toggle;   // 微調節モード用のトグルスイッチ
    DigitalIn   tweak_tact_up, tweak_tact_down; // 微調節モード用のタクトスイッチ

    CAN_controller can_controller;  // CAN通信に関するクラス
    
    // Pubs,Subs,Srvs
    ros::NodeHandle *nh_priv;
    ros::Publisher motor_status_pub;
    ros::Subscriber<AKROS_bridge_msgs::motor_cmd, AKROS_bridge> motor_cmd_sub;
    ros::ServiceServer<AKROS_bridge_msgs::Initialize::Request, AKROS_bridge_msgs::Initialize::Response, AKROS_bridge> enter_control_mode_srv;
    ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response, AKROS_bridge> exit_control_mode_srv;
    ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response, AKROS_bridge> set_zero_pos_srv;

    // callback functions
    void motor_cmd_Cb(const AKROS_bridge_msgs::motor_cmd&);
    void enter_control_mode_Cb(const AKROS_bridge_msgs::Initialize::Request&, AKROS_bridge_msgs::Initialize::Response&);
    void exit_control_mode_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    void set_zero_pos_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);


public:
    AKROS_bridge(ros::NodeHandle*);
    ~AKROS_bridge(void){};
    uint8_t getMotorNum(void);
    void loop(void);
};
#endif