#ifndef AKROS_BRIDGE_H_
#define AKROS_BRIDGE_H_

#include <mbed.h>
#include <vector>
#include <ros.h>
#include <config.h>
#include <std_srvs/Empty.h>
#include <AKROS_bridge_msgs/motor_can_cmd.h>
#include <AKROS_bridge_msgs/motor_can_reply.h>
#include <AKROS_bridge_msgs/Initialize_can.h>
#include <CAN_controller.h>


class AKROS_bridge{
private:
    // variables
    AKROS_bridge_msgs::motor_can_reply   can_reply_msg;   // ROSにモータの状態を返すmsg
    uint8_t motor_num = 0;  // モータの個数

    // Objects
    DigitalOut  myled;  // Nucleo上のLED
    DigitalIn   tweak_toggle;   // 微調節モード用のトグルスイッチ
    DigitalIn   tweak_tact_up, tweak_tact_down; // 微調節モード用のタクトスイッチ

    CAN_controller can_controller;  // CAN通信に関するクラス
    
    // Pubs,Subs,Srvs
    ros::NodeHandle *nh_priv;

    ros::Publisher can_reply_pub;
    ros::Subscriber<AKROS_bridge_msgs::motor_can_cmd, AKROS_bridge> can_cmd_sub;
    
    ros::ServiceServer<AKROS_bridge_msgs::Initialize_can::Request, AKROS_bridge_msgs::Initialize_can::Response, AKROS_bridge> enter_control_mode_srv;
    ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response, AKROS_bridge> exit_control_mode_srv;
    ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response, AKROS_bridge> set_zero_pos_srv;

    // callback functions
    void can_cmd_Cb(const AKROS_bridge_msgs::motor_can_cmd&);
    void enter_control_mode_Cb(const AKROS_bridge_msgs::Initialize_can::Request&, AKROS_bridge_msgs::Initialize_can::Response&);
    void exit_control_mode_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    void set_zero_pos_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);


    // functions
    // タクトスイッチで微調節する
    void incrementPosition(void);
    void decrementPosition(void);

public:
    AKROS_bridge(ros::NodeHandle*);
    ~AKROS_bridge(void){};
    uint8_t getMotorNum(void);
    void loop(void);
};
#endif