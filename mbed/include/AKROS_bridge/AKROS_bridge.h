#ifndef AKROS_BRIDGE_H_
#define AKROS_BRIDGE_H_

// 任意のタイミングでmotor_statusの中身（現在値）を返すserviceを実装すること
//　→ CANは常にsendすること！

#include <mbed.h>
#include <vector>
#include <ros.h>
#include <general_settings.h>
#include <CAN_controller.h>
#include <AKROS_bridge_msgs/motor_can_cmd.h>
#include <AKROS_bridge_msgs/motor_can_reply.h>
#include <AKROS_bridge_msgs/motor_config.h>
#include <AKROS_bridge_msgs/currentState.h>

class AKROS_bridge{
private:
    // Objects
    // BusOut leds;
    DigitalOut  Nucleo_led;  // Nucleo上のLED
    DigitalIn   tweak_toggle;   // 微調節モード用のトグルスイッチ
    DigitalIn   tweak_tact_up, tweak_tact_down; // 微調節モード用のタクトスイッチ

    CAN_controller can_controller;  // CAN通信に関するクラス
    
    ros::NodeHandle *nh_priv;
    AKROS_bridge_msgs::motor_can_reply   can_reply_msg;   // ROSにモータの状態を返すmsg

    // topics
    ros::Publisher can_reply_pub;
    ros::Subscriber<AKROS_bridge_msgs::motor_can_cmd, AKROS_bridge> can_cmd_sub;

    // services
    ros::ServiceServer<AKROS_bridge_msgs::motor_config::Request, AKROS_bridge_msgs::motor_config::Response, AKROS_bridge> motor_config_srv;
    ros::ServiceServer<AKROS_bridge_msgs::currentState::Request, AKROS_bridge_msgs::currentState::Response, AKROS_bridge> currentState_srv;

    // callback functions
    void can_cmd_Cb(const AKROS_bridge_msgs::motor_can_cmd&);
    void motor_config_Cb(const AKROS_bridge_msgs::motor_config::Request&, AKROS_bridge_msgs::motor_config::Response&);
    void currentState_Cb(const AKROS_bridge_msgs::currentState::Request&, AKROS_bridge_msgs::currentState::Response&);
    

    // functions
    // タクトスイッチで微調節する
    void incrementPosition(void);
    void decrementPosition(void);

public:
    AKROS_bridge(ros::NodeHandle*);
    ~AKROS_bridge(void){};
    void loop(void);
};
#endif