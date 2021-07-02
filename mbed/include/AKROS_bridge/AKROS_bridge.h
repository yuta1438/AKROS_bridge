#ifndef AKROS_BRIDGE_H_
#define AKROS_BRIDGE_H_

// 任意のタイミングでmotor_statusの中身（現在値）を返すserviceを実装すること
//　→ CANは常にsendすること！

#include <mbed.h>
#include <vector>
#include <ros.h>
#include <general_settings.h>
#include <CAN_controller.h>

#include <AKROS_bridge_msgs/motor_config.h>
#include <AKROS_bridge_msgs/currentState.h>

#ifndef USE_TIMESTAMP
#include <AKROS_bridge_msgs/motor_can_cmd.h>
#include <AKROS_bridge_msgs/motor_can_reply.h>
#else
#include <AKROS_bridge_msgs/motor_can_cmd_timestamped.h>
#include <AKROS_bridge_msgs/motor_can_reply_timestamped.h>
#endif
class AKROS_bridge{
private:
    // Objects
    // BusOut leds;
    DigitalOut green_led;
    DigitalOut yellow_led;
    DigitalOut red_led;

    Ticker ticker;
    uint8_t MotorNum;

    // 微調節モード用
    // DigitalIn   tweak_toggle1, tweak_toggle2;   // トグルスイッチ
    // DigitalIn   tweak_tact_up, tweak_tact_down; // タクトスイッチ

    CAN_controller can_controller;  // CAN通信に関するクラス
    
    ros::NodeHandle *nh_priv;
    AKROS_bridge_msgs::motor_can_reply   can_reply_msg;   // ROSにモータの状態を返すmsg

    // topics
    ros::Publisher can_reply_pub;
    ros::Subscriber<AKROS_bridge_msgs::motor_can_cmd, AKROS_bridge> can_cmd_sub;

    // services
    ros::ServiceServer<AKROS_bridge_msgs::motor_config::Request, AKROS_bridge_msgs::motor_config::Response, AKROS_bridge> motor_config_srv;

    // callback functions
    void can_cmd_Cb(const AKROS_bridge_msgs::motor_can_cmd&);
    void motor_config_Cb(const AKROS_bridge_msgs::motor_config::Request&, AKROS_bridge_msgs::motor_config::Response&);
    void can_send();
    // functions
    // タクトスイッチで微調節する
    void incrementPosition(void);
    void decrementPosition(void);

public:
    AKROS_bridge(ros::NodeHandle*);
    ~AKROS_bridge(void);
    void publish(void);
};
#endif
