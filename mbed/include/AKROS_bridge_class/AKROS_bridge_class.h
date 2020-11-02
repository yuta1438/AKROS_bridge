#ifndef AKROS_BRIDGE_CLASS_H_
#define AKROS_BRIDGE_CLASS_H_

#include <mbed.h>
#include <vector>
#include <ros.h>

#include <std_srvs/Empty.h>
#include <AKROS_bridge/motor_cmd.h>
#include <AKROS_bridge/motor_reply.h>
#include <AKROS_bridge/Initialize.h>

#include <CAN_controller.h>


class AKROS_bridge_class{
private:
    // variables
    AKROS_bridge::motor_reply   motor_reply_msg;
    uint8_t motor_num = 0;
    // Objects
    DigitalOut  myled;
    CAN_controller can_controller;
    
    // Pubs,Subs,Srvs
    ros::NodeHandle *nh_priv;
    ros::Publisher motor_status_pub;
    ros::Subscriber<AKROS_bridge::motor_cmd, AKROS_bridge_class> motor_cmd_sub;
    ros::ServiceServer<AKROS_bridge::Initialize::Request, AKROS_bridge::Initialize::Response, AKROS_bridge_class> enter_control_mode_srv;
    ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response, AKROS_bridge_class> exit_control_mode_srv;
    ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response, AKROS_bridge_class> set_zero_pos_srv;

    // callback functions
    void motor_cmd_Cb(const AKROS_bridge::motor_cmd&);
    void enter_control_mode_Cb(const AKROS_bridge::Initialize::Request&, AKROS_bridge::Initialize::Response&);
    void exit_control_mode_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    void set_zero_pos_Cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&);


public:
    AKROS_bridge_class(ros::NodeHandle*);
    ~AKROS_bridge_class(void){};
    uint8_t getMotorNum(void);
    void loop(void);
};
#endif