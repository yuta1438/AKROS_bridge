#ifndef AKROS_BRIDGE_CONVERTER_H_
#define AKROS_BRIDGE_CONVERTER_H_

#include <ros/ros.h>
#include <vector>
#include <mutex>
#include <unistd.h>
#include <set>

#include <AKROS_bridge_converter/motor_status.h>

// モータ制御
#include <AKROS_bridge_msgs/motor_can_cmd.h>
#include <AKROS_bridge_msgs/motor_can_reply.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_msgs/motor_reply.h>

#include "basic_op.h"

// モータ設定
#include <AKROS_bridge_msgs/enter_control_mode.h>
#include <AKROS_bridge_msgs/exit_control_mode.h>
#include <AKROS_bridge_msgs/set_position_zero.h>
#include <AKROS_bridge_msgs/servo_setting.h>
#include <AKROS_bridge_msgs/motor_config.h>
#include <AKROS_bridge_msgs/tweak.h>
#include <AKROS_bridge_msgs/currentState.h>
#include <AKROS_bridge_converter/tweak_control.h>
#include <AKROS_bridge_converter/motor_config.h>


#define ERROR_NUM   99

#define CAN_TX_DATA_LENGTH  8   // モータ指令値のDLC
#define CAN_RX_DATA_LENGTH  6   // モータ応答値のDLC



class AKROS_bridge_converter{
private:
    ros::NodeHandle *nh;
    bool initializeFlag;

    uint8_t motor_num;
    std::vector<motor_status> motor;
    std::set<std::string> valid_motor_models;

    ros::AsyncSpinner spinner;
    std::mutex motor_mutex;   // 排他処理

    /** トピック通信関係 **/
    AKROS_bridge_msgs::motor_can_cmd can_cmd;
    AKROS_bridge_msgs::motor_reply reply;
    ros::Publisher can_pub;     // Nucleoに整数指令値を送る
    ros::Publisher reply_pub;   // ROSに実数応答値を送る
    ros::Subscriber can_sub;    // Nucleoから整数応答値を受け取る
    ros::Subscriber cmd_sub;    // ROSから実数指令値を受け取る

    // for convertion
    void pack_can_cmd(AKROS_bridge_msgs::motor_can_cmd_single&, uint8_t);    // convert 「一つのモータに対する関数」 to CAN message
    void pack_reply(AKROS_bridge_msgs::motor_reply_single&, uint8_t);    // convert to ROS message
    void unpack_cmd(const AKROS_bridge_msgs::motor_cmd_single&);
    void unpack_can_reply(const AKROS_bridge_msgs::motor_can_reply_single&);
    
    // Callback Functions
    void motor_cmd_Cb(const AKROS_bridge_msgs::motor_cmd::ConstPtr&);
    void can_reply_Cb(const AKROS_bridge_msgs::motor_can_reply::ConstPtr&);


    /** サービス通信 **/
    ros::ServiceServer exit_CM_server;          // exit_control_mode_server
    ros::ServiceServer set_ZP_server;           // set_zero_position_server
    ros::ServiceServer servo_setting_server;    // set servo of the motor
    ros::ServiceServer motor_lock_server;
    ros::ServiceServer tweak_control_server;
    ros::ServiceServer currentState;
    ros::ServiceClient motor_config_client; // モータに関する各種設定
    

    // Callback Functions
    bool exit_CM_Cb(AKROS_bridge_msgs::exit_control_mode::Request&, AKROS_bridge_msgs::exit_control_mode::Response&);
    bool set_ZP_Cb(AKROS_bridge_msgs::set_position_zero::Request&, AKROS_bridge_msgs::set_position_zero::Response&);
    bool servo_setting_Cb(AKROS_bridge_msgs::servo_setting::Request&, AKROS_bridge_msgs::servo_setting::Response&);
    bool tweak_control_Cb(AKROS_bridge_msgs::tweak::Request&, AKROS_bridge_msgs::tweak::Response&);
    bool currentState_Cb(AKROS_bridge_msgs::currentState::Request&, AKROS_bridge_msgs::currentState::Response&);

    uint8_t find_index(uint8_t);

public:
    AKROS_bridge_converter(ros::NodeHandle*);
    ~AKROS_bridge_converter();
    void publish_can_cmd(void);
    void publish_reply(void);
};
#endif