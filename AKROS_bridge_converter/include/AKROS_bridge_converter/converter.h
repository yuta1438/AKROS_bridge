#ifndef CONVERTER_H_
#define CONVERTER_H_

#include <AKROS_bridge_msgs/motor_can_cmd_single.h>
#include <AKROS_bridge_msgs/motor_can_reply_single.h>
#include <AKROS_bridge_msgs/motor_cmd_single.h>
#include <AKROS_bridge_msgs/motor_reply_single.h>
#include "basic_op.h"

#define P_MIN   -95.5f
#define P_MAX   95.5f
#define V_MIN   -30.0f
#define V_MAX   30.0f
#define KP_MIN   0.0f
#define KP_MAX   500.0f
#define KD_MIN   0.0f
#define KD_MAX   5.0f
#define T_MIN   -18.0f
#define T_MAX   18.0f

#define POSITION_BIT_NUM    16
#define VELOCITY_BIT_NUM    12
#define EFFORT_BIT_NUM      12

#define CAN_TX_DATA_LENGTH  8   // モータ指令値のDLC
#define CAN_RX_DATA_LENGTH  6   // モータ応答値のDLC

// convert to CAN message
bool pack_cmd(const AKROS_bridge_msgs::motor_cmd_single&, AKROS_bridge_msgs::motor_can_cmd_single&);

// convert to ROS message 
bool unpack_reply(const AKROS_bridge_msgs::motor_can_reply_single&, AKROS_bridge_msgs::motor_reply_single&);

#endif