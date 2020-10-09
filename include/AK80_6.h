#ifndef AK80_6_H_
#define AK80_6_H_

#include <std_msgs/UInt8MultiArray.h>
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

#define TX_DATA_LENGTH  9   // motor_cmd(8byte) + motor_ID(1byte)
#define RX_DATA_LENGTH  6   // 「モータからの情報」バイト数

// convert motor_cmd to CAN message
bool pack_cmd(std_msgs::UInt8MultiArray *msg, uint8_t id, float p_des, float v_des, float kp, float kd, float t_ff);

// convert can_motor_reply to ROSmsg 
bool unpack_reply(std_msgs::UInt8MultiArray msg, uint8_t *id, float *pos_, float *vel_, float *tt_f_);

#endif