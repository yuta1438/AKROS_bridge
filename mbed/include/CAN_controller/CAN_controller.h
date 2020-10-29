#ifndef CAN_CONTROLLER_H_
#define CAN_CONTROLLER_H_

#include "mbed.h"
#include "CAN.h"
#include "../motor_status/motor_status.h"
#include "config.h"
#include "../basic_op/basic_op.h"

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

// CAN Settings
#define CAN_HOST_ID     0
#define CAN_FREQ        1000000

#if TARGET_BOARD==NUCLEO_F446RE
    #define CAN_RX_PIN  PB_8
    #define CAN_TX_PIN  PB_9
#elif TARGET_BOARD==NUCLEO_F303K8
    #define CAN_RX_PIN  PA_11
    #define CAN_TX_PIN  PA_12
#else
#endif


#define CAN_TX_DATA_LENGTH  8
#define CAN_RX_DATA_LENGTH  6


// motor_statusに保存された値を送信&受信データを保存
namespace CAN_controller{

void enter_control_mode(CAN *can_, uint8_t id_);
void exit_control_mode(CAN *can_, uint8_t id_);
void set_position_to_zero(CAN *can_, uint8_t id_);

// convert motor_cmd to CAN message
bool pack_cmd(CANMessage* msg_, motor_status motor_);

// convert can_motor_reply to ROSmsg 
bool unpack_reply(CANMessage msg, motor_status *motor_);
}
#endif
