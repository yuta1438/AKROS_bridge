#ifndef CAN_CONTROLLER_H_
#define CAN_CONTROLLER_H_


#include "mbed.h"
#include "CAN.h"
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

// F303
#define CAN_RX_PIN  PA_11
#define CAN_TX_PIN  PA_12

#define CAN_TX_DATA_LENGTH  8
#define CAN_RX_DATA_LENGTH  6



namespace CAN_controller{

void enter_control_mode(CAN *can_, uint8_t id_);
void exit_control_mode(CAN *can_, uint8_t id_);
void set_position_to_zero(CAN *can_, uint8_t id_);


// convert motor_cmd to CAN message
bool pack_cmd(CANMessage* msg_, float p_des, float v_des, float kp, float kd, float t_ff);

// convert can_motor_reply to ROSmsg 
bool unpack_reply(CANMessage msg, uint8_t *id, float *pos_, float *vel_, float *tt_f_);
}
#endif
