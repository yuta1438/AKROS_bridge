#ifndef CAN_CONTROLLER_H_
#define CAN_CONTROLLER_H_


#include "mbed.h"
#include "CAN.h"

// CAN Settings
#define CAN_HOST_ID     0
#define CAN_FREQ        1000000

// F303
#define CAN_RX_PIN  PA_11
#define CAN_TX_PIN  PA_12

#define CAN_TX_DATA_LENGTH  8
#define CAN_RX_DATA_LENGTH  6

namespace CAN_controller{

void enter_control_mode(const CAN can_, uint8_t id_);
void exit_control_mode(const CAN can_, uint8_t id_);
void set_position_to_zero(const CAN can_, uint8_t id_);

}
#endif
