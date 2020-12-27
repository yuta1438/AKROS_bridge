#ifndef CAN_CONTROLLER_H_
#define CAN_CONTROLLER_H_

#include "mbed.h"
#include "CAN.h"
#include <vector>
#include <motor_status.h>
#include "config.h"
#include "../basic_op/basic_op.h"


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


class CAN_controller{
private:
    CAN can;
    uint8_t motor_num = 0;
    
    bool pack_cmd(CANMessage&);
    bool unpack_reply(const CANMessage&);
    
public:
    CAN_controller();
    ~CAN_controller(){};
    std::vector<motor_status> motor;

    bool initializeFlag;
    
    void attach(void);
    void can_Cb(void);
    void can_send(uint8_t);
    void add_motor(void);
    uint8_t getMotorNum(void);

    void enter_control_mode(uint8_t id_);
    void exit_control_mode(uint8_t id_);
    void set_position_to_zero(uint8_t id_);

    
};
#endif
