#include "CAN_controller.h"

namespace CAN_controller{
void enter_control_mode(CAN can_, uint8_t id_){
    CANMessage msg_;
    msg_.id = id_;
    msg_.len = CAN_TX_DATA_LENGTH;
    msg_.data[0] = 0xFF;
    msg_.data[1] = 0xFF;
    msg_.data[2] = 0xFF;
    msg_.data[3] = 0xFF;
    msg_.data[4] = 0xFF;
    msg_.data[5] = 0xFF;
    msg_.data[6] = 0xFF;
    msg_.data[7] = 0xFC;
    
    can_.write(msg_);
}

// Exit motor control mode
void exit_control_mode(CAN can_, uint8_t id_){
    CANMessage msg_;
    msg_.id = id_;
    msg_.len = CAN_TX_DATA_LENGTH;
    msg_.data[0] = 0xFF;
    msg_.data[1] = 0xFF;
    msg_.data[2] = 0xFF;
    msg_.data[3] = 0xFF;
    msg_.data[4] = 0xFF;
    msg_.data[5] = 0xFF;
    msg_.data[6] = 0xFF;
    msg_.data[7] = 0xFD;
    
    if(can_.write(msg_)){
        // pc.printf("Exit motor control mode \r\n");
    }
}

// set the current motor position to zero
void set_position_to_zero(CAN can_, uint8_t id_){
    CANMessage msg_;
    msg_.id = id_;
    msg_.len = CAN_TX_DATA_LENGTH;
    msg_.data[0] = 0xFF;
    msg_.data[1] = 0xFF;
    msg_.data[2] = 0xFF;
    msg_.data[3] = 0xFF;
    msg_.data[4] = 0xFF;
    msg_.data[5] = 0xFF;
    msg_.data[6] = 0xFF;
    msg_.data[7] = 0xFE;
    
    if(can_.write(msg_)){
        // pc.printf("Set the current motor position to zero \r\n");
    }
}
}