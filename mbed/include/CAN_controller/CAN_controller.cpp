#include <CAN_controller/CAN_controller.h>

// Constructor
CAN_controller::CAN_controller()
  : can(CAN_RX_PIN, CAN_TX_PIN)
{
    can.frequency(CAN_FREQ);
    can.attach(callback(this, &CAN_controller::can_Cb));
    initializeFlag = false;
}



// モータから受け取った情報をmotor_statusに格納
void CAN_controller::can_Cb(void){
    CANMessage msg_;
    if(can.read(msg_)){
        if(msg_.id == CAN_HOST_ID){
            unpack_reply(msg_);
        }
    }
}


// モータから受け取った情報をmotor_statusに格納
// 関節番号とCAN_IDがずれることに留意！
void CAN_controller::can_send(uint8_t id_){
    CANMessage msg_;
    msg_.id = id_+1;
    pack_cmd(msg_);
    can.write(msg_);
}


// enter control mode(モータコントロールモードに入る)
// モータを制御するためには必須！
void CAN_controller::enter_control_mode(uint8_t id_){
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
    
    can.write(msg_);
}


// Exit motor control mode
void CAN_controller::exit_control_mode(uint8_t id_){
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
    
    can.write(msg_);
}

// set the current motor position to zero
void CAN_controller::set_position_to_zero(uint8_t id_){
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
    
    if(can.write(msg_)){
        // pc.printf("Set the current motor position to zero \r\n");    // Debug
    }
}