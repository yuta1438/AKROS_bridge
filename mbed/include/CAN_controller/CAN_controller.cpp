#include "CAN_controller.h"

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
void CAN_controller::can_send(uint8_t id_){
    CANMessage msg_;
    msg_.id = id_+1;
    pack_cmd(msg_);
    can.write(msg_);
}


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
        // pc.printf("Set the current motor position to zero \r\n");
    }
}


// この関数を呼び出す前に必ずIDを設定しておくこと！
bool CAN_controller::pack_cmd(CANMessage& msg_){
    // Set Limit 
    // convert float -> uint
    int p_int  = float_to_uint(fminf(fmaxf(P_MIN,  motor[msg_.id-1].q_ref),   P_MAX),  P_MIN,  P_MAX,  16);     // Position
    int v_int  = float_to_uint(fminf(fmaxf(V_MIN,  motor[msg_.id-1].dq_ref),  V_MAX),  V_MIN,  V_MAX,  12);     // Velocity
    int kp_int = float_to_uint(fminf(fmaxf(KP_MIN, motor[msg_.id-1].Kp),     KP_MAX), KP_MIN, KP_MAX,  12);     // Kp
    int kd_int = float_to_uint(fminf(fmaxf(KD_MIN, motor[msg_.id-1].Kd),     KD_MAX), KD_MIN, KD_MAX,  12);     // Kd
    int t_int  = float_to_uint(fminf(fmaxf(T_MIN,  motor[msg_.id-1].tau_ref), T_MAX),  T_MIN,  T_MAX,  12);      // Torque
    
    // Pack ints into the CAN buffer
    msg_.data[0] = p_int >> 8;
    msg_.data[1] = p_int & 0xFF;
    msg_.data[2] = v_int >> 4;
    msg_.data[3] = ((v_int & 0xF)<<4) | (kp_int >> 8);
    msg_.data[4] = kp_int & 0xFF;
    msg_.data[5] = kd_int >> 4;
    msg_.data[6] = ((kd_int & 0xF)<<4) | (t_int>>8);
    msg_.data[7] = t_int & 0xFF;

    return true;
}


// CANMessageのIDに対応したmotor_statusに格納
// motor_status_vectorの実体が見える必要がある．
bool CAN_controller::unpack_reply(const CANMessage& msg){
    uint8_t id_ = msg.data[0];

    // CAN_IDは1から始まる
    // idによって自動的に振り分けられるようにしたい！
    motor[id_-1].q   = uint_to_float(((msg.data[1]<<8) | msg.data[2]),       P_MIN, P_MAX, 16);
    motor[id_-1].dq  = uint_to_float(((msg.data[3]<<4) | (msg.data[4]>>4)),  V_MIN, V_MAX, 12);
    motor[id_-1].tau = uint_to_float((((msg.data[4]&0xF)<<8) | msg.data[5]), T_MIN, T_MAX, 12);
    return true;
}
