#include "CAN_controller.h"

namespace CAN_controller{
void enter_control_mode(CAN *can_, uint8_t id_){
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
    
    can_->write(msg_);
}

// Exit motor control mode
void exit_control_mode(CAN *can_, uint8_t id_){
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
    
    can_->write(msg_);
}

// set the current motor position to zero
void set_position_to_zero(CAN *can_, uint8_t id_){
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
    
    if(can_->write(msg_)){
        // pc.printf("Set the current motor position to zero \r\n");
    }
}

bool pack_cmd(CANMessage* msg_, float p_des, float v_des, float kp, float kd, float t_ff){

    // 
    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
    kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    
    // convert float -> uint
    int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);     // Position
    int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);     // Velocity
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);     // Kp
    int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);     // Kd
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);      // Torque
    
    // Pack ints into the CAN buffer
    msg_->data[0] = p_int >> 8;
    msg_->data[1] = p_int & 0xFF;
    msg_->data[2] = v_int >> 4;
    msg_->data[3] = ((v_int & 0xF)<<4) | (kp_int >> 8);
    msg_->data[4] = kp_int & 0xFF;
    msg_->data[5] = kd_int >> 4;
    msg_->data[6] = ((kd_int & 0xF)<<4) | (t_int>>8);
    msg_->data[7] = t_int & 0xFF;

    return true;
}


bool unpack_reply(CANMessage msg, uint8_t *id_, float *pos_, float *vel_, float *tt_f_){
    *id_ = msg.data[0];
    int p_int = (msg.data[1]<<8) | msg.data[2];
    int v_int = (msg.data[3]<<4) | (msg.data[4]>>4);
    int i_int = ((msg.data[4]&0xF)<<8) | msg.data[5];
    
    *pos_ = uint_to_float(p_int, P_MIN, P_MAX, 16);
    *vel_ = uint_to_float(v_int, V_MIN, V_MAX, 12);
    *tt_f_ = uint_to_float(i_int, T_MIN, T_MAX, 12);

    return true;
}
}