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

bool pack_cmd(CANMessage* msg_, motor_status motor_){

    // Set Limit 
    motor_.q_ref   = fminf(fmaxf(P_MIN, motor_.q_ref), P_MAX);
    motor_.dq_ref  = fminf(fmaxf(V_MIN, motor_.dq_ref), V_MAX);
    motor_.kp      = fminf(fmaxf(KP_MIN, motor_.kp), KP_MAX);
    motor_.kd      = fminf(fmaxf(KD_MIN, motor_.kd), KD_MAX);
    motor_.tau_ref = fminf(fmaxf(T_MIN, motor_.tau_ref), T_MAX);
    
    // convert float -> uint
    int p_int  = float_to_uint(motor_.q_ref, P_MIN, P_MAX, 16);     // Position
    int v_int  = float_to_uint(motor_.dq_ref, V_MIN, V_MAX, 12);     // Velocity
    int kp_int = float_to_uint(motor_.Kp, KP_MIN, KP_MAX, 12);     // Kp
    int kd_int = float_to_uint(motor_.kd, KD_MIN, KD_MAX, 12);     // Kd
    int t_int  = float_to_uint(motor_.tau_ref, T_MIN, T_MAX, 12);      // Torque
    
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


bool unpack_reply(CANMessage msg){
    uint8_t id_ = msg.data[0];
    int p_int = (msg.data[1]<<8) | msg.data[2];
    int v_int = (msg.data[3]<<4) | (msg.data[4]>>4);
    int i_int = ((msg.data[4]&0xF)<<8) | msg.data[5];
    
    // CAN_IDは1から始まる
    // idによって自動的に振り分けられるようにしたい！
    motor_->q   = uint_to_float(p_int, P_MIN, P_MAX, 16);
    motor_->dq[id_-1]  = uint_to_float(v_int, V_MIN, V_MAX, 12);
    motor_->tau[id_-1] = uint_to_float(i_int, T_MIN, T_MAX, 12);

    return true;
}
}