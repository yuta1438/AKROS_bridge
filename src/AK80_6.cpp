#include "AK80_6.h"

bool pack_cmd(std_msgs::UInt8MultiArray *msg, uint8_t id_, float p_des, float v_des, float kp, float kd, float t_ff){
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
    msg->data[0] = id_;
    msg->data[1] = p_int >> 8;
    msg->data[2] = p_int & 0xFF;
    msg->data[3] = v_int >> 4;
    msg->data[4] = ((v_int & 0xF)<<4) | (kp_int >> 8);
    msg->data[5] = kp_int & 0xFF;
    msg->data[6] = kd_int >> 4;
    msg->data[7] = ((kd_int & 0xF)<<4) | (t_int>>8);
    msg->data[8] = t_int & 0xFF;

    return true;
}


bool unpack_reply(std_msgs::UInt8MultiArray msg, uint8_t *id_, float *pos_, float *vel_, float *tt_f_){
    int p_int = (msg.data[1]<<8) | msg.data[2];
    int v_int = (msg.data[3]<<4) | (msg.data[4]>>4);
    int i_int = ((msg.data[4]&0xF)<<8) | msg.data[5];
    
    *id_ = msg.data[0];
    *pos_ = uint_to_float(p_int, P_MIN, P_MAX, 16);
    *vel_ = uint_to_float(v_int, V_MIN, V_MAX, 12);
    *tt_f_ = uint_to_float(i_int, T_MIN, T_MAX, 12);

    return true;
}
