#include <AKROS_bridge_converter/converter.h>

// 指令値の変換
// 一つのモータに対する関数
bool pack_cmd(const AKROS_bridge_msgs::motor_cmd_single &cmd_, AKROS_bridge_msgs::motor_can_cmd_single &can_cmd_){

    float p_des   = fminf(fmaxf(P_MIN, cmd_.position), P_MAX);
    float v_des   = fminf(fmaxf(V_MIN, cmd_.velocity), V_MAX);
    float kp      = fminf(fmaxf(KP_MIN, cmd_.Kp), KP_MAX);
    float kd      = fminf(fmaxf(KD_MIN, cmd_.Kd), KD_MAX);
    float tau_des = fminf(fmaxf(T_MIN, cmd_.effort), T_MAX);
    
    // convert float -> uint
    can_cmd_.position = float_to_uint(p_des, P_MIN, P_MAX, 16);     // Position
    can_cmd_.velocity = float_to_uint(v_des, V_MIN, V_MAX, 12);     // Velocity
    can_cmd_.Kp       = float_to_uint(kp, KP_MIN, KP_MAX, 12);     // Kp
    can_cmd_.Kd       = float_to_uint(kd, KD_MIN, KD_MAX, 12);     // Kd
    can_cmd_.effort   = float_to_uint(tau_des, T_MIN, T_MAX, 12);      // Torque
    
    // Pack uints into the CAN message
    /* Nucleo側に移植
    can_cmd_.data[0] = p_int >> 8;
    can_cmd_.data[1] = p_int & 0xFF;
    can_cmd_.data[2] = v_int >> 4;
    can_cmd_.data[3] = ((v_int & 0xF)<<4) | (kp_int >> 8);
    can_cmd_.data[4] = kp_int & 0xFF;
    can_cmd_.data[5] = kd_int >> 4;
    can_cmd_.data[6] = ((kd_int & 0xF)<<4) | (tau_int>>8);
    can_cmd_.data[7] = tau_int & 0xFF;
    */
    return true;
}


// 応答値の変換
bool unpack_reply(const AKROS_bridge_msgs::motor_can_reply_single &can_reply_, sensor_msgs::JointState &js_){
    // 
    /* Nucleo側に移植
    int p_int = (msg.data[1]<<8) | msg.data[2];
    int v_int = (msg.data[3]<<4) | (msg.data[4]>>4);
    int i_int = ((msg.data[4]&0xF)<<8) | msg.data[5];
    */
    
    // 関節IDとCAN_IDは異なるので注意！
    js_.position[can_reply_.id - 1] = uint_to_float(can_reply_.position, P_MIN, P_MAX, POSITION_BIT_NUM);
    js_.velocity[can_reply_.id - 1] = uint_to_float(can_reply_.velocity, V_MIN, V_MAX, VELOCITY_BIT_NUM);
    js_.effort[can_reply_.id - 1]   = uint_to_float(can_reply_.effort, T_MIN, T_MAX, EFFORT_BIT_NUM);

    return true;
}
