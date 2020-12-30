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
    can_cmd_.id       = cmd_.id;
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
// 一つのモータに対する関数
bool unpack_reply(const AKROS_bridge_msgs::motor_can_reply_single &can_reply_, AKROS_bridge_msgs::motor_reply_single &reply_){
    // 
    /* Nucleo側に移植
    int p_int = (msg.data[1]<<8) | msg.data[2];
    int v_int = (msg.data[3]<<4) | (msg.data[4]>>4);
    int i_int = ((msg.data[4]&0xF)<<8) | msg.data[5];
    */

    reply_.id = can_reply_.id;
    reply_.position = uint_to_float(can_reply_.position, P_MIN, P_MAX, POSITION_BIT_NUM);
    reply_.velocity = uint_to_float(can_reply_.velocity, V_MIN, V_MAX, VELOCITY_BIT_NUM);
    reply_.effort   = uint_to_float(can_reply_.effort, T_MIN, T_MAX, EFFORT_BIT_NUM);

    return true;
}

// /motor_cmdを受け取って/can_cmdに変換
void convert_cmd_Cb(const AKROS_bridge_msgs::motor_cmd::ConstPtr& cmd_){
    std::lock_guard<std::mutex> lock(m);

    for(uint8_t i=0; i<motor_num; i++){
        pack_cmd(cmd_->motor[i], can_cmd.motor[i]);
        
        // トルクモードがOFFならKp=Kd=0を代入
        if()
    }
    can_pub.publish(can_cmd);
}

// /can_replyを受け取って/motor_replyに変換
void convert_reply_Cb(const AKROS_bridge_msgs::motor_can_reply::ConstPtr& can_reply_){
    std::lock_guard<std::mutex> lock(m);

    for(uint8_t i=0; i<motor_num; i++){
        unpack_reply(can_reply_->motor[i], reply.motor[i]);
    }
    reply_pub.publish(reply);
}

// モータを追加
// Initialize_lockが呼ばれた後は無効
bool enter_CM_Cb(const AKROS_bridge_msgs::enter_control_mode::Request& req_, AKROS_bridge_msgs::enter_control_mode::Response& res_){
    if(!initializeFlag){
        motor_num++;    // モータ追加
        motor_config_srv.request.CAN_ID = req_.CAN_ID;
        motor_config_srv.request.configration_mode = ENTER_CONTROL_MODE;

        if(motor_config_client.call(motor_config_srv)){
            if(motor_config_srv.response.success){
                res_.success = true;
            }
        }
        return true;    
    }else{
        res_.success = false;
        return false;
    }
}


// 操作終了
// これを行うともう一度原点だしが必要になるので注意
bool exit_CM_Cb(const AKROS_bridge_msgs::exit_control_mode::Request& req_, AKROS_bridge_msgs::exit_control_mode::Response& res_){
    motor_config_srv.request.CAN_ID = req_.CAN_ID;
    motor_config_srv.request.configration_mode = EXIT_CONTROL_MODE;
    
    if(motor_config_client.call(motor_config_srv)){
        if(motor_config_srv.response.success)
            res_.success = true;
    }
}

// 
bool set_PZ_Cb(const AKROS_bridge_msgs::set_position_zero::Request& req_, AKROS_bridge_msgs::set_position_zero::Response& res_){
    motor_config_srv.request.CAN_ID = req_.CAN_ID;
    motor_config_srv.request.configration_mode = SET_POSITION_TO_ZERO;
    
    if(motor_config_client.call(motor_config_srv)){
        if(motor_config_srv.response.success)
            res_.success = true;
    }
}

// motor_cmdのKp，Kdを0にしてモータのサーボをOFFにする
// → どうやって特定のモータのゲインを0にすればよいか？(requestではCAN_IDを指定)
bool servo_setting_Cb(const AKROS_bridge_msgs::servo_setting::Request& req_, AKROS_bridge_msgs::servo_setting::Response& res_){

}


// モータの個数確定
// これ以上のモータ追加は不可能
bool motor_lock_Cb(std_srvs::Empty::Request& res_, std_srvs::Empty::Response& req_){
    motor_config_srv.request.CAN_ID = 0;
    motor_config_srv.request.configration_mode = INITIALIZE_LOCK;

    servo_mode.resize(motor_num);

    if(motor_config_client.call(motor_config_srv)){

    }
}
