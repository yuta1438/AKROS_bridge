#include <AKROS_bridge_converter/AKROS_bridge_converter.h>

AKROS_bridge_converter::AKROS_bridge_converter(ros::NodeHandle& nh)
 : nh_priv(nh),spinner(0){
    // Topics
    can_pub   = nh_priv.advertise<AKROS_bridge_msgs::motor_can_cmd>("can_cmd", 1);
    reply_pub = nh_priv.advertise<AKROS_bridge_msgs::motor_reply>("motor_reply", 1);
    cmd_sub   = nh_priv.subscribe("motor_cmd", 1, &AKROS_bridge_converter::motor_cmd_Cb, this);
    can_sub   = nh_priv.subscribe("can_reply", 1, &AKROS_bridge_converter::can_reply_Cb, this);

    // Servers
    enter_CM_server      = nh.advertiseService("enter_control_mode", &AKROS_bridge_converter::enter_CM_Cb, this);
    exit_CM_server       = nh.advertiseService("exit_control_mode", &AKROS_bridge_converter::exit_CM_Cb, this);
    set_PZ_server        = nh.advertiseService("set_position_to_zero", &AKROS_bridge_converter::set_PZ_Cb, this);
    servo_setting_server = nh.advertiseService("servo_setting", &AKROS_bridge_converter::servo_setting_Cb, this);
    motor_lock_server    = nh.advertiseService("motor_lock", &AKROS_bridge_converter::motor_lock_Cb, this);
    current_state_server = nh.advertiseService("current_state", &AKROS_bridge_converter::current_state_Cb, this);

    // Clients
    motor_config_client = nh.serviceClient<AKROS_bridge_msgs::motor_config>("motor_config");
    
    initializeFlag = false;
    motor_num = 0;
}

AKROS_bridge_converter::~AKROS_bridge_converter(void){
    spinner.stop();
}


// motor_statusからcan_cmdに格納
// 一つのモータに対する関数
bool AKROS_bridge_converter::pack_cmd(AKROS_bridge_msgs::motor_can_cmd_single &can_cmd_, uint8_t index_){
    std::lock_guard<std::mutex> lock(motor_mutex);
    
    can_cmd_.CAN_ID   = motor[index_].CAN_ID;
    can_cmd_.position = float_to_uint(fminf(fmaxf(P_MIN, motor[index_].position_ref), P_MAX), P_MIN, P_MAX, 16);   // Position
    can_cmd_.velocity = float_to_uint(fminf(fmaxf(V_MIN, motor[index_].velocity_ref), V_MAX), V_MIN, V_MAX, 12);   // Velocity
    can_cmd_.effort   = float_to_uint(fminf(fmaxf(T_MIN, motor[index_].effort_ref), T_MAX), T_MIN, T_MAX, 12); // Torque

    // servo_modeに準じてKp，Kdの値を調整
    if(motor[index_].servo_mode){
        can_cmd_.Kp   = float_to_uint(fminf(fmaxf(KP_MIN, motor[index_].Kp), KP_MAX), KP_MIN, KP_MAX, 12);    // Kp
        can_cmd_.Kd   = float_to_uint(fminf(fmaxf(KD_MIN, motor[index_].Kd), KD_MAX), KD_MIN, KD_MAX, 12);    // Kd
    }else{
        can_cmd_.Kp = 0;
        can_cmd_.Kd = 0;
    }
    
    return true;
}


// motor_statusからmotor_replyに格納
bool AKROS_bridge_converter::pack_reply(AKROS_bridge_msgs::motor_reply_single &reply_, uint8_t index_){
    std::lock_guard<std::mutex> lock(motor_mutex);
    
    reply_.CAN_ID   = motor[index_].CAN_ID;
    reply_.position = motor[index_].position;
    reply_.velocity = motor[index_].velocity;
    reply_.effort   = motor[index_].effort;
    
    return true;
}


void AKROS_bridge_converter::unpack_can_reply(const AKROS_bridge_msgs::motor_can_reply_single& can_reply_){
    uint8_t index_ = find_index(can_reply_.CAN_ID);
    
    motor[index_].CAN_ID   = can_reply_.CAN_ID;
    motor[index_].position = uint_to_float(can_reply_.position, P_MIN, P_MAX, POSITION_BIT_NUM);
    motor[index_].velocity = uint_to_float(can_reply_.velocity, V_MIN, V_MAX, VELOCITY_BIT_NUM);
    motor[index_].effort   = uint_to_float(can_reply_.effort, T_MIN, T_MAX, EFFORT_BIT_NUM);
}

// /motor_cmdを受け取ってmotor_statusに保存
void AKROS_bridge_converter::motor_cmd_Cb(const AKROS_bridge_msgs::motor_cmd& cmd_){
    // motor_statusに書き込むので排他処理
    std::lock_guard<std::mutex> lock(motor_mutex);
    for(uint8_t i=0; i<motor_num; i++){
        motor[i].CAN_ID       = cmd_.motor[i].CAN_ID;
        motor[i].position_ref = cmd_.motor[i].position;
        motor[i].velocity_ref = cmd_.motor[i].velocity;
        motor[i].effort_ref   = cmd_.motor[i].effort;
        motor[i].Kp           = cmd_.motor[i].Kp;
        motor[i].Kd           = cmd_.motor[i].Kd;
    }
}

// /can_replyを受け取って実数値に変換し，motor_statusに格納
void AKROS_bridge_converter::can_reply_Cb(const AKROS_bridge_msgs::motor_can_reply& can_reply_){
    // motor_statusに書き込むので排他処理
    std::lock_guard<std::mutex> lock(motor_mutex);

    // どうやって変換しようか？
    for(uint8_t i=0; i<motor_num; i++){
        unpack_can_reply(can_reply_.motor[i]);
    }
}


// モータを追加
// Initialize_lockが呼ばれた後は無効
bool AKROS_bridge_converter::enter_CM_Cb(AKROS_bridge_msgs::enter_control_mode::Request& req_, AKROS_bridge_msgs::enter_control_mode::Response& res_){
    if(!initializeFlag){
        // push_back
        motor_status motor_;
        motor_.CAN_ID = req_.CAN_ID;
        motor_.servo_mode = false;  // 最初はサーボOFF
        motor.push_back(motor_);

        motor_config_srv.request.CAN_ID = req_.CAN_ID;
        motor_config_srv.request.configration_mode = ENTER_CONTROL_MODE;

        if(motor_config_client.call(motor_config_srv)){
            if(motor_config_srv.response.success){
                res_.success = true;
                ROS_INFO("Motor %d initialized !", req_.CAN_ID);
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
bool AKROS_bridge_converter::exit_CM_Cb(AKROS_bridge_msgs::exit_control_mode::Request& req_, AKROS_bridge_msgs::exit_control_mode::Response& res_){
    motor_config_srv.request.CAN_ID = req_.CAN_ID;
    motor_config_srv.request.configration_mode = EXIT_CONTROL_MODE;
    
    if(motor_config_client.call(motor_config_srv)){
        if(motor_config_srv.response.success)
            res_.success = true;
    }
}

// 
bool AKROS_bridge_converter::set_PZ_Cb(AKROS_bridge_msgs::set_position_zero::Request& req_, AKROS_bridge_msgs::set_position_zero::Response& res_){
    motor_config_srv.request.CAN_ID = req_.CAN_ID;
    motor_config_srv.request.configration_mode = SET_POSITION_TO_ZERO;
    
    if(motor_config_client.call(motor_config_srv)){
        if(motor_config_srv.response.success)
            res_.success = true;
    }

    motor[find_index(req_.CAN_ID)].position_ref = 0.0;
    motor[find_index(req_.CAN_ID)].velocity_ref = 0.0;
    motor[find_index(req_.CAN_ID)].effort_ref = 0.0;

    return true;
}

// motor_cmdのKp，Kdを0にしてモータのサーボをOFFにする
// → どうやって特定のモータのゲインを0にすればよいか？(requestではCAN_IDを指定)
bool AKROS_bridge_converter::servo_setting_Cb(AKROS_bridge_msgs::servo_setting::Request& req_, AKROS_bridge_msgs::servo_setting::Response& res_){
    motor[find_index(req_.CAN_ID)].servo_mode = req_.servo;
    res_.success = true;
    return true;
}


// モータの個数確定
// これ以上のモータ追加は不可能
bool AKROS_bridge_converter::motor_lock_Cb(std_srvs::Empty::Request& res_, std_srvs::Empty::Response& req_){
    motor_config_srv.request.CAN_ID = 0;
    motor_config_srv.request.configration_mode = INITIALIZE_LOCK;
    initializeFlag = true;

    // メモリの動的確保
    motor_num = motor.size();
    can_cmd.motor.resize(motor_num);
    reply.motor.resize(motor_num);

    if(motor_config_client.call(motor_config_srv)){
        ROS_INFO("Motors has been locked !");
        ROS_INFO("AsyncSpinner Start !");
    }

    spinner.start();
}


// motor_statusの値を返す
bool AKROS_bridge_converter::current_state_Cb(AKROS_bridge_msgs::currentState::Request& req_, AKROS_bridge_msgs::currentState::Response& res_){
    pack_reply(res_.reply, find_index(req_.CAN_ID));
    res_.success = true;
    return true;
}



// CAN指令値をmotor_statusから引っ張り出して変換し，publish
void AKROS_bridge_converter::publish_cmd(void){
    for(uint8_t i=0; i<motor_num; i++){
        pack_cmd(can_cmd.motor[i], i);
    }
    
    // servo_modeが0ならKp=Kd=0を代入
    can_pub.publish(can_cmd);
}

// 応答値をmotor_statusから引っ張り出してpublish
void AKROS_bridge_converter::publish_reply(void){
    for(uint8_t i=0; i<motor_num; i++){
        pack_reply(reply.motor[i], i);
    }
    
    reply_pub.publish(reply);
}

// モータのCAN_IDからvector<motor_status>のindexを探し出す
uint8_t AKROS_bridge_converter::find_index(uint8_t CAN_ID_){
    for(uint8_t i=0; i<motor.size(); i++){
        if(motor[i].CAN_ID == CAN_ID_){
            return i;
        }
    }
    return ERROR_NUM;   // 該当するCAN_IDが無ければぬるぽになるはず
}