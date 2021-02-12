// 共有資源としてmotor_statusがあり，それをもとに書き込み，読み込みを行う．
// AsyncSpinnerによるマルチスレッドなため，排他処理が必須！

#include <AKROS_bridge_converter/AKROS_bridge_converter.h>

// ここで初期化を全て行うべき！
AKROS_bridge_converter::AKROS_bridge_converter(ros::NodeHandle* nh_) : spinner(0){
    // Topics
    usleep(1000*1000);  // wait for rosserial connection established
    nh = nh_;
    can_pub   = nh->advertise<AKROS_bridge_msgs::motor_can_cmd>("can_cmd", 1);
    reply_pub = nh->advertise<AKROS_bridge_msgs::motor_reply>("motor_reply", 1);
    cmd_sub   = nh->subscribe("motor_cmd", 1, &AKROS_bridge_converter::motor_cmd_Cb, this);
    can_sub   = nh->subscribe("can_reply", 1, &AKROS_bridge_converter::can_reply_Cb, this);

    // Servers
    exit_CM_server       = nh->advertiseService("exit_control_mode", &AKROS_bridge_converter::exit_CM_Cb, this);
    set_PZ_server        = nh->advertiseService("set_position_to_zero", &AKROS_bridge_converter::set_PZ_Cb, this);
    servo_setting_server = nh->advertiseService("servo_setting", &AKROS_bridge_converter::servo_setting_Cb, this);
    tweak_control_server = nh->advertiseService("tweak_control", &AKROS_bridge_converter::tweak_control_Cb, this);

    // Clients
    motor_config_client = nh->serviceClient<AKROS_bridge_msgs::motor_config>("motor_config");
    
    // These are valid motor ----
    valid_motor_models.insert("AK10-9");
    valid_motor_models.insert("AK80-6");
    valid_motor_models.insert("AK10-9_OLD");
    valid_motor_models.insert("AK80-6_OLD");
    // -------------------------

    // load motor configuration file (.yaml)
    XmlRpc::XmlRpcValue params;
    nh->getParam("/motor_list", params);

    // add motor written in yaml file
    for(auto params_iterator = params.begin(); params_iterator!= params.end(); params_iterator++){
        motor_status m;

        // get name, CAN-ID, model of the motor from rosparam
        m.name = static_cast<std::string>(params_iterator->first);
        m.CAN_ID = static_cast<int>(params_iterator->second["can_id"]);
        m.model = static_cast<std::string>(params_iterator->second["model"]);

        // check whether if the model written in .yaml file is valid or not.
        if(valid_motor_models.find(m.model) == valid_motor_models.end()){
            ROS_ERROR("Invalid motor has been detected at %s", m.name.c_str());
        }else{
            ROS_INFO("Add motor name: %s, CAN_ID: %i, model: %s", m.name.c_str(), m.CAN_ID, m.model.c_str());
            motor.push_back(m); // create new vector element
        }
    }

    motor_num = (uint8_t)motor.size();  // determine the number of motor

    for(int i=0; i<motor_num; i++){
        if(motor[i].model == "AK10-9"){
            motor[i].P_MAX = AK10_9_P_MAX;
            motor[i].P_MIN = AK10_9_P_MIN;
            motor[i].V_MAX = AK10_9_V_MAX;
            motor[i].V_MIN = AK10_9_V_MIN;
            }
        else if(motor[i].model == "AK80-6"){
            motor[i].P_MAX = AK80_6_P_MAX;
            motor[i].P_MIN = AK80_6_P_MIN;
            motor[i].V_MAX = AK80_6_V_MAX;
            motor[i].V_MIN = AK80_6_V_MIN;
        }
        else if(motor[i].model == "AK10-9_OLD"){
            motor[i].P_MAX = AK10_9_OLD_P_MAX;
            motor[i].P_MIN = AK10_9_OLD_P_MIN;
            motor[i].V_MAX = AK10_9_OLD_V_MAX;
            motor[i].V_MIN = AK10_9_OLD_V_MIN;
        }
        else if(motor[i].model == "AK80-6_OLD"){
            motor[i].P_MAX = AK80_6_OLD_P_MAX;
            motor[i].P_MIN = AK80_6_OLD_P_MIN;
            motor[i].V_MAX = AK80_6_OLD_V_MAX;
            motor[i].V_MIN = AK80_6_OLD_V_MIN;
        }

        // オフセット値や原点の計算
        // motor[i].error = ~

        // Enter control mode for each motor
        AKROS_bridge_msgs::motor_config enter_control_srv;
        enter_control_srv.request.CAN_ID = motor[i].CAN_ID;
        enter_control_srv.request.configration_mode = ENTER_CONTROL_MODE;

        if(motor_config_client.call(enter_control_srv)){
            if(enter_control_srv.response.success){
                ROS_INFO("Motor %d initialized !", motor[i].CAN_ID);
            }else{
                ROS_WARN("enter_control service failed !");
            }
        }else{
            ROS_WARN("There is no server for enter_control_mode !");
        }
        usleep(100*1000);
    }


    // motor_lock for all motors
    AKROS_bridge_msgs::motor_config motor_lock_srv;
    motor_lock_srv.request.CAN_ID = 0;
    motor_lock_srv.request.configration_mode = INITIALIZE_LOCK;
    if(motor_config_client.call(motor_lock_srv)){
        if(motor_lock_srv.response.success){
            ROS_INFO("Motor locked !");
        }else{
            ROS_WARN("Motor_lock failed !");
        }
    }else{
        ROS_WARN("There is no server for motor_lock !");
    }
    usleep(100*1000);


    // servo_on
    for(uint8_t i=0; i<motor_num; i++){
        motor[i].servo_mode = true;
    }
    ROS_INFO("All servo ON !");
    usleep(100*1000);

    // メモリの動的確保
    can_cmd.motor.resize(motor_num);
    reply.motor.resize(motor_num);

    initializeFlag = true;
    spinner.start();
}


AKROS_bridge_converter::~AKROS_bridge_converter(void){
    spinner.stop();
}


// motor_statusからcan_cmdに格納
// 一つのモータに対する関数
void AKROS_bridge_converter::pack_cmd(AKROS_bridge_msgs::motor_can_cmd_single &can_cmd_, uint8_t index_){
    std::lock_guard<std::mutex> lock(motor_mutex);
    
    can_cmd_.CAN_ID   = motor[index_].CAN_ID;
    can_cmd_.position = motor[index_].position_ref - motor[index_].error;
    can_cmd_.velocity = motor[index_].velocity_ref;
    can_cmd_.effort   = motor[index_].effort_ref;

    // servo_modeに準じてKp，Kdの値を調整
    if(motor[index_].servo_mode){
        can_cmd_.Kp = motor[index_].Kp;
        can_cmd_.Kd = motor[index_].Kd;
    }else{
        can_cmd_.Kp = 0;
        can_cmd_.Kd = 0;
    }
}


// motor_statusをfloat型に変換してmotor_replyに格納
void AKROS_bridge_converter::pack_reply(AKROS_bridge_msgs::motor_reply_single &reply_, uint8_t index_){
    std::lock_guard<std::mutex> lock(motor_mutex);
    reply_.CAN_ID   = motor[index_].CAN_ID;
    reply_.position = uint_to_float(motor[index_].position + motor[index_].error, motor[index_].P_MIN, motor[index_].P_MAX, POSITION_BIT_NUM);
    reply_.velocity = uint_to_float(motor[index_].velocity, motor[index_].V_MIN, motor[index_].V_MAX, VELOCITY_BIT_NUM);
    reply_.effort   = uint_to_float(motor[index_].effort,   T_MIN, T_MAX, EFFORT_BIT_NUM);
}


// motor_cmdをint値に変換してmotor_statusに格納
void AKROS_bridge_converter::unpack_cmd(const AKROS_bridge_msgs::motor_cmd_single& cmd_){
    std::lock_guard<std::mutex> lock(motor_mutex);
    uint8_t index_ = find_index(cmd_.CAN_ID);
    motor[index_].position_ref = float_to_uint(fminf(fmaxf(motor[index_].P_MIN, cmd_.position), motor[index_].P_MAX), motor[index_].P_MIN, motor[index_].P_MAX, POSITION_BIT_NUM);
    motor[index_].velocity_ref = float_to_uint(fminf(fmaxf(motor[index_].V_MIN, cmd_.velocity), motor[index_].V_MAX), motor[index_].V_MIN, motor[index_].V_MAX, VELOCITY_BIT_NUM);
    motor[index_].effort_ref   = float_to_uint(fminf(fmaxf(T_MIN, cmd_.effort), T_MAX), T_MIN, T_MAX, EFFORT_BIT_NUM);
    motor[index_].Kp           = float_to_uint(fminf(fmaxf(KP_MIN, cmd_.Kp), KP_MAX), KP_MIN, KP_MAX, KP_BIT_NUM);
    motor[index_].Kd           = float_to_uint(fminf(fmaxf(KD_MIN, cmd_.Kd), KD_MAX), KD_MIN, KD_MAX, KD_BIT_NUM);
}


// motor_can_replyをそのままmotor_statusに格納
void AKROS_bridge_converter::unpack_can_reply(const AKROS_bridge_msgs::motor_can_reply_single& can_reply_){
    std::lock_guard<std::mutex> lock(motor_mutex);
    uint8_t index_ = find_index(can_reply_.CAN_ID);
    motor[index_].CAN_ID   = can_reply_.CAN_ID;
    motor[index_].position = can_reply_.position;
    motor[index_].velocity = can_reply_.velocity;
    motor[index_].effort   = can_reply_.effort;
}


// motor_cmdを受け取ってmotor_statusに保存
void AKROS_bridge_converter::motor_cmd_Cb(const AKROS_bridge_msgs::motor_cmd::ConstPtr& cmd_){
    for(uint8_t i=0; i<cmd_->motor.size();i++){
        unpack_cmd(cmd_->motor[i]);
    }
    publish_cmd();
}


// /can_replyを受け取って実数値に変換し，motor_statusに格納
void AKROS_bridge_converter::can_reply_Cb(const AKROS_bridge_msgs::motor_can_reply::ConstPtr& can_reply_){
    for(uint8_t i=0; i<can_reply_->motor.size(); i++){
        unpack_can_reply(can_reply_->motor[i]);
    }
    publish_reply();
}



// 操作終了
// これを行うともう一度原点だしが必要になるので注意
// To deprecated
bool AKROS_bridge_converter::exit_CM_Cb(AKROS_bridge_msgs::exit_control_mode::Request& req_, AKROS_bridge_msgs::exit_control_mode::Response& res_){
    AKROS_bridge_msgs::motor_config motor_config_srv;
    motor_config_srv.request.CAN_ID = req_.CAN_ID;
    motor_config_srv.request.configration_mode = EXIT_CONTROL_MODE;

    // 指定したvector要素のみを削除したい
    // motor.erase(motor.begin() + (unsigned int)find_index[req_.CAN_ID]);    // vectorを削除
    
    if(motor_config_client.call(motor_config_srv)){
        if(motor_config_srv.response.success)
            res_.success = true;
    }
}


// モータの原点と関節の原点との誤差値を設定
// initialize部でしか使用できないようにする！ To deprecate!
bool AKROS_bridge_converter::set_PZ_Cb(AKROS_bridge_msgs::set_position_zero::Request& req_, AKROS_bridge_msgs::set_position_zero::Response& res_){
    // error = joint - motor
    motor[find_index(req_.CAN_ID)].error = CENTER_POSITION - motor[find_index(req_.CAN_ID)].position;
    motor[find_index(req_.CAN_ID)].position_ref = CENTER_POSITION;

    res_.success = true;

    return true;
}


// motor_cmdのKp，Kdを0にしてモータのサーボをOFFにする
// サーボOFF時に実数目標値を0（整数値ではCENTER_〇〇）に設定
bool AKROS_bridge_converter::servo_setting_Cb(AKROS_bridge_msgs::servo_setting::Request& req_, AKROS_bridge_msgs::servo_setting::Response& res_){
    // CAN_ID = 0ならすべてのモータに対して一括設定
    if(req_.CAN_ID == 0){
        for(uint8_t i=0; i<motor.size(); i++){
            motor[i].servo_mode = req_.servo;
        }

        if(req_.servo){
            ROS_INFO("All servo ON");
        }else{
            ROS_INFO("All servo OFF");
            // 指令値をすべて0に！
            for(size_t i=0; i<motor.size(); i++){
                motor[i].position_ref = CENTER_POSITION;
                motor[i].velocity_ref = CENTER_VELOCITY;
                motor[i].effort_ref = CENTER_EFFORT;
            }
        }
    }else{  // そうでなければ個別に設定
        motor[find_index(req_.CAN_ID)].servo_mode = req_.servo;
        if(req_.servo){
            ROS_INFO("Motor %d servo ON", req_.CAN_ID);
        }else{
            ROS_INFO("Motor %d servo OFF", req_.CAN_ID);
            motor[find_index(req_.CAN_ID)].position_ref = CENTER_POSITION;
            motor[find_index(req_.CAN_ID)].velocity_ref = CENTER_VELOCITY;
            motor[find_index(req_.CAN_ID)].effort_ref = CENTER_EFFORT;
        }
    }
    
    res_.success = true;
    return true;
}


// 微調節
bool AKROS_bridge_converter::tweak_control_Cb(AKROS_bridge_msgs::tweak::Request& req_, AKROS_bridge_msgs::tweak::Response& res_){
switch (req_.control){
    case TWEAK_UP:  // +1
        motor[find_index(req_.CAN_ID)].position_ref += tweak_delta;
        res_.success = true;
        return true;
        break;

    case TWEAK_DOWN:    // -1
        motor[find_index(req_.CAN_ID)].position_ref -= tweak_delta;
        res_.success = true;
        return true;
        break;

    case UP:    // +10
        motor[find_index(req_.CAN_ID)].position_ref += delta;
        res_.success = true;
        return true;
        break;

    case DOWN:  // -10
        motor[find_index(req_.CAN_ID)].position_ref -= delta;
        res_.success = true;
        return true;
        break;

    case BIG_UP:    // +100
        motor[find_index(req_.CAN_ID)].position_ref += big_delta;
        res_.success = true;
        return true;
        break;

    case BIG_DOWN:  // -100
        motor[find_index(req_.CAN_ID)].position_ref -= big_delta;
        res_.success = true;
        return true;
        break;

    default:
        return false;
        break;
    }
}


// CAN指令値をmotor_statusから引っ張り出して変換し，publish 
void AKROS_bridge_converter::publish_cmd(void){
    if(initializeFlag){
        for(uint8_t i=0; i<motor.size(); i++){
            pack_cmd(can_cmd.motor[i], i);
        }
        can_pub.publish(can_cmd);
    }
}


// 応答値をmotor_statusから引っ張り出してpublish
void AKROS_bridge_converter::publish_reply(void){
    if(initializeFlag){
        for(uint8_t i=0; i<motor.size(); i++){
            pack_reply(reply.motor[i], i);
        }
        reply_pub.publish(reply);
    }
}


// モータのCAN_IDからvector<motor_status>のindexを探し出す
uint8_t AKROS_bridge_converter::find_index(uint8_t id_){
    for(uint8_t i=0; i<motor.size(); i++){
        if(motor[i].CAN_ID == id_){
            return i;
        }
    }
    return ERROR_NUM;   // 該当するCAN_IDが無ければぬるぽになるはず
}