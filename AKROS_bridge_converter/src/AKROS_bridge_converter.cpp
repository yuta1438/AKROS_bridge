// 共有資源としてmotor_statusがあり，それをもとに書き込み，読み込みを行う．
// AsyncSpinnerによるマルチスレッドなため，排他処理が必須！

#include <AKROS_bridge_converter/AKROS_bridge_converter.h>

// コンストラクタ
AKROS_bridge_converter::AKROS_bridge_converter(ros::NodeHandle* nh_) : spinner(0){
    // Topic初期設定
    usleep(1000*1000);  // wait for rosserial connection established
    nh = nh_;
    can_pub   = nh->advertise<AKROS_bridge_msgs::motor_can_cmd>("can_cmd", 1);
    reply_pub = nh->advertise<AKROS_bridge_msgs::motor_reply>("motor_reply", 1);
    cmd_sub   = nh->subscribe("motor_cmd", 1, &AKROS_bridge_converter::motor_cmd_Cb, this);
    can_sub   = nh->subscribe("can_reply", 1, &AKROS_bridge_converter::can_reply_Cb, this);

    // Server初期設定
    exit_CM_server       = nh->advertiseService("exit_control_mode", &AKROS_bridge_converter::exit_CM_Cb, this);
    set_ZP_server        = nh->advertiseService("set_position_to_zero", &AKROS_bridge_converter::set_ZP_Cb, this);
    servo_setting_server = nh->advertiseService("servo_setting", &AKROS_bridge_converter::servo_setting_Cb, this);
    tweak_control_server = nh->advertiseService("tweak_control", &AKROS_bridge_converter::tweak_control_Cb, this);
    currentState         = nh->advertiseService("current_state", &AKROS_bridge_converter::currentState_Cb, this);

    // Client初期設定（マイコンに対してcall）
    motor_config_client = nh->serviceClient<AKROS_bridge_msgs::motor_config>("motor_config");
    
    // 有効なモータの型番は以下の通り(std::mapを使用して有効かどうか確認する) ----
    valid_motor_models.insert(AK10_9::model_name);
    valid_motor_models.insert(AK80_6::model_name);
    valid_motor_models.insert(AK10_9_OLD::model_name);
    valid_motor_models.insert(AK80_6_OLD::model_name);
    // -------------------------

    // .yamlファイルからデータを読み込み，rosparamに登録する
    XmlRpc::XmlRpcValue params;
    nh->getParam("/motor_list", params);

    // .yamlファイルに記載されたモータを登録する
    for(auto params_iterator = params.begin(); params_iterator!= params.end(); params_iterator++){
        motor_status m;

        // get name, CAN-ID, model, offset of the motor from rosparam
        m.name = static_cast<std::string>(params_iterator->first);
        m.CAN_ID = static_cast<int>(params_iterator->second["can_id"]);
        m.model = static_cast<std::string>(params_iterator->second["model"]);

        // .yamlファイルに記載されたモータ型番が有効かどうかを確認
        // もし有効ならそれにあったPDゲインの上下限値を設定．（AD/DA変換時に使用）
        if(valid_motor_models.find(m.model) == valid_motor_models.end()){
            ROS_ERROR("Invalid motor has been detected at %s", m.name.c_str());
        }else{
            if(m.model == AK10_9::model_name){
                m.P_MAX = AK10_9::P_MAX;
                m.P_MIN = AK10_9::P_MIN;
                m.V_MAX = AK10_9::V_MAX;
                m.V_MIN = AK10_9::V_MIN;
                m.T_MAX = AK10_9::T_MAX;
                m.T_MIN = AK10_9::T_MIN;
            }
            else if(m.model == AK80_6::model_name){
                m.P_MAX = AK80_6::P_MAX;
                m.P_MIN = AK80_6::P_MIN;
                m.V_MAX = AK80_6::V_MAX;
                m.V_MIN = AK80_6::V_MIN;
                m.T_MAX = AK80_6::T_MAX;
                m.T_MIN = AK80_6::T_MIN;
            }
            else if(m.model == AK10_9_OLD::model_name){
                m.P_MAX = AK10_9_OLD::P_MAX;
                m.P_MIN = AK10_9_OLD::P_MIN;
                m.V_MAX = AK10_9_OLD::V_MAX;
                m.V_MIN = AK10_9_OLD::V_MIN;
                m.T_MAX = AK10_9_OLD::T_MAX;
                m.T_MIN = AK10_9_OLD::T_MIN;
            }
            else if(m.model == AK80_6_OLD::model_name){
                m.P_MAX = AK80_6_OLD::P_MAX;
                m.P_MIN = AK80_6_OLD::P_MIN;
                m.V_MAX = AK80_6_OLD::V_MAX;
                m.V_MIN = AK80_6_OLD::V_MIN;
                m.T_MAX = AK80_6_OLD::T_MAX;
                m.T_MIN = AK80_6_OLD::T_MIN;
            }
        }

        // 関節可動角を設定
        if(params_iterator->second["joint_limit"].valid()){
            // iterator経由で取得する方法が分からなかったのでNodeHandle経由で取得するようにする．
            std::vector<double> limit;
            std::string string1 = "/motor_list/";
            std::string string2 = "/joint_limit";
            std::string full_path = string1 + static_cast<std::string>(params_iterator->first) + string2;
            nh->getParam(full_path, limit);
            m.lower_limit = deg2rad(limit[0]);
            m.upper_limit = deg2rad(limit[1]);
            m.isLimitExist = true;
        }

        // オフセット値の計算
        if(params_iterator->second["offset"].valid()){
            // offsetをアナログ値[deg]からデジタル値に変換する
            m.offset = convertOffset(deg2rad(static_cast<double>(params_iterator->second["offset"])), m.P_MIN, m.P_MAX, POSITION_BIT_NUM);
        }


        // モータ出力を逆にするかどうか
        if(params_iterator->second["inverseDirection"].valid()){
            m.inverseDirection = static_cast<bool>(params_iterator->second["inverseDirection"]);
        }else{
            m.inverseDirection = false;
        }

        ROS_INFO("Add motor name: %s, CAN_ID: %i, model: %s", m.name.c_str(), m.CAN_ID, m.model.c_str());
        motor.push_back(m); // 新しいベクトル要素を作成
    }

    motor_num = (uint8_t)motor.size();  // モータ個数を固定

    for(auto& e : motor){
        // Enter control mode for each motor
        AKROS_bridge_msgs::motor_config enter_control_srv;
        enter_control_srv.request.CAN_ID = e.CAN_ID;
        enter_control_srv.request.configration_mode = ENTER_CONTROL_MODE;

        if(motor_config_client.call(enter_control_srv)){
            if(enter_control_srv.response.success){
                ROS_INFO("Motor %d initialized !", e.CAN_ID);
            }else{
                ROS_WARN("enter_control service failed !");
            }
        }else{
            ROS_WARN("There is no server for enter_control_mode !");
        }
        usleep(100*1000);

        // motor[i].errorの計算
        // error = (理想位置) - (初期位置)
        // e.error = CENTER_POSITION - e.position;
        // e.position_ref = CENTER_POSITION;
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


// デストラクタ
AKROS_bridge_converter::~AKROS_bridge_converter(void){
    spinner.stop();
}


// motor_statusの内容をcan_cmdに格納
// 一つのモータに対する関数
void AKROS_bridge_converter::pack_can_cmd(AKROS_bridge_msgs::motor_can_cmd_single &can_cmd_, uint8_t index_){
    std::lock_guard<std::mutex> lock(motor_mutex);
    
    can_cmd_.CAN_ID   = motor[index_].CAN_ID;
    can_cmd_.position = motor[index_].position_ref - (motor[index_].offset+motor[index_].error);
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


// motor_statusの内容をfloat型に変換してmotor_replyに格納
void AKROS_bridge_converter::pack_reply(AKROS_bridge_msgs::motor_reply_single &reply_, uint8_t index_){
    std::lock_guard<std::mutex> lock(motor_mutex);
    reply_.CAN_ID   = motor[index_].CAN_ID;
    reply_.position = signChange(uint_to_float(motor[index_].position + (motor[index_].offset+motor[index_].error), motor[index_].P_MIN, motor[index_].P_MAX, POSITION_BIT_NUM), motor[index_].inverseDirection);
    reply_.velocity = signChange(uint_to_float(motor[index_].velocity, motor[index_].V_MIN, motor[index_].V_MAX, VELOCITY_BIT_NUM), motor[index_].inverseDirection);
    reply_.effort   = signChange(uint_to_float(motor[index_].effort,   motor[index_].T_MIN, motor[index_].T_MAX, EFFORT_BIT_NUM), motor[index_].inverseDirection);
}


// motor_cmdをint値に変換してmotor_statusに格納
// ソフト上で可動角内で動くように制限を掛ける
void AKROS_bridge_converter::unpack_cmd(const AKROS_bridge_msgs::motor_cmd_single& cmd_){
    std::lock_guard<std::mutex> lock(motor_mutex);
    uint8_t index_ = find_index(cmd_.CAN_ID);

    // ソフト上で回転角を制限
    // 制限角がある場合は制限角を超えないようにフィルタ．
    // ロボットとモータの回転方向が逆だったら反転
    if(motor[index_].isLimitExist){
        motor[index_].position_ref = float_to_uint(fminf(fmaxf(motor[index_].lower_limit, signChange(cmd_.position, motor[index_].inverseDirection)), motor[index_].upper_limit), motor[index_].P_MIN, motor[index_].P_MAX, POSITION_BIT_NUM);
    }else{  // 制限がない場合は特にフィルタしない
        motor[index_].position_ref = float_to_uint(fminf(fmaxf(motor[index_].P_MIN, signChange(cmd_.position, motor[index_].inverseDirection)), motor[index_].P_MAX), motor[index_].P_MIN, motor[index_].P_MAX, POSITION_BIT_NUM);
    }
    
    motor[index_].velocity_ref = float_to_uint(fminf(fmaxf(motor[index_].V_MIN, signChange(cmd_.velocity, motor[index_].inverseDirection)), motor[index_].V_MAX), motor[index_].V_MIN, motor[index_].V_MAX, VELOCITY_BIT_NUM);
    motor[index_].effort_ref   = float_to_uint(fminf(fmaxf(motor[index_].T_MIN, signChange(cmd_.effort, motor[index_].inverseDirection)), motor[index_].T_MAX), motor[index_].T_MIN, motor[index_].T_MAX, EFFORT_BIT_NUM);
    motor[index_].Kp           = float_to_uint(fminf(fmaxf(KP_MIN, cmd_.Kp), KP_MAX), KP_MIN, KP_MAX, KP_BIT_NUM);
    motor[index_].Kd           = float_to_uint(fminf(fmaxf(KD_MIN, cmd_.Kd), KD_MAX), KD_MIN, KD_MAX, KD_BIT_NUM);
}


// motor_can_replyをそのままmotor_statusに格納
void AKROS_bridge_converter::unpack_can_reply(const AKROS_bridge_msgs::motor_can_reply_single& can_reply_){
    std::lock_guard<std::mutex> lock(motor_mutex);
    uint8_t index_ = find_index(can_reply_.CAN_ID);
    motor[index_].CAN_ID   = can_reply_.CAN_ID;     // 無駄かもしれないが...
    // 現在のデータを古いデータに変更
    //motor[index_].position_old = motor[index_].position;
    //motor[index_].velocity_old = motor[index_].velocity;
    //motor[index_].effort_old   = motor[index_].effort;
    // 最新のデータをmotor_statusに格納
    motor[index_].position = can_reply_.position;
    motor[index_].velocity = can_reply_.velocity;
    motor[index_].effort   = can_reply_.effort;
}


// motor_cmdを受け取ってmotor_statusに保存
void AKROS_bridge_converter::motor_cmd_Cb(const AKROS_bridge_msgs::motor_cmd::ConstPtr& cmd_){
    for(uint8_t i=0; i<cmd_->motor.size();i++){
        unpack_cmd(cmd_->motor[i]);
    }
    publish_can_cmd();
}


// /can_replyを受け取って実数値に変換し，motor_statusに格納
// 値がオーバーフローしていないかを確認
void AKROS_bridge_converter::can_reply_Cb(const AKROS_bridge_msgs::motor_can_reply::ConstPtr& can_reply_){
    for(uint8_t i=0; i<can_reply_->motor.size(); i++){
        unpack_can_reply(can_reply_->motor[i]);
    }

    #ifdef USE_TIMESTAMP
    reply.header = can_reply_->header;
    #endif
    
    publish_reply();
}



// 操作終了
// これを行うともう一度原点だしが必要になるので注意
bool AKROS_bridge_converter::exit_CM_Cb(AKROS_bridge_msgs::exit_control_mode::Request& req_, AKROS_bridge_msgs::exit_control_mode::Response& res_){
    AKROS_bridge_msgs::motor_config motor_config_srv;
    motor_config_srv.request.CAN_ID = req_.CAN_ID;
    motor_config_srv.request.configration_mode = EXIT_CONTROL_MODE;

    if(motor_config_client.call(motor_config_srv)){
        if(motor_config_srv.response.success)
            res_.success = true;
            return true;
    }
    return false;
}


// モータの原点と関節の原点との誤差値を設定
bool AKROS_bridge_converter::set_ZP_Cb(AKROS_bridge_msgs::set_position_zero::Request& req_, AKROS_bridge_msgs::set_position_zero::Response& res_){
    // error = joint - motor
    if(req_.CAN_ID == 0){   // 0なら全てのモータに対して
        for(auto& e : motor){
            e.error = CENTER_POSITION - e.position;
            e.position_ref = CENTER_POSITION;
        }
        ROS_INFO("Set zero position of all motors !");
    }else{
        motor[find_index(req_.CAN_ID)].error = CENTER_POSITION - motor[find_index(req_.CAN_ID)].position;
        motor[find_index(req_.CAN_ID)].position_ref = CENTER_POSITION;
        ROS_INFO("Set zero position of motor %d !", req_.CAN_ID);
    }
    
    res_.success = true;
    return true;
}


// motor_cmdのKp，Kdを0にしてモータのサーボをOFFにする
// サーボOFF時に実数目標値を0（整数値ではCENTER_〇〇）に設定
bool AKROS_bridge_converter::servo_setting_Cb(AKROS_bridge_msgs::servo_setting::Request& req_, AKROS_bridge_msgs::servo_setting::Response& res_){
    // CAN_ID = 0ならすべてのモータに対して一括設定
    if(req_.CAN_ID == 0){
        for(auto& e : motor){
            e.servo_mode = req_.servo;
        }

        if(req_.servo){
            ROS_INFO("All servo ON");
        }else{
            ROS_INFO("All servo OFF");
            // 指令値をすべて0に！
            for(auto& e : motor){
                // OFF->ONにしたときに0に動いてしまう！
                // e.position_ref = CENTER_POSITION;
                e.velocity_ref = CENTER_VELOCITY;
                e.effort_ref = CENTER_EFFORT;
            }
        }
    }else{  // そうでなければ個別に設定
        motor[find_index(req_.CAN_ID)].servo_mode = req_.servo;
        if(req_.servo){
            ROS_INFO("Motor %d servo ON", req_.CAN_ID);
        }else{
            ROS_INFO("Motor %d servo OFF", req_.CAN_ID);
            // OFF->ONにしたときに0に動いてしまう！
            // motor[find_index(req_.CAN_ID)].position_ref = CENTER_POSITION;
            motor[find_index(req_.CAN_ID)].velocity_ref = CENTER_VELOCITY;
            motor[find_index(req_.CAN_ID)].effort_ref = CENTER_EFFORT;
        }
    }
    
    res_.success = true;
    return true;
}


// 現在のモータ情報を返す
// controllerのはじめに取得するために使う
bool AKROS_bridge_converter::currentState_Cb(AKROS_bridge_msgs::currentState::Request& req_, AKROS_bridge_msgs::currentState::Response& res_){
    res_.reply.motor.resize(motor_num);

    for(int index_=0; index_<motor_num; index_++){
        res_.reply.motor[index_].CAN_ID   = motor[index_].CAN_ID;
        res_.reply.motor[index_].position = signChange(uint_to_float(motor[index_].position + (motor[index_].offset+motor[index_].error), motor[index_].P_MIN, motor[index_].P_MAX, POSITION_BIT_NUM), motor[index_].inverseDirection);
        res_.reply.motor[index_].velocity = uint_to_float(motor[index_].velocity, motor[index_].V_MIN, motor[index_].V_MAX, VELOCITY_BIT_NUM);
        res_.reply.motor[index_].effort   = uint_to_float(motor[index_].effort,   motor[index_].T_MIN, motor[index_].T_MAX, EFFORT_BIT_NUM);
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
void AKROS_bridge_converter::publish_can_cmd(void){
    if(initializeFlag){
        for(uint8_t i=0; i<motor.size(); i++){
            pack_can_cmd(can_cmd.motor[i], i);
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


