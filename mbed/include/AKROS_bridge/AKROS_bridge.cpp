#include <AKROS_bridge.h>

AKROS_bridge::AKROS_bridge(ros::NodeHandle* n_)
  : myled(LED1),
    tweak_toggle(TWEAK_TOGGLE_PIN),
    tweak_tact_up(TWEAK_TACT_UP_PIN),
    tweak_tact_down(TWEAK_TACT_DOWN_PIN),
    can_reply_pub("/can_reply", &can_reply_msg),
    can_cmd_sub("/can_cmd", &AKROS_bridge::can_cmd_Cb, this),
    enter_control_mode_srv("initialize_can", &AKROS_bridge::enter_control_mode_Cb, this),
    initialize_lock_srv("initialize_can_lock", &AKROS_bridge::initialize_lock_Cb, this),
    exit_control_mode_srv("exit_control_mode", &AKROS_bridge::exit_control_mode_Cb, this),
    set_zero_pos_srv("set_zero_pos", &AKROS_bridge::set_zero_pos_Cb, this)
    {
    wait_ms(10);
    nh_priv = n_;
    myled = 0;
    
    // 各種トピック，サービスの設定
    nh_priv->advertise(can_reply_pub);
    nh_priv->subscribe(can_cmd_sub);
    nh_priv->advertiseService(enter_control_mode_srv);
    nh_priv->advertiseService(exit_control_mode_srv);
    nh_priv->advertiseService(set_zero_pos_srv);

    wait_ms(10);
}


// モータCAN指令に対するsubscriberのコールバック関数
// 指令を受け取ったらcan_controllerクラスのメンバ変数（motor）に格納
void AKROS_bridge::can_cmd_Cb(const AKROS_bridge_msgs::motor_can_cmd& cmd_){
    for(uint8_t i=0; i<can_controller.motor.size(); i++){
        can_controller.motor[i].position_ref = cmd_.motor[i].position;
        can_controller.motor[i].velocity_ref = cmd_.motor[i].velocity;
        can_controller.motor[i].effort_ref   = cmd_.motor[i].effort;
        can_controller.motor[i].Kp           = cmd_.motor[i].Kp;
        can_controller.motor[i].Kd           = cmd_.motor[i].Kd;
    }
}


// 初期化を終了する
void AKROS_bridge::initialize_lock_Cb(const AKROS_bridge_msgs::Initialize_lock::Request&, AKROS_bridge_msgs::Initialize_lock::Response&){
    // メモリの動的確保の方法がわからん
    can_reply_msg.motor_length = can_controller.getMotorNum();
    can_reply_msg.motor = (can_reply_msg.motor *)malloc(can_reply_msg.motor_length * sizeof(can_reply_msg.motor));
    myled = 1;
}

// モータ起動サービスに対するserverのコールバック関数
// モータ1つに対して行う
void AKROS_bridge::enter_control_mode_Cb(const AKROS_bridge_msgs::Initialize_can::Request& req_, AKROS_bridge_msgs::Initialize_can::Response& res_){
    can_controller.enter_control_mode(req_.CAN_ID);
    wait_ms(10);

    // motor_replyの値をROSに返す
    // motor_statusから値を持ってきてpublish
    uint8_t iterator = can_controller.find_iterator(req_.CAN_ID);
    res_.can_reply.position = can_controller.motor[iterator].position;
    res_.can_reply.velocity = can_controller.motor[iterator].velocity;
    res_.can_reply.effort   = can_controller.motor[iterator].effort;
}


// モータ終了サービスに対するserverのコールバック関数
void AKROS_bridge::exit_control_mode_Cb(const AKROS_bridge_msgs::finalize::Request& req_, AKROS_bridge_msgs::finalize::Response& res_){
    can_controller.exit_control_mode(req_.CAN_ID);
    wait_ms(10);
}


// 原点設定サービスに対するserverのコールバック関数
void AKROS_bridge::set_zero_pos_Cb(const AKROS_bridge_msgs::set_zero_pos::Request& req_, AKROS_bridge_msgs::set_zero_pos::Response& res_){
    can_controller.set_position_to_zero(req_.CAN_ID);
    wait_ms(10);
}



// メインで回す部分
// PCにmotor_replyをPublish
// モータにcan_cmdを送信
void AKROS_bridge::loop(void){
    if(can_controller.getInitializeFlag()){
        for(uint8_t i=0; i<can_controller.getMotorNum(); i++){
            __disable_irq();
            can_controller.can_send(i);
            __enable_irq();
            can_reply_msg.motor[i].position = can_controller.motor[i].position;
            can_reply_msg.motor[i].velocity = can_controller.motor[i].velocity;
            can_reply_msg.motor[i].effort   = can_controller.motor[i].effort;
        }
        can_reply_pub.publish(&can_reply_msg);
    }
}