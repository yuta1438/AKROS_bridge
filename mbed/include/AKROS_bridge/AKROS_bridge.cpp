#include <AKROS_bridge.h>

AKROS_bridge::AKROS_bridge(ros::NodeHandle* n_)
  : Nucleo_led(LED1),
    tweak_toggle(TWEAK_TOGGLE_PIN),
    tweak_tact_up(TWEAK_TACT_UP_PIN),
    tweak_tact_down(TWEAK_TACT_DOWN_PIN),
    can_reply_pub("can_reply", &can_reply_msg),
    can_cmd_sub("can_cmd", &AKROS_bridge::can_cmd_Cb, this),
    motor_config_srv("motor_config", &AKROS_bridge::motor_config_Cb, this),
    currentState_srv("current_state", &AKROS_bridge::currentState_Cb, this)
    {
    wait_ms(10);
    nh_priv = n_;
    Nucleo_led = 0;
    
    // 各種トピック，サービスの設定
    nh_priv->advertise(can_reply_pub);
    nh_priv->subscribe(can_cmd_sub);
    nh_priv->advertiseService(motor_config_srv);
    nh_priv->advertiseService(currentState_srv);
    wait_ms(10);
}


// モータCAN指令に対するsubscriberのコールバック関数
// 指令を受け取ったらcan_controllerクラスのメンバ変数（motor）に格納
void AKROS_bridge::can_cmd_Cb(const AKROS_bridge_msgs::motor_can_cmd& cmd_){
    for(uint8_t i=0; i<can_controller.getMotorNum(); i++){
        uint8_t index_ = can_controller.find_index(cmd_.motor[i].id);
        can_controller.motor[index_].position_ref = cmd_.motor[i].position;
        can_controller.motor[index_].velocity_ref = cmd_.motor[i].velocity;
        can_controller.motor[index_].effort_ref   = cmd_.motor[i].effort;
        can_controller.motor[index_].Kp           = cmd_.motor[i].Kp;
        can_controller.motor[index_].Kd           = cmd_.motor[i].Kd;
    }
}


// 各種モータ設定
// configuration_mode 
// 1 -> enter_control_mode
// 2 -> exit_control_mode
// 3 -> set_position_to_zero
// 4 -> initialize_lock
// 5 -> servo Off
void AKROS_bridge::motor_config_Cb(const AKROS_bridge_msgs::motor_config::Request& req_, AKROS_bridge_msgs::motor_config::Response& res_){
    __disable_irq();
    switch(req_.configration_mode){
        case ENTER_CONTROL_MODE:
            // すでに初期化が終わっていたらこれ以上のモータ追加を許可しない
            // msg, srvの初期値は0なのでこれに入らなかったらfalseが返される！
            if(!can_controller.getInitializeFlag()){
                can_controller.add_motor(req_.CAN_ID);
                can_controller.enter_control_mode(req_.CAN_ID);
                res_.success = true;
            }
            break;

        case EXIT_CONTROL_MODE:
            can_controller.exit_control_mode(req_.CAN_ID);
            wait_ms(10);
            res_.success = true;
            break;

        case SET_POSITION_TO_ZERO:
            can_controller.set_position_to_zero(req_.CAN_ID);
            wait_ms(10);
            res_.success = true;
            break;

        case INITIALIZE_LOCK:
            // Publishするトピック用にメモリ確保
            can_reply_msg.motor_length = can_controller.getMotorNum();
            can_reply_msg.motor = new AKROS_bridge_msgs::motor_can_reply::_motor_type[can_reply_msg.motor_length];

            can_controller.setInitializeFlag(true);
            Nucleo_led = 1;
            res_.success = true;
            break;

        case SERVO_OFF: 
            // set servo of the motor OFF (Kp = Kd = 0.0)
            if(can_controller.find_index(req_.CAN_ID) != ERROR_VALUE){
                can_controller.motor[can_controller.find_index(req_.CAN_ID)].Kp = 0.0;
                can_controller.motor[can_controller.find_index(req_.CAN_ID)].Kd = 0.0;
                res_.success = true;
            }
            break;

        default:
            res_.success = false;
            break;
    }
    __enable_irq();
}


// 現在のモータ情報を返す
// Clientのcallでは必ずif(service.call())とすること！
void AKROS_bridge::currentState_Cb(const AKROS_bridge_msgs::currentState::Request& req_, AKROS_bridge_msgs::currentState::Response& res_){
   if(can_controller.find_index(req_.CAN_ID) != ERROR_VALUE){
       can_controller.unpack_reply(res_.reply, req_.CAN_ID);
       res_.success = true;
   }else{
       res_.reply.id = ERROR_VALUE;
       res_.reply.position = -1.0;
       res_.reply.velocity = -1.0;
       res_.reply.effort   = -1.0;
       res_.success = false;
   }
}


// メインで回す部分
// PCにmotor_replyをPublish
// モータにcan_cmdを送信
void AKROS_bridge::loop(void){
    if(can_controller.getInitializeFlag()){
        for(uint8_t i=0; i<can_controller.getMotorNum(); i++){
            // 割り込み禁止
            // __disable_irq();

            // 
            if(can_controller.motor[i].control_mode){
                can_controller.can_send(i);
            }
            // 割り込み許可
            // __enable_irq();

            // can_reply_pub
            can_reply_msg.motor[i].id = can_controller.motor[i].id;
            can_reply_msg.motor[i].position = can_controller.motor[i].position;
            can_reply_msg.motor[i].velocity = can_controller.motor[i].velocity;
            can_reply_msg.motor[i].effort   = can_controller.motor[i].effort;
        }
        can_reply_pub.publish(&can_reply_msg);
    }
    else{
        wait_ms(10);
    }
}