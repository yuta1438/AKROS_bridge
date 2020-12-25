#include <AKROS_bridge/AKROS_bridge.h>

AKROS_bridge::AKROS_bridge(ros::NodeHandle* n_)
  : myled(LED1),
    tweak_toggle(TWEAK_TOGGLE_PIN),
    tweak_tact_up(TWEAK_TACT_UP_PIN),
    tweak_tact_up(TWEAK_TACT_DOWN_PIN),
    can_reply_pub("/can_reply", &can_reply_msg),
    can_cmd_sub("/can_cmd", &AKROS_bridge::can_cmd_Cb, this),
    enter_control_mode_srv("initialize_can", &AKROS_bridge::enter_control_mode_Cb, this),
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
void AKROS_bridge::motor_cmd_Cb(const AKROS_bridge_msgs::motor_can_cmd& cmd_){
    for(uint8_t i=0; i<motor_num; i++){
        can_controller.motor[i].position_ref = cmd_.motor[i].position;
        can_controller.motor[i].velocity_ref = cmd_.motor[i].velocity;
        can_controller.motor[i].effort_ref   = cmd_.motor[i].effort;
        can_controller.motor[i].Kp           = cmd_.motor[i].Kp;
        can_controller.motor[i].Kd           = cmd_.motor[i].Kd;
    }
}


// モータ起動サービスに対するserverのコールバック関数
void AKROS_bridge::enter_control_mode_Cb(const AKROS_bridge_msgs::Initialize_can::Request& req_, AKROS_bridge_msgs::Initialize_can::Response& res_){
    motor_num = req_.joint_num;
    can_controller.motor.resize(motor_num);
    
    // モータにenter_control_modeを送信
    for(uint8_t i=0; i<motor_num; i++){
        CANMessage msg_;
        can_controller.enter_control_mode(i+1);
        wait_ms(50);
    }

    wait_ms(500);

    // motor_replyの値をROSに返す
    // motor_statusから値を持ってきてpublish
    for(uint8_t i=0; i<motor_num; i++){
        res_.can_reply.motor[i].position = can_controller.motor[i].position;
        res_.can_reply.motor[i].velocity = can_controller.motor[i].velocity;
        res_.can_reply.motor[i].effort   = can_controller.motor[i].effort;
    }

    can_controller.initializeFlag = true;
    myled = 1;
}


// モータ終了サービスに対するserverのコールバック関数
void AKROS_bridge::exit_control_mode_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    for(uint8_t i=0; i<motor_num; i++){
        can_controller.exit_control_mode(i+1);
        wait_ms(10);
    }
    myled = 0;
}


// 原点設定サービスに対するserverのコールバック関数
void AKROS_bridge::set_zero_pos_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    for(uint8_t i=0; i<motor_num; i++){
        can_controller.set_position_to_zero(i+1);
        wait_ms(10);
    }
}


// モータの個数を返す
// モータの個数は「enter_control_mode」サービスのrequestで設定する必要あり
uint8_t AKROS_bridge::getMotorNum(void){
    return motor_num;
}


// メインで回す部分
// PCにmotor_replyをPublish
// モータにcan_cmdを送信
void AKROS_bridge::loop(void){
    if(can_controller.initializeFlag){
        for(uint8_t i=0; i<motor_num; i++){
            __disable_irq();
            can_controller.can_send(i);
            __enable_irq();
            can_reply_msg.motor[i].position = can_controller.motor[i].position;
            can_reply_msg.motor[i].velocity = can_controller.motor[i].velocity;
            can_reply_msg.motor[i].effort   = can_controller.motor[i].effort;
        }
        motor_reply_pub.publish(&can_reply_msg);
    }
}