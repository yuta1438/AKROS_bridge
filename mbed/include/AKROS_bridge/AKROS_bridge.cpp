#include <AKROS_bridge.h>

AKROS_bridge::AKROS_bridge(ros::NodeHandle* n_)
  : myled(LED1),
    motor_status_pub("/reply/motor_status", &motor_reply_msg),
    motor_cmd_sub("/cmd/motor_cmd", &AKROS_bridge::motor_cmd_Cb, this),
    enter_control_mode_srv("/cmd/enter_control_mode", &AKROS_bridge::enter_control_mode_Cb, this),
    exit_control_mode_srv("/cmd/exit_control_mode", &AKROS_bridge::exit_control_mode_Cb, this),
    set_zero_pos_srv("/cmd/set_zero_pos", &AKROS_bridge::set_zero_pos_Cb, this)
{
    wait_ms(10);
    nh_priv = n_;
    myled = 0;
    
    nh_priv->advertise(motor_status_pub);
    nh_priv->subscribe(motor_cmd_sub);
    nh_priv->advertiseService(enter_control_mode_srv);
    nh_priv->advertiseService(exit_control_mode_srv);
    nh_priv->advertiseService(set_zero_pos_srv);

    wait_ms(10);
}


void AKROS_bridge::motor_cmd_Cb(const AKROS_bridge_msgs::motor_cmd& cmd_){
    for(uint8_t i=0; i<motor_num; i++){
        can_controller.motor[i].q_ref   = cmd_.cmd.positions[i];
        can_controller.motor[i].dq_ref  = cmd_.cmd.velocities[i];
        can_controller.motor[i].tau_ref = cmd_.cmd.effort[i];
        can_controller.motor[i].Kp      = cmd_.Kp[i];
        can_controller.motor[i].Kd      = cmd_.Kd[i];
    }
}


void AKROS_bridge::enter_control_mode_Cb(const AKROS_bridge_msgs::Initialize::Request& req_, AKROS_bridge_msgs::Initialize::Response& res_){
    motor_num = req_.joint_num;

    // responseのjointstateに対するメモリ動的確保
    res_.jointstate.position_length = motor_num;
    res_.jointstate.velocity_length = motor_num;
    res_.jointstate.effort_length   = motor_num;
    res_.jointstate.position = (double *)malloc(sizeof(double) * motor_num);
    res_.jointstate.velocity = (double *)malloc(sizeof(double) * motor_num);
    res_.jointstate.effort   = (double *)malloc(sizeof(double) * motor_num);

    // motor_reply_msgのjointstateに対するメモリ動的確保
    motor_reply_msg.state.position_length = motor_num;
    motor_reply_msg.state.velocity_length = motor_num;
    motor_reply_msg.state.effort_length   = motor_num;
    motor_reply_msg.state.position = (double *)malloc(sizeof(double) * motor_num);
    motor_reply_msg.state.velocity = (double *)malloc(sizeof(double) * motor_num);
    motor_reply_msg.state.effort   = (double *)malloc(sizeof(double) * motor_num);

    can_controller.motor.resize(motor_num);

    for(uint8_t i=0; i<motor_num; i++){
        res_.jointstate.position[i] = 0.0;
        res_.jointstate.velocity[i] = 0.0;
        res_.jointstate.effort[i] = 0.0;
    }


    // モータにenter_control_modeを送信
    for(uint8_t i=0; i<motor_num; i++){
        CANMessage msg_;
        can_controller.enter_control_mode(i+1);
        wait_ms(50);
    }

    wait_ms(500);

    // motor_replyの値をROSに返す
    for(uint8_t i=0; i<motor_num; i++){
        res_.jointstate.position[i] = can_controller.motor[i].q;
        res_.jointstate.velocity[i] = can_controller.motor[i].dq;
        res_.jointstate.effort[i]   = can_controller.motor[i].tau;
    }

    can_controller.initializeFlag = true;
    myled = 1;
}


// exit control mode of motor-1
void AKROS_bridge::exit_control_mode_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    for(uint8_t i=0; i<motor_num; i++){
        can_controller.exit_control_mode(i+1);
        wait_ms(10);
    }
    myled = 0;
}


// set the angle of motor-1 to zero
void AKROS_bridge::set_zero_pos_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    for(uint8_t i=0; i<motor_num; i++){
        can_controller.set_position_to_zero(i+1);
        wait_ms(10);
    }
}


uint8_t AKROS_bridge::getMotorNum(void){
    return motor_num;
}


// メインで回す部分
// motor_replyをPublish
// can_cmdを送信
void AKROS_bridge::loop(void){
    if(can_controller.initializeFlag){
        for(uint8_t i=0; i<motor_num; i++){
            __disable_irq();
            can_controller.can_send(i);
            __enable_irq();
            motor_reply_msg.state.position[i] = can_controller.motor[i].q;
            motor_reply_msg.state.velocity[i] = can_controller.motor[i].dq;
            motor_reply_msg.state.effort[i]   = can_controller.motor[i].tau;
        }
        motor_status_pub.publish(&motor_reply_msg);
    }
}