#include <AKROS_bridge_class.h>

AKROS_bridge_class::AKROS_bridge_class(ros::NodeHandle* n_)
  : myled(LED1),
    motor_status_pub("/reply/motor_status", &motor_reply_msg),
    motor_cmd_sub("/cmd/motor_cmd", &AKROS_bridge_class::motor_cmd_Cb, this),
    enter_control_mode_srv("/cmd/enter_control_mode", &AKROS_bridge_class::enter_control_mode_Cb, this),
    exit_control_mode_srv("/cmd/exit_control_mode", &AKROS_bridge_class::exit_control_mode_Cb, this),
    set_zero_pos_srv("/cmd/set_zero_pos", &AKROS_bridge_class::set_zero_pos_Cb, this)
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


void AKROS_bridge_class::motor_cmd_Cb(const AKROS_bridge::motor_cmd& cmd_){
    for(uint8_t i=0; i<motor_num; i++){
        can_controller.motor[i].q_ref   = cmd_.cmd.positions[i];
        can_controller.motor[i].dq_ref  = cmd_.cmd.velocities[i];
        can_controller.motor[i].tau_ref = cmd_.cmd.effort[i];
        can_controller.motor[i].Kp      = cmd_.Kp[i];
        can_controller.motor[i].Kd      = cmd_.Kd[i];
    }
}


void AKROS_bridge_class::enter_control_mode_Cb(const AKROS_bridge::Initialize::Request& req_, AKROS_bridge::Initialize::Response& res_){
    motor_num = req_.joint_num;
    res_.jointstate.position_length = motor_num;
    res_.jointstate.velocity_length = motor_num;
    res_.jointstate.effort_length   = motor_num;

    res_.jointstate.position = (double *)malloc(sizeof(double) * motor_num);
    res_.jointstate.velocity = (double *)malloc(sizeof(double) * motor_num);
    res_.jointstate.effort   = (double *)malloc(sizeof(double) * motor_num);

    can_controller.motor.resize(motor_num);
    // can_controller.attach();


    // モータにenter_control_modeを送信
    for(uint8_t i=0; i<motor_num; i++){
        CANMessage msg_;
        can_controller.enter_control_mode(i+1);
        wait_ms(10);
    }

    wait_ms(500);

    // motor_replyの値をROSに返す
    for(uint8_t i=0; i<motor_num; i++){
        res_.jointstate.position[i] = can_controller.motor[i].q;
        res_.jointstate.velocity[i] = can_controller.motor[i].dq;
        res_.jointstate.effort[i]   = can_controller.motor[i].tau;
    }

    myled = 1;
}


// exit control mode of motor-1
void AKROS_bridge_class::exit_control_mode_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    for(int i=0; i<motor_num; i++){
        can_controller.exit_control_mode(i+1);
        wait_ms(10);
    }
    myled = 0;
}


// set the angle of motor-1 to zero
void AKROS_bridge_class::set_zero_pos_Cb(const std_srvs::Empty::Request& req_, std_srvs::Empty::Response& res_){
    for(int i=0; i<motor_num; i++){
        can_controller.set_position_to_zero(i+1);
        wait_ms(10);
    }
}


uint8_t AKROS_bridge_class::getMotorNum(void){
    return motor_num;
}


// メインで回す部分
// motor_replyをPublish
// can_cmdを送信
void AKROS_bridge_class::loop(void){
    motor_status_pub.publish(&motor_reply_msg);
    
    for(uint8_t i=0; i<motor_num; i++){
        can_controller.can_send(i);
    }
}