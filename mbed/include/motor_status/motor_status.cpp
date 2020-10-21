#include "motor_status.h"

motor_status::motor_status(){};

void motor_status::initialize(uint8_t n_){
    motor_num = n_;
    q.resize(motor_num);
    q_ref.resize(motor_num);
    dq.resize(motor_num);
    dq_ref.resize(motor_num);
    effort.resize(motor_num);
    effort_ref.resize(motor_num);
    // Kp.resize(motor_num);
    // Kd.resize(motor_num);
}

uint8_t motor_status::size(void){
    return motor_num;
}