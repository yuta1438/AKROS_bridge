#include "motor_status.h"

void motor_status::initialize(uint8_t n_){
    motor_num = n_;
    q.resize(motor_num);
    q_ref.resize(motor_num);
    dq.resize(motor_num);
    dq_ref.resize(motor_num);
    tau.resize(motor_num);
    tau_ref.resize(motor_num);
    // Kp.resize(motor_num);
    // Kd.resize(motor_num);
}

uint8_t motor_status::getSize(void){
    return motor_num;
}