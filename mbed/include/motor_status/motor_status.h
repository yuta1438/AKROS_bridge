// モータの状態をここで管理
#ifndef MOTOR_STATUS_H
#define MOTOR_STATUS_H_

#include "mbed.h"
#include <vector>

class motor_status{
public:
    motor_status(){}
    ~motor_status(){};
    float q;
    float q_ref;
    float dq;
    float dq_ref;
    float tau;
    float tau_ref;
    float Kp;
    float Kd;
};
#endif