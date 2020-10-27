// モータの状態をここで管理

#ifndef MOTOR_STATUS_H
#define MOTOR_STATUS_H_

#include "mbed.h"
#include <vector>

class motor_status{
private:
    uint8_t motor_num;

public:
    motor_status(){}
    ~motor_status(){};
    std::vector<float> q;
    std::vector<float> q_ref;
    std::vector<float> dq;
    std::vector<float> dq_ref;
    std::vector<float> tau;
    std::vector<float> tau_ref;

    void initialize(uint8_t);
    uint8_t getSize(void);
};
#endif