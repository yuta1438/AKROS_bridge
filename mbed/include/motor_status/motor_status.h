// モータの状態をここで管理

#ifndef MOTOR_STATUS_H
#define MOTOR_STATUS_H_

#include "mbed.h"
#include <vector>

class motor_status{
private:
    uint8_t motor_num;

public:
    motor_status();
    std::vector<float> q;
    std::vector<float> q_ref;
    std::vector<float> dq;
    std::vector<float> dq_ref;
    std::vector<float> effort;
    std::vector<float> effort_ref;

    void initialize(uint8_t);
    uint8_t size(void);
};
#endif