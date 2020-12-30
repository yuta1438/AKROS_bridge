#ifndef MOTOR_STATUS_H_
#define MOTOR_STATUS_H_
#include <stdint.h>

typedef struct motor_status_{
    uint8_t CAN_ID;
    bool servo_mode;    // サーボ状態
    float position;
    float position_ref;
    float velocity;
    float velocity_ref;
    float effort;
    float effort_ref;
    float Kp;
    float Kd;
}motor_status;

#endif