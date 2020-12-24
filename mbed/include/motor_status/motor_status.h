// モータの状態を構造体によって管理する
#ifndef MOTOR_STATUS_H_
#define MOTOR_STATUS_H_

typedef struct motor_status_{
    uint8_t position;
    uint8_t position_ref;
    uint8_t velocity;
    uint8_t velocity_ref;
    uint8_t effort;
    uint8_t effort_ref;
    uint8_t Kp;
    uint8_t Kd;
} motor_status;
#endif