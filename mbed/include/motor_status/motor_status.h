// モータの状態を構造体によって管理する
#ifndef MOTOR_STATUS_H_
#define MOTOR_STATUS_H_

typedef struct motor_status_{
    uint8_t id;
    uint16_t position;
    uint16_t position_ref;
    uint16_t velocity;
    uint16_t velocity_ref;
    uint16_t effort;
    uint16_t effort_ref;
    uint16_t Kp;
    uint16_t Kd;
} motor_status;
#endif