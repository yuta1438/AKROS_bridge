// モータの状態を構造体によって管理する
#ifndef MOTOR_STATUS_H_
#define MOTOR_STATUS_H_

typedef struct motor_status_{
    uint8_t id = 0;
    float q = 0.0;
    float q_ref = 0.0;
    float dq = 0.0;
    float dq_ref = 0.0;
    float tau = 0.0;
    float tau_ref = 0.0;
    float Kp = 0.0;
    float Kd = 0.0;
} motor_status;
#endif