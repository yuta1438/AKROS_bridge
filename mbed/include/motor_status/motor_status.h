// モータの状態をここで管理
#ifndef MOTOR_STATUS_H_
#define MOTOR_STATUS_H_

typedef struct motor_status_{
    float q;
    float q_ref;
    float dq;
    float dq_ref;
    float tau;
    float tau_ref;
    float Kp;
    float Kd;
} motor_status;
#endif