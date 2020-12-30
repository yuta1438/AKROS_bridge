// モータの状態を構造体によって管理する
#ifndef MOTOR_STATUS_H_
#define MOTOR_STATUS_H_

typedef struct motor_status_{
    uint8_t id;
    bool control_mode;
    int position = 32767;
    int position_ref = 32767;
    int velocity = 2047;
    int velocity_ref = 2047; 
    int effort = 2047;
    int effort_ref = 2047;
    int Kp = 0;     // 初期状態では脱力
    int Kd = 0;
} motor_status;
#endif