#ifndef MOTOR_STATUS_H_
#define MOTOR_STATUS_H_
#include <stdint.h>

// 実数で0に対応する値
#define CENTER_POSITION     32768
#define CENTER_VELOCITY     2048
#define CENTER_EFFORT       2048

typedef struct motor_status_{
    uint8_t CAN_ID = 0;
    bool servo_mode = false;    // サーボ状態
    int16_t error = 0;  // モータの原点と関節の原点の誤差値
    uint16_t position = CENTER_POSITION;
    uint16_t position_ref = CENTER_POSITION;
    uint16_t velocity = CENTER_VELOCITY;
    uint16_t velocity_ref = CENTER_VELOCITY;
    uint16_t effort = CENTER_EFFORT;
    uint16_t effort_ref = CENTER_EFFORT;
    uint16_t Kp = 0;
    uint16_t Kd = 0;
}motor_status;

#endif