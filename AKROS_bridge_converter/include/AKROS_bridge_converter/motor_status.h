#ifndef MOTOR_STATUS_H_
#define MOTOR_STATUS_H_
#include <stdint.h>

typedef struct motor_status_{
    uint8_t CAN_ID = 0;
    bool servo_mode = false;    // サーボ状態
    uint16_t position = 32767;
    uint16_t position_ref = 32767;
    uint16_t velocity = 2047;
    uint16_t velocity_ref = 2047;
    uint16_t effort = 2047;
    uint16_t effort_ref = 2047;
    uint16_t Kp = 0.0;
    uint16_t Kd = 0.0;
}motor_status;

#endif