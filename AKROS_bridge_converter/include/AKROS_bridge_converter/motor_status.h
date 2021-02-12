#ifndef MOTOR_STATUS_H_
#define MOTOR_STATUS_H_
#include <stdint.h>

// AK-series models
/*
#define AK10_9          0
#define AK80_6          1
#define AK10_9_OLD      2
#define AK80_6_OLD      3
*/
/*
enum models{
    AK10_9,
    AK80_6,
    AK10_9_OLD,
    AK80_6_OLD,
    num
};*/

// AK10-9(new)
#define AK10_9_P_MIN   -12.5f
#define AK10_9_P_MAX   12.5f
#define AK10_9_V_MIN   -45.0f
#define AK10_9_V_MAX   45.0f

// AK80-6(new)
#define AK80_6_P_MIN   -12.5f
#define AK80_6_P_MAX   12.5f
#define AK80_6_V_MIN   -45.0f
#define AK80_6_V_MAX   45.0f

// AK10-9(old)
#define AK10_9_OLD_P_MIN   -95.5f
#define AK10_9_OLD_P_MAX   95.5f
#define AK10_9_OLD_V_MIN   -12.5f
#define AK10_9_OLD_V_MAX   12.5f

// AK80-6(old)
#define AK80_6_OLD_P_MIN   -95.5f
#define AK80_6_OLD_P_MAX   95.5f
#define AK80_6_OLD_V_MIN   -22.5f
#define AK80_6_OLD_V_MAX   22.5f

// Gain Parameters
#define KP_MIN   0.0f
#define KP_MAX   500.0f
#define KD_MIN   0.0f
#define KD_MAX   5.0f
#define T_MIN   -18.0f
#define T_MAX   18.0f

// number of bits
#define POSITION_BIT_NUM    16
#define VELOCITY_BIT_NUM    12
#define EFFORT_BIT_NUM      12
#define KP_BIT_NUM          12
#define KD_BIT_NUM          12

// 実数で0に対応する値
#define CENTER_POSITION     32768
#define CENTER_VELOCITY     2048
#define CENTER_EFFORT       2048


typedef struct motor_status_{
    std::string name;   // モータ名前
    std::string model;  // モータ型番
    uint8_t CAN_ID = 0; // モータのCAN_ID．初期値は0．

    bool servo_mode = false;    // サーボ状態
    bool isExceedLimit = false; // 可動角を
    int error = 0;              // モータの原点と関節の原点の誤差値（デジタル値）．初期値は0．
    
    float P_MAX;
    float P_MIN;
    float V_MAX;
    float V_MIN;

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
