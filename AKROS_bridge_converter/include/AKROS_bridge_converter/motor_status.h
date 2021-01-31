#ifndef MOTOR_STATUS_H_
#define MOTOR_STATUS_H_
#include <stdint.h>


// models
#define AK10_9          0
#define AK80_6          1
#define AK10_9_OLD      2
#define AK80_6_OLD      3

#define AK10_9_P_MIN   -12.5f
#define AK10_9_P_MAX   12.5f
#define AK80_6_P_MIN   -12.5f
#define AK80_6_P_MAX   12.5f

#define AK10_9_OLD_P_MIN   -95.5f
#define AK10_9_OLD_P_MAX   95.5f
#define AK80_6_OLD_P_MIN   -95.5f
#define AK80_6_OLD_P_MAX   95.5f


/*
#define V_MIN   -30.0f
#define V_MAX   30.0f
*/

// for AK80-6(48V)
#define AK80_6_V_MIN   -45.0
#define AK80_6_V_MAX   45.0

#define KP_MIN   0.0f
#define KP_MAX   500.0f
#define KD_MIN   0.0f
#define KD_MAX   5.0f
#define T_MIN   -18.0f
#define T_MAX   18.0f

#define POSITION_BIT_NUM    16
#define VELOCITY_BIT_NUM    12
#define EFFORT_BIT_NUM      12
#define KP_BIT_NUM          12
#define KD_BIT_NUM          12

// modelについての注意事項
// AK10-9, AK80-6           ->  P_MAX = 12.5
// AK10-9_old, AK80-6_old   ->  P_MAX = 95.5

// 実数で0に対応する値
#define CENTER_POSITION     32768
#define CENTER_VELOCITY     2048
#define CENTER_EFFORT       2048

typedef struct motor_status_{
    std::string name;   // モータ名前
    std::string model;  // モータ型番
    
    uint8_t CAN_ID = 0; 
    bool servo_mode = false;    // サーボ状態
    int error = 0;  // モータの原点と関節の原点の誤差値
    
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