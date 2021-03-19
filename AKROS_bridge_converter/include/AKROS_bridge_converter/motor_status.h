#ifndef MOTOR_STATUS_H_
#define MOTOR_STATUS_H_
#include <stdint.h>
#include <string>

#include <AKROS_bridge_converter/motors/AK10-9.h>
#include <AKROS_bridge_converter/motors/AK80-6.h>
#include <AKROS_bridge_converter/motors/AK10-9_old.h>
#include <AKROS_bridge_converter/motors/AK80-6_old.h>

// Gain Parameters
#define KP_MIN   0.0f
#define KP_MAX   500.0f
#define KD_MIN   0.0f
#define KD_MAX   5.0f

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
    bool isExceedLimit = false; // 可動角を超えているかどうか？
    int error = 0;              // モータの原点と関節の原点の誤差値（デジタル値）．初期値は0．
    int offset = 0;             // モータのゼロ点合わせ位置と実際のゼロ点との差（符号注意！）

    float P_MAX;
    float P_MIN;
    float V_MAX;
    float V_MIN;
    float T_MAX;
    float T_MIN;

    // 可動角
    bool isLimitExist = false;  // 無限回転可能かそうでないか？
    bool inverseDirection = false;  // モータの回転を逆にする
    float upper_limit;  // 上限可動角[rad]
    float lower_limit;  // 下限可動角[rad]

    uint16_t position = CENTER_POSITION;
    uint16_t position_old = CENTER_POSITION;
    uint16_t position_ref = CENTER_POSITION;
    int position_overflow_count = 0; // 上限下限を超えた回数

    uint16_t velocity = CENTER_VELOCITY;
    uint16_t velocity_old = CENTER_VELOCITY;
    uint16_t velocity_ref = CENTER_VELOCITY;
    int velocity_overflow_count = 0;

    uint16_t effort = CENTER_EFFORT;
    uint16_t effort_old = CENTER_EFFORT;
    uint16_t effort_ref = CENTER_EFFORT;
    int effort_overflow_count = 0;

    uint16_t Kp = 0;
    uint16_t Kd = 0;
}motor_status;

#endif
