#ifndef INITIALIZE_SETTINGS_H_
#define INITIALIZE_SETTINGS_H_

// configuration modes
#define ENTER_CONTROL_MODE      1
#define EXIT_CONTROL_MODE       2
#define SET_POSITION_TO_ZERO    3
#define INITIALIZE_LOCK         4
#define SERVO_OFF               5

// CAN_IDs
enum CAN_ID {
    HIP_MOTOR   = 1,
    KNEE_MOTOR  = 2,
    WHEEL_MOTOR = 3,
    
    MOTOR_NUM = 3
};

#endif