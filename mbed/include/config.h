#ifndef CONFIG_H_
#define CONFIG_H_

// System Configurations
#define ROS_FREQ        100 //[Hz]

// STM_Board
#define NUCLEO_F303K8   0
#define NUCLEO_F446RE   1

// select your target board
#define TARGET_BOARD    NUCLEO_F446RE
//#define TARGET_BOARD    NUCLEO_F303K8


// Pin Settings
#define TWEAK_TOGGLE_PIN     D7
#define TWEAK_TACT_UP_PIN    D8
#define TWEAK_TACT_DOWN_PIN  D9

// Value Settings
#define P_MIN   -95.5f
#define P_MAX   95.5f
#define V_MIN   -30.0f
#define V_MAX   30.0f
#define KP_MIN   0.0f
#define KP_MAX   500.0f
#define KD_MIN   0.0f
#define KD_MAX   5.0f
#define T_MIN   -18.0f
#define T_MAX   18.0f

#endif