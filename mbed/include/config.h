#ifndef CONFIG_H_
#define CONFIG_H_

// System Configurations
#define ROS_FREQ        100 //[Hz]

// Pin Settings
#define CAN_RX_PIN  PB_8
#define CAN_TX_PIN  PB_9
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