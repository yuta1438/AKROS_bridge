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