#ifndef CONFIG_H_
#define CONFIG_H_

// System Configurations
#define ROS_FREQ        100 //[Hz]

// Pin Settings
#define CAN_RX_PIN  PB_8
#define CAN_TX_PIN  PB_9


#define GREEN_LED_PIN   D13 // = PA_5
#define YELLOW_LED_PIN  D12 // = PA_6
#define RED_LED_PIN     D11 // = PA_7

/*
#define TWEAK_TOGGLE1_PIN   PC_8
#define TWEAK_TOGGLE2_PIN   PC_9
#define TWEAK_TACT_UP_PIN   PC_5    // or PC_6
#define TWEAK_TACT_DOWN_PIN PC_6    // or PC_5
*/

// configuration modes
#define ENTER_CONTROL_MODE      1
#define EXIT_CONTROL_MODE       2
#define SET_POSITION_TO_ZERO    3
#define INITIALIZE_LOCK         4


#define ERROR_VALUE         99
#endif