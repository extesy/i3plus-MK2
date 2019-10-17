#ifndef __AVR_ATmega644P__
  #ifndef __AVR_ATmega1284P__
    #error "Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu."
  #endif
#endif
#ifndef BOARD_NAME
  #define BOARD_NAME "Gen6"
#endif
#define X_STOP_PIN         20
#define Y_STOP_PIN         25
#define Z_STOP_PIN         30
#define X_STEP_PIN         15
#define X_DIR_PIN          18
#define X_ENABLE_PIN       19
#define Y_STEP_PIN         23
#define Y_DIR_PIN          22
#define Y_ENABLE_PIN       24
#define Z_STEP_PIN         27
#define Z_DIR_PIN          28
#define Z_ENABLE_PIN       29
#define E0_STEP_PIN         4   
#define E0_DIR_PIN          2   
#define E0_ENABLE_PIN       3   
#define TEMP_0_PIN          5   
#define HEATER_0_PIN       14   
#if !MB(GEN6)
  #define HEATER_BED_PIN    1   
  #define TEMP_BED_PIN      0   
#endif
#define SDSS               17
#define DEBUG_PIN           0
#define CASE_LIGHT_PIN     16   
#define TX_ENABLE_PIN      12
#define RX_ENABLE_PIN      13
#define SPINDLE_LASER_ENABLE_PIN  5   
#define SPINDLE_LASER_PWM_PIN    16   
#define SPINDLE_DIR_PIN           6
