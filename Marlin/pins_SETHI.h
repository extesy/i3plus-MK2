#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
  #error "Oops!  Make sure you have 'Sethi 3D' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME "Sethi 3D_1"
#ifndef GEN7_VERSION
  #define GEN7_VERSION 12 
#endif
#define X_STOP_PIN          2
#define Y_STOP_PIN          0
#define Z_MIN_PIN           1
#define Z_MAX_PIN           0
#define X_STEP_PIN         19
#define X_DIR_PIN          18
#define X_ENABLE_PIN       24
#define Y_STEP_PIN         23
#define Y_DIR_PIN          22
#define Y_ENABLE_PIN       24
#define Z_STEP_PIN         26
#define Z_DIR_PIN          25
#define Z_ENABLE_PIN       24
#define E0_STEP_PIN        28
#define E0_DIR_PIN         27
#define E0_ENABLE_PIN      24
#define TEMP_0_PIN          1   
#define TEMP_BED_PIN        2   
#define HEATER_0_PIN        4
#define HEATER_BED_PIN      3
#if GEN7_VERSION >= 13
  #define FAN_PIN          -1
#else
  #define FAN_PIN          31
#endif
#define PS_ON_PIN          15
#define BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
#define DEBUG_PIN           0
#define TX_ENABLE_PIN      12
#define RX_ENABLE_PIN      13
