#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega1284P__)
  #error "Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME         "Gen3+"
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
#define E0_STEP_PIN        17
#define E0_DIR_PIN         21
#define E0_ENABLE_PIN      13
#define TEMP_0_PIN          0   
#define TEMP_BED_PIN        5   
#define HEATER_0_PIN       12
#define HEATER_BED_PIN     16
#define SDSS                4
#define PS_ON_PIN          14
