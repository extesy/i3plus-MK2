#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME         "Leapfrog"
#define X_MIN_PIN          47
#define X_MAX_PIN           2
#define Y_MIN_PIN          48
#define Y_MAX_PIN          15
#define Z_MIN_PIN          49
#define Z_MAX_PIN          -1
#define X_STEP_PIN         28
#define X_DIR_PIN          63
#define X_ENABLE_PIN       29
#define Y_STEP_PIN         14 
#define Y_DIR_PIN          15 
#define Y_ENABLE_PIN       39
#define Z_STEP_PIN         31 
#define Z_DIR_PIN          32 
#define Z_ENABLE_PIN       30 
#define E0_STEP_PIN        34 
#define E0_DIR_PIN         35 
#define E0_ENABLE_PIN      33 
#define E1_STEP_PIN        37 
#define E1_DIR_PIN         40 
#define E1_ENABLE_PIN      36 
#define TEMP_0_PIN         13   
#define TEMP_1_PIN         15   
#define TEMP_BED_PIN       14   
#define HEATER_0_PIN        9
#define HEATER_1_PIN        8 
#define HEATER_2_PIN       11 
#define HEATER_BED_PIN     10 
#define FAN_PIN             7
#define SDSS               11
#define LED_PIN            13
#define SOL1_PIN           16
#define SOL2_PIN           17
