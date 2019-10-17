#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif
#ifndef BOARD_NAME
  #define BOARD_NAME "RAMPS 1.4"
#endif
#define LARGE_FLASH true
#define X_STEP_PIN         61
#define X_DIR_PIN          62
#define X_ENABLE_PIN       60
#define X_MIN_PIN          54
#define X_MAX_PIN           2
#define Y_STEP_PIN         64
#define Y_DIR_PIN          65
#define Y_ENABLE_PIN       63
#define Y_MIN_PIN          24
#define Y_MAX_PIN          15
#define Z_STEP_PIN         67
#define Z_DIR_PIN          69
#define Z_ENABLE_PIN       66
#define Z_MIN_PIN          23
#define Z_MAX_PIN          19
#define E0_STEP_PIN        58 
#define E0_DIR_PIN         59 
#define E0_ENABLE_PIN      57 
#define FAN_PIN            5
#define KILL_PIN           -1
#define HEATER_0_PIN       4
#define HEATER_1_PIN       7
#define HEATER_2_PIN       -1
#define HEATER_BED_PIN     3
#define TEMP_0_PIN         1
#define TEMP_1_PIN         14
#define TEMP_2_PIN         -1   
#define TEMP_BED_PIN       14   
#define SD_DETECT_PIN       49
#define SERVO0_PIN         25
#define SDSS               53
#define LED_PIN            13
