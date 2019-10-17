#if HOTENDS > 5 || E_STEPPERS > 5
  #error "Azteeg X3 Pro supports up to 5 hotends / E-steppers. Comment out this line to continue."
#endif
#if ENABLED(CASE_LIGHT_ENABLE)  && !PIN_EXISTS(CASE_LIGHT)
  #define CASE_LIGHT_PIN 44     
#endif
#define BOARD_NAME "Azteeg X3 Pro"
#include "pins_RAMPS.h"
#ifndef __AVR_ATmega2560__
  #error "Oops! Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu."
#endif
#undef SERVO0_PIN
#define SERVO0_PIN         47
#if ENABLED(DELTA)
  #undef X_MIN_PIN
  #undef X_MAX_PIN
  #undef Y_MIN_PIN
  #undef Y_MAX_PIN
  #undef Z_MIN_PIN
  #undef Z_MAX_PIN
  #define X_MIN_PIN         2
  #define X_MAX_PIN         3
  #define Y_MIN_PIN        15
  #define Y_MAX_PIN        14
  #define Z_MIN_PIN        19
  #define Z_MAX_PIN        18
#endif
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  18
#endif
#define E2_STEP_PIN        23
#define E2_DIR_PIN         25
#define E2_ENABLE_PIN      40
#define E3_STEP_PIN        27
#define E3_DIR_PIN         29
#define E3_ENABLE_PIN      41
#define E4_STEP_PIN        43
#define E4_DIR_PIN         37
#define E4_ENABLE_PIN      42
#define TEMP_2_PIN         12   
#define TEMP_3_PIN         11   
#define TEMP_4_PIN         10   
#define TC1                 4   
#define TC2                 5   
#define HEATER_2_PIN       16
#define HEATER_3_PIN       17
#define HEATER_4_PIN        4
#define HEATER_5_PIN        5
#define HEATER_6_PIN        6
#define HEATER_7_PIN       11
#undef FAN_PIN
#define FAN_PIN             6 
#ifndef CONTROLLER_FAN_PIN
  #define CONTROLLER_FAN_PIN 4 
#endif
#define ORIG_E0_AUTO_FAN_PIN 5
#define ORIG_E1_AUTO_FAN_PIN 5
#define ORIG_E2_AUTO_FAN_PIN 5
#define ORIG_E3_AUTO_FAN_PIN 5
#undef BEEPER_PIN
#define BEEPER_PIN         33
#if ENABLED(VIKI2) || ENABLED(miniVIKI)
  #undef SD_DETECT_PIN
  #define SD_DETECT_PIN    49   
  #undef BEEPER_PIN
  #define  BEEPER_PIN      12   
#else
  #define STAT_LED_RED_PIN 32
  #define STAT_LED_BLUE_PIN 35
#endif
#if ENABLED(CASE_LIGHT_ENABLE)  && PIN_EXISTS(CASE_LIGHT) && defined(DOGLCD_A0) && DOGLCD_A0 == CASE_LIGHT_PIN
  #undef DOGLCD_A0            
  #define DOGLCD_A0        57 
#endif
#undef SPINDLE_LASER_PWM_PIN    
#undef SPINDLE_LASER_ENABLE_PIN
#undef SPINDLE_DIR_PIN
#if ENABLED(SPINDLE_LASER_ENABLE)   
  #if ENABLED(VIKI2) || ENABLED(miniVIKI)
    #undef BTN_EN2
    #define BTN_EN2             31  
  #endif
  #define SPINDLE_LASER_PWM_PIN     7  
  #define SPINDLE_LASER_ENABLE_PIN 20  
  #define SPINDLE_DIR_PIN          21
#endif
