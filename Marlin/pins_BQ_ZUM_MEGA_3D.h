#ifndef __AVR_ATmega2560__
  #error "Oops! Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME "ZUM Mega 3D"
#define RAMPS_D8_PIN  10
#define RAMPS_D9_PIN  12
#define RAMPS_D10_PIN  9
#define MOSFET_D_PIN   7
#define ORIG_E0_AUTO_FAN_PIN 11
#define ORIG_E1_AUTO_FAN_PIN  6
#define ORIG_E2_AUTO_FAN_PIN  6
#define ORIG_E3_AUTO_FAN_PIN  6
#define CASE_LIGHT_PIN   44     
#define SPINDLE_LASER_ENABLE_PIN 40  
#define SPINDLE_LASER_PWM_PIN    44  
#define SPINDLE_DIR_PIN          42
#include "pins_RAMPS_13.h"
#undef X_MAX_PIN
#define X_MAX_PIN         79 
#undef Z_MIN_PROBE_PIN
#define Z_MIN_PROBE_PIN   19 
#undef Z_ENABLE_PIN
#define Z_ENABLE_PIN      77 
#define DIGIPOTSS_PIN     22
#define DIGIPOT_CHANNELS  { 4, 5, 3, 0, 1 }
#undef TEMP_1_PIN
#define TEMP_1_PIN        14   
#undef TEMP_BED_PIN
#define TEMP_BED_PIN      15   
#undef PS_ON_PIN             
#define PS_ON_PIN         81 
#ifdef Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
  #undef Z_MIN_PIN
  #undef Z_MAX_PIN
  #define Z_MIN_PIN       19 
  #define Z_MAX_PIN       18 
#endif
