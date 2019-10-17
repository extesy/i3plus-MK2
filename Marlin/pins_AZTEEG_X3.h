#ifndef __AVR_ATmega2560__
  #error "Oops! Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu."
#endif
#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Azteeg X3 supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif
#if ENABLED(CASE_LIGHT_ENABLE)  && !PIN_EXISTS(CASE_LIGHT)
  #define CASE_LIGHT_PIN 6     
#endif
#define BOARD_NAME "Azteeg X3"
#include "pins_RAMPS_13.h"
#undef SERVO0_PIN
#undef SERVO1_PIN
#define SERVO0_PIN  44  
#define SERVO1_PIN  55  
#if ENABLED(VIKI2) || ENABLED(miniVIKI)
  #undef DOGLCD_A0
  #undef DOGLCD_CS
  #undef BTN_ENC
  #define DOGLCD_A0         31
  #define DOGLCD_CS         32
  #define BTN_ENC           12
  #undef STAT_LED_RED_PIN
  #undef STAT_LED_BLUE_PIN
  #define STAT_LED_RED_PIN  64
  #define STAT_LED_BLUE_PIN 63
#else
  #define STAT_LED_RED_PIN   6
  #define STAT_LED_BLUE_PIN 11
#endif
#if ENABLED(CASE_LIGHT_ENABLE)  && PIN_EXISTS(CASE_LIGHT) && PIN_EXISTS(STAT_LED_RED) && STAT_LED_RED_PIN == CASE_LIGHT_PIN
  #undef STAT_LED_RED_PIN
#endif
#undef SPINDLE_LASER_PWM_PIN    
#undef SPINDLE_LASER_ENABLE_PIN
#undef SPINDLE_DIR_PIN
#if ENABLED(SPINDLE_LASER_ENABLE)
  #undef SDA                       
  #undef SCL
  #if SERVO0_PIN == 7
    #undef SERVO0_PIN
    #def SERVO0_PIN 11
  #endif
  #define SPINDLE_LASER_PWM_PIN     7  
  #define SPINDLE_LASER_ENABLE_PIN 20  
  #define SPINDLE_DIR_PIN          21
#endif
