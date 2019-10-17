#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME         "MEGA/RAMPS <1.2"
#define X_MIN_PIN           3
#define X_MAX_PIN           2
#define Y_MIN_PIN          16
#define Y_MAX_PIN          17
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  19
#endif
#define X_STEP_PIN         26
#define X_DIR_PIN          28
#define X_ENABLE_PIN       24
#define Y_STEP_PIN         38
#define Y_DIR_PIN          40
#define Y_ENABLE_PIN       36
#define Z_STEP_PIN         44
#define Z_DIR_PIN          46
#define Z_ENABLE_PIN       42
#define E0_STEP_PIN        32
#define E0_DIR_PIN         34
#define E0_ENABLE_PIN      30
#define TEMP_0_PIN          2   
#define TEMP_BED_PIN        1   
#if DISABLED(SDSUPPORT)
  #define MAX6675_SS       66 
#else
  #define MAX6675_SS       66 
#endif
#if ENABLED(RAMPS_V_1_0)
  #define HEATER_0_PIN     12
  #define HEATER_BED_PIN   -1
  #define FAN_PIN          11
#else 
  #define HEATER_0_PIN     10
  #define HEATER_BED_PIN    8
  #define FAN_PIN           9
#endif
#define SDPOWER            48
#define SDSS               53
#define LED_PIN            13
#define CASE_LIGHT_PIN     45     
#define SPINDLE_LASER_ENABLE_PIN 41  
#define SPINDLE_LASER_PWM_PIN    45  
#define SPINDLE_DIR_PIN          43
