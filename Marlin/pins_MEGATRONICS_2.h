#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME         "Megatronics v2.0"
#define LARGE_FLASH        true
#define X_MIN_PIN          37
#define X_MAX_PIN          40
#define Y_MIN_PIN          41
#define Y_MAX_PIN          38
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  19
#endif
#define X_STEP_PIN         26
#define X_DIR_PIN          27
#define X_ENABLE_PIN       25
#define Y_STEP_PIN          4 
#define Y_DIR_PIN          54 
#define Y_ENABLE_PIN        5
#define Z_STEP_PIN         56 
#define Z_DIR_PIN          60 
#define Z_ENABLE_PIN       55 
#define E0_STEP_PIN        35
#define E0_DIR_PIN         36
#define E0_ENABLE_PIN      34
#define E1_STEP_PIN        29
#define E1_DIR_PIN         39
#define E1_ENABLE_PIN      28
#define E2_STEP_PIN        23 
#define E2_DIR_PIN         24 
#define E2_ENABLE_PIN      22
#if TEMP_SENSOR_0 == -1
  #define TEMP_0_PIN        4   
#else
  #define TEMP_0_PIN       13   
#endif
#if TEMP_SENSOR_1 == -1
  #define TEMP_1_PIN        8   
#else
  #define TEMP_1_PIN       15   
#endif
#if TEMP_SENSOR_BED == -1
  #define TEMP_BED_PIN      8   
#else
  #define TEMP_BED_PIN     14   
#endif
#define HEATER_0_PIN        9
#define HEATER_1_PIN        8
#define HEATER_BED_PIN     10
#define FAN_PIN             7
#define FAN1_PIN            6
#define SDSS               53
#define LED_PIN            13
#define PS_ON_PIN          12
#define CASE_LIGHT_PIN      2
#define BEEPER_PIN         64
#define LCD_PINS_RS        14
#define LCD_PINS_ENABLE    15
#define LCD_PINS_D4        30
#define LCD_PINS_D5        31
#define LCD_PINS_D6        32
#define LCD_PINS_D7        33
#define BTN_EN1            61
#define BTN_EN2            59
#define BTN_ENC            43
#define SHIFT_CLK 63
#define SHIFT_LD 42
#define SHIFT_OUT 17
#define SHIFT_EN 17
#define SPINDLE_LASER_PWM_PIN          3  
#define SPINDLE_LASER_ENABLE_PIN      16  
#define SPINDLE_DIR_PIN               11
