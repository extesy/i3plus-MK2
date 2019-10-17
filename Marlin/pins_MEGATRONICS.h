#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME         "Megatronics"
#define LARGE_FLASH        true
#define X_MIN_PIN          41
#define X_MAX_PIN          37
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  19
#endif
#define X_STEP_PIN         26
#define X_DIR_PIN          28
#define X_ENABLE_PIN       24
#define Y_STEP_PIN         60 
#define Y_DIR_PIN          61 
#define Y_ENABLE_PIN       22
#define Z_STEP_PIN         54 
#define Z_DIR_PIN          55 
#define Z_ENABLE_PIN       56 
#define E0_STEP_PIN        31
#define E0_DIR_PIN         32
#define E0_ENABLE_PIN      38
#define E1_STEP_PIN        34
#define E1_DIR_PIN         36
#define E1_ENABLE_PIN      30
#if TEMP_SENSOR_0 == -1
  #define TEMP_0_PIN        8   
#else
  #define TEMP_0_PIN       13   
#endif
#define TEMP_1_PIN         15   
#define TEMP_BED_PIN       14   
#define HEATER_0_PIN        9
#define HEATER_1_PIN        8
#define HEATER_BED_PIN     10
#define FAN_PIN             7   
#define SDSS               53
#define LED_PIN            13
#define PS_ON_PIN          12
#define CASE_LIGHT_PIN      2
#define BEEPER_PIN         33
#if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)
  #define LCD_PINS_RS     16
  #define LCD_PINS_ENABLE 17
  #define LCD_PINS_D4     23
  #define LCD_PINS_D5     25
  #define LCD_PINS_D6     27
  #define LCD_PINS_D7     29
  #define BTN_EN1         59
  #define BTN_EN2         64
  #define BTN_ENC         43
  #define SD_DETECT_PIN   -1   
#endif 
#define SPINDLE_LASER_PWM_PIN     3  
#define SPINDLE_LASER_ENABLE_PIN  4  
#define SPINDLE_DIR_PIN          11
