#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif
#if ENABLED(MEGATRONICS_31)
  #define BOARD_NAME       "Megatronics v3.1"
#else
  #define BOARD_NAME       "Megatronics v3.0"
#endif
#define LARGE_FLASH        true
#define SERVO0_PIN         46 
#define SERVO1_PIN         47 
#define SERVO2_PIN         48 
#define SERVO3_PIN         49 
#define X_MIN_PIN          37
#define X_MAX_PIN          40
#define Y_MIN_PIN          41
#define Y_MAX_PIN          38
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  19
#endif
#define X_STEP_PIN         58
#define X_DIR_PIN          57
#define X_ENABLE_PIN       59
#define Y_STEP_PIN          5
#define Y_DIR_PIN          17
#define Y_ENABLE_PIN        4
#define Z_STEP_PIN         16
#define Z_DIR_PIN          11
#define Z_ENABLE_PIN        3
#define E0_STEP_PIN        28
#define E0_DIR_PIN         27
#define E0_ENABLE_PIN      29
#define E1_STEP_PIN        25
#define E1_DIR_PIN         24
#define E1_ENABLE_PIN      26
#define E2_STEP_PIN        22
#define E2_DIR_PIN         60
#define E2_ENABLE_PIN      23
#if TEMP_SENSOR_0 == -1
  #define TEMP_0_PIN       11   
#else
  #define TEMP_0_PIN       15   
#endif
#if TEMP_SENSOR_1 == -1
  #define TEMP_1_PIN       10   
#else
  #define TEMP_1_PIN       13   
#endif
#if TEMP_SENSOR_2 == -1
  #define TEMP_2_PIN        9   
#else
  #define TEMP_2_PIN       12   
#endif
#if TEMP_SENSOR_BED == -1
  #define TEMP_BED_PIN      8   
#else
  #define TEMP_BED_PIN     14   
#endif
#define HEATER_0_PIN        2
#define HEATER_1_PIN        9
#define HEATER_2_PIN        8
#define HEATER_BED_PIN     10
#define FAN_PIN             6
#define FAN1_PIN            7
#define SDSS               53
#define LED_PIN            13
#define PS_ON_PIN          12
#define CASE_LIGHT_PIN     45   
#define BEEPER_PIN         61
#define BTN_EN1            44
#define BTN_EN2            45
#define BTN_ENC            33
#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
  #define LCD_PINS_RS      56   
  #define LCD_PINS_ENABLE  51   
  #define LCD_PINS_D4      52   
  #define SD_DETECT_PIN    35
#else
  #define LCD_PINS_RS      32
  #define LCD_PINS_ENABLE  31
  #define LCD_PINS_D4      14
  #define LCD_PINS_D5      30
  #define LCD_PINS_D6      39
  #define LCD_PINS_D7      15
  #define SHIFT_CLK        43
  #define SHIFT_LD         35
  #define SHIFT_OUT        34
  #define SHIFT_EN         44
  #if ENABLED(MEGATRONICS_31)
    #define SD_DETECT_PIN  56
  #else
    #define SD_DETECT_PIN  -1
  #endif
#endif
#if DISABLED(REPRAPWORLD_KEYPAD)       
  #define SPINDLE_LASER_PWM_PIN    44  
  #define SPINDLE_LASER_ENABLE_PIN 43  
  #define SPINDLE_DIR_PIN          42
#elif EXTRUDERS <= 2
  #undef Y_ENABLE_PIN  
  #undef Y_STEP_PIN    
  #undef Y_DIR_PIN     
  #undef E2_ENABLE_PIN 
  #undef E2_STEP_PIN   
  #undef E2_DIR_PIN    
  #define Y_ENABLE_PIN             23
  #define Y_STEP_PIN               22
  #define Y_DIR_PIN                60
  #define SPINDLE_LASER_PWM_PIN     4  
  #define SPINDLE_LASER_ENABLE_PIN 17  
  #define SPINDLE_DIR_PIN           5
#endif
