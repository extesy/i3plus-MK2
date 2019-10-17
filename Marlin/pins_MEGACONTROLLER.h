#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif
#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Mega Controller supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif
#define BOARD_NAME "Mega Controller"
#define SERVO0_PIN         30
#define SERVO1_PIN         31
#define SERVO2_PIN         32
#define SERVO3_PIN         33
#define X_MIN_PIN          43
#define X_MAX_PIN          42
#define Y_MIN_PIN          38
#define Y_MAX_PIN          41
#define Z_MIN_PIN          40
#define Z_MAX_PIN          37
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  37
#endif
#define X_STEP_PIN         62 
#define X_DIR_PIN          63 
#define X_ENABLE_PIN       61 
#define Y_STEP_PIN         65 
#define Y_DIR_PIN          66 
#define Y_ENABLE_PIN       64 
#define Z_STEP_PIN         68 
#define Z_DIR_PIN          69 
#define Z_ENABLE_PIN       67 
#define E0_STEP_PIN        23
#define E0_DIR_PIN         24
#define E0_ENABLE_PIN      22
#define E1_STEP_PIN        26
#define E1_DIR_PIN         27
#define E1_ENABLE_PIN      25
#if TEMP_SENSOR_0 == -1
  #define TEMP_0_PIN        4   
#else
  #define TEMP_0_PIN        0   
#endif
#if TEMP_SENSOR_1 == -1
  #define TEMP_1_PIN        5   
#else
  #define TEMP_1_PIN        2   
#endif
#define TEMP_2_PIN          3   
#if TEMP_SENSOR_BED == -1
  #define TEMP_BED_PIN      6   
#else
  #define TEMP_BED_PIN      1   
#endif
#define HEATER_0_PIN       29
#define HEATER_1_PIN       34
#define HEATER_BED_PIN     28
#define FAN_PIN            39
#define FAN1_PIN           35
#define FAN2_PIN           36
#ifndef CONTROLLER_FAN_PIN
  #define CONTROLLER_FAN_PIN FAN2_PIN
#endif
#define FAN_SOFT_PWM
#define SDSS               53
#define LED_PIN            13
#define CASE_LIGHT_PIN      2
#if ENABLED(MINIPANEL)
  #define BEEPER_PIN       46
  #define DOGLCD_A0        47
  #define DOGLCD_CS        45
  #define LCD_BACKLIGHT_PIN 44  
  #define KILL_PIN         12
  #define BTN_EN1          48
  #define BTN_EN2          11
  #define BTN_ENC          10
  #define SD_DETECT_PIN    49
#endif 
#define SPINDLE_LASER_PWM_PIN     6  
#define SPINDLE_LASER_ENABLE_PIN  7  
#define SPINDLE_DIR_PIN           8
