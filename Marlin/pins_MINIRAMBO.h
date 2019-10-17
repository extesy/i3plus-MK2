#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega 2560 or Rambo' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME          "Mini Rambo"
#define LARGE_FLASH         true
#define X_MIN_PIN          12
#define X_MAX_PIN          30
#define Y_MIN_PIN          11
#define Y_MAX_PIN          24
#define Z_MIN_PIN          10
#define Z_MAX_PIN          23
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  23
#endif
#define X_STEP_PIN         37
#define X_DIR_PIN          48
#define X_ENABLE_PIN       29
#define Y_STEP_PIN         36
#define Y_DIR_PIN          49
#define Y_ENABLE_PIN       28
#define Z_STEP_PIN         35
#define Z_DIR_PIN          47
#define Z_ENABLE_PIN       27
#define E0_STEP_PIN        34
#define E0_DIR_PIN         43
#define E0_ENABLE_PIN      26
#define E1_STEP_PIN        -1
#define E1_DIR_PIN         -1
#define E1_ENABLE_PIN      -1
#define X_MS1_PIN          40
#define X_MS2_PIN          41
#define Y_MS1_PIN          69
#define Y_MS2_PIN          39
#define Z_MS1_PIN          68
#define Z_MS2_PIN          67
#define E0_MS1_PIN         65
#define E0_MS2_PIN         66
#define MOTOR_CURRENT_PWM_XY_PIN 46
#define MOTOR_CURRENT_PWM_Z_PIN  45
#define MOTOR_CURRENT_PWM_E_PIN  44
#ifndef MOTOR_CURRENT_PWM_RANGE
  #define MOTOR_CURRENT_PWM_RANGE 2000
#endif
#define DEFAULT_PWM_MOTOR_CURRENT  {1300, 1300, 1250}
#define TEMP_0_PIN          0   
#define TEMP_1_PIN          1   
#define TEMP_BED_PIN        2   
#define HEATER_0_PIN        3
#define HEATER_1_PIN        7
#define HEATER_2_PIN        6
#define HEATER_BED_PIN      4
#define FAN_PIN             8
#define FAN1_PIN            6
#define SDSS               53
#define LED_PIN            13
#define CASE_LIGHT_PIN      9
#define SPINDLE_LASER_PWM_PIN     9  
#define SPINDLE_LASER_ENABLE_PIN 18  
#define SPINDLE_DIR_PIN          19
#define E_MUX0_PIN         17
#define E_MUX1_PIN         16
#define E_MUX2_PIN         78 
#if ENABLED(ULTRA_LCD)
  #define KILL_PIN         32
  #if ENABLED(NEWPANEL)
    #define BEEPER_PIN     84
    #define LCD_PINS_RS    82
    #define LCD_PINS_ENABLE 18
    #define LCD_PINS_D4    19
    #define LCD_PINS_D5    70
    #define LCD_PINS_D6    85
    #define LCD_PINS_D7    71
    #define BTN_EN1        14
    #define BTN_EN2        72
    #define BTN_ENC         9  
    #define SD_DETECT_PIN  15
  #endif 
#endif 
