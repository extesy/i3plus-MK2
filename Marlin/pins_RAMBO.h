#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME "Rambo"
#define LARGE_FLASH true
#define SERVO0_PIN         22 
#define SERVO1_PIN         23 
#define SERVO2_PIN         24 
#define SERVO3_PIN          5 
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  30
#endif
#define X_MIN_PIN          12
#define X_MAX_PIN          24
#define Y_MIN_PIN          11
#define Y_MAX_PIN          23
#define Z_MIN_PIN          10
#define Z_MAX_PIN          30
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
#define E1_STEP_PIN        33
#define E1_DIR_PIN         42
#define E1_ENABLE_PIN      25
#define X_MS1_PIN          40
#define X_MS2_PIN          41
#define Y_MS1_PIN          69
#define Y_MS2_PIN          39
#define Z_MS1_PIN          68
#define Z_MS2_PIN          67
#define E0_MS1_PIN         65
#define E0_MS2_PIN         66
#define E1_MS1_PIN         63
#define E1_MS2_PIN         64
#define DIGIPOTSS_PIN      38
#define DIGIPOT_CHANNELS {4,5,3,0,1} 
#define TEMP_0_PIN          0   
#define TEMP_1_PIN          1   
#define TEMP_BED_PIN        2   
#define HEATER_0_PIN        9
#define HEATER_1_PIN        7
#define HEATER_2_PIN        6
#define HEATER_BED_PIN      3
#define FAN_PIN             8
#define FAN1_PIN            6
#define FAN2_PIN            2
#define SDSS               53
#define LED_PIN            13
#define PS_ON_PIN           4
#define CASE_LIGHT_PIN     46
#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN      3   
#endif
#define SPINDLE_LASER_PWM_PIN    45  
#define SPINDLE_LASER_ENABLE_PIN 31  
#define SPINDLE_DIR_PIN          32
#define E_MUX0_PIN         17
#define E_MUX1_PIN         16
#define E_MUX2_PIN         84 
#if ENABLED(ULTRA_LCD)
  #define KILL_PIN 80
  #if ENABLED(NEWPANEL)
    #define LCD_PINS_RS     70
    #define LCD_PINS_ENABLE 71
    #define LCD_PINS_D4     72
    #define LCD_PINS_D5     73
    #define LCD_PINS_D6     74
    #define LCD_PINS_D7     75
    #if ENABLED(VIKI2) || ENABLED(miniVIKI)
      #define BEEPER_PIN 44
      #define DOGLCD_A0  70
      #define DOGLCD_CS  71
      #define LCD_SCREEN_ROT_180
      #define BTN_EN1 85
      #define BTN_EN2 84
      #define BTN_ENC 83
      #define SD_DETECT_PIN -1 
      #define STAT_LED_RED_PIN 22
      #define STAT_LED_BLUE_PIN 32
    #else
      #define BEEPER_PIN 79 
      #define BTN_EN1 76
      #define BTN_EN2 77
      #define BTN_ENC 78
      #define SD_DETECT_PIN 81
    #endif 
  #else 
    #define BEEPER_PIN 33
    #define LCD_PINS_RS     75
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4     23
    #define LCD_PINS_D5     25
    #define LCD_PINS_D6     27
    #define LCD_PINS_D7     29
  #endif 
#endif 
