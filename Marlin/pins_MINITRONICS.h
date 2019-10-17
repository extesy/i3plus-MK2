#ifndef __AVR_ATmega1281__
  #error "Oops!  Make sure you have 'Minitronics' selected from the 'Tools -> Boards' menu."
#endif
#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Minitronics supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif
#define BOARD_NAME         "Minitronics v1.0 / v1.1"
#define LARGE_FLASH        true
#define X_MIN_PIN           5
#define X_MAX_PIN           2
#define Y_MIN_PIN           2
#define Y_MAX_PIN          15
#define Z_MIN_PIN           6
#define Z_MAX_PIN          -1
#define X_STEP_PIN         48
#define X_DIR_PIN          47
#define X_ENABLE_PIN       49
#define Y_STEP_PIN         39 
#define Y_DIR_PIN          40 
#define Y_ENABLE_PIN       38
#define Z_STEP_PIN         42 
#define Z_DIR_PIN          43 
#define Z_ENABLE_PIN       41 
#define E0_STEP_PIN        45
#define E0_DIR_PIN         44
#define E0_ENABLE_PIN      27
#define E1_STEP_PIN        36
#define E1_DIR_PIN         35
#define E1_ENABLE_PIN      37
#define TEMP_0_PIN          7   
#define TEMP_1_PIN          6   
#define TEMP_BED_PIN        6   
#define HEATER_0_PIN        7 
#define HEATER_1_PIN        8 
#define HEATER_BED_PIN      3 
#define FAN_PIN             9
#define SDSS               16
#define LED_PIN            46
#define BEEPER_PIN         -1
#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
  #define LCD_PINS_RS      15   
  #define LCD_PINS_ENABLE  11   
  #define LCD_PINS_D4      10   
  #define BTN_EN1          18
  #define BTN_EN2          17
  #define BTN_ENC          25
  #define SD_DETECT_PIN    30
#else
  #define LCD_PINS_RS      -1
  #define LCD_PINS_ENABLE  -1
  #define LCD_PINS_D4      -1
  #define LCD_PINS_D5      -1
  #define LCD_PINS_D6      -1
  #define LCD_PINS_D7      -1
  #define BTN_EN1          -1
  #define BTN_EN2          -1
  #define BTN_ENC          -1
  #define SD_DETECT_PIN    -1 
#endif
#if ENABLED(SPINDLE_LASER_ENABLE)  
  #undef HEATER_BED_PIN
  #undef TEMP_BED_PIN           
  #undef TEMP_0_PIN             
  #undef TEMP_1_PIN             
  #define HEATER_BED_PIN      4  
  #define TEMP_BED_PIN       50
  #define TEMP_0_PIN         51
  #define SPINDLE_LASER_ENABLE_PIN      52 
  #define SPINDLE_LASER_PWM_PIN          3 
  #define SPINDLE_DIR_PIN               53
#endif
