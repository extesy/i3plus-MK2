#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
  #error "Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME      "Gen7 Custom"
#define X_STOP_PIN       0
#define Y_STOP_PIN       1
#define Z_STOP_PIN       2
#define X_STEP_PIN      21   
#define X_DIR_PIN       20   
#define X_ENABLE_PIN    24
#define Y_STEP_PIN      23
#define Y_DIR_PIN       22
#define Y_ENABLE_PIN    24
#define Z_STEP_PIN      26
#define Z_DIR_PIN       25
#define Z_ENABLE_PIN    24
#define E0_STEP_PIN     28
#define E0_DIR_PIN      27
#define E0_ENABLE_PIN   24
#define TEMP_0_PIN       2   
#define TEMP_BED_PIN     1   
#define HEATER_0_PIN     4
#define HEATER_BED_PIN   3   
#define SDSS            31   
#define PS_ON_PIN       19
#define CASE_LIGHT_PIN  15   
#define DEBUG_PIN       -1
#define BEEPER_PIN      -1
#define LCD_PINS_RS     18
#define LCD_PINS_ENABLE 17
#define LCD_PINS_D4     16
#define LCD_PINS_D5     15
#define LCD_PINS_D6     13
#define LCD_PINS_D7     14
#define BTN_EN1         11
#define BTN_EN2         10
#define BTN_ENC         12
#define SPINDLE_LASER_ENABLE_PIN  5  
#define SPINDLE_LASER_PWM_PIN    16  
#define SPINDLE_DIR_PIN           6
