#define BOARD_NAME "Melzi"
#if defined(__AVR_ATmega1284P__)
  #define LARGE_FLASH true
#endif
#define SANGUINOLOLU_V_1_2
#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega1284P__)
  #error "Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu."
#endif
#ifndef BOARD_NAME
  #define BOARD_NAME "Wanhao Melzi V1.0"
#endif
#define IS_MELZI (MB(MELZI) || MB(MELZI_MAKR3D))
#define X_STEP_PIN         15
#define X_DIR_PIN          21
#define X_MIN_PIN          18
#define Y_STEP_PIN         22
#define Y_DIR_PIN          23
#define Y_MIN_PIN          19
#define Z_STEP_PIN         3
#define Z_DIR_PIN          2
#define Z_MIN_PIN          20
#define E0_STEP_PIN         1
#define E0_DIR_PIN          0
#define SDSS               24
#define TEMP_0_PIN          A7   
#define TEMP_BED_PIN        A6   
#define HEATER_0_PIN       13 
#define HEATER_BED_PIN     12 
#define X_ENABLE_PIN     14
#define Y_ENABLE_PIN     14
#define Z_ENABLE_PIN     29 
#define E0_ENABLE_PIN    14
#define FAN_PIN           4
#define LCD_PINS_RS         27 
#define LCD_PINS_ENABLE     26  
#define LCD_PINS_D4         10
#define LCD_PINS_D5         11
#define LCD_PINS_D6         16
#define LCD_PINS_D7         17
#define BTN_ENC            8
