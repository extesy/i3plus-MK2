#if !defined(__AVR_ATmega1284P__)
  #error "Oops!  Make sure you have 'Anet V1.0', 'Anet V1.0 (Optiboot)' or 'Sanguino' selected from the 'Tools -> Boards' menu."
#endif
#ifndef BOARD_NAME
  #define BOARD_NAME "Anet"
#endif
#define LARGE_FLASH true
#define X_STOP_PIN         18
#define Y_STOP_PIN         19
#define Z_STOP_PIN         20
#define X_STEP_PIN         15
#define X_DIR_PIN          21
#define X_ENABLE_PIN       14
#define Y_STEP_PIN         22
#define Y_DIR_PIN          23
#define Y_ENABLE_PIN       14
#define Z_STEP_PIN          3
#define Z_DIR_PIN           2
#define Z_ENABLE_PIN       26
#define E0_STEP_PIN         1
#define E0_DIR_PIN          0
#define E0_ENABLE_PIN      14
#define TEMP_0_PIN          7  
#define TEMP_BED_PIN        6  
#define HEATER_0_PIN       13  
#define HEATER_BED_PIN     12  
#define FAN_PIN             4
#define SDSS               31
#define LED_PIN            -1
#if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)
  #define LCD_SDSS           28
  #if ENABLED(ADC_KEYPAD)
    #define SERVO0_PIN         27 
    #define LCD_PINS_RS        28
    #define LCD_PINS_ENABLE    29
    #define LCD_PINS_D4        10
    #define LCD_PINS_D5        11
    #define LCD_PINS_D6        16
    #define LCD_PINS_D7        17
    #define BTN_EN1            -1
    #define BTN_EN2            -1
    #define BTN_ENC            -1
    #define ADC_KEYPAD_PIN      1
    #define ENCODER_FEEDRATE_DEADZONE 2
  #elif ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER) || ENABLED(ANET_FULL_GRAPHICS_LCD)
    #define SERVO0_PIN         29 
    #define BEEPER_PIN         17
    #define LCD_PINS_RS        27
    #define LCD_PINS_ENABLE    28
    #define LCD_PINS_D4        30
    #define BTN_EN1            11
    #define BTN_EN2            10
    #define BTN_ENC            16
    #define ST7920_DELAY_1 DELAY_0_NOP
    #define ST7920_DELAY_2 DELAY_1_NOP
    #define ST7920_DELAY_3 DELAY_2_NOP
    #ifndef ENCODER_STEPS_PER_MENU_ITEM
      #define ENCODER_STEPS_PER_MENU_ITEM 1
    #endif
    #ifndef ENCODER_PULSES_PER_STEP
      #define ENCODER_PULSES_PER_STEP 4
    #endif
  #endif
#endif  
