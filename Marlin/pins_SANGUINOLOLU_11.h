#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega1284P__)
  #error "Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu."
#endif
#ifndef BOARD_NAME
  #define BOARD_NAME "Sanguinololu <1.2"
#endif
#define IS_MELZI (MB(MELZI) || MB(MELZI_MAKR3D))
#define X_STOP_PIN         18
#define Y_STOP_PIN         19
#define Z_STOP_PIN         20
#define X_STEP_PIN         15
#define X_DIR_PIN          21
#define Y_STEP_PIN         22
#define Y_DIR_PIN          23
#define Z_STEP_PIN          3
#define Z_DIR_PIN           2
#define E0_STEP_PIN         1
#define E0_DIR_PIN          0
#define TEMP_0_PIN          7   
#define TEMP_BED_PIN        6   
#define HEATER_0_PIN       13 
#if ENABLED(SANGUINOLOLU_V_1_2)
  #define HEATER_BED_PIN   12 
  #define X_ENABLE_PIN     14
  #define Y_ENABLE_PIN     14
  #define Z_ENABLE_PIN     26
  #define E0_ENABLE_PIN    14
  #if ENABLED(LCD_I2C_PANELOLU2)
    #define FAN_PIN         4 
  #endif
#else
  #define HEATER_BED_PIN   14 
  #define X_ENABLE_PIN     -1
  #define Y_ENABLE_PIN     -1
  #define Z_ENABLE_PIN     -1
  #define E0_ENABLE_PIN    -1
#endif
#if MB(AZTEEG_X1) || MB(STB_11) || IS_MELZI
  #define FAN_PIN           4 
#endif
#define SDSS               31
#if IS_MELZI
  #define LED_PIN           27
#elif MB(STB_11)
  #define LCD_BACKLIGHT_PIN 17 
#endif
#if DISABLED(SPINDLE_LASER_ENABLE) && ENABLED(SANGUINOLOLU_V_1_2) && !(ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL))  
  #define CASE_LIGHT_PIN         4   
#endif
#if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)
  #if ENABLED(DOGLCD)
    #if ENABLED(U8GLIB_ST7920) 
      #if IS_MELZI 
        #define LCD_PINS_RS     30 
        #define LCD_PINS_ENABLE 29 
        #define LCD_PINS_D4     17 
        #define BEEPER_PIN      27
      #else        
        #define LCD_PINS_RS      4
        #define LCD_PINS_ENABLE 17
        #define LCD_PINS_D4     30
        #define LCD_PINS_D5     29
        #define LCD_PINS_D6     28
        #define LCD_PINS_D7     27
      #endif
    #else 
      #define DOGLCD_A0         30
      #define LCD_CONTRAST       1
      #if ENABLED(MAKRPANEL)
        #define BEEPER_PIN      29
        #define DOGLCD_CS       17
        #define LCD_BACKLIGHT_PIN 28 
      #else 
        #define DOGLCD_CS       29
      #endif
    #endif
    #define LCD_SCREEN_ROT_0
  #else 
    #define LCD_PINS_RS          4
    #define LCD_PINS_ENABLE     17
    #define LCD_PINS_D4         30
    #define LCD_PINS_D5         29
    #define LCD_PINS_D6         28
    #define LCD_PINS_D7         27
  #endif 
  #define BTN_EN1               11
  #define BTN_EN2               10
  #if ENABLED(LCD_I2C_PANELOLU2)
    #if IS_MELZI
      #define BTN_ENC           29
      #define LCD_SDSS          30 
    #else
      #define BTN_ENC           30
    #endif
  #elif ENABLED(LCD_FOR_MELZI)
    #define LCD_PINS_RS         17
    #define LCD_PINS_ENABLE     16
    #define LCD_PINS_D4         11
    #define BTN_ENC             28
    #define BTN_EN1             29
    #define BTN_EN2             30
    #ifndef ST7920_DELAY_1
      #define ST7920_DELAY_1 DELAY_0_NOP
    #endif
    #ifndef ST7920_DELAY_3
      #define ST7920_DELAY_2 DELAY_3_NOP
    #endif
    #ifndef ST7920_DELAY_3
      #define ST7920_DELAY_3 DELAY_0_NOP
    #endif
  #else  
    #define BTN_ENC             16
    #define LCD_SDSS            28 
  #endif
  #define SD_DETECT_PIN         -1
#endif 
#if ENABLED(SPINDLE_LASER_ENABLE)
  #if !MB(AZTEEG_X1) && ENABLED(SANGUINOLOLU_V_1_2) && !(ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL))  
    #define SPINDLE_LASER_ENABLE_PIN 10  
    #define SPINDLE_LASER_PWM_PIN     4  
    #define SPINDLE_DIR_PIN          11
  #elif !MB(MELZI)  
    #undef X_DIR_PIN
    #undef X_ENABLE_PIN
    #undef X_STEP_PIN
    #define X_DIR_PIN                 0
    #define X_ENABLE_PIN             14
    #define X_STEP_PIN                1
    #define SPINDLE_LASER_PWM_PIN    15  
    #define SPINDLE_LASER_ENABLE_PIN 21  
    #define SPINDLE_DIR_PIN          -1  
  #endif
#endif 
