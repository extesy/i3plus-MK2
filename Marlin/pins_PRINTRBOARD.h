#ifndef __AVR_AT90USB1286__
  #error "Oops!  Make sure you have 'Teensy++ 2.0' or 'Printrboard' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME         "Printrboard"
#define USBCON 1286  
#define LARGE_FLASH        true
#define DISABLE_JTAG       true
#define X_STOP_PIN         47   
#if ENABLED(SDSUPPORT)
  #define Y_STOP_PIN       37   
#else
  #define Y_STOP_PIN       20   
#endif
#define Z_STOP_PIN         36   
#define X_STEP_PIN         28   
#define X_DIR_PIN          29   
#define X_ENABLE_PIN       19   
#define Y_STEP_PIN         30   
#define Y_DIR_PIN          31   
#define Y_ENABLE_PIN       18   
#define Z_STEP_PIN         32   
#define Z_DIR_PIN          33   
#define Z_ENABLE_PIN       17   
#define E0_STEP_PIN        34   
#define E0_DIR_PIN         35   
#define E0_ENABLE_PIN      13   
#define TEMP_0_PIN          1   
#define TEMP_BED_PIN        0   
#define HEATER_0_PIN       15   
#define HEATER_1_PIN       44   
#define HEATER_2_PIN       45   
#define HEATER_BED_PIN     14   
#define FAN_PIN            16   
#define SDSS               20   
#define FILWIDTH_PIN        2   
#if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)
  #define BEEPER_PIN       -1
  #if ENABLED(LCD_I2C_PANELOLU2)
    #define BTN_EN1         3   
    #define BTN_EN2         2   
    #define BTN_ENC        41   
    #define SDSS           38   
  #else
    #define BTN_EN1        10   
    #define BTN_EN2        11   
    #define BTN_ENC        12   
  #endif
  #define SD_DETECT_PIN    -1
  #define LCD_PINS_RS       9   
  #define LCD_PINS_ENABLE   8   
  #define LCD_PINS_D4       7   
  #define LCD_PINS_D5       6   
  #define LCD_PINS_D6       5   
  #define LCD_PINS_D7       4   
#endif 
#if ENABLED(VIKI2) || ENABLED(miniVIKI)
  #define BEEPER_PIN        8   
  #define DOGLCD_A0        40   
  #define DOGLCD_CS        41   
  #define LCD_SCREEN_ROT_180
  #define BTN_EN1           2   
  #define BTN_EN2           3   
  #define BTN_ENC          45   
  #define SDSS             43   
  #define SD_DETECT_PIN    -1
  #define STAT_LED_RED_PIN  12  
  #define STAT_LED_BLUE_PIN 10  
#endif
