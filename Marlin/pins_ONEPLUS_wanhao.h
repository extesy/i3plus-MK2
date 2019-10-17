#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
 Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif
#define LARGE_FLASH true
#ifdef IS_RAMPS_13
  #define SERVO0_PIN        7 
#else
  #define SERVO0_PIN       11
#endif
#define SERVO1_PIN          6
#define SERVO2_PIN          5
#define SERVO3_PIN          4
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#ifndef X_MAX_PIN
  #define X_MAX_PIN         2
#endif
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          40
#define Y_MAX_PIN          41
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30
#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13
#define FILWIDTH_PIN        5 
#if ENABLED(Z_MIN_PROBE_ENDSTOP)
  #define Z_MIN_PROBE_PIN  32
#endif
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define FILRUNOUT_PIN     4
#endif
#if MB(RAMPS_14_EFF) || MB(RAMPS_13_EFF) || ENABLED(IS_RAMPS_EFB)
  #define FAN_PIN           9 
  #if MB(RAMPS_14_EFF) || MB(RAMPS_13_EFF)
    #define CONTROLLERFAN_PIN  -1 
  #endif
#elif MB(RAMPS_14_EEF) || MB(RAMPS_14_SF) || MB(RAMPS_13_EEF) || MB(RAMPS_13_SF)
  #define FAN_PIN           8
#else
  #define FAN_PIN           4 
#endif
#define PS_ON_PIN          12
#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL)
  #define KILL_PIN         41
#endif
#if MB(RAMPS_14_EFF) || MB(RAMPS_13_EFF)
  #define HEATER_0_PIN      8
#else
  #define HEATER_0_PIN     10   
#endif
#if MB(RAMPS_14_SF) || MB(RAMPS_13_SF) || ENABLED(IS_RAMPS_EFB)
  #define HEATER_1_PIN     -1
#else
  #define HEATER_1_PIN      9   
#endif
#define HEATER_2_PIN       -1
#define TEMP_0_PIN         13   
#define TEMP_1_PIN         15   
#define TEMP_2_PIN         -1   
#if MB(RAMPS_14_EFF) || MB(RAMPS_14_EEF) || MB(RAMPS_14_SF) || MB(RAMPS_13_EFF) || MB(RAMPS_13_EEF) || MB(RAMPS_13_SF)
  #define HEATER_BED_PIN   -1    
#else
  #define HEATER_BED_PIN    8    
#endif
#define TEMP_BED_PIN         14   
#if ENABLED(Z_PROBE_SLED)
  #define SLED_PIN           -1
#endif
#if ENABLED(ULTRA_LCD)|| ENABLED(FYS_ULTRA_LCD_WANHAO_ONEPLUS)
  #if ENABLED(NEWPANEL)
    #if ENABLED(PANEL_ONE)
      #define LCD_PINS_RS 40
      #define LCD_PINS_ENABLE 42
      #define LCD_PINS_D4 65
      #define LCD_PINS_D5 66
      #define LCD_PINS_D6 44
      #define LCD_PINS_D7 64
    #else
    #endif
    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
      #define BEEPER_PIN 37
      #define BTN_EN1 31
      #define BTN_EN2 33
      #define BTN_ENC 35
      #define SD_DETECT_PIN 49
    #elif ENABLED(LCD_I2C_PANELOLU2)
      #define BTN_EN1 47  
      #define BTN_EN2 43
      #define BTN_ENC 32
      #define LCD_SDSS 53
      #define SD_DETECT_PIN -1
      #define KILL_PIN 41
    #elif ENABLED(LCD_I2C_VIKI)
      #define BTN_EN1 22  
      #define BTN_EN2 7   
      #define BTN_ENC -1
      #define LCD_SDSS 53
      #define SD_DETECT_PIN 49
    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
      #define BTN_EN1 35  
      #define BTN_EN2 37
      #define BTN_ENC 31
      #define SD_DETECT_PIN 49
      #define LCD_SDSS 53
      #define KILL_PIN 41
      #define BEEPER_PIN 23
      #define DOGLCD_CS 29
      #define DOGLCD_A0 27
      #define LCD_PIN_BL 33
    #elif ENABLED(MINIPANEL)
      #define BEEPER_PIN 37
      #define DOGLCD_A0  27
      #define DOGLCD_CS  29
      #define LCD_BACKLIGHT_PIN 65 
      #define LCD_RESET_PIN  25
      #define SDSS   53
    #define BTN_EN1 33
    #define BTN_EN2 31
    #define BTN_ENC 35  
      #define SD_DETECT_PIN 49
    #else
      #define BEEPER_PIN 33  
      #if ENABLED(REPRAPWORLD_KEYPAD)
        #define BTN_EN1 64 
        #define BTN_EN2 59 
        #define BTN_ENC 63 
        #define SHIFT_OUT 40 
        #define SHIFT_CLK 44 
        #define SHIFT_LD 42 
      #elif ENABLED(PANEL_ONE)
        #define BTN_EN1 59 
        #define BTN_EN2 63 
        #define BTN_ENC 49 
      #else
        #define BTN_EN1 37
        #define BTN_EN2 35
        #define BTN_ENC 31  
      #endif
      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN 49
      #else
      #endif
    #endif
  #else 
    #define BEEPER_PIN 33   
    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29
  #endif 
#endif 
#if DISABLED(SDSUPPORT)
  #define MAX6675_SS       66 
#else
  #define MAX6675_SS       66 
#endif
#if DISABLED(SDSUPPORT)
  #define SCK_PIN          52
  #define MISO_PIN         50
  #define MOSI_PIN         51
#endif
#ifndef KILL_PIN
#endif
