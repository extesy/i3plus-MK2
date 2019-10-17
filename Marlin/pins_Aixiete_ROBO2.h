#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
#error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif
#define SERVO0_PIN          11
#define SERVO1_PIN          6
#define SERVO2_PIN          5
#define SERVO3_PIN          4
#define X_MIN_PIN          3
#define Y_MIN_PIN          14
#define Z_MAX_PIN          18
#ifndef Z_MIN_PROBE_PIN
#define Z_MIN_PROBE_PIN  32
#endif
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_CS_PIN           53
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_CS_PIN           49
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_CS_PIN           40
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24
#define E0_CS_PIN          42
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30
#define E1_CS_PIN          44
#define TEMP_0_PIN         13   
#define TEMP_1_PIN         15   
#define TEMP_BED_PIN       14   
#if DISABLED(SDSUPPORT)
#define MAX6675_SS       66 
#else
#define MAX6675_SS       66 
#endif
#if DISABLED(IS_RAMPS_EEB) && DISABLED(IS_RAMPS_EEF) && DISABLED(IS_RAMPS_EFB) && DISABLED(IS_RAMPS_EFF) && DISABLED(IS_RAMPS_SF) && !PIN_EXISTS(MOSFET_D)
#if HOTENDS > 1
#if TEMP_SENSOR_BED
#define IS_RAMPS_EEB
#else
#define IS_RAMPS_EEF
#endif
#elif TEMP_SENSOR_BED
#define IS_RAMPS_EFB
#else
#define IS_RAMPS_EFF
#endif
#endif
#ifndef MOSFET_D_PIN
#define MOSFET_D_PIN  -1
#endif
#ifndef RAMPS_D8_PIN
#define RAMPS_D8_PIN   8
#endif
#ifndef RAMPS_D9_PIN
#define RAMPS_D9_PIN   9
#endif
#ifndef RAMPS_D10_PIN
#define RAMPS_D10_PIN 10
#endif
#define HEATER_0_PIN     RAMPS_D10_PIN
#if ENABLED(IS_RAMPS_EFB)                      
#define FAN_PIN        RAMPS_D9_PIN
#define HEATER_BED_PIN RAMPS_D8_PIN
#elif ENABLED(IS_RAMPS_EEF)                    
#define HEATER_1_PIN   RAMPS_D9_PIN
#define FAN_PIN        RAMPS_D8_PIN
#elif ENABLED(IS_RAMPS_EEB)                    
#define HEATER_1_PIN   RAMPS_D9_PIN
#define HEATER_BED_PIN RAMPS_D8_PIN
#elif ENABLED(IS_RAMPS_EFF)                    
#define FAN_PIN        RAMPS_D9_PIN
#define FAN1_PIN       RAMPS_D8_PIN
#elif ENABLED(IS_RAMPS_SF)                     
#define FAN_PIN        RAMPS_D8_PIN
#else                                          
#define FAN_PIN        RAMPS_D9_PIN
#define HEATER_BED_PIN RAMPS_D8_PIN
#if HOTENDS == 1
#define FAN1_PIN     MOSFET_D_PIN
#else
#define HEATER_1_PIN MOSFET_D_PIN
#endif
#endif
#ifndef FAN_PIN
#define FAN_PIN 4      
#endif
#define SDSS               53
#define LED_PIN            13
#ifndef FILWIDTH_PIN
#define FILWIDTH_PIN      5   
#endif
#define FIL_RUNOUT_PIN      4
#define PS_ON_PIN          12
#if ENABLED(CASE_LIGHT_ENABLE) && !PIN_EXISTS(CASE_LIGHT) && !defined(SPINDLE_LASER_ENABLE_PIN)
#if !defined(NUM_SERVOS) || NUM_SERVOS == 0 
#define CASE_LIGHT_PIN   6      
#elif !(ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)     && (ENABLED(PANEL_ONE) || ENABLED(VIKI2) || ENABLED(miniVIKI) || ENABLED(MINIPANEL) || ENABLED(REPRAPWORLD_KEYPAD)))  
#define CASE_LIGHT_PIN   44     
#endif
#endif
#if ENABLED(SPINDLE_LASER_ENABLE) && !PIN_EXISTS(SPINDLE_LASER_ENABLE)
#if !defined(NUM_SERVOS) || NUM_SERVOS == 0 
#define SPINDLE_LASER_ENABLE_PIN  4  
#define SPINDLE_LASER_PWM_PIN     6  
#define SPINDLE_DIR_PIN           5
#elif !(ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)     && (ENABLED(PANEL_ONE) || ENABLED(VIKI2) || ENABLED(miniVIKI) || ENABLED(MINIPANEL) || ENABLED(REPRAPWORLD_KEYPAD)))  
#define SPINDLE_LASER_ENABLE_PIN 40  
#define SPINDLE_LASER_PWM_PIN    44  
#define SPINDLE_DIR_PIN          65
#endif
#endif
#define E_MUX0_PIN         40   
#define E_MUX1_PIN         42   
#define E_MUX2_PIN         44   
#if ENABLED(ULTRA_LCD)
#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
#define LCD_PINS_RS         49 
#define LCD_PINS_ENABLE     51 
#define LCD_PINS_D4         52 
#elif ENABLED(NEWPANEL) && ENABLED(PANEL_ONE)
#define LCD_PINS_RS         40
#define LCD_PINS_ENABLE     42
#define LCD_PINS_D4         65
#define LCD_PINS_D5         66
#define LCD_PINS_D6         44
#define LCD_PINS_D7         64
#else
#define LCD_PINS_RS         16
#define LCD_PINS_ENABLE     17
#define LCD_PINS_D4         23
#define LCD_PINS_D5         25
#define LCD_PINS_D6         27
#define LCD_PINS_D7         29
#if DISABLED(NEWPANEL)
#define BEEPER_PIN        33
#endif
#endif
#if ENABLED(NEWPANEL)
#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
#define BEEPER_PIN        37
#define BTN_EN1           31
#define BTN_EN2           33
#define BTN_ENC           35
#define SD_DETECT_PIN     49
#define KILL_PIN          41
#if ENABLED(BQ_LCD_SMART_CONTROLLER)
#define LCD_BACKLIGHT_PIN 39
#endif
#elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
#define BTN_EN1           64
#define BTN_EN2           59
#define BTN_ENC           63
#define SD_DETECT_PIN     42
#elif ENABLED(LCD_I2C_PANELOLU2)
#define BTN_EN1           47
#define BTN_EN2           43
#define BTN_ENC           32
#define LCD_SDSS          53
#define SD_DETECT_PIN     -1
#define KILL_PIN          41
#elif ENABLED(LCD_I2C_VIKI)
#define BTN_EN1           22 
#define BTN_EN2            7 
#define BTN_ENC           -1
#define LCD_SDSS          53
#define SD_DETECT_PIN     49
#elif ENABLED(VIKI2) || ENABLED(miniVIKI)
#define BEEPER_PIN        33
#define DOGLCD_A0         44
#define DOGLCD_CS         45
#define LCD_SCREEN_ROT_180
#define BTN_EN1           22
#define BTN_EN2            7
#define BTN_ENC           39
#define SDSS              53
#define SD_DETECT_PIN     -1 
#define KILL_PIN          31
#define STAT_LED_RED_PIN  32
#define STAT_LED_BLUE_PIN 35
#elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
#define BTN_EN1           35
#define BTN_EN2           37
#define BTN_ENC           31
#define SD_DETECT_PIN     49
#define LCD_SDSS          53
#define KILL_PIN          41
#define BEEPER_PIN        23
#define DOGLCD_CS         29
#define DOGLCD_A0         27
#define LCD_BACKLIGHT_PIN 33
#elif ENABLED(MINIPANEL)
#define BEEPER_PIN        42
#define DOGLCD_A0         44
#define DOGLCD_CS         66
#define LCD_BACKLIGHT_PIN 65 
#define SDSS              53
#define KILL_PIN          64
#define BTN_EN1           40
#define BTN_EN2           63
#define BTN_ENC           59
#define SD_DETECT_PIN     49
#else
#define BEEPER_PIN        33
#if ENABLED(REPRAPWORLD_KEYPAD)
#define BTN_EN1         64
#define BTN_EN2         59
#define BTN_ENC         63
#define SHIFT_OUT       40
#define SHIFT_CLK       44
#define SHIFT_LD        42
#elif ENABLED(PANEL_ONE)
#define BTN_EN1         59 
#define BTN_EN2         63 
#define BTN_ENC         49 
#else
#define BTN_EN1         37
#define BTN_EN2         35
#define BTN_ENC         31
#endif
#if ENABLED(G3D_PANEL)
#define SD_DETECT_PIN   49
#define KILL_PIN        41
#else
#endif
#endif
#endif 
#endif 
