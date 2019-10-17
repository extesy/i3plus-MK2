#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'FYSETC_F6 V1.3' selected from the 'Tools -> Boards' menu."
#endif
#if MB(FYSETC_F6_13)
  #define BOARD_NAME "F6"
#endif
#define X_MIN_PIN          63
#define X_MAX_PIN          64
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15
#define Z_MIN_PIN          12
#define Z_MAX_PIN          2
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  -1 
#endif
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Z_STEP_PIN         43
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       58
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30
#define E2_STEP_PIN        59
#define E2_DIR_PIN         57
#define E2_ENABLE_PIN      40
#define X_CS_PIN        70
#define Y_CS_PIN        39
#define Z_CS_PIN        77
#define E0_CS_PIN       47
#define E1_CS_PIN       32
#define E2_CS_PIN       42
#define X_TMC2130_DIAG     -1 
#define Y_TMC2130_DIAG     -1 
#define Z_TMC2130_DIAG     -1 
#define E0_TMC2130_DIAG    -1
#define E1_TMC2130_DIAG    -1 
#define E2_TMC2130_DIAG    -1
#if ENABLED(HAVE_TMC2208)
 #define X_SERIAL_RX_PIN       71
 #define X_SERIAL_TX_PIN       72
 #define Y_SERIAL_RX_PIN       73
 #define Y_SERIAL_TX_PIN       74
 #define Z_SERIAL_RX_PIN       78
 #define Z_SERIAL_TX_PIN       79
 #define E0_SERIAL_RX_PIN      75
 #define E0_SERIAL_TX_PIN      76
 #define E1_SERIAL_RX_PIN      80
 #define E1_SERIAL_TX_PIN      81
 #define E2_SERIAL_RX_PIN      22
 #define E2_SERIAL_TX_PIN      82
#endif
#define TEMP_0_PIN         12   
#define TEMP_1_PIN         13   
#define TEMP_2_PIN         14   
#define TEMP_BED_PIN       15   
#define HEATER_0_PIN       5
#define HEATER_1_PIN       6
#define HEATER_2_PIN       7
#define HEATER_BED_PIN     8
#define FAN0_PIN           2
#define FAN1_PIN           3
#define FAN2_PIN           4
#define SDSS               53
#define LED_PIN            13
#if ENABLED(RGB_LED) || ENABLED(RGBW_LED)
  #define RGB_LED_R_PIN    44
  #define RGB_LED_G_PIN    45
  #define RGB_LED_B_PIN    46
  #define RGB_LED_W_PIN    -1
#endif
#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN      9   
#endif
#define FIL_RUNOUT_PIN      10
#ifndef PS_ON_PIN
  #define PS_ON_PIN         11
#endif
#if ENABLED(ULTRA_LCD)
    #define LCD_PINS_RS         16
    #define LCD_PINS_ENABLE     17
    #define LCD_PINS_D4         23
    #define LCD_PINS_D5         25
    #define LCD_PINS_D6         27
    #define LCD_PINS_D7         29
    #define BEEPER_PIN          37
    #define BTN_EN1             31
    #define BTN_EN2             33
    #define BTN_ENC             35
    #define SD_DETECT_PIN       49
    #define KILL_PIN            41
#endif 
