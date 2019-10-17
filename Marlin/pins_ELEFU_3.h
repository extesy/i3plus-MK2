#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME "Elefu Ra v3"
#define X_MIN_PIN          35
#define X_MAX_PIN          34
#define Y_MIN_PIN          33
#define Y_MAX_PIN          32
#define Z_MIN_PIN          31
#define Z_MAX_PIN          30
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  30
#endif
#define X_STEP_PIN         49
#define X_DIR_PIN          13
#define X_ENABLE_PIN       48
#define Y_STEP_PIN         11
#define Y_DIR_PIN           9
#define Y_ENABLE_PIN       12
#define Z_STEP_PIN          7
#define Z_DIR_PIN           6
#define Z_ENABLE_PIN        8
#define E0_STEP_PIN        40
#define E0_DIR_PIN         41
#define E0_ENABLE_PIN      37
#define E1_STEP_PIN        18
#define E1_DIR_PIN         19
#define E1_ENABLE_PIN      38
#define E2_STEP_PIN        43
#define E2_DIR_PIN         47
#define E2_ENABLE_PIN      42
#define TEMP_0_PIN          3   
#define TEMP_1_PIN          2   
#define TEMP_2_PIN          1   
#define TEMP_BED_PIN        0   
#define HEATER_0_PIN       45 
#define HEATER_1_PIN       46 
#define HEATER_2_PIN       17 
#define HEATER_BED_PIN     44 
#define FAN_PIN            16 
#define PS_ON_PIN          10 
#define SLEEP_WAKE_PIN     26 
#define PHOTOGRAPH_PIN     29
#define BEEPER_PIN         36
#if ENABLED(RA_CONTROL_PANEL)
  #define SDSS             53
  #define SD_DETECT_PIN    28
  #define BTN_EN1          14
  #define BTN_EN2          39
  #define BTN_ENC          15
#endif 
#if ENABLED(RA_DISCO)
  #define TLC_CLOCK_PIN    25
  #define TLC_BLANK_PIN    23
  #define TLC_XLAT_PIN     22
  #define TLC_DATA_PIN     24
  #define TLC_CLOCK_BIT 3 
  #define TLC_CLOCK_PORT &PORTA 
  #define TLC_BLANK_BIT 1 
  #define TLC_BLANK_PORT &PORTA 
  #define TLC_DATA_BIT 2 
  #define TLC_DATA_PORT &PORTA 
  #define TLC_XLAT_BIT 0 
  #define TLC_XLAT_PORT &PORTA 
  #define NUM_TLCS 2
  #define TRANS_ARRAY {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8} 
#endif 
