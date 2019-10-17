#ifndef __AVR_AT90USB1286__
  #error "Oops!  Make sure you have 'Teensy++ 2.0' or 'Printrboard' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME         "Brainwave Pro"
#define USBCON 1286  
#define LARGE_FLASH        true
#define X_STOP_PIN         45   
#define Y_STOP_PIN         12   
#define Z_STOP_PIN         36   
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  11   
#endif
#define X_STEP_PIN          9   
#define X_DIR_PIN           8   
#define X_ENABLE_PIN       23   
#define Y_STEP_PIN          7   
#define Y_DIR_PIN           6   
#define Y_ENABLE_PIN       20   
#define Z_STEP_PIN          5   
#define Z_DIR_PIN           4   
#define Z_ENABLE_PIN       37   
#define E0_STEP_PIN        47   
#define E0_DIR_PIN         46   
#define E0_ENABLE_PIN      25   
#define TEMP_0_PIN          2   
#define TEMP_1_PIN          1   
#define TEMP_BED_PIN        0   
#define HEATER_0_PIN       27   
#define HEATER_BED_PIN     26   
#define FAN_PIN            16   
#define SDSS               20   
#define SD_DETECT_PIN      24   
#define LED_PIN            13   
