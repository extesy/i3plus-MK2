#ifndef __AVR_AT90USB1286__
  #error "Oops!  Make sure you have 'Teensy++ 2.0' or 'Printrboard' selected from the 'Tools -> Boards' menu."
#endif
#define DEFAULT_MACHINE_NAME "Makibox"
#define BOARD_NAME           "5DPrint D8"
#define USBCON 1286  
#define LARGE_FLASH        true
#define X_STOP_PIN         37   
#define Y_STOP_PIN         36   
#define Z_STOP_PIN         19   
#define X_STEP_PIN         28   
#define X_DIR_PIN          29   
#define X_ENABLE_PIN       17   
#define Y_STEP_PIN         30   
#define Y_DIR_PIN          31   
#define Y_ENABLE_PIN       13   
#define Z_STEP_PIN         32   
#define Z_DIR_PIN          33   
#define Z_ENABLE_PIN       12   
#define E0_STEP_PIN        34   
#define E0_DIR_PIN         35   
#define E0_ENABLE_PIN      11   
#define X_MS1_PIN          25   
#define X_MS2_PIN          26   
#define Y_MS1_PIN           9   
#define Y_MS2_PIN           8   
#define Z_MS1_PIN           7   
#define Z_MS2_PIN           6   
#define E0_MS1_PIN          5   
#define E0_MS2_PIN          4   
#define TEMP_0_PIN          1   
#define TEMP_BED_PIN        0   
#define HEATER_0_PIN       15   
#define HEATER_BED_PIN     14   
#define FAN_PIN            16   
#define SDSS               20   
