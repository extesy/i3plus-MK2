#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__)
  #error "Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu. (Final OMCA board)"
#endif
#define BOARD_NAME         "Final OMCA"
#define X_STOP_PIN         0
#define Y_STOP_PIN         1
#define Z_STOP_PIN         2
#define X_STEP_PIN         26
#define X_DIR_PIN          25
#define X_ENABLE_PIN       10
#define Y_STEP_PIN         28
#define Y_DIR_PIN          27
#define Y_ENABLE_PIN       10
#define Z_STEP_PIN         23
#define Z_DIR_PIN          22
#define Z_ENABLE_PIN       10
#define E0_STEP_PIN        24
#define E0_DIR_PIN         21
#define E0_ENABLE_PIN      10
#define E1_STEP_PIN        -1 
#define E1_DIR_PIN         -1 
#define E1_ENABLE_PIN      -1 
#define E2_STEP_PIN        -1 
#define E2_DIR_PIN         -1 
#define E2_ENABLE_PIN      -1 
#define TEMP_0_PIN          0   
#define TEMP_1_PIN          1   
#define TEMP_BED_PIN        2   
#define HEATER_0_PIN        3 
#define HEATER_BED_PIN      4
#define FAN_PIN            14 
#define SDSS               11
#define I2C_SCL_PIN        16
#define I2C_SDA_PIN        17
#define __FS  20
#define __FD  19
#define __GS  18
#define __GD  13
#define UNUSED_PWM         14 
