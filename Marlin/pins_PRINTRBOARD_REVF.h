#ifndef __AVR_AT90USB1286__
  #error "Oops!  Make sure you have 'Teensy++ 2.0' or 'Printrboard' selected from the 'Tools -> Boards' menu."
#endif
#define BOARD_NAME         "Printrboard Rev F"
#define LARGE_FLASH        true
#define X_STOP_PIN         47   
#define Y_STOP_PIN         24   
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
#define DAC_STEPPER_CURRENT
#ifndef DAC_MOTOR_CURRENT_DEFAULT 
  #define DAC_MOTOR_CURRENT_DEFAULT { 70, 70, 50, 70 }    
#endif
#define DAC_STEPPER_ORDER { 3, 2, 1, 0 }
#define DAC_STEPPER_SENSE    0.11
#define DAC_STEPPER_ADDRESS  0
#define DAC_STEPPER_MAX   3520
#define DAC_STEPPER_VREF     1   
#define DAC_STEPPER_GAIN     0
#define DAC_OR_ADDRESS    0x00
#define TEMP_0_PIN          1   
#define TEMP_BED_PIN        0   
#define HEATER_0_PIN       15   
#define HEATER_1_PIN       44   
#define HEATER_2_PIN       45   
#define HEATER_BED_PIN     14   
#define FAN_PIN            16   
#if ENABLED(ULTRA_LCD)
  #define BEEPER_PIN       -1
  #define LCD_PINS_RS       9   
  #define LCD_PINS_ENABLE   8   
  #define LCD_PINS_D4       7   
  #define LCD_PINS_D5       6   
  #define LCD_PINS_D6       5   
  #define LCD_PINS_D7       4   
  #define BTN_EN1          10   
  #define BTN_EN2          11   
  #define BTN_ENC          12   
  #define SD_DETECT_PIN    -1
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
#if ENABLED(MINIPANEL)
  #if ENABLED(USE_INTERNAL_SD)
    #define SDSS               20  
    #define SD_DETECT_PIN      -1  
  #else
    #define SDSS               11  
    #define SD_DETECT_PIN       9  
  #endif
    #define DOGLCD_A0           4  
    #define DOGLCD_CS           5  
    #define BTN_ENC             6  
    #define BEEPER_PIN          7  
    #define KILL_PIN            8  
    #define BTN_EN1            10  
    #define BTN_EN2            12  
  #define BLEN_A 0
  #define BLEN_B 1
  #define BLEN_C 2
  #define encrot0 0
  #define encrot1 2
  #define encrot2 3
  #define encrot3 1
  #define ST7920_DELAY_1 DELAY_5_NOP
  #define ST7920_DELAY_2 DELAY_5_NOP
  #define ST7920_DELAY_3 DELAY_5_NOP
#endif
#ifndef SDSS
  #define SDSS               20   
#endif
#define FILWIDTH_PIN        2   
