#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error "Oops! Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif
#define DEFAULT_MACHINE_NAME    "MB Replicator"
#define BOARD_NAME              "Mightyboard"
#define LARGE_FLASH true
#define SERVO0_PIN         36   
#define SERVO1_PIN         37   
#define SERVO2_PIN         40   
#define SERVO3_PIN         41   
#define X_MIN_PIN          49   
#define X_MAX_PIN          48   
#define Y_MIN_PIN          47   
#define Y_MAX_PIN          46   
#define Z_MIN_PIN          43   
#define Z_MAX_PIN          42   
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  42
#endif
#define X_STEP_PIN         55   
#define X_DIR_PIN          54   
#define X_ENABLE_PIN       56   
#define Y_STEP_PIN         59   
#define Y_DIR_PIN          58   
#define Y_ENABLE_PIN       60   
#define Z_STEP_PIN         63   
#define Z_DIR_PIN          62   
#define Z_ENABLE_PIN       64   
#define E0_STEP_PIN        25   
#define E0_DIR_PIN         24   
#define E0_ENABLE_PIN      26   
#define E1_STEP_PIN        29   
#define E1_DIR_PIN         28   
#define E1_ENABLE_PIN      39   
#define DIGIPOTS_I2C_SCL    76   
#define DIGIPOTS_I2C_SDA_X  57   
#define DIGIPOTS_I2C_SDA_Y  61   
#define DIGIPOTS_I2C_SDA_Z  65   
#define DIGIPOTS_I2C_SDA_E0 27   
#define DIGIPOTS_I2C_SDA_E1 77   
#define TEMP_BED_PIN        69   
#define THERMO_SCK_PIN      78   
#define THERMO_DO_PIN        3   
#define THERMO_CS1           5   
#define THERMO_CS2           2   
#define MAX6675_SS          THERMO_CS1
#define MAX6675_SCK_PIN     THERMO_SCK_PIN
#define MAX6675_DO_PIN      THERMO_DO_PIN
#define MOSFET_A_PIN         6   
#define MOSFET_B_PIN        11   
#define MOSFET_C_PIN        45   
#define MOSFET_D_PIN        44   
#if HOTENDS > 1
  #if TEMP_SENSOR_BED
    #define IS_EEB
  #else
    #define IS_EEF
  #endif
#elif TEMP_SENSOR_BED
  #define IS_EFB
#else
  #define IS_EFF
#endif
#define HEATER_0_PIN     MOSFET_A_PIN
#if ENABLED(IS_EFB)                            
  #define FAN_PIN        MOSFET_B_PIN
  #define HEATER_BED_PIN MOSFET_C_PIN
#elif ENABLED(IS_EEF)                          
  #define HEATER_1_PIN   MOSFET_B_PIN
  #define FAN_PIN        MOSFET_C_PIN
#elif ENABLED(IS_EEB)                          
  #define HEATER_1_PIN   MOSFET_B_PIN
  #define HEATER_BED_PIN MOSFET_C_PIN
#elif ENABLED(IS_EFF)                          
  #define FAN_PIN        MOSFET_B_PIN
  #define FAN1_PIN       MOSFET_C_PIN
#elif ENABLED(IS_SF)                           
  #define FAN_PIN        MOSFET_C_PIN
#endif
#ifndef FAN_PIN
  #define FAN_PIN MOSFET_D_PIN
#endif
#define ORIG_E0_AUTO_FAN_PIN  7   
#define ORIG_E1_AUTO_FAN_PIN 12   
#define LED_PIN             13   
#define CUTOFF_RESET_PIN    16   
#define CUTOFF_TEST_PIN     17   
#define CASE_LIGHT_PIN      44   
#ifdef REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
  #define LCD_PINS_RS       33   
  #define LCD_PINS_ENABLE   72   
  #define LCD_PINS_D4       35   
  #define LCD_PINS_D5       32   
  #define LCD_PINS_D6       34   
  #define LCD_PINS_D7       31   
  #define BTN_EN2           75   
  #define BTN_EN1           73   
  #define KILL_PIN          14   
  #define BEEPER_PIN         8   
  #define BTN_CENTER        15   
  #define BTN_ENC           BTN_CENTER
  #define STAT_LED_RED_LED  SERVO0_PIN 
  #define STAT_LED_BLUE_PIN SERVO1_PIN 
#else
  #define SAV_3DLCD
  #define SR_DATA_PIN       34   
  #define SR_CLK_PIN        35   
  #define SR_STROBE_PIN     33   
  #define BTN_UP            75   
  #define BTN_DOWN          73   
  #define BTN_LEFT          72   
  #define BTN_RIGHT         14   
  #define BTN_CENTER        15   
  #define BTN_ENC           BTN_CENTER
  #define BEEPER_PIN         4   
  #define STAT_LED_RED_PIN  32   
  #define STAT_LED_BLUE_PIN 31   
#endif
#define SDSS                53   
#define SD_DETECT_PIN        9   
#define MAX_PIN             THERMO_SCK_PIN
#define SPINDLE_LASER_ENABLE_PIN 66  
#define SPINDLE_LASER_PWM_PIN     8  
#define SPINDLE_DIR_PIN          67  
#include <Arduino.h>
static_assert(NUM_DIGITAL_PINS > MAX_PIN, "add missing pins to [arduino dir]/hardware/arduino/avr/variants/mega/pins_arduino.h based on fastio.h"
                                          "to digital_pin_to_port_PGM, digital_pin_to_bit_mask_PGM, digital_pin_to_timer_PGM, NUM_DIGITAL_PINS, see below");
