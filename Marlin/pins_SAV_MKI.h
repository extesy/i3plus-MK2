#ifndef __AVR_AT90USB1286__
  #error "Oops!  Make sure you have 'Teensy++ 2.0' or 'Printrboard' selected from the 'Tools -> Boards' menu."
#endif
#define DEFAULT_MACHINE_NAME    "SAV MkI"
#define DEFAULT_SOURCE_CODE_URL "https://github.com/fmalpartida/Marlin/tree/SAV-MkI-config"
#define BOARD_NAME              "SAV MkI"
#define USBCON 1286  // Disable MarlinSerial etc.
#define LARGE_FLASH        true
//
// Servos
//
#define SERVO0_PIN         39   // F1  In teensy's pin definition for pinMode (in servo.cpp)
//
// Limit Switches
//
#define X_STOP_PIN         25   // B5
#define Y_STOP_PIN         26   // B6
//#define Z_STOP_PIN         27   // B7
#define Z_STOP_PIN         36   // E4 For inductive sensor.
//#define E_STOP_PIN         36   // E4
//
// Steppers
//
#define X_STEP_PIN         28   // A0
#define X_DIR_PIN          29   // A1
#define X_ENABLE_PIN       19   // E7
#define Y_STEP_PIN         30   // A2
#define Y_DIR_PIN          31   // A3
#define Y_ENABLE_PIN       18   // E6
#define Z_STEP_PIN         32   // A4
#define Z_DIR_PIN          33   // A5
#define Z_ENABLE_PIN       17   // C7
#define E0_STEP_PIN        34   // A6
#define E0_DIR_PIN         35   // A7
#define E0_ENABLE_PIN      13   // C3
//
// Temperature Sensors
//
#define TEMP_0_PIN          7   // F7  Analog Input (Extruder)
#define TEMP_BED_PIN        6   // F6  Analog Input (Bed)
//
// Heaters / Fans
//
#define HEATER_0_PIN       15   // C5 PWM3B - Extruder
#define HEATER_BED_PIN     14   // C4 PWM3C - Bed
#define FAN_PIN            16   // C6 PWM3A
//
// Misc. Functions
//
#define SDSS               20   // B0
// Extension header pin mapping
// ----------------------------
//  SCL (I2C)-D0    A0 (An), IO
//  SDA (I2C)-D1    A1 (An), IO
//  RX1-D2          A2 (An), IO
//  TX1-D3          A3 (An), IO
//  PWM-D24         A4 (An), IO
//  5V              GND
//  12V             GND
#define EXT_AUX_SCL_D0      0   // D0  PWM0B
#define EXT_AUX_SDA_D1      1   // D1
#define EXT_AUX_RX1_D2      2   // D2
#define EXT_AUX_TX1_D3      3   // D3
#define EXT_AUX_PWM_D24    24   // B4  PWM2A
#define EXT_AUX_A0          0   // F0  Analog Input
#define EXT_AUX_A0_IO      38   // F0  Digital IO
#define EXT_AUX_A1          1   // F1  Analog Input
#define EXT_AUX_A1_IO      39   // F1  Digital IO
#define EXT_AUX_A2          2   // F2  Analog Input
#define EXT_AUX_A2_IO      40   // F2  Digital IO
#define EXT_AUX_A3          3   // F3  Analog Input
#define EXT_AUX_A3_IO      41   // F3  Digital IO
#define EXT_AUX_A4          4   // F4  Analog Input
#define EXT_AUX_A4_IO      42   // F4  Digital IO
//
// LCD / Controller
//
#define BEEPER_PIN         -1
#define LCD_PINS_RS        -1
#define LCD_PINS_ENABLE    -1
#define LCD_PINS_D4        -1
#define LCD_PINS_D5        -1
#define LCD_PINS_D6        -1
#define LCD_PINS_D7        -1
#if ENABLED(SAV_3DLCD)
  // For LCD SHIFT register LCD
  #define SR_DATA_PIN      EXT_AUX_SDA_D1
  #define SR_CLK_PIN       EXT_AUX_SCL_D0
#endif
#if ENABLED(SAV_3DLCD) || ENABLED(SAV_3DGLCD)
  #define BTN_EN1          EXT_AUX_A1_IO
  #define BTN_EN2          EXT_AUX_A0_IO
  #define BTN_ENC          EXT_AUX_PWM_D24
  #define KILL_PIN         EXT_AUX_A2_IO
  #define HOME_PIN         EXT_AUX_A4_IO
#else // Use the expansion header for spindle control
  //
  // M3/M4/M5 - Spindle/Laser Control
  //
  #define SPINDLE_LASER_PWM_PIN    24  // B4  PWM2A
  #define SPINDLE_LASER_ENABLE_PIN 39  // F1  Pin should have a pullup!
  #define SPINDLE_DIR_PIN          40  // F2
  #define CASE_LIGHT_PIN            0  // D0  PWM0B
#endif
