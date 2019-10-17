#define board_rev_1_1_TO_1_3
#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif
#define DEFAULT_MACHINE_NAME    "Ultimaker"
#define DEFAULT_SOURCE_CODE_URL "https://github.com/Ultimaker/Marlin"
#define BOARD_NAME              "Ultimaker <1.5.4"
#define LARGE_FLASH true
//
// Limit Switches
//
#if ENABLED(board_rev_1_1_TO_1_3)
  #define X_MIN_PIN          15  // SW1
  #define X_MAX_PIN          14  // SW2
  #define Y_MIN_PIN          17  // SW3
  #define Y_MAX_PIN          16  // SW4
  #define Z_MIN_PIN          19  // SW5
  #define Z_MAX_PIN          18  // SW6
#endif
#if ENABLED(board_rev_1_0)
  #define X_MIN_PIN          13  // SW1
  #define X_MAX_PIN          12  // SW2
  #define Y_MIN_PIN          11  // SW3
  #define Y_MAX_PIN          10  // SW4
  #define Z_MIN_PIN           9  // SW5
  #define Z_MAX_PIN           8  // SW6
#endif
#if ENABLED(board_rev_1_5)
  #define X_MIN_PIN          22
  #define X_MAX_PIN          24
  #define Y_MIN_PIN          26
  #define Y_MAX_PIN          28
  #define Z_MIN_PIN          30
  #define Z_MAX_PIN          32
#endif
//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  Z_MAX_PIN
#endif
//
// Steppers
//
#define X_STEP_PIN         25
#define X_DIR_PIN          23
#define X_ENABLE_PIN       27
#define Y_STEP_PIN         31
#define Y_DIR_PIN          33
#define Y_ENABLE_PIN       29
#define Z_STEP_PIN         37
#define Z_DIR_PIN          39
#define Z_ENABLE_PIN       35
#define E0_STEP_PIN        43
#define E0_DIR_PIN         45
#define E0_ENABLE_PIN      41
#define E1_STEP_PIN        -1  // 49
#define E1_DIR_PIN         -1  // 47
#define E1_ENABLE_PIN      -1  // 48
//
// Temperature Sensors
//
#define TEMP_0_PIN          8   // Analog Input
#define TEMP_1_PIN          1   // Analog Input
//
// Heaters / Fans
//
#define HEATER_0_PIN        2
//#define HEATER_1_PIN        3 // used for case light   Rev A said "1"
#define HEATER_BED_PIN      4
//
// LCD / Controller
//
#if ENABLED(board_rev_1_0) || ENABLED(board_rev_1_1_TO_1_3)
  #define LCD_PINS_RS        24
  #define LCD_PINS_ENABLE    22
  #define LCD_PINS_D4        36
  #define LCD_PINS_D5        34
  #define LCD_PINS_D6        32
  #define LCD_PINS_D7        30
#elif ENABLED(board_rev_1_5) && ENABLED(ULTRA_LCD)
  #define BEEPER_PIN 18
  #if ENABLED(NEWPANEL)
    #define LCD_PINS_RS 20
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 16
    #define LCD_PINS_D5 21
    #define LCD_PINS_D6 5
    #define LCD_PINS_D7 6
    // buttons are directly attached
    #define BTN_EN1 40
    #define BTN_EN2 42
    #define BTN_ENC 19
    #define SD_DETECT_PIN 38
  #else // !NEWPANEL - Old style panel with shift register
    // buttons are attached to a shift register
    #define SHIFT_CLK 38
    #define SHIFT_LD 42
    #define SHIFT_OUT 40
    #define SHIFT_EN 17
    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 5
    #define LCD_PINS_D4 6
    #define LCD_PINS_D5 21
    #define LCD_PINS_D6 20
    #define LCD_PINS_D7 19
    #define SD_DETECT_PIN -1
  #endif // !NEWPANEL
#endif // ULTRA_LCD
//
// case light  - see spindle section for more info on available hardware PWMs
//
#if !PIN_EXISTS(CASE_LIGHT) && ENABLED(board_rev_1_5)
  #define CASE_LIGHT_PIN        7  // use PWM -  MUST BE HARDWARE PWM
#endif
//
// M3/M4/M5 - Spindle/Laser Control
//
#if ENABLED(SPINDLE_LASER_ENABLE)
  #if ENABLED(board_rev_1_0)       // use the last three SW positions
    #undef Z_MIN_PROBE_PIN
    #undef X_MIN_PIN              // SW1
    #undef X_MAX_PIN              // SW2
    #undef Y_MIN_PIN              // SW3
    #undef Y_MAX_PIN              // SW4
    #undef Z_MIN_PIN              // SW5
    #undef Z_MAX_PIN              // SW6
    #define X_STOP_PIN         13  // SW1  (didn't change) - also has a useable hardware PWM
    #define Y_STOP_PIN         12  // SW2
    #define Z_STOP_PIN         11  // SW3
    #define SPINDLE_DIR_PIN          10  
    #define SPINDLE_LASER_PWM_PIN     9  
    #define SPINDLE_LASER_ENABLE_PIN  8  
  #elif ENABLED(board_rev_1_5)      
    #define SPINDLE_DIR_PIN          10  
    #define SPINDLE_LASER_PWM_PIN     9  
    #define SPINDLE_LASER_ENABLE_PIN  8  
  #elif ENABLED(board_rev_1_1_TO_1_3)
    #if EXTRUDERS == 1                     
      #undef E0_STEP_PIN
      #undef E0_DIR_PIN
      #undef E0_ENABLE_PIN
      #define E0_STEP_PIN              49
      #define E0_DIR_PIN               47
      #define E0_ENABLE_PIN            48
      #define SPINDLE_DIR_PIN          43
      #define SPINDLE_LASER_PWM_PIN    45  
      #define SPINDLE_LASER_ENABLE_PIN 41  
    #elif TEMP_SENSOR_BED == 0  
      #undef HEATER_BED_PIN
      #define SPINDLE_DIR_PIN          38  
      #define SPINDLE_LASER_PWM_PIN     4  
      #define SPINDLE_LASER_ENABLE_PIN 40  
    #endif
  #endif
#endif
