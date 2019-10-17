#define BOARD_NAME "RigidBoard"
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  19    
#endif
#define RAMPS_D10_PIN       9 
#define MOSFET_D_PIN       12 
#include "pins_RAMPS.h"
#undef E0_STEP_PIN
#undef E0_DIR_PIN
#undef E0_ENABLE_PIN
#define E0_STEP_PIN        36
#define E0_DIR_PIN         34
#define E0_ENABLE_PIN      30
#undef E1_STEP_PIN
#undef E1_DIR_PIN
#undef E1_ENABLE_PIN
#define E1_STEP_PIN        26
#define E1_DIR_PIN         28
#define E1_ENABLE_PIN      24
#define STEPPER_RESET_PIN  41   
#undef TEMP_0_PIN
#undef TEMP_1_PIN
#undef TEMP_BED_PIN
#define TEMP_0_PIN         14   
#define TEMP_1_PIN         13   
#define TEMP_BED_PIN       15   
#undef MAX6675_SS
#if DISABLED(SDSUPPORT)
  #define MAX6675_SS       53 
#else
  #define MAX6675_SS       49 
#endif
#undef HEATER_BED_PIN
#define HEATER_BED_PIN     10
#undef FAN_PIN
#define FAN_PIN             8 
#undef PS_ON_PIN
#define PS_ON_PIN          -1
#if ENABLED(RIGIDBOT_PANEL)
  #undef BEEPER_PIN
  #define BEEPER_PIN -1
  #define BTN_UP           37
  #define BTN_DWN          35
  #define BTN_LFT          33
  #define BTN_RT           32
  #undef BTN_ENC
  #define BTN_ENC 31
  #undef BTN_EN1
  #define BTN_EN1 -1
  #undef BTN_EN2
  #define BTN_EN2 -1
  #undef SD_DETECT_PIN
  #define SD_DETECT_PIN 22
#elif ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
  #undef SD_DETECT_PIN
  #define SD_DETECT_PIN 22
  #undef KILL_PIN
  #define KILL_PIN 32
#endif
