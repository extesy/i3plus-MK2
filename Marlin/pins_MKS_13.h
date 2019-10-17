#if HOTENDS > 2 || E_STEPPERS > 2
  #error "MKS 1.3/1.4 supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif
#define BOARD_NAME "MKS > v1.3"
#define MOSFET_D_PIN 7
#include "pins_RAMPS.h"
#if ENABLED(VIKI2) || ENABLED(miniVIKI)
  #undef BTN_EN1
  #undef BTN_EN2
  #undef BTN_ENC
  #undef DOGLCD_A0
  #undef DOGLCD_CS
  #undef SD_DETECT_PIN
  #undef BEEPER_PIN
  #undef KILL_PIN
  #define SD_DETECT_PIN   49
  #define BTN_EN1         35
  #define BTN_EN2         37
  #define BTN_ENC         31
  #define DOGLCD_A0       27
  #define DOGLCD_CS       29
  #define KILL_PIN        23
  #define BEEPER_PIN      25
  #define STAT_LED_RED_PIN 16
  #define STAT_LED_BLUE_PIN 17
#endif
