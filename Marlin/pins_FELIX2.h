#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Felix 2.0+ supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif
#define BOARD_NAME "Felix 2.0+"
#define MOSFET_D_PIN 7
#include "pins_RAMPS.h"
#undef SDPOWER
#define SDPOWER             1
#define PS_ON_PIN          12
#if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)
  #define SD_DETECT_PIN 6
#endif 
#undef SPINDLE_LASER_PWM_PIN     
#undef SPINDLE_LASER_ENABLE_PIN
#undef SPINDLE_DIR_PIN
