#if HOTENDS > 2 || E_STEPPERS > 2
  #error "2PrintBeta Due supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif
#define BOARD_NAME "2PrintBeta Due"
#define SPINDLE_LASER_ENABLE_PIN 66  
#define SPINDLE_DIR_PIN          67
#define SPINDLE_LASER_PWM_PIN    44  
#include "pins_RAMPS.h"
#undef TEMP_0_PIN
#undef TEMP_1_PIN
#define TEMP_0_PIN          9   
#define TEMP_1_PIN         11   
