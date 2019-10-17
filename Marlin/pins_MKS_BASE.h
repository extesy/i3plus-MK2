#if (HOTENDS > 2 || E_STEPPERS > 2)&&CLIENT_VAR!=NEW_FUNCTION_yszTEST
  #error "MKS BASE 1.0 supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif
#define BOARD_NAME "MKS BASE 1.0"
#define MOSFET_D_PIN 7
#define CASE_LIGHT_PIN            2
#define SPINDLE_LASER_PWM_PIN     2  
#define SPINDLE_LASER_ENABLE_PIN 15  
#define SPINDLE_DIR_PIN          19
#include "pins_RAMPS.h"
