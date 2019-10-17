#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Sainsmart 2-in-1 supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif
#define BOARD_NAME "Sainsmart"
#define RAMPS_D10_PIN 9 
#define RAMPS_D9_PIN  7 
#define MOSFET_D_PIN 10 
#include "pins_RAMPS.h"
