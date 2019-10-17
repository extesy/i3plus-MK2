#include "pins_RIGIDBOARD.h"
#undef BOARD_NAME
#define BOARD_NAME "RigidBoard V2"
#define DAC_STEPPER_CURRENT
#define DAC_STEPPER_ORDER { 0, 1, 2, 3 }
#define DAC_STEPPER_SENSE    0.05 
#define DAC_STEPPER_ADDRESS  0
#define DAC_STEPPER_MAX   4096 
#define DAC_STEPPER_VREF     1 
#define DAC_STEPPER_GAIN     1 
#define DAC_DISABLE_PIN     42 
#define DAC_OR_ADDRESS    0x01
#ifndef DAC_MOTOR_CURRENT_DEFAULT
  #define DAC_MOTOR_CURRENT_DEFAULT { 70, 80, 90, 80 } 
#endif
