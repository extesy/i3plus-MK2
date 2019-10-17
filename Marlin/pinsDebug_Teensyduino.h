#ifndef __PINSDEBUG_TEENSYDUINO_H__
#define __PINSDEBUG_TEENSYDUINO_H__
#undef NUM_DIGITAL_PINS
#define NUM_DIGITAL_PINS 48  
#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
const uint8_t PROGMEM digital_pin_to_port_PGM_Teensy[] = {
  PD, 
  PD, 
  PD, 
  PD, 
  PD, 
  PD, 
  PD, 
  PD, 
  PE, 
  PE, 
  PC, 
  PC, 
  PC, 
  PC, 
  PC, 
  PC, 
  PC, 
  PC, 
  PE, 
  PE, 
  PB, 
  PB, 
  PB, 
  PB, 
  PB, 
  PB, 
  PB, 
  PB, 
  PA, 
  PA, 
  PA, 
  PA, 
  PA, 
  PA, 
  PA, 
  PA, 
  PE, 
  PE, 
  PF, 
  PF, 
  PF, 
  PF, 
  PF, 
  PF, 
  PF, 
  PF, 
  PE, 
  PE, 
};
#define digitalPinToPort_Teensy(P) ( pgm_read_byte( digital_pin_to_port_PGM_Teensy + (P) ) )
#define digitalRead_mod(p)  digitalRead(p)   
#endif 
