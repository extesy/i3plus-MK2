#ifndef _ENDSTOP_INTERRUPTS_H_
#define _ENDSTOP_INTERRUPTS_H_
#include "macros.h"
#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA)
  #undef  digitalPinToPCICR
  #define digitalPinToPCICR(p)    ( WITHIN(p, 10, 15) || \
                                    WITHIN(p, 50, 53) || \
                                    WITHIN(p, 62, 69) ? &PCICR : (uint8_t*)0 )
  #undef  digitalPinToPCICRbit
  #define digitalPinToPCICRbit(p) ( WITHIN(p, 10, 13) || WITHIN(p, 50, 53) ? 0 : \
                                    WITHIN(p, 14, 15) ? 1 : \
                                    WITHIN(p, 62, 69) ? 2 : \
                                    0 )
  #undef  digitalPinToPCMSK
  #define digitalPinToPCMSK(p)    ( WITHIN(p, 10, 13) || WITHIN(p, 50, 53) ? &PCMSK0 : \
                                    WITHIN(p, 14, 15) ? &PCMSK1 : \
                                    WITHIN(p, 62, 69) ? &PCMSK2 : \
                                    (uint8_t *)0 )
  #undef  digitalPinToPCMSKbit
  #define digitalPinToPCMSKbit(p) ( WITHIN(p, 10, 13) ? ((p) - 6) : \
                                    (p) == 14 || (p) == 51 ? 2 : \
                                    (p) == 15 || (p) == 52 ? 1 : \
                                    (p) == 50 ? 3 : \
                                    (p) == 53 ? 0 : \
                                    WITHIN(p, 62, 69) ? ((p) - 62) : \
                                    0 )
#endif
volatile uint8_t e_hit = 0; 
void pciSetup(byte pin) {
  SBI(*digitalPinToPCMSK(pin), digitalPinToPCMSKbit(pin));  
  SBI(PCIFR, digitalPinToPCICRbit(pin)); 
  SBI(PCICR, digitalPinToPCICRbit(pin)); 
}
FORCE_INLINE void endstop_ISR_worker( void ) {
  e_hit = 2; 
}
void endstop_ISR(void) { endstop_ISR_worker(); }
#ifdef PCINT0_vect
  ISR(PCINT0_vect) { endstop_ISR_worker(); }
#endif
#ifdef PCINT1_vect
  ISR(PCINT1_vect) { endstop_ISR_worker(); }
#endif
#ifdef PCINT2_vect
  ISR(PCINT2_vect) { endstop_ISR_worker(); }
#endif
#ifdef PCINT3_vect
  ISR(PCINT3_vect) { endstop_ISR_worker(); }
#endif
void setup_endstop_interrupts( void ) {
  #if HAS_X_MAX
    #if (digitalPinToInterrupt(X_MAX_PIN) != NOT_AN_INTERRUPT) 
      attachInterrupt(digitalPinToInterrupt(X_MAX_PIN), endstop_ISR, CHANGE); 
    #else
      static_assert(digitalPinToPCICR(X_MAX_PIN) != NULL, "X_MAX_PIN is not interrupt-capable"); 
      pciSetup(X_MAX_PIN);                                                            
    #endif
  #endif
  #if HAS_X_MIN
    #if (digitalPinToInterrupt(X_MIN_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(X_MIN_PIN), endstop_ISR, CHANGE);
    #else
      static_assert(digitalPinToPCICR(X_MIN_PIN) != NULL, "X_MIN_PIN is not interrupt-capable");
      pciSetup(X_MIN_PIN);
    #endif
  #endif
  #if HAS_Y_MAX
    #if (digitalPinToInterrupt(Y_MAX_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Y_MAX_PIN), endstop_ISR, CHANGE);
    #else
      static_assert(digitalPinToPCICR(Y_MAX_PIN) != NULL, "Y_MAX_PIN is not interrupt-capable");
      pciSetup(Y_MAX_PIN);
    #endif
  #endif
  #if HAS_Y_MIN
    #if (digitalPinToInterrupt(Y_MIN_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Y_MIN_PIN), endstop_ISR, CHANGE);
    #else
      static_assert(digitalPinToPCICR(Y_MIN_PIN) != NULL, "Y_MIN_PIN is not interrupt-capable");
      pciSetup(Y_MIN_PIN);
    #endif
  #endif
  #if HAS_Z_MAX
    #if (digitalPinToInterrupt(Z_MAX_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Z_MAX_PIN), endstop_ISR, CHANGE);
    #else
      static_assert(digitalPinToPCICR(Z_MAX_PIN) != NULL, "Z_MAX_PIN is not interrupt-capable");
      pciSetup(Z_MAX_PIN);
    #endif
  #endif
  #if HAS_Z_MIN
    #if (digitalPinToInterrupt(Z_MIN_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Z_MIN_PIN), endstop_ISR, CHANGE);
    #else
      static_assert(digitalPinToPCICR(Z_MIN_PIN) != NULL, "Z_MIN_PIN is not interrupt-capable");
      pciSetup(Z_MIN_PIN);
    #endif
  #endif
  #if HAS_Z2_MAX
    #if (digitalPinToInterrupt(Z2_MAX_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Z2_MAX_PIN), endstop_ISR, CHANGE);
    #else
      static_assert(digitalPinToPCICR(Z2_MAX_PIN) != NULL, "Z2_MAX_PIN is not interrupt-capable");
      pciSetup(Z2_MAX_PIN);
    #endif
  #endif
  #if HAS_Z2_MIN
    #if (digitalPinToInterrupt(Z2_MIN_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Z2_MIN_PIN), endstop_ISR, CHANGE);
    #else
      static_assert(digitalPinToPCICR(Z2_MIN_PIN) != NULL, "Z2_MIN_PIN is not interrupt-capable");
      pciSetup(Z2_MIN_PIN);
    #endif
  #endif
  #if HAS_Z_MIN_PROBE_PIN
    #if (digitalPinToInterrupt(Z_MIN_PROBE_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Z_MIN_PROBE_PIN), endstop_ISR, CHANGE);
    #else
      static_assert(digitalPinToPCICR(Z_MIN_PROBE_PIN) != NULL, "Z_MIN_PROBE_PIN is not interrupt-capable");
      pciSetup(Z_MIN_PROBE_PIN);
    #endif
  #endif
}
#endif 
