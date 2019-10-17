#ifndef _FASTIO_ARDUINO_H
#define _FASTIO_ARDUINO_H
#include <avr/io.h>
#include "macros.h"
#define AVR_AT90USB1286_FAMILY (defined(__AVR_AT90USB1287__) || defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1286P__) || defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB646P__)  || defined(__AVR_AT90USB647__))
#define AVR_ATmega1284_FAMILY (defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__))
#define AVR_ATmega2560_FAMILY (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__))
#define AVR_ATmega2561_FAMILY (defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__))
#define AVR_ATmega328_FAMILY (defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328p__))
#if AVR_ATmega328_FAMILY
  #include "fastio_168.h"
#elif AVR_ATmega1284_FAMILY
  #include "fastio_644.h"
#elif AVR_ATmega2560_FAMILY
  #include "fastio_1280.h"
#elif AVR_AT90USB1286_FAMILY
  #include "fastio_AT90USB.h"
#elif AVR_ATmega2561_FAMILY
  #include "fastio_1281.h"
#else
  #error "Pins for this chip not defined in Arduino.h! If you have a working pins definition, please contribute!"
#endif
#ifndef _BV
  #define _BV(PIN) (1UL << PIN)
#endif
#define _READ(IO) ((bool)(DIO ## IO ## _RPORT & _BV(DIO ## IO ## _PIN)))
#define _WRITE_NC(IO, v)  do { if (v) {DIO ##  IO ## _WPORT |= _BV(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~_BV(DIO ## IO ## _PIN); }; } while (0)
#define _WRITE_C(IO, v)   do { if (v) { \
                                         CRITICAL_SECTION_START; \
                                         {DIO ##  IO ## _WPORT |= _BV(DIO ## IO ## _PIN); } \
                                         CRITICAL_SECTION_END; \
                                       } \
                                       else { \
                                         CRITICAL_SECTION_START; \
                                         {DIO ##  IO ## _WPORT &= ~_BV(DIO ## IO ## _PIN); } \
                                         CRITICAL_SECTION_END; \
                                       } \
                                     } \
                                     while (0)
#define _WRITE(IO, v) do { if (&(DIO ## IO ## _RPORT) >= (uint8_t *)0x100) {_WRITE_C(IO, v); } else {_WRITE_NC(IO, v); }; } while (0)
#define _TOGGLE(IO) do {DIO ## IO ## _RPORT ^= _BV(DIO ## IO ## _PIN); } while (0)
#define _SET_INPUT(IO) do {DIO ## IO ## _DDR &= ~_BV(DIO ## IO ## _PIN); } while (0)
#define _SET_OUTPUT(IO) do {DIO ## IO ## _DDR |= _BV(DIO ## IO ## _PIN); } while (0)
#define _GET_INPUT(IO) ((DIO ## IO ## _DDR & _BV(DIO ## IO ## _PIN)) == 0)
#define _GET_OUTPUT(IO) ((DIO ## IO ## _DDR & _BV(DIO ## IO ## _PIN)) != 0)
#define _GET_TIMER(IO) (DIO ## IO ## _PWM)
#define READ(IO) _READ(IO)
#define WRITE(IO,V) _WRITE(IO,V)
#define TOGGLE(IO) _TOGGLE(IO)
#define SET_INPUT(IO) _SET_INPUT(IO)
#define SET_INPUT_PULLUP(IO) do{ _SET_INPUT(IO); _WRITE(IO, HIGH); }while(0)
#define SET_OUTPUT(IO) _SET_OUTPUT(IO)
#define GET_INPUT(IO) _GET_INPUT(IO)
#define GET_OUTPUT(IO) _GET_OUTPUT(IO)
#define GET_TIMER(IO) _GET_TIMER(IO)
#define OUT_WRITE(IO, v) do{ SET_OUTPUT(IO); WRITE(IO, v); }while(0)
typedef enum {
  WGM_NORMAL,          
  WGM_PWM_PC_8,        
  WGM_PWM_PC_9,        
  WGM_PWM_PC_10,       
  WGM_CTC_OCRnA,       
  WGM_FAST_PWM_8,      
  WGM_FAST_PWM_9,      
  WGM_FAST_PWM_10,     
  WGM_PWM_PC_FC_ICRn,  
  WGM_PWM_PC_FC_OCRnA, 
  WGM_PWM_PC_ICRn,     
  WGM_PWM_PC_OCRnA,    
  WGM_CTC_ICRn,        
  WGM_reserved,        
  WGM_FAST_PWM_ICRn,   
  WGM_FAST_PWM_OCRnA   
} WaveGenMode;
typedef enum {
  COM_NORMAL,          
  COM_TOGGLE,          
  COM_CLEAR_SET,       
  COM_SET_CLEAR        
} CompareMode;
typedef enum {
  CS_NONE,             
  CS_PRESCALER_1,      
  CS_PRESCALER_8,      
  CS_PRESCALER_64,     
  CS_PRESCALER_256,    
  CS_PRESCALER_1024,   
  CS_EXT_FALLING,      
  CS_EXT_RISING        
} ClockSource;
typedef enum {
  CS2_NONE,            
  CS2_PRESCALER_1,     
  CS2_PRESCALER_8,     
  CS2_PRESCALER_32,    
  CS2_PRESCALER_64,    
  CS2_PRESCALER_128,   
  CS2_PRESCALER_256,   
  CS2_PRESCALER_1024   
} ClockSource2;
#define GET_WGM(T)   (((TCCR##T##A >> WGM##T##0) & 0x3) | ((TCCR##T##B >> WGM##T##2 << 2) & 0xC))
#define GET_CS(T)    ((TCCR##T##B >> CS##T##0) & 0x7)
#define GET_COM(T,Q) ((TCCR##T##Q >> COM##T##Q##0) & 0x3)
#define GET_COMA(T)  GET_COM(T,A)
#define GET_COMB(T)  GET_COM(T,B)
#define GET_COMC(T)  GET_COM(T,C)
#define GET_ICNC(T)  (!!(TCCR##T##B & _BV(ICNC##T)))
#define GET_ICES(T)  (!!(TCCR##T##B & _BV(ICES##T)))
#define GET_FOC(T,Q) (!!(TCCR##T##C & _BV(FOC##T##Q)))
#define GET_FOCA(T)  GET_FOC(T,A)
#define GET_FOCB(T)  GET_FOC(T,B)
#define GET_FOCC(T)  GET_FOC(T,C)
#define _SET_WGM(T,V) do{ \
    TCCR##T##A = (TCCR##T##A & ~(0x3 << WGM##T##0)) | (( int(V)       & 0x3) << WGM##T##0); \
    TCCR##T##B = (TCCR##T##B & ~(0x3 << WGM##T##2)) | (((int(V) >> 2) & 0x3) << WGM##T##2); \
  }while(0)
#define SET_WGM(T,V) _SET_WGM(T,WGM_##V)
#define _SET_CS(T,V) (TCCR##T##B = (TCCR##T##B & ~(0x7 << CS##T##0)) | ((int(V) & 0x7) << CS##T##0))
#define _SET_CS0(V) _SET_CS(0,V)
#define _SET_CS1(V) _SET_CS(1,V)
#ifdef TCCR2
  #define _SET_CS2(V) (TCCR2 = (TCCR2 & ~(0x7 << CS20)) | (int(V) << CS20))
#else
  #define _SET_CS2(V) _SET_CS(2,V)
#endif
#define _SET_CS3(V) _SET_CS(3,V)
#define _SET_CS4(V) _SET_CS(4,V)
#define _SET_CS5(V) _SET_CS(5,V)
#define SET_CS0(V) _SET_CS0(CS_##V)
#define SET_CS1(V) _SET_CS1(CS_##V)
#ifdef TCCR2
  #define SET_CS2(V) _SET_CS2(CS2_##V)
#else
  #define SET_CS2(V) _SET_CS2(CS_##V)
#endif
#define SET_CS3(V) _SET_CS3(CS_##V)
#define SET_CS4(V) _SET_CS4(CS_##V)
#define SET_CS5(V) _SET_CS5(CS_##V)
#define SET_CS(T,V) SET_CS##T(V)
#define _SET_COM(T,Q,V) (TCCR##T##Q = (TCCR##T##Q & ~(0x3 << COM##T##Q##0)) | (int(V) << COM##T##Q##0))
#define SET_COM(T,Q,V) _SET_COM(T,Q,COM_##V)
#define SET_COMA(T,V) SET_COM(T,A,V)
#define SET_COMB(T,V) SET_COM(T,B,V)
#define SET_COMC(T,V) SET_COM(T,C,V)
#define SET_COMS(T,V1,V2,V3) do{ SET_COMA(T,V1); SET_COMB(T,V2); SET_COMC(T,V3); }while(0)
#define SET_ICNC(T,V) (TCCR##T##B = (V) ? TCCR##T##B | _BV(ICNC##T) : TCCR##T##B & ~_BV(ICNC##T))
#define SET_ICES(T,V) (TCCR##T##B = (V) ? TCCR##T##B | _BV(ICES##T) : TCCR##T##B & ~_BV(ICES##T))
#define SET_FOC(T,Q,V) (TCCR##T##C = (V) ? TCCR##T##C | _BV(FOC##T##Q) : TCCR##T##C & ~_BV(FOC##T##Q))
#define SET_FOCA(T,V) SET_FOC(T,A,V)
#define SET_FOCB(T,V) SET_FOC(T,B,V)
#define SET_FOCC(T,V) SET_FOC(T,C,V)
#if PIN_EXISTS(CONTROLLER_FAN)
  #define PWM_CHK_FAN_B(p) (p == CONTROLLER_FAN_PIN || p == E0_AUTO_FAN_PIN || p ==  E1_AUTO_FAN_PIN || p ==  E2_AUTO_FAN_PIN || p ==  E3_AUTO_FAN_PIN || p ==  E4_AUTO_FAN_PIN)
#else
  #define PWM_CHK_FAN_B(p) (p == E0_AUTO_FAN_PIN || p ==  E1_AUTO_FAN_PIN || p ==  E2_AUTO_FAN_PIN || p ==  E3_AUTO_FAN_PIN || p ==  E4_AUTO_FAN_PIN)
#endif
#if PIN_EXISTS(FAN) || PIN_EXISTS(FAN1) || PIN_EXISTS(FAN2)
  #if PIN_EXISTS(FAN2)
    #define PWM_CHK_FAN_A(p) (p == FAN_PIN || p == FAN1_PIN || p == FAN2_PIN)
  #elif PIN_EXISTS(FAN1)
    #define PWM_CHK_FAN_A(p) (p == FAN_PIN || p == FAN1_PIN)
  #else
    #define PWM_CHK_FAN_A(p) p == FAN_PIN
  #endif
#else
  #define PWM_CHK_FAN_A(p) false
#endif
#if HAS_MOTOR_CURRENT_PWM
  #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
    #define PWM_CHK_MOTOR_CURRENT(p) (p == MOTOR_CURRENT_PWM_E || p == MOTOR_CURRENT_PWM_Z || p == MOTOR_CURRENT_PWM_XY)
  #elif PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
    #define PWM_CHK_MOTOR_CURRENT(p) (p == MOTOR_CURRENT_PWM_E || p == MOTOR_CURRENT_PWM_Z)
  #else
    #define PWM_CHK_MOTOR_CURRENT(p) (p == MOTOR_CURRENT_PWM_E)
  #endif
#else
  #define PWM_CHK_MOTOR_CURRENT(p) false
#endif
#if defined(NUM_SERVOS)
  #if AVR_ATmega2560_FAMILY
    #define PWM_CHK_SERVO(p) ( p == 5 || NUM_SERVOS > 12 && p == 6 || NUM_SERVOS > 24 && p == 46)  
  #elif AVR_ATmega2561_FAMILY
    #define PWM_CHK_SERVO(p)   p ==  5  
  #elif AVR_ATmega1284_FAMILY
    #define PWM_CHK_SERVO(p)   false
  #elif AVR_AT90USB1286_FAMILY
    #define PWM_CHK_SERVO(p)   p ==  16 
  #elif AVR_ATmega328_FAMILY
    #define PWM_CHK_SERVO(p)   false
  #endif
#else
  #define PWM_CHK_SERVO(p) false
#endif
#if ENABLED(BARICUDA)
  #if HAS_HEATER_1 && HAS_HEATER_2
    #define PWM_CHK_HEATER(p) (p == HEATER_1_PIN || p == HEATER_2_PIN)
  #elif HAS_HEATER_1
    #define PWM_CHK_HEATER(p) (p == HEATER_1_PIN)
  #endif
#else
    #define PWM_CHK_HEATER(p) false
#endif
#define PWM_CHK(p) (PWM_CHK_HEATER(p) || PWM_CHK_SERVO(p)  || PWM_CHK_MOTOR_CURRENT(p)\
                     || PWM_CHK_FAN_A(p) || PWM_CHK_FAN_B(p))
#if AVR_ATmega2560_FAMILY
  #define PWM_PINS(p)  ((p >= 2 && p <= 10 ) || p ==  13 || p ==  44 || p ==  45 || p ==  46 )
#elif AVR_ATmega2561_FAMILY
  #define PWM_PINS(p)  ((p >= 2 && p <= 6 ) || p ==  9)
#elif AVR_ATmega1284_FAMILY
  #define PWM_PINS(p)  (p == 3 || p ==  4 || p ==  14 || p ==  15)
#elif AVR_AT90USB1286_FAMILY
  #define PWM_PINS(p)  (p == 0 || p ==  1 || p ==  14 || p ==  15 || p ==  16 || p ==  24)
#elif AVR_ATmega328_FAMILY
  #define PWM_PINS(p)  (p == 3 || p ==  5 || p ==  6 || p ==  11)
#else
  #error "unknown CPU"
#endif
#define USEABLE_HARDWARE_PWM(p) (PWM_PINS(p) && !PWM_CHK(p))
#endif 
