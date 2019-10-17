#include <Arduino.h>
#ifdef __arm__
#ifdef CORE_TEENSY
static inline __attribute__((always_inline))
bool fastDigitalRead(uint8_t pin) {
  return *portInputRegister(pin);
}
static inline __attribute__((always_inline))
void fastDigitalWrite(uint8_t pin, bool value) {
  if (value) {
    *portSetRegister(pin) = 1;
  } else {
    *portClearRegister(pin) = 1;
  }
}
#else  
static inline __attribute__((always_inline))
bool fastDigitalRead(uint8_t pin){
  return g_APinDescription[pin].pPort->PIO_PDSR & g_APinDescription[pin].ulPin;
}
static inline __attribute__((always_inline))
void fastDigitalWrite(uint8_t pin, bool value){
  if(value) {
    g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;
  } else {
    g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin;
  }
}
#endif  
inline void fastDigitalToggle(uint8_t pin) {
 fastDigitalWrite(pin, !fastDigitalRead(pin));
 }
inline void fastPinMode(uint8_t pin, bool mode) {pinMode(pin, mode);}
#else  
#include <avr/io.h>
#include <util/atomic.h>
struct pin_map_t {
  volatile uint8_t* ddr;   
  volatile uint8_t* pin;   
  volatile uint8_t* port;  
  uint8_t bit;             
};
#if defined(__AVR_ATmega168__)||defined(__AVR_ATmega168P__)||defined(__AVR_ATmega328P__)
const static pin_map_t pinMap[] = {
  {&DDRD, &PIND, &PORTD, 0},  
  {&DDRD, &PIND, &PORTD, 1},  
  {&DDRD, &PIND, &PORTD, 2},  
  {&DDRD, &PIND, &PORTD, 3},  
  {&DDRD, &PIND, &PORTD, 4},  
  {&DDRD, &PIND, &PORTD, 5},  
  {&DDRD, &PIND, &PORTD, 6},  
  {&DDRD, &PIND, &PORTD, 7},  
  {&DDRB, &PINB, &PORTB, 0},  
  {&DDRB, &PINB, &PORTB, 1},  
  {&DDRB, &PINB, &PORTB, 2},  
  {&DDRB, &PINB, &PORTB, 3},  
  {&DDRB, &PINB, &PORTB, 4},  
  {&DDRB, &PINB, &PORTB, 5},  
  {&DDRC, &PINC, &PORTC, 0},  
  {&DDRC, &PINC, &PORTC, 1},  
  {&DDRC, &PINC, &PORTC, 2},  
  {&DDRC, &PINC, &PORTC, 3},  
  {&DDRC, &PINC, &PORTC, 4},  
  {&DDRC, &PINC, &PORTC, 5}   
};
#elif defined(__AVR_ATmega1280__)|| defined(__AVR_ATmega2560__)
static const pin_map_t pinMap[] = {
  {&DDRE, &PINE, &PORTE, 0},  
  {&DDRE, &PINE, &PORTE, 1},  
  {&DDRE, &PINE, &PORTE, 4},  
  {&DDRE, &PINE, &PORTE, 5},  
  {&DDRG, &PING, &PORTG, 5},  
  {&DDRE, &PINE, &PORTE, 3},  
  {&DDRH, &PINH, &PORTH, 3},  
  {&DDRH, &PINH, &PORTH, 4},  
  {&DDRH, &PINH, &PORTH, 5},  
  {&DDRH, &PINH, &PORTH, 6},  
  {&DDRB, &PINB, &PORTB, 4},  
  {&DDRB, &PINB, &PORTB, 5},  
  {&DDRB, &PINB, &PORTB, 6},  
  {&DDRB, &PINB, &PORTB, 7},  
  {&DDRJ, &PINJ, &PORTJ, 1},  
  {&DDRJ, &PINJ, &PORTJ, 0},  
  {&DDRH, &PINH, &PORTH, 1},  
  {&DDRH, &PINH, &PORTH, 0},  
  {&DDRD, &PIND, &PORTD, 3},  
  {&DDRD, &PIND, &PORTD, 2},  
  {&DDRD, &PIND, &PORTD, 1},  
  {&DDRD, &PIND, &PORTD, 0},  
  {&DDRA, &PINA, &PORTA, 0},  
  {&DDRA, &PINA, &PORTA, 1},  
  {&DDRA, &PINA, &PORTA, 2},  
  {&DDRA, &PINA, &PORTA, 3},  
  {&DDRA, &PINA, &PORTA, 4},  
  {&DDRA, &PINA, &PORTA, 5},  
  {&DDRA, &PINA, &PORTA, 6},  
  {&DDRA, &PINA, &PORTA, 7},  
  {&DDRC, &PINC, &PORTC, 7},  
  {&DDRC, &PINC, &PORTC, 6},  
  {&DDRC, &PINC, &PORTC, 5},  
  {&DDRC, &PINC, &PORTC, 4},  
  {&DDRC, &PINC, &PORTC, 3},  
  {&DDRC, &PINC, &PORTC, 2},  
  {&DDRC, &PINC, &PORTC, 1},  
  {&DDRC, &PINC, &PORTC, 0},  
  {&DDRD, &PIND, &PORTD, 7},  
  {&DDRG, &PING, &PORTG, 2},  
  {&DDRG, &PING, &PORTG, 1},  
  {&DDRG, &PING, &PORTG, 0},  
  {&DDRL, &PINL, &PORTL, 7},  
  {&DDRL, &PINL, &PORTL, 6},  
  {&DDRL, &PINL, &PORTL, 5},  
  {&DDRL, &PINL, &PORTL, 4},  
  {&DDRL, &PINL, &PORTL, 3},  
  {&DDRL, &PINL, &PORTL, 2},  
  {&DDRL, &PINL, &PORTL, 1},  
  {&DDRL, &PINL, &PORTL, 0},  
  {&DDRB, &PINB, &PORTB, 3},  
  {&DDRB, &PINB, &PORTB, 2},  
  {&DDRB, &PINB, &PORTB, 1},  
  {&DDRB, &PINB, &PORTB, 0},  
  {&DDRF, &PINF, &PORTF, 0},  
  {&DDRF, &PINF, &PORTF, 1},  
  {&DDRF, &PINF, &PORTF, 2},  
  {&DDRF, &PINF, &PORTF, 3},  
  {&DDRF, &PINF, &PORTF, 4},  
  {&DDRF, &PINF, &PORTF, 5},  
  {&DDRF, &PINF, &PORTF, 6},  
  {&DDRF, &PINF, &PORTF, 7},  
  {&DDRK, &PINK, &PORTK, 0},  
  {&DDRK, &PINK, &PORTK, 1},  
  {&DDRK, &PINK, &PORTK, 2},  
  {&DDRK, &PINK, &PORTK, 3},  
  {&DDRK, &PINK, &PORTK, 4},  
  {&DDRK, &PINK, &PORTK, 5},  
  {&DDRK, &PINK, &PORTK, 6},  
  {&DDRK, &PINK, &PORTK, 7},  
  {&DDRG, &PING, &PORTG, 4},  
  {&DDRG, &PING, &PORTG, 3},  
  {&DDRJ, &PINJ, &PORTJ, 2},  
  {&DDRJ, &PINJ, &PORTJ, 3},  
  {&DDRJ, &PINJ, &PORTJ, 7},  
  {&DDRJ, &PINJ, &PORTJ, 4},  
  {&DDRJ, &PINJ, &PORTJ, 5},  
  {&DDRJ, &PINJ, &PORTJ, 6},  
  {&DDRE, &PINE, &PORTE, 2},  
  {&DDRE, &PINE, &PORTE, 6}   
};
#elif defined(__AVR_ATmega1284P__)|| defined(__AVR_ATmega1284__)|| defined(__AVR_ATmega644P__)|| defined(__AVR_ATmega644__)|| defined(__AVR_ATmega64__)|| defined(__AVR_ATmega32__)|| defined(__AVR_ATmega324__)|| defined(__AVR_ATmega16__)
#if defined(VARIANT_MIGHTY)
static const pin_map_t pinMap[] = {
  {&DDRB, &PINB, &PORTB, 0},  
  {&DDRB, &PINB, &PORTB, 1},  
  {&DDRB, &PINB, &PORTB, 2},  
  {&DDRB, &PINB, &PORTB, 3},  
  {&DDRB, &PINB, &PORTB, 4},  
  {&DDRB, &PINB, &PORTB, 5},  
  {&DDRB, &PINB, &PORTB, 6},  
  {&DDRB, &PINB, &PORTB, 7},  
  {&DDRD, &PIND, &PORTD, 0},  
  {&DDRD, &PIND, &PORTD, 1},  
  {&DDRD, &PIND, &PORTD, 2},  
  {&DDRD, &PIND, &PORTD, 3},  
  {&DDRD, &PIND, &PORTD, 4},  
  {&DDRD, &PIND, &PORTD, 5},  
  {&DDRD, &PIND, &PORTD, 6},  
  {&DDRD, &PIND, &PORTD, 7},  
  {&DDRC, &PINC, &PORTC, 0},  
  {&DDRC, &PINC, &PORTC, 1},  
  {&DDRC, &PINC, &PORTC, 2},  
  {&DDRC, &PINC, &PORTC, 3},  
  {&DDRC, &PINC, &PORTC, 4},  
  {&DDRC, &PINC, &PORTC, 5},  
  {&DDRC, &PINC, &PORTC, 6},  
  {&DDRC, &PINC, &PORTC, 7},  
  {&DDRA, &PINA, &PORTA, 0},  
  {&DDRA, &PINA, &PORTA, 1},  
  {&DDRA, &PINA, &PORTA, 2},  
  {&DDRA, &PINA, &PORTA, 3},  
  {&DDRA, &PINA, &PORTA, 4},  
  {&DDRA, &PINA, &PORTA, 5},  
  {&DDRA, &PINA, &PORTA, 6},  
  {&DDRA, &PINA, &PORTA, 7}   
};
#elif defined(VARIANT_BOBUINO)
static const pin_map_t pinMap[] = {
  {&DDRD, &PIND, &PORTD, 0},  
  {&DDRD, &PIND, &PORTD, 1},  
  {&DDRD, &PIND, &PORTD, 2},  
  {&DDRD, &PIND, &PORTD, 3},  
  {&DDRB, &PINB, &PORTB, 0},  
  {&DDRB, &PINB, &PORTB, 1},  
  {&DDRB, &PINB, &PORTB, 2},  
  {&DDRB, &PINB, &PORTB, 3},  
  {&DDRD, &PIND, &PORTD, 5},  
  {&DDRD, &PIND, &PORTD, 6},  
  {&DDRB, &PINB, &PORTB, 4},  
  {&DDRB, &PINB, &PORTB, 5},  
  {&DDRB, &PINB, &PORTB, 6},  
  {&DDRB, &PINB, &PORTB, 7},  
  {&DDRA, &PINA, &PORTA, 7},  
  {&DDRA, &PINA, &PORTA, 6},  
  {&DDRA, &PINA, &PORTA, 5},  
  {&DDRA, &PINA, &PORTA, 4},  
  {&DDRA, &PINA, &PORTA, 3},  
  {&DDRA, &PINA, &PORTA, 2},  
  {&DDRA, &PINA, &PORTA, 1},  
  {&DDRA, &PINA, &PORTA, 0},  
  {&DDRC, &PINC, &PORTC, 0},  
  {&DDRC, &PINC, &PORTC, 1},  
  {&DDRC, &PINC, &PORTC, 2},  
  {&DDRC, &PINC, &PORTC, 3},  
  {&DDRC, &PINC, &PORTC, 4},  
  {&DDRC, &PINC, &PORTC, 5},  
  {&DDRC, &PINC, &PORTC, 6},  
  {&DDRC, &PINC, &PORTC, 7},  
  {&DDRD, &PIND, &PORTD, 4},  
  {&DDRD, &PIND, &PORTD, 7}   
};
#elif defined(VARIANT_STANDARD)
static const pin_map_t pinMap[] = {
  {&DDRB, &PINB, &PORTB, 0},  
  {&DDRB, &PINB, &PORTB, 1},  
  {&DDRB, &PINB, &PORTB, 2},  
  {&DDRB, &PINB, &PORTB, 3},  
  {&DDRB, &PINB, &PORTB, 4},  
  {&DDRB, &PINB, &PORTB, 5},  
  {&DDRB, &PINB, &PORTB, 6},  
  {&DDRB, &PINB, &PORTB, 7},  
  {&DDRD, &PIND, &PORTD, 0},  
  {&DDRD, &PIND, &PORTD, 1},  
  {&DDRD, &PIND, &PORTD, 2},  
  {&DDRD, &PIND, &PORTD, 3},  
  {&DDRD, &PIND, &PORTD, 4},  
  {&DDRD, &PIND, &PORTD, 5},  
  {&DDRD, &PIND, &PORTD, 6},  
  {&DDRD, &PIND, &PORTD, 7},  
  {&DDRC, &PINC, &PORTC, 0},  
  {&DDRC, &PINC, &PORTC, 1},  
  {&DDRC, &PINC, &PORTC, 2},  
  {&DDRC, &PINC, &PORTC, 3},  
  {&DDRC, &PINC, &PORTC, 4},  
  {&DDRC, &PINC, &PORTC, 5},  
  {&DDRC, &PINC, &PORTC, 6},  
  {&DDRC, &PINC, &PORTC, 7},  
  {&DDRA, &PINA, &PORTA, 7},  
  {&DDRA, &PINA, &PORTA, 6},  
  {&DDRA, &PINA, &PORTA, 5},  
  {&DDRA, &PINA, &PORTA, 4},  
  {&DDRA, &PINA, &PORTA, 3},  
  {&DDRA, &PINA, &PORTA, 2},  
  {&DDRA, &PINA, &PORTA, 1},  
  {&DDRA, &PINA, &PORTA, 0}   
};
#else  
#error Undefined variant 1284, 644, 324, 64, 32
#endif  
#elif defined(__AVR_ATmega32U4__)
#ifdef CORE_TEENSY
static const pin_map_t pinMap[] = {
  {&DDRB, &PINB, &PORTB, 0},  
  {&DDRB, &PINB, &PORTB, 1},  
  {&DDRB, &PINB, &PORTB, 2},  
  {&DDRB, &PINB, &PORTB, 3},  
  {&DDRB, &PINB, &PORTB, 7},  
  {&DDRD, &PIND, &PORTD, 0},  
  {&DDRD, &PIND, &PORTD, 1},  
  {&DDRD, &PIND, &PORTD, 2},  
  {&DDRD, &PIND, &PORTD, 3},  
  {&DDRC, &PINC, &PORTC, 6},  
  {&DDRC, &PINC, &PORTC, 7},  
  {&DDRD, &PIND, &PORTD, 6},  
  {&DDRD, &PIND, &PORTD, 7},  
  {&DDRB, &PINB, &PORTB, 4},  
  {&DDRB, &PINB, &PORTB, 5},  
  {&DDRB, &PINB, &PORTB, 6},  
  {&DDRF, &PINF, &PORTF, 7},  
  {&DDRF, &PINF, &PORTF, 6},  
  {&DDRF, &PINF, &PORTF, 5},  
  {&DDRF, &PINF, &PORTF, 4},  
  {&DDRF, &PINF, &PORTF, 1},  
  {&DDRF, &PINF, &PORTF, 0},  
  {&DDRD, &PIND, &PORTD, 4},  
  {&DDRD, &PIND, &PORTD, 5},  
  {&DDRE, &PINE, &PORTE, 6}   
};
#else  
static const pin_map_t pinMap[] = {
  {&DDRD, &PIND, &PORTD, 2},  
  {&DDRD, &PIND, &PORTD, 3},  
  {&DDRD, &PIND, &PORTD, 1},  
  {&DDRD, &PIND, &PORTD, 0},  
  {&DDRD, &PIND, &PORTD, 4},  
  {&DDRC, &PINC, &PORTC, 6},  
  {&DDRD, &PIND, &PORTD, 7},  
  {&DDRE, &PINE, &PORTE, 6},  
  {&DDRB, &PINB, &PORTB, 4},  
  {&DDRB, &PINB, &PORTB, 5},  
  {&DDRB, &PINB, &PORTB, 6},  
  {&DDRB, &PINB, &PORTB, 7},  
  {&DDRD, &PIND, &PORTD, 6},  
  {&DDRC, &PINC, &PORTC, 7},  
  {&DDRB, &PINB, &PORTB, 3},  
  {&DDRB, &PINB, &PORTB, 1},  
  {&DDRB, &PINB, &PORTB, 2},  
  {&DDRB, &PINB, &PORTB, 0},  
  {&DDRF, &PINF, &PORTF, 7},  
  {&DDRF, &PINF, &PORTF, 6},  
  {&DDRF, &PINF, &PORTF, 5},  
  {&DDRF, &PINF, &PORTF, 4},  
  {&DDRF, &PINF, &PORTF, 1},  
  {&DDRF, &PINF, &PORTF, 0},  
  {&DDRD, &PIND, &PORTD, 4},  
  {&DDRD, &PIND, &PORTD, 7},  
  {&DDRB, &PINB, &PORTB, 4},  
  {&DDRB, &PINB, &PORTB, 5},  
  {&DDRB, &PINB, &PORTB, 6},  
  {&DDRD, &PIND, &PORTD, 6}   
};
#endif  
#elif defined(__AVR_AT90USB646__)|| defined(__AVR_AT90USB1286__)
static const pin_map_t pinMap[] = {
  {&DDRD, &PIND, &PORTD, 0},  
  {&DDRD, &PIND, &PORTD, 1},  
  {&DDRD, &PIND, &PORTD, 2},  
  {&DDRD, &PIND, &PORTD, 3},  
  {&DDRD, &PIND, &PORTD, 4},  
  {&DDRD, &PIND, &PORTD, 5},  
  {&DDRD, &PIND, &PORTD, 6},  
  {&DDRD, &PIND, &PORTD, 7},  
  {&DDRE, &PINE, &PORTE, 0},  
  {&DDRE, &PINE, &PORTE, 1},  
  {&DDRC, &PINC, &PORTC, 0},  
  {&DDRC, &PINC, &PORTC, 1},  
  {&DDRC, &PINC, &PORTC, 2},  
  {&DDRC, &PINC, &PORTC, 3},  
  {&DDRC, &PINC, &PORTC, 4},  
  {&DDRC, &PINC, &PORTC, 5},  
  {&DDRC, &PINC, &PORTC, 6},  
  {&DDRC, &PINC, &PORTC, 7},  
  {&DDRE, &PINE, &PORTE, 6},  
  {&DDRE, &PINE, &PORTE, 7},  
  {&DDRB, &PINB, &PORTB, 0},  
  {&DDRB, &PINB, &PORTB, 1},  
  {&DDRB, &PINB, &PORTB, 2},  
  {&DDRB, &PINB, &PORTB, 3},  
  {&DDRB, &PINB, &PORTB, 4},  
  {&DDRB, &PINB, &PORTB, 5},  
  {&DDRB, &PINB, &PORTB, 6},  
  {&DDRB, &PINB, &PORTB, 7},  
  {&DDRA, &PINA, &PORTA, 0},  
  {&DDRA, &PINA, &PORTA, 1},  
  {&DDRA, &PINA, &PORTA, 2},  
  {&DDRA, &PINA, &PORTA, 3},  
  {&DDRA, &PINA, &PORTA, 4},  
  {&DDRA, &PINA, &PORTA, 5},  
  {&DDRA, &PINA, &PORTA, 6},  
  {&DDRA, &PINA, &PORTA, 7},  
  {&DDRE, &PINE, &PORTE, 4},  
  {&DDRE, &PINE, &PORTE, 5},  
  {&DDRF, &PINF, &PORTF, 0},  
  {&DDRF, &PINF, &PORTF, 1},  
  {&DDRF, &PINF, &PORTF, 2},  
  {&DDRF, &PINF, &PORTF, 3},  
  {&DDRF, &PINF, &PORTF, 4},  
  {&DDRF, &PINF, &PORTF, 5},  
  {&DDRF, &PINF, &PORTF, 6},  
  {&DDRF, &PINF, &PORTF, 7}   
};
#else  
#error unknown CPU type
#endif  
static const uint8_t digitalPinCount = sizeof(pinMap)/sizeof(pin_map_t);
void badPinNumber(void)
  __attribute__((error("Pin number is too large or not a constant")));
static inline __attribute__((always_inline))
void badPinCheck(uint8_t pin) {
  if (!__builtin_constant_p(pin) || pin >= digitalPinCount) {
     badPinNumber();
  }
}
static inline __attribute__((always_inline))
void fastBitWriteSafe(volatile uint8_t* address, uint8_t bit, bool level) {
  uint8_t oldSREG;
  if (address > (uint8_t*)0X5F) {
    oldSREG = SREG;
    cli();
  }
  if (level) {
    *address |= 1 << bit;
  } else {
    *address &= ~(1 << bit);
  }
  if (address > (uint8_t*)0X5F) {
    SREG = oldSREG;
  }
}
static inline __attribute__((always_inline))
bool fastDigitalRead(uint8_t pin) {
  badPinCheck(pin);
  return (*pinMap[pin].pin >> pinMap[pin].bit) & 1;
}
static inline __attribute__((always_inline))
void fastDigitalToggle(uint8_t pin) {
  badPinCheck(pin);
    if (pinMap[pin].pin > (uint8_t*)0X5F) {
      *pinMap[pin].pin = 1 << pinMap[pin].bit;
    } else {
      *pinMap[pin].pin |= 1 << pinMap[pin].bit;
    }
}
static inline __attribute__((always_inline))
void fastDigitalWrite(uint8_t pin, bool level) {
  badPinCheck(pin);
  fastBitWriteSafe(pinMap[pin].port, pinMap[pin].bit, level);
}
static inline __attribute__((always_inline))
void fastPinMode(uint8_t pin, bool mode) {
  badPinCheck(pin);
  fastBitWriteSafe(pinMap[pin].ddr, pinMap[pin].bit, mode);
}
#endif  
static inline __attribute__((always_inline))
void fastPinConfig(uint8_t pin, bool mode, bool level) {
  fastPinMode(pin, mode);
  fastDigitalWrite(pin, level);
}
template<uint8_t PinNumber>
class DigitalPin {
 public:
  DigitalPin() {}
  explicit DigitalPin(bool pinMode) {
    mode(pinMode);
  }
  DigitalPin(bool mode, bool level) {
    config(mode, level);
  }
  inline DigitalPin & operator = (bool value) __attribute__((always_inline)) {
    write(value);
    return *this;
  }
  inline operator bool () const __attribute__((always_inline)) {
    return read();
  }
  inline __attribute__((always_inline))
  void config(bool mode, bool level) {
    fastPinConfig(PinNumber, mode, level);
  }
  inline __attribute__((always_inline))
  void high() {write(true);}
  inline __attribute__((always_inline))
  void low() {write(false);}
  inline __attribute__((always_inline))
  void mode(bool pinMode) {
    fastPinMode(PinNumber, pinMode);
  }
  inline __attribute__((always_inline))
  bool read() const {
    return fastDigitalRead(PinNumber);
  }
  inline __attribute__((always_inline))
  void toggle() {
    fastDigitalToggle(PinNumber);
  }
  inline __attribute__((always_inline))
  void write(bool value) {
    fastDigitalWrite(PinNumber, value);
  }
};
#define nop asm volatile ("nop\n\t")
const bool MISO_MODE  = false;
const bool MISO_LEVEL = false;
const bool MOSI_MODE  = true;
const bool SCK_MODE   = true;
template<uint8_t MisoPin, uint8_t MosiPin, uint8_t SckPin, uint8_t Mode = 0>
class SoftSPI {
 public:
  void begin() {
    fastPinConfig(MisoPin, MISO_MODE, MISO_LEVEL);
    fastPinConfig(MosiPin, MOSI_MODE, !MODE_CPHA(Mode));
    fastPinConfig(SckPin, SCK_MODE, MODE_CPOL(Mode));
  }
  inline __attribute__((always_inline))
  uint8_t receive() {
    uint8_t data = 0;
    receiveBit(7, &data);
    receiveBit(6, &data);
    receiveBit(5, &data);
    receiveBit(4, &data);
    receiveBit(3, &data);
    receiveBit(2, &data);
    receiveBit(1, &data);
    receiveBit(0, &data);
    return data;
  }
  inline __attribute__((always_inline))
  void send(uint8_t data) {
    sendBit(7, data);
    sendBit(6, data);
    sendBit(5, data);
    sendBit(4, data);
    sendBit(3, data);
    sendBit(2, data);
    sendBit(1, data);
    sendBit(0, data);
  }
  inline __attribute__((always_inline))
  uint8_t transfer(uint8_t txData) {
    uint8_t rxData = 0;
    transferBit(7, &rxData, txData);
    transferBit(6, &rxData, txData);
    transferBit(5, &rxData, txData);
    transferBit(4, &rxData, txData);
    transferBit(3, &rxData, txData);
    transferBit(2, &rxData, txData);
    transferBit(1, &rxData, txData);
    transferBit(0, &rxData, txData);
    return rxData;
  }
 private:
  inline __attribute__((always_inline))
  bool MODE_CPHA(uint8_t mode) {return (mode & 1) != 0;}
  inline __attribute__((always_inline))
  bool MODE_CPOL(uint8_t mode) {return (mode & 2) != 0;}
  inline __attribute__((always_inline))
  void receiveBit(uint8_t bit, uint8_t* data) {
    if (MODE_CPHA(Mode)) {
      fastDigitalWrite(SckPin, !MODE_CPOL(Mode));
    }
    nop;
    nop;
    fastDigitalWrite(SckPin,
      MODE_CPHA(Mode) ? MODE_CPOL(Mode) : !MODE_CPOL(Mode));
    if (fastDigitalRead(MisoPin)) *data |= 1 << bit;
    if (!MODE_CPHA(Mode)) {
      fastDigitalWrite(SckPin, MODE_CPOL(Mode));
    }
  }
  inline __attribute__((always_inline))
  void sendBit(uint8_t bit, uint8_t data) {
    if (MODE_CPHA(Mode)) {
      fastDigitalWrite(SckPin, !MODE_CPOL(Mode));
    }
    fastDigitalWrite(MosiPin, data & (1 << bit));
    fastDigitalWrite(SckPin,
      MODE_CPHA(Mode) ? MODE_CPOL(Mode) : !MODE_CPOL(Mode));
    nop;
    nop;
    if (!MODE_CPHA(Mode)) {
      fastDigitalWrite(SckPin, MODE_CPOL(Mode));
    }
  }
  inline __attribute__((always_inline))
  void transferBit(uint8_t bit, uint8_t* rxData, uint8_t txData) {
    if (MODE_CPHA(Mode)) {
      fastDigitalWrite(SckPin, !MODE_CPOL(Mode));
    }
    fastDigitalWrite(MosiPin, txData & (1 << bit));
    fastDigitalWrite(SckPin,
      MODE_CPHA(Mode) ? MODE_CPOL(Mode) : !MODE_CPOL(Mode));
    if (fastDigitalRead(MisoPin)) *rxData |= 1 << bit;
    if (!MODE_CPHA(Mode)) {
      fastDigitalWrite(SckPin, MODE_CPOL(Mode));
    }
  }
};
