#ifndef servo_h
#define servo_h
#include <inttypes.h>
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define _useTimer3
  #define _useTimer4
  #if !HAS_MOTOR_CURRENT_PWM
    #define _useTimer5 
  #endif
#elif defined(__AVR_ATmega32U4__)
  #define _useTimer3
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
  #define _useTimer3
#elif defined(__AVR_ATmega128__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega2561__)
  #define _useTimer3
#else
#endif
typedef enum {
  #if ENABLED(_useTimer1)
    _timer1,
  #endif
  #if ENABLED(_useTimer3)
    _timer3,
  #endif
  #if ENABLED(_useTimer4)
    _timer4,
  #endif
  #if ENABLED(_useTimer5)
    _timer5,
  #endif
  _Nbr_16timers
} timer16_Sequence_t;
#define Servo_VERSION           2     
#define MIN_PULSE_WIDTH       544     
#define MAX_PULSE_WIDTH      2400     
#define DEFAULT_PULSE_WIDTH  1500     
#define REFRESH_INTERVAL    20000     
#define SERVOS_PER_TIMER       12     
#define MAX_SERVOS   (_Nbr_16timers  * SERVOS_PER_TIMER)
#define INVALID_SERVO         255     
typedef struct {
  uint8_t nbr        : 6 ;            
  uint8_t isActive   : 1 ;            
} ServoPin_t;
typedef struct {
  ServoPin_t Pin;
  unsigned int ticks;
} ServoInfo_t;
class Servo {
  public:
    Servo();
    int8_t attach(int pin);            
    int8_t attach(int pin, int min, int max); 
    void detach();
    void write(int value);             
    void writeMicroseconds(int value); 
    void move(int value);              
    int read();                        
    int readMicroseconds();            
    bool attached();                   
  private:
    uint8_t servoIndex;               
    int8_t min;                       
    int8_t max;                       
};
#endif
