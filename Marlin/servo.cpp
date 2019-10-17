#include "MarlinConfig.h"
#if HAS_SERVOS
#include <avr/interrupt.h>
#include <Arduino.h>
#include "servo.h"
#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) / 8)     
#define ticksToUs(_ticks) (( (unsigned)_ticks * 8)/ clockCyclesPerMicrosecond() ) 
#define TRIM_DURATION       2                               
static ServoInfo_t servo_info[MAX_SERVOS];                  
static volatile int8_t Channel[_Nbr_16timers ];             
uint8_t ServoCount = 0;                                     
#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / (SERVOS_PER_TIMER))) 
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % (SERVOS_PER_TIMER))       
#define SERVO_INDEX(_timer,_channel)  ((_timer*(SERVOS_PER_TIMER)) + _channel)     
#define SERVO(_timer,_channel)  (servo_info[SERVO_INDEX(_timer,_channel)])       
#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)  
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)  
static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t* TCNTn, volatile uint16_t* OCRnA) {
  if (Channel[timer] < 0)
    *TCNTn = 0; 
  else {
    if (SERVO_INDEX(timer, Channel[timer]) < ServoCount && SERVO(timer, Channel[timer]).Pin.isActive)
      digitalWrite(SERVO(timer, Channel[timer]).Pin.nbr, LOW); 
  }
  Channel[timer]++;    
  if (SERVO_INDEX(timer, Channel[timer]) < ServoCount && Channel[timer] < SERVOS_PER_TIMER) {
    *OCRnA = *TCNTn + SERVO(timer, Channel[timer]).ticks;
    if (SERVO(timer, Channel[timer]).Pin.isActive)    
      digitalWrite(SERVO(timer, Channel[timer]).Pin.nbr, HIGH); 
  }
  else {
    if (((unsigned)*TCNTn) + 4 < usToTicks(REFRESH_INTERVAL))    
      *OCRnA = (unsigned int)usToTicks(REFRESH_INTERVAL);
    else
      *OCRnA = *TCNTn + 4;  
    Channel[timer] = -1; 
  }
}
#ifndef WIRING 
  #if ENABLED(_useTimer1)
    SIGNAL (TIMER1_COMPA_vect) { handle_interrupts(_timer1, &TCNT1, &OCR1A); }
  #endif
  #if ENABLED(_useTimer3)
    SIGNAL (TIMER3_COMPA_vect) { handle_interrupts(_timer3, &TCNT3, &OCR3A); }
  #endif
  #if ENABLED(_useTimer4)
    SIGNAL (TIMER4_COMPA_vect) { handle_interrupts(_timer4, &TCNT4, &OCR4A); }
  #endif
  #if ENABLED(_useTimer5)
    SIGNAL (TIMER5_COMPA_vect) { handle_interrupts(_timer5, &TCNT5, &OCR5A); }
  #endif
#else 
  #if ENABLED(_useTimer1)
    void Timer1Service() { handle_interrupts(_timer1, &TCNT1, &OCR1A); }
  #endif
  #if ENABLED(_useTimer3)
    void Timer3Service() { handle_interrupts(_timer3, &TCNT3, &OCR3A); }
  #endif
#endif 
static void initISR(timer16_Sequence_t timer) {
  #if ENABLED(_useTimer1)
    if (timer == _timer1) {
      TCCR1A = 0;             
      TCCR1B = _BV(CS11);     
      TCNT1 = 0;              
      #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
        SBI(TIFR, OCF1A);      
        SBI(TIMSK, OCIE1A);    
      #else
        SBI(TIFR1, OCF1A);     
        SBI(TIMSK1, OCIE1A);   
      #endif
      #ifdef WIRING
        timerAttach(TIMER1OUTCOMPAREA_INT, Timer1Service);
      #endif
    }
  #endif
  #if ENABLED(_useTimer3)
    if (timer == _timer3) {
      TCCR3A = 0;             
      TCCR3B = _BV(CS31);     
      TCNT3 = 0;              
      #ifdef __AVR_ATmega128__
        SBI(TIFR, OCF3A);     
        SBI(ETIMSK, OCIE3A);  
      #else
        SBI(TIFR3, OCF3A);   
        SBI(TIMSK3, OCIE3A); 
      #endif
      #ifdef WIRING
        timerAttach(TIMER3OUTCOMPAREA_INT, Timer3Service);  
      #endif
    }
  #endif
  #if ENABLED(_useTimer4)
    if (timer == _timer4) {
      TCCR4A = 0;             
      TCCR4B = _BV(CS41);     
      TCNT4 = 0;              
      TIFR4 = _BV(OCF4A);     
      TIMSK4 = _BV(OCIE4A);   
    }
  #endif
  #if ENABLED(_useTimer5)
    if (timer == _timer5) {
      TCCR5A = 0;             
      TCCR5B = _BV(CS51);     
      TCNT5 = 0;              
      TIFR5 = _BV(OCF5A);     
      TIMSK5 = _BV(OCIE5A);   
    }
  #endif
}
static void finISR(timer16_Sequence_t timer) {
  #ifdef WIRING
    if (timer == _timer1) {
      CBI(
        #if defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)
          TIMSK1
        #else
          TIMSK
        #endif
          , OCIE1A);    
      timerDetach(TIMER1OUTCOMPAREA_INT);
    }
    else if (timer == _timer3) {
      CBI(
        #if defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)
          TIMSK3
        #else
          ETIMSK
        #endif
          , OCIE3A);    
      timerDetach(TIMER3OUTCOMPAREA_INT);
    }
  #else 
    UNUSED(timer);
  #endif
}
static bool isTimerActive(timer16_Sequence_t timer) {
  for (uint8_t channel = 0; channel < SERVOS_PER_TIMER; channel++) {
    if (SERVO(timer, channel).Pin.isActive)
      return true;
  }
  return false;
}
Servo::Servo() {
  if (ServoCount < MAX_SERVOS) {
    this->servoIndex = ServoCount++;                    
    servo_info[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   
  }
  else
    this->servoIndex = INVALID_SERVO;  
}
int8_t Servo::attach(int pin) {
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}
int8_t Servo::attach(int pin, int min, int max) {
  if (this->servoIndex >= MAX_SERVOS) return -1;
  if (pin > 0) servo_info[this->servoIndex].Pin.nbr = pin;
  pinMode(servo_info[this->servoIndex].Pin.nbr, OUTPUT); 
  this->min = (MIN_PULSE_WIDTH - min) / 4; 
  this->max = (MAX_PULSE_WIDTH - max) / 4;
  timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
  if (!isTimerActive(timer)) initISR(timer);
  servo_info[this->servoIndex].Pin.isActive = true;  
  return this->servoIndex;
}
void Servo::detach() {
  servo_info[this->servoIndex].Pin.isActive = false;
  timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
  if (!isTimerActive(timer)) finISR(timer);
}
void Servo::write(int value) {
  if (value < MIN_PULSE_WIDTH) { 
    value = map(constrain(value, 0, 180), 0, 180, SERVO_MIN(), SERVO_MAX());
  }
  this->writeMicroseconds(value);
}
void Servo::writeMicroseconds(int value) {
  byte channel = this->servoIndex;
  if (channel < MAX_SERVOS) {  
    value = constrain(value, SERVO_MIN(), SERVO_MAX()) - (TRIM_DURATION);
    value = usToTicks(value);  
    CRITICAL_SECTION_START;
    servo_info[channel].ticks = value;
    CRITICAL_SECTION_END;
  }
}
int Servo::read() { return map(this->readMicroseconds() + 1, SERVO_MIN(), SERVO_MAX(), 0, 180); }
int Servo::readMicroseconds() {
  return (this->servoIndex == INVALID_SERVO) ? 0 : ticksToUs(servo_info[this->servoIndex].ticks) + TRIM_DURATION;
}
bool Servo::attached() { return servo_info[this->servoIndex].Pin.isActive; }
void Servo::move(int value) {
  if (this->attach(0) >= 0) {
    this->write(value);
    delay(SERVO_DELAY);
    #if ENABLED(DEACTIVATE_SERVOS_AFTER_MOVE)
      this->detach();
    #endif
  }
}
#endif
