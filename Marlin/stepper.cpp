#include "Marlin.h"
#include "stepper.h"
#include "endstops.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"
#include "cardreader.h"
#include "speed_lookuptable.h"
#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif
Stepper stepper; 
block_t* Stepper::current_block = NULL;  
#if ENABLED(FYS_POWERBREAK_STEPPER_STATUS)
volatile uint8_t Stepper::powerBreakStatus;
#endif
#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  bool Stepper::abort_on_endstop_hit = false;
#endif
#if ENABLED(Z_DUAL_ENDSTOPS)
  bool Stepper::performing_homing = false;
#endif
#if HAS_MOTOR_CURRENT_PWM
  uint32_t Stepper::motor_current_setting[3] = PWM_MOTOR_CURRENT;
#endif
uint8_t Stepper::last_direction_bits = 0;        
uint16_t Stepper::cleaning_buffer_counter = 0;
#if ENABLED(Z_DUAL_ENDSTOPS)
  bool Stepper::locked_z_motor = false;
  bool Stepper::locked_z2_motor = false;
#endif
long Stepper::counter_X = 0,
     Stepper::counter_Y = 0,
     Stepper::counter_Z = 0,
     Stepper::counter_E = 0;
volatile uint32_t Stepper::step_events_completed = 0; 
#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  constexpr uint16_t ADV_NEVER = 65535;
  uint16_t Stepper::nextMainISR = 0,
           Stepper::nextAdvanceISR = ADV_NEVER,
           Stepper::eISR_Rate = ADV_NEVER;
  #if ENABLED(LIN_ADVANCE)
    volatile int Stepper::e_steps[E_STEPPERS];
    int Stepper::final_estep_rate,
        Stepper::current_estep_rate[E_STEPPERS],
        Stepper::current_adv_steps[E_STEPPERS];
  #else
    long Stepper::e_steps[E_STEPPERS],
         Stepper::final_advance = 0,
         Stepper::old_advance = 0,
         Stepper::advance_rate,
         Stepper::advance;
  #endif
  FORCE_INLINE uint16_t adv_rate(const int steps, const uint16_t timer, const uint8_t loops) {
    if (steps) {
      const uint16_t rate = (timer * loops) / abs(steps);
      return rate ? rate : 1;
    }
    return ADV_NEVER;
  }
#endif 
long Stepper::acceleration_time, Stepper::deceleration_time;
volatile long Stepper::count_position[NUM_AXIS] = { 0 };
volatile signed char Stepper::count_direction[NUM_AXIS] = { 1, 1, 1, 1 };
#if ENABLED(MIXING_EXTRUDER)
  long Stepper::counter_m[MIXING_STEPPERS];
#endif
unsigned short Stepper::acc_step_rate; 
uint8_t Stepper::step_loops, Stepper::step_loops_nominal;
unsigned short Stepper::OCR1A_nominal;
volatile long Stepper::endstops_trigsteps[XYZ];
#if ENABLED(X_DUAL_STEPPER_DRIVERS)
  #define X_APPLY_DIR(v,Q) do{ X_DIR_WRITE(v); X2_DIR_WRITE((v) != INVERT_X2_VS_X_DIR); }while(0)
  #define X_APPLY_STEP(v,Q) do{ X_STEP_WRITE(v); X2_STEP_WRITE(v); }while(0)
#elif ENABLED(DUAL_X_CARRIAGE)
  #define X_APPLY_DIR(v,ALWAYS) \
    if (extruder_duplication_enabled || ALWAYS) { \
      X_DIR_WRITE(v); \
      X2_DIR_WRITE(v); \
    } \
    else { \
      if (current_block->active_extruder) X2_DIR_WRITE(v); else X_DIR_WRITE(v); \
    }
  #define X_APPLY_STEP(v,ALWAYS) \
    if (extruder_duplication_enabled || ALWAYS) { \
      X_STEP_WRITE(v); \
      X2_STEP_WRITE(v); \
    } \
    else { \
      if (current_block->active_extruder) X2_STEP_WRITE(v); else X_STEP_WRITE(v); \
    }
#else
  #define X_APPLY_DIR(v,Q) X_DIR_WRITE(v)
  #define X_APPLY_STEP(v,Q) X_STEP_WRITE(v)
#endif
#if ENABLED(Y_DUAL_STEPPER_DRIVERS)
  #define Y_APPLY_DIR(v,Q) do{ Y_DIR_WRITE(v); Y2_DIR_WRITE((v) != INVERT_Y2_VS_Y_DIR); }while(0)
  #define Y_APPLY_STEP(v,Q) do{ Y_STEP_WRITE(v); Y2_STEP_WRITE(v); }while(0)
#else
  #define Y_APPLY_DIR(v,Q) Y_DIR_WRITE(v)
  #define Y_APPLY_STEP(v,Q) Y_STEP_WRITE(v)
#endif
#if ENABLED(Z_DUAL_STEPPER_DRIVERS)
  #define Z_APPLY_DIR(v,Q) do{ Z_DIR_WRITE(v); Z2_DIR_WRITE(v); }while(0)
  #if ENABLED(Z_DUAL_ENDSTOPS)
    #define Z_APPLY_STEP(v,Q) \
    if (performing_homing) { \
      if (Z_HOME_DIR < 0) { \
        if (!(TEST(endstops.old_endstop_bits, Z_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z2_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
      } \
      else { \
        if (!(TEST(endstops.old_endstop_bits, Z_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z2_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
      } \
    } \
    else { \
      Z_STEP_WRITE(v); \
      Z2_STEP_WRITE(v); \
    }
  #else
    #define Z_APPLY_STEP(v,Q) do{ Z_STEP_WRITE(v); Z2_STEP_WRITE(v); }while(0)
  #endif
#else
  #define Z_APPLY_DIR(v,Q) Z_DIR_WRITE(v)
  #define Z_APPLY_STEP(v,Q) Z_STEP_WRITE(v)
#endif
#if DISABLED(MIXING_EXTRUDER)
  #define E_APPLY_STEP(v,Q) E_STEP_WRITE(v)
#endif
#define MultiU24X32toH16(intRes, longIn1, longIn2) \
  asm volatile ( \
                 "clr r26 \n\t" \
                 "mul %A1, %B2 \n\t" \
                 "mov r27, r1 \n\t" \
                 "mul %B1, %C2 \n\t" \
                 "movw %A0, r0 \n\t" \
                 "mul %C1, %C2 \n\t" \
                 "add %B0, r0 \n\t" \
                 "mul %C1, %B2 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %A1, %C2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %B2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %C1, %A2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %A2 \n\t" \
                 "add r27, r1 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "lsr r27 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %D2, %A1 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %D2, %B1 \n\t" \
                 "add %B0, r0 \n\t" \
                 "clr r1 \n\t" \
                 : \
                 "=&r" (intRes) \
                 : \
                 "d" (longIn1), \
                 "d" (longIn2) \
                 : \
                 "r26" , "r27" \
               )
void Stepper::wake_up() {
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}
void Stepper::set_directions() {
  #define SET_STEP_DIR(AXIS) \
    if (motor_direction(AXIS ##_AXIS)) { \
      AXIS ##_APPLY_DIR(INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = -1; \
    } \
    else { \
      AXIS ##_APPLY_DIR(!INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = 1; \
    }
  #if HAS_X_DIR
    SET_STEP_DIR(X); 
  #endif
  #if HAS_Y_DIR
    SET_STEP_DIR(Y); 
  #endif
  #if HAS_Z_DIR
    SET_STEP_DIR(Z); 
  #endif
  #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
    if (motor_direction(E_AXIS)) {
      REV_E_DIR();
      count_direction[E_AXIS] = -1;
    }
    else {
      NORM_E_DIR();
      count_direction[E_AXIS] = 1;
    }
  #endif 
}
#if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
  extern volatile uint8_t e_hit;
#endif
ISR(TIMER1_COMPA_vect) {
  #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
    Stepper::advance_isr_scheduler();
  #else
    Stepper::isr();
  #endif
}
#define _ENABLE_ISRs() do { cli(); if (thermalManager.in_temp_isr) CBI(TIMSK0, OCIE0B); else SBI(TIMSK0, OCIE0B); ENABLE_STEPPER_DRIVER_INTERRUPT(); } while(0)
void Stepper::isr() {
  #if ENABLED(FYS_POWERBREAK_STEPPER_STATUS)  
    powerStepCheck();
  #endif
  uint16_t ocr_val;
  #define ENDSTOP_NOMINAL_OCR_VAL 3000    
  #define OCR_VAL_TOLERANCE 1000          
  #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
    CBI(TIMSK0, OCIE0B); 
    DISABLE_STEPPER_DRIVER_INTERRUPT();
    sei();
  #endif
  #define _SPLIT(L) (ocr_val = (uint16_t)L)
  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    #define SPLIT(L) _SPLIT(L)
  #else                 
    static uint32_t step_remaining = 0;
    #define SPLIT(L) do { \
      _SPLIT(L); \
      if (ENDSTOPS_ENABLED && L > ENDSTOP_NOMINAL_OCR_VAL) { \
        const uint16_t remainder = (uint16_t)L % (ENDSTOP_NOMINAL_OCR_VAL); \
        ocr_val = (remainder < OCR_VAL_TOLERANCE) ? ENDSTOP_NOMINAL_OCR_VAL + remainder : ENDSTOP_NOMINAL_OCR_VAL; \
        step_remaining = (uint16_t)L - ocr_val; \
      } \
    }while(0)
    if (step_remaining && ENDSTOPS_ENABLED) {   
      endstops.update();
      if (step_remaining > ENDSTOP_NOMINAL_OCR_VAL) {
        step_remaining -= ENDSTOP_NOMINAL_OCR_VAL;
        ocr_val = ENDSTOP_NOMINAL_OCR_VAL;
      }
      else {
        ocr_val = step_remaining;
        step_remaining = 0;  
      }
      _NEXT_ISR(ocr_val);
      NOLESS(OCR1A, TCNT1 + 16);
      _ENABLE_ISRs(); 
      return;
    }
  #endif
  if (cleaning_buffer_counter) {
    --cleaning_buffer_counter;
    current_block = NULL;
    planner.discard_current_block();
    #ifdef SD_FINISHED_RELEASECOMMAND
      if (!cleaning_buffer_counter && (SD_FINISHED_STEPPERRELEASE)) enqueue_and_echo_commands_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    #endif
    _NEXT_ISR(200); 
    _ENABLE_ISRs(); 
    return;
  }
  if (!current_block) {
    current_block = planner.get_current_block();
    if (current_block) {
      trapezoid_generator_reset();
      counter_X = counter_Y = counter_Z = counter_E = -(current_block->step_event_count >> 1);
      #if ENABLED(MIXING_EXTRUDER)
        MIXING_STEPPERS_LOOP(i)
          counter_m[i] = -(current_block->mix_event_count[i] >> 1);
      #endif
      step_events_completed = 0;
      #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
        e_hit = 2; 
      #endif
      #if ENABLED(Z_LATE_ENABLE)
        if (current_block->steps[Z_AXIS] > 0) {
          enable_Z();
          _NEXT_ISR(2000); 
          _ENABLE_ISRs(); 
          return;
        }
      #endif
    }
    else {
      _NEXT_ISR(2000); 
      _ENABLE_ISRs(); 
      return;
    }
  }
  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    if (e_hit && ENDSTOPS_ENABLED) {
      endstops.update();
      e_hit--;
    }
  #else
    if (ENDSTOPS_ENABLED) endstops.update();
  #endif
  bool all_steps_done = false;
  for (uint8_t i = step_loops; i--;) {
    #if ENABLED(LIN_ADVANCE)
      counter_E += current_block->steps[E_AXIS];
      if (counter_E > 0) {
        counter_E -= current_block->step_event_count;
        #if DISABLED(MIXING_EXTRUDER)
          count_position[E_AXIS] += count_direction[E_AXIS];
          motor_direction(E_AXIS) ? --e_steps[TOOL_E_INDEX] : ++e_steps[TOOL_E_INDEX];
        #endif
      }
      #if ENABLED(MIXING_EXTRUDER)
        const bool dir = motor_direction(E_AXIS);
        MIXING_STEPPERS_LOOP(j) {
          counter_m[j] += current_block->steps[E_AXIS];
          if (counter_m[j] > 0) {
            counter_m[j] -= current_block->mix_event_count[j];
            dir ? --e_steps[j] : ++e_steps[j];
          }
        }
      #endif
    #elif ENABLED(ADVANCE)
      counter_E += current_block->steps[E_AXIS];
      if (counter_E > 0) {
        counter_E -= current_block->step_event_count;
        #if DISABLED(MIXING_EXTRUDER)
          motor_direction(E_AXIS) ? --e_steps[TOOL_E_INDEX] : ++e_steps[TOOL_E_INDEX];
        #endif
      }
      #if ENABLED(MIXING_EXTRUDER)
        const bool dir = motor_direction(E_AXIS);
        MIXING_STEPPERS_LOOP(j) {
          counter_m[j] += current_block->steps[E_AXIS];
          if (counter_m[j] > 0) {
            counter_m[j] -= current_block->mix_event_count[j];
            dir ? --e_steps[j] : ++e_steps[j];
          }
        }
      #endif 
    #endif 
    #define _COUNTER(AXIS) counter_## AXIS
    #define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
    #define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN
    #define PULSE_START(AXIS) \
      _COUNTER(AXIS) += current_block->steps[_AXIS(AXIS)]; \
      if (_COUNTER(AXIS) > 0) { _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS),0); }
    #define PULSE_STOP(AXIS) \
      if (_COUNTER(AXIS) > 0) { \
        _COUNTER(AXIS) -= current_block->step_event_count; \
        count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
        _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS),0); \
      }
    #if HAS_X_STEP
      #define _CYCLE_APPROX_1 5
    #else
      #define _CYCLE_APPROX_1 0
    #endif
    #if ENABLED(X_DUAL_STEPPER_DRIVERS)
      #define _CYCLE_APPROX_2 _CYCLE_APPROX_1 + 4
    #else
      #define _CYCLE_APPROX_2 _CYCLE_APPROX_1
    #endif
    #if HAS_Y_STEP
      #define _CYCLE_APPROX_3 _CYCLE_APPROX_2 + 5
    #else
      #define _CYCLE_APPROX_3 _CYCLE_APPROX_2
    #endif
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS)
      #define _CYCLE_APPROX_4 _CYCLE_APPROX_3 + 4
    #else
      #define _CYCLE_APPROX_4 _CYCLE_APPROX_3
    #endif
    #if HAS_Z_STEP
      #define _CYCLE_APPROX_5 _CYCLE_APPROX_4 + 5
    #else
      #define _CYCLE_APPROX_5 _CYCLE_APPROX_4
    #endif
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS)
      #define _CYCLE_APPROX_6 _CYCLE_APPROX_5 + 4
    #else
      #define _CYCLE_APPROX_6 _CYCLE_APPROX_5
    #endif
    #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
      #if ENABLED(MIXING_EXTRUDER)
        #define _CYCLE_APPROX_7 _CYCLE_APPROX_6 + (MIXING_STEPPERS) * 6
      #else
        #define _CYCLE_APPROX_7 _CYCLE_APPROX_6 + 5
      #endif
    #else
      #define _CYCLE_APPROX_7 _CYCLE_APPROX_6
    #endif
    #define CYCLES_EATEN_XYZE _CYCLE_APPROX_7
    #define EXTRA_CYCLES_XYZE (STEP_PULSE_CYCLES - (CYCLES_EATEN_XYZE))
    #if EXTRA_CYCLES_XYZE > 20
      uint32_t pulse_start = TCNT0;
    #endif
    #if HAS_X_STEP
      PULSE_START(X);
    #endif
    #if HAS_Y_STEP
      PULSE_START(Y);
    #endif
    #if HAS_Z_STEP
      PULSE_START(Z);
    #endif
    #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
      #if ENABLED(MIXING_EXTRUDER)
        counter_E += current_block->steps[E_AXIS];
        MIXING_STEPPERS_LOOP(j) {
          counter_m[j] += current_block->steps[E_AXIS];
          if (counter_m[j] > 0) En_STEP_WRITE(j, !INVERT_E_STEP_PIN);
        }
      #else 
        PULSE_START(E);
      #endif
    #endif 
    #if EXTRA_CYCLES_XYZE > 20
      while (EXTRA_CYCLES_XYZE > (uint32_t)(TCNT0 - pulse_start) * (INT0_PRESCALER)) {  }
      pulse_start = TCNT0;
    #elif EXTRA_CYCLES_XYZE > 0
      DELAY_NOPS(EXTRA_CYCLES_XYZE);
    #endif
    #if HAS_X_STEP
      PULSE_STOP(X);
    #endif
    #if HAS_Y_STEP
      PULSE_STOP(Y);
    #endif
    #if HAS_Z_STEP
      PULSE_STOP(Z);
    #endif
    #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
      #if ENABLED(MIXING_EXTRUDER)
        if (counter_E > 0) {
          counter_E -= current_block->step_event_count;
          count_position[E_AXIS] += count_direction[E_AXIS];
        }
        MIXING_STEPPERS_LOOP(j) {
          if (counter_m[j] > 0) {
            counter_m[j] -= current_block->mix_event_count[j];
            En_STEP_WRITE(j, INVERT_E_STEP_PIN);
          }
        }
      #else 
        PULSE_STOP(E);
      #endif
    #endif 
    if (++step_events_completed >= current_block->step_event_count) {
      all_steps_done = true;
      break;
    }
    #if EXTRA_CYCLES_XYZE > 20
      if (i) while (EXTRA_CYCLES_XYZE > (uint32_t)(TCNT0 - pulse_start) * (INT0_PRESCALER)) {  }
    #elif EXTRA_CYCLES_XYZE > 0
      if (i) DELAY_NOPS(EXTRA_CYCLES_XYZE);
    #endif
  } 
  #if ENABLED(LIN_ADVANCE)
    if (current_block->use_advance_lead) {
      const int delta_adv_steps = current_estep_rate[TOOL_E_INDEX] - current_adv_steps[TOOL_E_INDEX];
      current_adv_steps[TOOL_E_INDEX] += delta_adv_steps;
      #if ENABLED(MIXING_EXTRUDER)
        MIXING_STEPPERS_LOOP(j)
          e_steps[j] += delta_adv_steps * current_block->step_event_count / current_block->mix_event_count[j];
      #else
        e_steps[TOOL_E_INDEX] += delta_adv_steps;
      #endif
   }
  #endif
  #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
    if (e_steps[TOOL_E_INDEX]) nextAdvanceISR = 0;
  #endif
  if (step_events_completed <= (uint32_t)current_block->accelerate_until) {
    MultiU24X32toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
    acc_step_rate += current_block->initial_rate;
    NOMORE(acc_step_rate, current_block->nominal_rate);
    const uint16_t timer = calc_timer(acc_step_rate);
    SPLIT(timer);  
    _NEXT_ISR(ocr_val);
    acceleration_time += timer;
    #if ENABLED(LIN_ADVANCE)
      if (current_block->use_advance_lead) {
        #if ENABLED(MIXING_EXTRUDER)
          MIXING_STEPPERS_LOOP(j)
            current_estep_rate[j] = ((uint32_t)acc_step_rate * current_block->abs_adv_steps_multiplier8 * current_block->step_event_count / current_block->mix_event_count[j]) >> 17;
        #else
          current_estep_rate[TOOL_E_INDEX] = ((uint32_t)acc_step_rate * current_block->abs_adv_steps_multiplier8) >> 17;
        #endif
      }
    #elif ENABLED(ADVANCE)
      advance += advance_rate * step_loops;
      const long advance_whole = advance >> 8,
                 advance_factor = advance_whole - old_advance;
      #if ENABLED(MIXING_EXTRUDER)
        MIXING_STEPPERS_LOOP(j)
          e_steps[j] += advance_factor * current_block->step_event_count / current_block->mix_event_count[j];
      #else
        e_steps[TOOL_E_INDEX] += advance_factor;
      #endif
      old_advance = advance_whole;
    #endif 
    #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
      eISR_Rate = adv_rate(e_steps[TOOL_E_INDEX], timer, step_loops);
    #endif
  }
  else if (step_events_completed > (uint32_t)current_block->decelerate_after) {
    uint16_t step_rate;
    MultiU24X32toH16(step_rate, deceleration_time, current_block->acceleration_rate);
    if (step_rate < acc_step_rate) { 
      step_rate = acc_step_rate - step_rate;
      NOLESS(step_rate, current_block->final_rate);
    }
    else
      step_rate = current_block->final_rate;
    const uint16_t timer = calc_timer(step_rate);
    SPLIT(timer);  
    _NEXT_ISR(ocr_val);
    deceleration_time += timer;
    #if ENABLED(LIN_ADVANCE)
      if (current_block->use_advance_lead) {
        #if ENABLED(MIXING_EXTRUDER)
          MIXING_STEPPERS_LOOP(j)
            current_estep_rate[j] = ((uint32_t)step_rate * current_block->abs_adv_steps_multiplier8 * current_block->step_event_count / current_block->mix_event_count[j]) >> 17;
        #else
          current_estep_rate[TOOL_E_INDEX] = ((uint32_t)step_rate * current_block->abs_adv_steps_multiplier8) >> 17;
        #endif
      }
    #elif ENABLED(ADVANCE)
      advance -= advance_rate * step_loops;
      NOLESS(advance, final_advance);
      const long advance_whole = advance >> 8,
                 advance_factor = advance_whole - old_advance;
      #if ENABLED(MIXING_EXTRUDER)
        MIXING_STEPPERS_LOOP(j)
          e_steps[j] += advance_factor * current_block->step_event_count / current_block->mix_event_count[j];
      #else
        e_steps[TOOL_E_INDEX] += advance_factor;
      #endif
      old_advance = advance_whole;
    #endif 
    #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
      eISR_Rate = adv_rate(e_steps[TOOL_E_INDEX], timer, step_loops);
    #endif
  }
  else {
    #if ENABLED(LIN_ADVANCE)
      if (current_block->use_advance_lead)
        current_estep_rate[TOOL_E_INDEX] = final_estep_rate;
      eISR_Rate = adv_rate(e_steps[TOOL_E_INDEX], OCR1A_nominal, step_loops_nominal);
    #endif
    SPLIT(OCR1A_nominal);  
    _NEXT_ISR(ocr_val);
    step_loops = step_loops_nominal;
  }
  #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
    NOLESS(OCR1A, TCNT1 + 16);
  #endif
  if (all_steps_done) {
    current_block = NULL;
    planner.discard_current_block();
  }
  #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
    _ENABLE_ISRs(); 
  #endif
}
#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  #define CYCLES_EATEN_E (E_STEPPERS * 5)
  #define EXTRA_CYCLES_E (STEP_PULSE_CYCLES - (CYCLES_EATEN_E))
  void Stepper::advance_isr() {
    nextAdvanceISR = eISR_Rate;
    #if ENABLED(MK2_MULTIPLEXER)
      #define SET_E_STEP_DIR(INDEX) \
        if (e_steps[INDEX]) E## INDEX ##_DIR_WRITE(e_steps[INDEX] < 0 ? !INVERT_E## INDEX ##_DIR ^ TEST(INDEX, 0) : INVERT_E## INDEX ##_DIR ^ TEST(INDEX, 0))
    #else
      #define SET_E_STEP_DIR(INDEX) \
        if (e_steps[INDEX]) E## INDEX ##_DIR_WRITE(e_steps[INDEX] < 0 ? INVERT_E## INDEX ##_DIR : !INVERT_E## INDEX ##_DIR)
    #endif
    #define START_E_PULSE(INDEX) \
      if (e_steps[INDEX]) E## INDEX ##_STEP_WRITE(!INVERT_E_STEP_PIN)
    #define STOP_E_PULSE(INDEX) \
      if (e_steps[INDEX]) { \
        e_steps[INDEX] < 0 ? ++e_steps[INDEX] : --e_steps[INDEX]; \
        E## INDEX ##_STEP_WRITE(INVERT_E_STEP_PIN); \
      }
    SET_E_STEP_DIR(0);
    #if E_STEPPERS > 1
      SET_E_STEP_DIR(1);
      #if E_STEPPERS > 2
        SET_E_STEP_DIR(2);
        #if E_STEPPERS > 3
          SET_E_STEP_DIR(3);
          #if E_STEPPERS > 4
            SET_E_STEP_DIR(4);
          #endif
        #endif
      #endif
    #endif
    for (uint8_t i = step_loops; i--;) {
      #if EXTRA_CYCLES_E > 20
        uint32_t pulse_start = TCNT0;
      #endif
      START_E_PULSE(0);
      #if E_STEPPERS > 1
        START_E_PULSE(1);
        #if E_STEPPERS > 2
          START_E_PULSE(2);
          #if E_STEPPERS > 3
            START_E_PULSE(3);
            #if E_STEPPERS > 4
              START_E_PULSE(4);
            #endif
          #endif
        #endif
      #endif
      #if EXTRA_CYCLES_E > 20
        while (EXTRA_CYCLES_E > (uint32_t)(TCNT0 - pulse_start) * (INT0_PRESCALER)) {  }
        pulse_start = TCNT0;
      #elif EXTRA_CYCLES_E > 0
        DELAY_NOPS(EXTRA_CYCLES_E);
      #endif
      STOP_E_PULSE(0);
      #if E_STEPPERS > 1
        STOP_E_PULSE(1);
        #if E_STEPPERS > 2
          STOP_E_PULSE(2);
          #if E_STEPPERS > 3
            STOP_E_PULSE(3);
            #if E_STEPPERS > 4
              STOP_E_PULSE(4);
            #endif
          #endif
        #endif
      #endif
      #if EXTRA_CYCLES_E > 20
        if (i) while (EXTRA_CYCLES_E > (uint32_t)(TCNT0 - pulse_start) * (INT0_PRESCALER)) {  }
      #elif EXTRA_CYCLES_E > 0
        if (i) DELAY_NOPS(EXTRA_CYCLES_E);
      #endif
    } 
  }
  void Stepper::advance_isr_scheduler() {
    CBI(TIMSK0, OCIE0B); 
    DISABLE_STEPPER_DRIVER_INTERRUPT();
    sei();
    if (!nextMainISR) isr();
    if (!nextAdvanceISR) advance_isr();
    if (nextAdvanceISR <= nextMainISR) {
      OCR1A = nextAdvanceISR;
      if (nextMainISR) nextMainISR -= nextAdvanceISR;
      nextAdvanceISR = 0;
    }
    else {
      OCR1A = nextMainISR;
      if (nextAdvanceISR && nextAdvanceISR != ADV_NEVER)
        nextAdvanceISR -= nextMainISR;
      nextMainISR = 0;
    }
    NOLESS(OCR1A, TCNT1 + 16);
    _ENABLE_ISRs();
  }
#endif 
void Stepper::init() {
  #if ENABLED(FYS_POWERBREAK_STEPPER_STATUS)
    powerBreakStatus = 0;
  #endif
  #if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM
    digipot_init();
  #endif
  #if HAS_MICROSTEPS
    microstep_init();
  #endif
  #if ENABLED(HAVE_TMCDRIVER)
    tmc_init();
  #endif
  #if ENABLED(HAVE_TMC2130)
    tmc2130_init();
  #endif
  #if ENABLED(HAVE_L6470DRIVER)
    L6470_init();
  #endif
  #if HAS_X_DIR
    X_DIR_INIT;
  #endif
  #if HAS_X2_DIR
    X2_DIR_INIT;
  #endif
  #if HAS_Y_DIR
    Y_DIR_INIT;
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS) && HAS_Y2_DIR
      Y2_DIR_INIT;
    #endif
  #endif
  #if HAS_Z_DIR
    Z_DIR_INIT;
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && HAS_Z2_DIR
      Z2_DIR_INIT;
    #endif
  #endif
  #if HAS_E0_DIR
    E0_DIR_INIT;
  #endif
  #if HAS_E1_DIR
    E1_DIR_INIT;
  #endif
  #if HAS_E2_DIR
    E2_DIR_INIT;
  #endif
  #if HAS_E3_DIR
    E3_DIR_INIT;
  #endif
  #if HAS_E4_DIR
    E4_DIR_INIT;
  #endif
  #if HAS_X_ENABLE
    X_ENABLE_INIT;
    if (!X_ENABLE_ON) X_ENABLE_WRITE(HIGH);
    #if ENABLED(DUAL_X_CARRIAGE) && HAS_X2_ENABLE
      X2_ENABLE_INIT;
      if (!X_ENABLE_ON) X2_ENABLE_WRITE(HIGH);
    #endif
  #endif
  #if HAS_Y_ENABLE
    Y_ENABLE_INIT;
    if (!Y_ENABLE_ON) Y_ENABLE_WRITE(HIGH);
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS) && HAS_Y2_ENABLE
      Y2_ENABLE_INIT;
      if (!Y_ENABLE_ON) Y2_ENABLE_WRITE(HIGH);
    #endif
  #endif
  #if HAS_Z_ENABLE
    Z_ENABLE_INIT;
    if (!Z_ENABLE_ON) Z_ENABLE_WRITE(HIGH);
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && HAS_Z2_ENABLE
      Z2_ENABLE_INIT;
      if (!Z_ENABLE_ON) Z2_ENABLE_WRITE(HIGH);
    #endif
  #endif
  #if HAS_E0_ENABLE
    E0_ENABLE_INIT;
    if (!E_ENABLE_ON) E0_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E1_ENABLE
    E1_ENABLE_INIT;
    if (!E_ENABLE_ON) E1_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E2_ENABLE
    E2_ENABLE_INIT;
    if (!E_ENABLE_ON) E2_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E3_ENABLE
    E3_ENABLE_INIT;
    if (!E_ENABLE_ON) E3_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E4_ENABLE
    E4_ENABLE_INIT;
    if (!E_ENABLE_ON) E4_ENABLE_WRITE(HIGH);
  #endif
  endstops.init();
  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE(AXIS) disable_## AXIS()
  #define AXIS_INIT(AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, _INVERT_STEP_PIN(PIN)); \
    _DISABLE(AXIS)
  #define E_AXIS_INIT(NUM) AXIS_INIT(E## NUM, E)
  #if HAS_X_STEP
    #if ENABLED(X_DUAL_STEPPER_DRIVERS) || ENABLED(DUAL_X_CARRIAGE)
      X2_STEP_INIT;
      X2_STEP_WRITE(INVERT_X_STEP_PIN);
    #endif
    AXIS_INIT(X, X);
  #endif
  #if HAS_Y_STEP
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS)
      Y2_STEP_INIT;
      Y2_STEP_WRITE(INVERT_Y_STEP_PIN);
    #endif
    AXIS_INIT(Y, Y);
  #endif
  #if HAS_Z_STEP
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS)
      Z2_STEP_INIT;
      Z2_STEP_WRITE(INVERT_Z_STEP_PIN);
    #endif
    AXIS_INIT(Z, Z);
  #endif
  #if HAS_E0_STEP
    E_AXIS_INIT(0);
  #endif
  #if HAS_E1_STEP
    E_AXIS_INIT(1);
  #endif
  #if HAS_E2_STEP
    E_AXIS_INIT(2);
  #endif
  #if HAS_E3_STEP
    E_AXIS_INIT(3);
  #endif
  #if HAS_E4_STEP
    E_AXIS_INIT(4);
  #endif
  SET_WGM(1, CTC_OCRnA);
  SET_COMA(1, NORMAL);
  SET_CS(1, PRESCALER_8);  
  OCR1A = 0x4000;
  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
  #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
    for (uint8_t i = 0; i < COUNT(e_steps); i++) e_steps[i] = 0;
    #if ENABLED(LIN_ADVANCE)
      ZERO(current_adv_steps);
    #endif
  #endif 
  endstops.enable(true); 
  sei();
  set_directions(); 
}
void Stepper::synchronize() { while (planner.blocks_queued()) idle(); }
void Stepper::set_position(const long &a, const long &b, const long &c, const long &e) {
  synchronize(); 
  CRITICAL_SECTION_START;
  #if CORE_IS_XY
    count_position[A_AXIS] = a + b;
    count_position[B_AXIS] = CORESIGN(a - b);
    count_position[Z_AXIS] = c;
  #elif CORE_IS_XZ
    count_position[A_AXIS] = a + c;
    count_position[Y_AXIS] = b;
    count_position[C_AXIS] = CORESIGN(a - c);
  #elif CORE_IS_YZ
    count_position[X_AXIS] = a;
    count_position[B_AXIS] = b + c;
    count_position[C_AXIS] = CORESIGN(b - c);
  #else
    count_position[X_AXIS] = a;
    count_position[Y_AXIS] = b;
    count_position[Z_AXIS] = c;
  #endif
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}
void Stepper::set_position(const AxisEnum &axis, const long &v) {
  CRITICAL_SECTION_START;
  count_position[axis] = v;
  CRITICAL_SECTION_END;
}
void Stepper::set_e_position(const long &e) {
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}
long Stepper::position(AxisEnum axis) {
  CRITICAL_SECTION_START;
  const long count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}
float Stepper::get_axis_position_mm(AxisEnum axis) {
  float axis_steps;
  #if IS_CORE
    if (axis == CORE_AXIS_1 || axis == CORE_AXIS_2) {
      CRITICAL_SECTION_START;
      axis_steps = 0.5f * (
        axis == CORE_AXIS_2 ? CORESIGN(count_position[CORE_AXIS_1] - count_position[CORE_AXIS_2])
                            : count_position[CORE_AXIS_1] + count_position[CORE_AXIS_2]
      );
      CRITICAL_SECTION_END;
    }
    else
      axis_steps = position(axis);
  #else
    axis_steps = position(axis);
  #endif
  return axis_steps * planner.steps_to_mm[axis];
}
void Stepper::finish_and_disable() {
  synchronize();
  disable_all_steppers();
}
void Stepper::quick_stop() {
  cleaning_buffer_counter = 5000;
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while (planner.blocks_queued()) planner.discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
  #if ENABLED(ULTRA_LCD)
    planner.clear_block_buffer_runtime();
  #endif
}
void Stepper::endstop_triggered(AxisEnum axis) {
  #if IS_CORE
    endstops_trigsteps[axis] = 0.5f * (
      axis == CORE_AXIS_2 ? CORESIGN(count_position[CORE_AXIS_1] - count_position[CORE_AXIS_2])
                          : count_position[CORE_AXIS_1] + count_position[CORE_AXIS_2]
    );
  #else 
    endstops_trigsteps[axis] = count_position[axis];
  #endif 
  kill_current_block();
}
void Stepper::report_positions() {
  CRITICAL_SECTION_START;
  const long xpos = count_position[X_AXIS],
             ypos = count_position[Y_AXIS],
             zpos = count_position[Z_AXIS];
  CRITICAL_SECTION_END;
  #if CORE_IS_XY || CORE_IS_XZ || IS_SCARA
    SERIAL_PROTOCOLPGM(MSG_COUNT_A);
  #else
    SERIAL_PROTOCOLPGM(MSG_COUNT_X);
  #endif
  SERIAL_PROTOCOL(xpos);
  #if CORE_IS_XY || CORE_IS_YZ || IS_SCARA
    SERIAL_PROTOCOLPGM(" B:");
  #else
    SERIAL_PROTOCOLPGM(" Y:");
  #endif
  SERIAL_PROTOCOL(ypos);
  #if CORE_IS_XZ || CORE_IS_YZ
    SERIAL_PROTOCOLPGM(" C:");
  #else
    SERIAL_PROTOCOLPGM(" Z:");
  #endif
  SERIAL_PROTOCOL(zpos);
  SERIAL_EOL();
}
#if ENABLED(BABYSTEPPING)
  #if ENABLED(DELTA)
    #define CYCLES_EATEN_BABYSTEP (2 * 15)
  #else
    #define CYCLES_EATEN_BABYSTEP 0
  #endif
  #define EXTRA_CYCLES_BABYSTEP (STEP_PULSE_CYCLES - (CYCLES_EATEN_BABYSTEP))
  #define _ENABLE(AXIS) enable_## AXIS()
  #define _READ_DIR(AXIS) AXIS ##_DIR_READ
  #define _INVERT_DIR(AXIS) INVERT_## AXIS ##_DIR
  #define _APPLY_DIR(AXIS, INVERT) AXIS ##_APPLY_DIR(INVERT, true)
  #if EXTRA_CYCLES_BABYSTEP > 20
    #define _SAVE_START const uint32_t pulse_start = TCNT0
    #define _PULSE_WAIT while (EXTRA_CYCLES_BABYSTEP > (uint32_t)(TCNT0 - pulse_start) * (INT0_PRESCALER)) {  }
  #else
    #define _SAVE_START NOOP
    #if EXTRA_CYCLES_BABYSTEP > 0
      #define _PULSE_WAIT DELAY_NOPS(EXTRA_CYCLES_BABYSTEP)
    #elif STEP_PULSE_CYCLES > 0
      #define _PULSE_WAIT NOOP
    #elif ENABLED(DELTA)
      #define _PULSE_WAIT delayMicroseconds(2);
    #else
      #define _PULSE_WAIT delayMicroseconds(4);
    #endif
  #endif
  #define BABYSTEP_AXIS(AXIS, INVERT) {                     \
      const uint8_t old_dir = _READ_DIR(AXIS);              \
      _ENABLE(AXIS);                                        \
      _SAVE_START;                                          \
      _APPLY_DIR(AXIS, _INVERT_DIR(AXIS)^direction^INVERT); \
      _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS), true);     \
      _PULSE_WAIT;                                          \
      _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS), true);      \
      _APPLY_DIR(AXIS, old_dir);                            \
    }
  void Stepper::babystep(const AxisEnum axis, const bool direction) {
    cli();
    switch (axis) {
      #if ENABLED(BABYSTEP_XY)
        case X_AXIS:
          BABYSTEP_AXIS(X, false);
          break;
        case Y_AXIS:
          BABYSTEP_AXIS(Y, false);
          break;
      #endif
      case Z_AXIS: {
        #if DISABLED(DELTA)
          BABYSTEP_AXIS(Z, BABYSTEP_INVERT_Z);
        #else 
          const bool z_direction = direction ^ BABYSTEP_INVERT_Z;
          enable_X();
          enable_Y();
          enable_Z();
          const uint8_t old_x_dir_pin = X_DIR_READ,
                        old_y_dir_pin = Y_DIR_READ,
                        old_z_dir_pin = Z_DIR_READ;
          X_DIR_WRITE(INVERT_X_DIR ^ z_direction);
          Y_DIR_WRITE(INVERT_Y_DIR ^ z_direction);
          Z_DIR_WRITE(INVERT_Z_DIR ^ z_direction);
          _SAVE_START;
          X_STEP_WRITE(!INVERT_X_STEP_PIN);
          Y_STEP_WRITE(!INVERT_Y_STEP_PIN);
          Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
          _PULSE_WAIT;
          X_STEP_WRITE(INVERT_X_STEP_PIN);
          Y_STEP_WRITE(INVERT_Y_STEP_PIN);
          Z_STEP_WRITE(INVERT_Z_STEP_PIN);
          X_DIR_WRITE(old_x_dir_pin);
          Y_DIR_WRITE(old_y_dir_pin);
          Z_DIR_WRITE(old_z_dir_pin);
        #endif
      } break;
      default: break;
    }
    sei();
  }
#endif 
#if HAS_DIGIPOTSS
  void Stepper::digitalPotWrite(const int16_t address, const int16_t value) {
    WRITE(DIGIPOTSS_PIN, LOW);  
    SPI.transfer(address);      
    SPI.transfer(value);
    WRITE(DIGIPOTSS_PIN, HIGH); 
  }
#endif 
#if HAS_MOTOR_CURRENT_PWM
  void Stepper::refresh_motor_power() {
    for (uint8_t i = 0; i < COUNT(motor_current_setting); ++i) {
      switch (i) {
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
          case 0:
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
          case 1:
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
          case 2:
        #endif
            digipot_current(i, motor_current_setting[i]);
        default: break;
      }
    }
  }
#endif 
#if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM
  void Stepper::digipot_current(const uint8_t driver, const int current) {
    #if HAS_DIGIPOTSS
      const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
      digitalPotWrite(digipot_ch[driver], current);
    #elif HAS_MOTOR_CURRENT_PWM
      if (WITHIN(driver, 0, 2))
        motor_current_setting[driver] = current; 
      #define _WRITE_CURRENT_PWM(P) analogWrite(MOTOR_CURRENT_PWM_## P ##_PIN, 255L * current / (MOTOR_CURRENT_PWM_RANGE))
      switch (driver) {
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
          case 0: _WRITE_CURRENT_PWM(XY); break;
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
          case 1: _WRITE_CURRENT_PWM(Z); break;
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
          case 2: _WRITE_CURRENT_PWM(E); break;
        #endif
      }
    #endif
  }
  void Stepper::digipot_init() {
    #if HAS_DIGIPOTSS
      static const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;
      SPI.begin();
      SET_OUTPUT(DIGIPOTSS_PIN);
      for (uint8_t i = 0; i < COUNT(digipot_motor_current); i++) {
        digipot_current(i, digipot_motor_current[i]);
      }
    #elif HAS_MOTOR_CURRENT_PWM
      #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
        SET_OUTPUT(MOTOR_CURRENT_PWM_XY_PIN);
      #endif
      #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
        SET_OUTPUT(MOTOR_CURRENT_PWM_Z_PIN);
      #endif
      #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
        SET_OUTPUT(MOTOR_CURRENT_PWM_E_PIN);
      #endif
      refresh_motor_power();
      SET_CS5(PRESCALER_1);
    #endif
  }
#endif
#if HAS_MICROSTEPS
  void Stepper::microstep_init() {
    SET_OUTPUT(X_MS1_PIN);
    SET_OUTPUT(X_MS2_PIN);
    #if HAS_Y_MICROSTEPS
      SET_OUTPUT(Y_MS1_PIN);
      SET_OUTPUT(Y_MS2_PIN);
    #endif
    #if HAS_Z_MICROSTEPS
      SET_OUTPUT(Z_MS1_PIN);
      SET_OUTPUT(Z_MS2_PIN);
    #endif
    #if HAS_E0_MICROSTEPS
      SET_OUTPUT(E0_MS1_PIN);
      SET_OUTPUT(E0_MS2_PIN);
    #endif
    #if HAS_E1_MICROSTEPS
      SET_OUTPUT(E1_MS1_PIN);
      SET_OUTPUT(E1_MS2_PIN);
    #endif
    #if HAS_E2_MICROSTEPS
      SET_OUTPUT(E2_MS1_PIN);
      SET_OUTPUT(E2_MS2_PIN);
    #endif
    #if HAS_E3_MICROSTEPS
      SET_OUTPUT(E3_MS1_PIN);
      SET_OUTPUT(E3_MS2_PIN);
    #endif
    #if HAS_E4_MICROSTEPS
      SET_OUTPUT(E4_MS1_PIN);
      SET_OUTPUT(E4_MS2_PIN);
    #endif
    static const uint8_t microstep_modes[] = MICROSTEP_MODES;
    for (uint16_t i = 0; i < COUNT(microstep_modes); i++)
      microstep_mode(i, microstep_modes[i]);
  }
  void Stepper::microstep_ms(const uint8_t driver, const int8_t ms1, const int8_t ms2) {
    if (ms1 >= 0) switch (driver) {
      case 0: WRITE(X_MS1_PIN, ms1); break;
      #if HAS_Y_MICROSTEPS
        case 1: WRITE(Y_MS1_PIN, ms1); break;
      #endif
      #if HAS_Z_MICROSTEPS
        case 2: WRITE(Z_MS1_PIN, ms1); break;
      #endif
      #if HAS_E0_MICROSTEPS
        case 3: WRITE(E0_MS1_PIN, ms1); break;
      #endif
      #if HAS_E1_MICROSTEPS
        case 4: WRITE(E1_MS1_PIN, ms1); break;
      #endif
      #if HAS_E2_MICROSTEPS
        case 5: WRITE(E2_MS1_PIN, ms1); break;
      #endif
      #if HAS_E3_MICROSTEPS
        case 6: WRITE(E3_MS1_PIN, ms1); break;
      #endif
      #if HAS_E4_MICROSTEPS
        case 7: WRITE(E4_MS1_PIN, ms1); break;
      #endif
    }
    if (ms2 >= 0) switch (driver) {
      case 0: WRITE(X_MS2_PIN, ms2); break;
      #if HAS_Y_MICROSTEPS
        case 1: WRITE(Y_MS2_PIN, ms2); break;
      #endif
      #if HAS_Z_MICROSTEPS
        case 2: WRITE(Z_MS2_PIN, ms2); break;
      #endif
      #if HAS_E0_MICROSTEPS
        case 3: WRITE(E0_MS2_PIN, ms2); break;
      #endif
      #if HAS_E1_MICROSTEPS
        case 4: WRITE(E1_MS2_PIN, ms2); break;
      #endif
      #if HAS_E2_MICROSTEPS
        case 5: WRITE(E2_MS2_PIN, ms2); break;
      #endif
      #if HAS_E3_MICROSTEPS
        case 6: WRITE(E3_MS2_PIN, ms2); break;
      #endif
      #if HAS_E4_MICROSTEPS
        case 7: WRITE(E4_MS2_PIN, ms2); break;
      #endif
    }
  }
  void Stepper::microstep_mode(const uint8_t driver, const uint8_t stepping_mode) {
    switch (stepping_mode) {
      case 1: microstep_ms(driver, MICROSTEP1); break;
      case 2: microstep_ms(driver, MICROSTEP2); break;
      case 4: microstep_ms(driver, MICROSTEP4); break;
      case 8: microstep_ms(driver, MICROSTEP8); break;
      case 16: microstep_ms(driver, MICROSTEP16); break;
    }
  }
  void Stepper::microstep_readings() {
    SERIAL_PROTOCOLLNPGM("MS1,MS2 Pins");
    SERIAL_PROTOCOLPGM("X: ");
    SERIAL_PROTOCOL(READ(X_MS1_PIN));
    SERIAL_PROTOCOLLN(READ(X_MS2_PIN));
    #if HAS_Y_MICROSTEPS
      SERIAL_PROTOCOLPGM("Y: ");
      SERIAL_PROTOCOL(READ(Y_MS1_PIN));
      SERIAL_PROTOCOLLN(READ(Y_MS2_PIN));
    #endif
    #if HAS_Z_MICROSTEPS
      SERIAL_PROTOCOLPGM("Z: ");
      SERIAL_PROTOCOL(READ(Z_MS1_PIN));
      SERIAL_PROTOCOLLN(READ(Z_MS2_PIN));
    #endif
    #if HAS_E0_MICROSTEPS
      SERIAL_PROTOCOLPGM("E0: ");
      SERIAL_PROTOCOL(READ(E0_MS1_PIN));
      SERIAL_PROTOCOLLN(READ(E0_MS2_PIN));
    #endif
    #if HAS_E1_MICROSTEPS
      SERIAL_PROTOCOLPGM("E1: ");
      SERIAL_PROTOCOL(READ(E1_MS1_PIN));
      SERIAL_PROTOCOLLN(READ(E1_MS2_PIN));
    #endif
    #if HAS_E2_MICROSTEPS
      SERIAL_PROTOCOLPGM("E2: ");
      SERIAL_PROTOCOL(READ(E2_MS1_PIN));
      SERIAL_PROTOCOLLN(READ(E2_MS2_PIN));
    #endif
    #if HAS_E3_MICROSTEPS
      SERIAL_PROTOCOLPGM("E3: ");
      SERIAL_PROTOCOL(READ(E3_MS1_PIN));
      SERIAL_PROTOCOLLN(READ(E3_MS2_PIN));
    #endif
    #if HAS_E4_MICROSTEPS
      SERIAL_PROTOCOLPGM("E4: ");
      SERIAL_PROTOCOL(READ(E4_MS1_PIN));
      SERIAL_PROTOCOLLN(READ(E4_MS2_PIN));
    #endif
  }
#endif 
