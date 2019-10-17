#ifndef STEPPER_H
#define STEPPER_H
#include "planner.h"
#include "speed_lookuptable.h"
#include "stepper_indirection.h"
#include "language.h"
#include "types.h"
class Stepper;
extern Stepper stepper;
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
  asm volatile ( \
                 "clr r26 \n\t" \
                 "mul %A1, %B2 \n\t" \
                 "movw %A0, r0 \n\t" \
                 "mul %A1, %A2 \n\t" \
                 "add %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "lsr r0 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "clr r1 \n\t" \
                 : \
                 "=&r" (intRes) \
                 : \
                 "d" (charIn1), \
                 "d" (intIn2) \
                 : \
                 "r26" \
               )
#define ENABLE_STEPPER_DRIVER_INTERRUPT()  SBI(TIMSK1, OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() CBI(TIMSK1, OCIE1A)
class Stepper {
  public:
    #if ENABLED(FYS_POWERBREAK_STEPPER_STATUS)
      static volatile uint8_t powerBreakStatus;
    #endif
    static block_t* current_block;  
    #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
      static bool abort_on_endstop_hit;
    #endif
    #if ENABLED(Z_DUAL_ENDSTOPS)
      static bool performing_homing;
    #endif
    #if HAS_MOTOR_CURRENT_PWM
      #ifndef PWM_MOTOR_CURRENT
        #define PWM_MOTOR_CURRENT DEFAULT_PWM_MOTOR_CURRENT
      #endif
      static uint32_t motor_current_setting[3];
    #endif
  private:
    static uint8_t last_direction_bits;        
    static uint16_t cleaning_buffer_counter;
    #if ENABLED(Z_DUAL_ENDSTOPS)
      static bool locked_z_motor, locked_z2_motor;
    #endif
    static long counter_X, counter_Y, counter_Z, counter_E;
    static volatile uint32_t step_events_completed; 
    #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
      static uint16_t nextMainISR, nextAdvanceISR, eISR_Rate;
      #define _NEXT_ISR(T) nextMainISR = T
      #if ENABLED(LIN_ADVANCE)
        static volatile int e_steps[E_STEPPERS];
        static int final_estep_rate;
        static int current_estep_rate[E_STEPPERS]; 
        static int current_adv_steps[E_STEPPERS];  
      #else
        static long e_steps[E_STEPPERS];
        static long advance_rate, advance, final_advance;
        static long old_advance;
      #endif
    #else
      #define _NEXT_ISR(T) OCR1A = T
    #endif 
    static long acceleration_time, deceleration_time;
    static unsigned short acc_step_rate; 
    static uint8_t step_loops, step_loops_nominal;
    static unsigned short OCR1A_nominal;
    static volatile long endstops_trigsteps[XYZ];
    static volatile long endstops_stepsTotal, endstops_stepsDone;
    static volatile long count_position[NUM_AXIS];
    static volatile signed char count_direction[NUM_AXIS];
    #if ENABLED(MIXING_EXTRUDER)
      static long counter_m[MIXING_STEPPERS];
      #define MIXING_STEPPERS_LOOP(VAR) \
        for (uint8_t VAR = 0; VAR < MIXING_STEPPERS; VAR++) \
          if (current_block->mix_event_count[VAR])
    #endif
  public:
    Stepper() { };
    static void init();
    static void isr();
    #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
      static void advance_isr();
      static void advance_isr_scheduler();
    #endif
    static void synchronize();
    static void set_position(const long &a, const long &b, const long &c, const long &e);
    static void set_position(const AxisEnum &a, const long &v);
    static void set_e_position(const long &e);
    static void set_directions();
    static long position(AxisEnum axis);
    static void report_positions();
    static float get_axis_position_mm(AxisEnum axis);
    #if IS_SCARA
      static FORCE_INLINE float get_axis_position_degrees(AxisEnum axis) { return get_axis_position_mm(axis); }
    #endif
    static void wake_up();
    static void finish_and_disable();
    static void quick_stop();
    static FORCE_INLINE bool motor_direction(AxisEnum axis) { return TEST(last_direction_bits, axis); }
    #if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM
      static void digitalPotWrite(const int16_t address, const int16_t value);
      static void digipot_current(const uint8_t driver, const int16_t current);
    #endif
    #if HAS_MICROSTEPS
      static void microstep_ms(const uint8_t driver, const int8_t ms1, const int8_t ms2);
      static void microstep_mode(const uint8_t driver, const uint8_t stepping);
      static void microstep_readings();
    #endif
    #if ENABLED(Z_DUAL_ENDSTOPS)
      static FORCE_INLINE void set_homing_flag(const bool state) { performing_homing = state; }
      static FORCE_INLINE void set_z_lock(const bool state) { locked_z_motor = state; }
      static FORCE_INLINE void set_z2_lock(const bool state) { locked_z2_motor = state; }
    #endif
    #if ENABLED(BABYSTEPPING)
      static void babystep(const AxisEnum axis, const bool direction); 
      static void babystepforce(const AxisEnum axis, const bool direction);
    #endif
    static inline void kill_current_block() {
      step_events_completed = current_block->step_event_count;
    }
    static void endstop_triggered(AxisEnum axis);
    static FORCE_INLINE float triggered_position_mm(AxisEnum axis) {
      return endstops_trigsteps[axis] * planner.steps_to_mm[axis];
    }
    #if HAS_MOTOR_CURRENT_PWM
      static void refresh_motor_power();
    #endif
  private:
    static FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
      unsigned short timer;
      NOMORE(step_rate, MAX_STEP_FREQUENCY);
      if (step_rate > 20000) { 
        step_rate >>= 2;
        step_loops = 4;
      }
      else if (step_rate > 10000) { 
        step_rate >>= 1;
        step_loops = 2;
      }
      else {
        step_loops = 1;
      }
      NOLESS(step_rate, F_CPU / 500000);
      step_rate -= F_CPU / 500000; 
      if (step_rate >= (8 * 256)) { 
        unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate >> 8)][0];
        unsigned char tmp_step_rate = (step_rate & 0x00FF);
        unsigned short gain = (unsigned short)pgm_read_word_near(table_address + 2);
        MultiU16X8toH16(timer, tmp_step_rate, gain);
        timer = (unsigned short)pgm_read_word_near(table_address) - timer;
      }
      else { 
        unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
        table_address += ((step_rate) >> 1) & 0xFFFC;
        timer = (unsigned short)pgm_read_word_near(table_address);
        timer -= (((unsigned short)pgm_read_word_near(table_address + 2) * (unsigned char)(step_rate & 0x0007)) >> 3);
      }
      if (timer < 100) { 
        timer = 100;
        MYSERIAL.print(MSG_STEPPER_TOO_HIGH);
        MYSERIAL.println(step_rate);
      }
      return timer;
    }
    static FORCE_INLINE void trapezoid_generator_reset() {
      static int8_t last_extruder = -1;
      if (current_block->direction_bits != last_direction_bits || current_block->active_extruder != last_extruder) {
        last_direction_bits = current_block->direction_bits;
        last_extruder = current_block->active_extruder;
        set_directions();
      }
      #if ENABLED(ADVANCE)
        advance = current_block->initial_advance;
        final_advance = current_block->final_advance;
        #if ENABLED(MIXING_EXTRUDER)
          long advance_factor = (advance >> 8) - old_advance;
          MIXING_STEPPERS_LOOP(j)
            e_steps[j] += advance_factor * current_block->step_event_count / current_block->mix_event_count[j];
        #else
          e_steps[TOOL_E_INDEX] += ((advance >> 8) - old_advance);
        #endif
        old_advance = advance >> 8;
      #endif
      deceleration_time = 0;
      OCR1A_nominal = calc_timer(current_block->nominal_rate);
      step_loops_nominal = step_loops;
      acc_step_rate = current_block->initial_rate;
      acceleration_time = calc_timer(acc_step_rate);
      _NEXT_ISR(acceleration_time);
      #if ENABLED(LIN_ADVANCE)
        if (current_block->use_advance_lead) {
          current_estep_rate[current_block->active_extruder] = ((unsigned long)acc_step_rate * current_block->abs_adv_steps_multiplier8) >> 17;
          final_estep_rate = (current_block->nominal_rate * current_block->abs_adv_steps_multiplier8) >> 17;
        }
      #endif
    }
    #if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM
      static void digipot_init();
    #endif
    #if HAS_MICROSTEPS
      static void microstep_init();
    #endif
    #if ENABLED(FYS_POWERBREAK_STEPPER_STATUS)
      static inline void powerStepCheck() 
      {
          if (powerBreakStatus == 1 || powerBreakStatus == 2)
          {
              powerBreakStatus = 2;
              current_block = NULL;
              planner.clearBlock();
          }
      }
    #endif
};
#endif 
