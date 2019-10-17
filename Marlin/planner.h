#ifndef PLANNER_H
#define PLANNER_H
#include "types.h"
#include "enum.h"
#include "Marlin.h"
#if HAS_ABL
  #include "vector_3.h"
#endif
enum BlockFlagBit {
  BLOCK_BIT_RECALCULATE,
  BLOCK_BIT_NOMINAL_LENGTH,
  BLOCK_BIT_START_FROM_FULL_HALT,
  BLOCK_BIT_BUSY
};
enum BlockFlag {
  BLOCK_FLAG_RECALCULATE          = _BV(BLOCK_BIT_RECALCULATE),
  BLOCK_FLAG_NOMINAL_LENGTH       = _BV(BLOCK_BIT_NOMINAL_LENGTH),
  BLOCK_FLAG_START_FROM_FULL_HALT = _BV(BLOCK_BIT_START_FROM_FULL_HALT),
  BLOCK_FLAG_BUSY                 = _BV(BLOCK_BIT_BUSY)
};
typedef struct {
  uint8_t flag;                             
  unsigned char active_extruder;            
#ifdef FYS_SAFE_PRINT_BREAK
  uint32_t sdPos;
#endif
  int32_t steps[NUM_AXIS];                  
  uint32_t step_event_count;                
  #if ENABLED(MIXING_EXTRUDER)
    uint32_t mix_event_count[MIXING_STEPPERS]; 
  #endif
  int32_t accelerate_until,                 
          decelerate_after,                 
          acceleration_rate;                
  uint8_t direction_bits;                   
  #if ENABLED(LIN_ADVANCE)
    bool use_advance_lead;
    uint32_t abs_adv_steps_multiplier8; 
  #elif ENABLED(ADVANCE)
    int32_t advance_rate;
    volatile int32_t initial_advance;
    volatile int32_t final_advance;
    float advance;
  #endif
  float nominal_speed,                      
        entry_speed,                        
        max_entry_speed,                    
        millimeters,                        
        acceleration;                       
  uint32_t nominal_rate,                    
           initial_rate,                    
           final_rate,                      
           acceleration_steps_per_s2;       
  #if FAN_COUNT > 0
    uint16_t fan_speed[FAN_COUNT];
  #endif
  #if ENABLED(BARICUDA)
    uint32_t valve_pressure, e_to_p_pressure;
  #endif
  uint32_t segment_time;
} block_t;
#define BLOCK_MOD(n) ((n)&(BLOCK_BUFFER_SIZE-1))
class Planner {
  public:
    static block_t block_buffer[BLOCK_BUFFER_SIZE];
    static volatile uint8_t block_buffer_head,  
                            block_buffer_tail;
    #if ENABLED(DISTINCT_E_FACTORS)
      static uint8_t last_extruder;             
    #endif
    static float max_feedrate_mm_s[XYZE_N],     
                 axis_steps_per_mm[XYZE_N],
                 steps_to_mm[XYZE_N];
    static uint32_t max_acceleration_steps_per_s2[XYZE_N],
                    max_acceleration_mm_per_s2[XYZE_N]; 
    static millis_t min_segment_time;
    static float min_feedrate_mm_s,
                 acceleration,         
                 retract_acceleration, 
                 travel_acceleration,  
                 max_jerk[XYZE],       
                 min_travel_feedrate_mm_s;
    #if HAS_ABL
    static char abl_enabled;
      #if ABL_PLANAR
        static matrix_3x3 bed_level_matrix; 
      #endif
    #endif
    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      static float z_fade_height, inverse_z_fade_height;
    #endif
    #if ENABLED(LIN_ADVANCE)
      static float extruder_advance_k, advance_ed_ratio;
    #endif
    #if ENABLED(FYS_POWERBREAK_STEPPER_STATUS)
      inline void clearBlock()
      {
          if (blocks_queued())
              block_buffer_head = block_buffer_tail;
      }
    #endif
  private:
    static long position[NUM_AXIS];
    static float previous_speed[NUM_AXIS];
    static float previous_nominal_speed;
    static uint32_t cutoff_long;
    #if ENABLED(DISABLE_INACTIVE_EXTRUDER)
      static uint8_t g_uc_extruder_last_move[EXTRUDERS];
    #endif 
    #ifdef XY_FREQUENCY_LIMIT
      #define MAX_FREQ_TIME long(1000000.0/XY_FREQUENCY_LIMIT)
      static unsigned char old_direction_bits;
      static long axis_segment_time[2][3];
    #endif
    #if ENABLED(LIN_ADVANCE)
      static float position_float[NUM_AXIS];
    #endif
    #if ENABLED(ULTRA_LCD)
      volatile static uint32_t block_buffer_runtime_us; 
    #endif
  public:
    Planner();
    void init();
    static void report_positon();
    static void reset_acceleration_rates();
    static void refresh_positioning();
    static void check_axes_activity();
    static uint8_t movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE); }
    static bool is_full() { return (block_buffer_tail == BLOCK_MOD(block_buffer_head + 1)); }
    #if PLANNER_LEVELING
      #define ARG_X float lx
      #define ARG_Y float ly
      #define ARG_Z float lz
      static void apply_leveling(float &lx, float &ly, float &lz);
      static void apply_leveling(float logical[XYZ]) { apply_leveling(logical[X_AXIS], logical[Y_AXIS], logical[Z_AXIS]); }
      static void unapply_leveling(float logical[XYZ]);
    #else
      #define ARG_X const float &lx
      #define ARG_Y const float &ly
      #define ARG_Z const float &lz
    #endif
    static void _buffer_line(const float &a, const float &b, const float &c, const float &e, float fr_mm_s, const uint8_t extruder);
    static void _set_position_mm(const float &a, const float &b, const float &c, const float &e);
    static FORCE_INLINE void buffer_line(ARG_X, ARG_Y, ARG_Z, const float &e, const float &fr_mm_s, const uint8_t extruder) {
      #if PLANNER_LEVELING && IS_CARTESIAN
        apply_leveling(lx, ly, lz);
      #endif
      _buffer_line(lx, ly, lz, e, fr_mm_s, extruder);
    }
    static FORCE_INLINE void buffer_line_and_report(ARG_X, ARG_Y, ARG_Z, const float &e, const float &fr_mm_s, const uint8_t extruder) {
            SERIAL_PROTOCOLPGM("lx:");
      SERIAL_PROTOCOL(lx);
      SERIAL_PROTOCOLPGM(" ly:");
      SERIAL_PROTOCOL(ly);
      SERIAL_PROTOCOLPGM(" lz:");
      SERIAL_PROTOCOL(lz);
      SERIAL_PROTOCOLPGM(" le:");
      SERIAL_PROTOCOLLN(e);
      #if PLANNER_LEVELING && IS_CARTESIAN
        apply_leveling(lx, ly, lz);
                  SERIAL_PROTOCOLPGM("lx:");
      SERIAL_PROTOCOL(lx);
      SERIAL_PROTOCOLPGM(" ly:");
      SERIAL_PROTOCOL(ly);
      SERIAL_PROTOCOLPGM(" lz:");
      SERIAL_PROTOCOL(lz);
      SERIAL_PROTOCOLPGM(" le:");
      SERIAL_PROTOCOLLN(e);
      #endif
      _buffer_line(lx, ly, lz, e, fr_mm_s, extruder);
            SERIAL_PROTOCOLPGM("lx:");
      SERIAL_PROTOCOL(lx);
      SERIAL_PROTOCOLPGM(" ly:");
      SERIAL_PROTOCOL(ly);
      SERIAL_PROTOCOLPGM(" lz:");
      SERIAL_PROTOCOL(lz);
      SERIAL_PROTOCOLPGM(" le:");
      SERIAL_PROTOCOLLN(e);
    }
    static FORCE_INLINE void buffer_line_kinematic(const float ltarget[XYZE], const float &fr_mm_s, const uint8_t extruder) {
      #if PLANNER_LEVELING
        float lpos[XYZ] = { ltarget[X_AXIS], ltarget[Y_AXIS], ltarget[Z_AXIS] };
        apply_leveling(lpos);
      #else
        const float * const lpos = ltarget;
      #endif
      #if IS_KINEMATIC
        inverse_kinematics(lpos);
        _buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], ltarget[E_AXIS], fr_mm_s, extruder);
      #else
        _buffer_line(lpos[X_AXIS], lpos[Y_AXIS], lpos[Z_AXIS], ltarget[E_AXIS], fr_mm_s, extruder);
      #endif
    }
    static FORCE_INLINE void set_position_mm(ARG_X, ARG_Y, ARG_Z, const float &e) {
      #if PLANNER_LEVELING && IS_CARTESIAN
        apply_leveling(lx, ly, lz);
      #endif
      _set_position_mm(lx, ly, lz, e);
    }
    static FORCE_INLINE void set_position_mm_and_report(ARG_X, ARG_Y, ARG_Z, const float &e) {
      SERIAL_PROTOCOLPGM("lx:");
      SERIAL_PROTOCOL(lx);
      SERIAL_PROTOCOLPGM(" ly:");
      SERIAL_PROTOCOL(ly);
      SERIAL_PROTOCOLPGM(" lz:");
      SERIAL_PROTOCOL(lz);
      SERIAL_PROTOCOLPGM(" le:");
      SERIAL_PROTOCOLLN(e);
      #if PLANNER_LEVELING && IS_CARTESIAN
        apply_leveling(lx, ly, lz);
            SERIAL_PROTOCOLPGM("lx:");
      SERIAL_PROTOCOL(lx);
      SERIAL_PROTOCOLPGM(" ly:");
      SERIAL_PROTOCOL(ly);
      SERIAL_PROTOCOLPGM(" lz:");
      SERIAL_PROTOCOL(lz);
      SERIAL_PROTOCOLPGM(" le:");
      SERIAL_PROTOCOLLN(e);
      #endif
      _set_position_mm(lx, ly, lz, e);
      SERIAL_PROTOCOLPGM("lx:");
      SERIAL_PROTOCOL(lx);
      SERIAL_PROTOCOLPGM(" ly:");
      SERIAL_PROTOCOL(ly);
      SERIAL_PROTOCOLPGM(" lz:");
      SERIAL_PROTOCOL(lz);
      SERIAL_PROTOCOLPGM(" le:");
      SERIAL_PROTOCOLLN(e);
    }
    static void set_position_mm_kinematic(const float position[NUM_AXIS]);
    static void set_position_mm(const AxisEnum axis, const float &v);
    static FORCE_INLINE void set_z_position_mm(const float &z) { set_position_mm(Z_AXIS, z); }
    static FORCE_INLINE void set_e_position_mm(const float &e) { set_position_mm(AxisEnum(E_AXIS), e); }
    static void sync_from_steppers();
    static bool blocks_queued() { return (block_buffer_head != block_buffer_tail); }
    static void discard_current_block() {
        #ifdef FYS_SAFE_PRINT_BREAK
        cli();
        #endif
      if (blocks_queued())
        block_buffer_tail = BLOCK_MOD(block_buffer_tail + 1);
        #ifdef FYS_SAFE_PRINT_BREAK
        sei();
        #endif
    }
    static block_t* get_current_block() {
      if (blocks_queued()) {
        block_t* block = &block_buffer[block_buffer_tail];
        #if ENABLED(ULTRA_LCD)
          block_buffer_runtime_us -= block->segment_time; 
        #endif
        SBI(block->flag, BLOCK_BIT_BUSY);
        return block;
      }
      else {
        #if ENABLED(ULTRA_LCD)
          clear_block_buffer_runtime(); 
        #endif
        return NULL;
      }
    }
    #if ENABLED(ULTRA_LCD)
      static uint16_t block_buffer_runtime() {
        CRITICAL_SECTION_START
          millis_t bbru = block_buffer_runtime_us;
        CRITICAL_SECTION_END
        bbru >>= 10;
        NOMORE(bbru, 0xFFFFul);
        return bbru;
      }
      static void clear_block_buffer_runtime(){
        CRITICAL_SECTION_START
          block_buffer_runtime_us = 0;
        CRITICAL_SECTION_END
      }
    #endif
    #if ENABLED(AUTOTEMP)
      static float autotemp_min, autotemp_max, autotemp_factor;
      static bool autotemp_enabled;
      static void getHighESpeed();
      static void autotemp_M104_M109();
    #endif
  private:
    static int8_t next_block_index(int8_t block_index) { return BLOCK_MOD(block_index + 1); }
    static int8_t prev_block_index(int8_t block_index) { return BLOCK_MOD(block_index - 1); }
    static float estimate_acceleration_distance(const float &initial_rate, const float &target_rate, const float &accel) {
      if (accel == 0) return 0; 
      return (sq(target_rate) - sq(initial_rate)) / (accel * 2);
    }
    static float intersection_distance(const float &initial_rate, const float &final_rate, const float &accel, const float &distance) {
      if (accel == 0) return 0; 
      return (accel * 2 * distance - sq(initial_rate) + sq(final_rate)) / (accel * 4);
    }
    static float max_allowable_speed(const float &accel, const float &target_velocity, const float &distance) {
      return SQRT(sq(target_velocity) - 2 * accel * distance);
    }
    static void calculate_trapezoid_for_block(block_t* const block, const float &entry_factor, const float &exit_factor);
    static void reverse_pass_kernel(block_t* const current, const block_t *next);
    static void forward_pass_kernel(const block_t *previous, block_t* const current);
    static void reverse_pass();
    static void forward_pass();
    static void recalculate_trapezoids();
    static void recalculate();
};
#define PLANNER_XY_FEEDRATE() (min(planner.max_feedrate_mm_s[X_AXIS], planner.max_feedrate_mm_s[Y_AXIS]))
extern Planner planner;
#endif 
