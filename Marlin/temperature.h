#ifndef TEMPERATURE_H
#define TEMPERATURE_H
#include "thermistortables.h"
#include "MarlinConfig.h"
#if ENABLED(PID_EXTRUSION_SCALING)
  #include "stepper.h"
#endif
#if ENABLED(FYS_LCD_EVENT)
#include "ultralcd.h" 
#endif
#ifndef SOFT_PWM_SCALE
  #define SOFT_PWM_SCALE 0
#endif
#define HOTEND_LOOP() for (int8_t e = 0; e < HOTENDS; e++)
#if HOTENDS == 1
  #define HOTEND_INDEX  0
  #define EXTRUDER_IDX  0
#else
  #define HOTEND_INDEX  e
  #define EXTRUDER_IDX  active_extruder
#endif
enum ADCSensorState {
  #if HAS_TEMP_0
    PrepareTemp_0,
    MeasureTemp_0,
  #endif
  #if HAS_TEMP_1
    PrepareTemp_1,
    MeasureTemp_1,
  #endif
  #if HAS_TEMP_2
    PrepareTemp_2,
    MeasureTemp_2,
  #endif
  #if HAS_TEMP_3
    PrepareTemp_3,
    MeasureTemp_3,
  #endif
  #if HAS_TEMP_4
    PrepareTemp_4,
    MeasureTemp_4,
  #endif
  #if HAS_TEMP_BED
    PrepareTemp_BED,
    MeasureTemp_BED,
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    Prepare_FILWIDTH,
    Measure_FILWIDTH,
  #endif
  #if ENABLED(ADC_KEYPAD)
    Prepare_ADC_KEY,
    Measure_ADC_KEY,
  #endif
  SensorsReady, 
  StartupDelay  
};
#define MIN_ADC_ISR_LOOPS 10
#define ACTUAL_ADC_SAMPLES max(int(MIN_ADC_ISR_LOOPS), int(SensorsReady))
#if !HAS_HEATER_BED
  constexpr int16_t target_temperature_bed = 0;
#endif
class Temperature {
  public:
    static float current_temperature[HOTENDS],
                 current_temperature_bed;
    static int16_t current_temperature_raw[HOTENDS],
                   target_temperature[HOTENDS],
                   current_temperature_bed_raw;
    #if HAS_HEATER_BED
      static int16_t target_temperature_bed;
    #endif
    static volatile bool in_temp_isr;
    static uint8_t soft_pwm_amount[HOTENDS],
                   soft_pwm_amount_bed;
    #if ENABLED(FAN_SOFT_PWM)
      static uint8_t soft_pwm_amount_fan[FAN_COUNT],
                     soft_pwm_count_fan[FAN_COUNT];
    #endif
    #if ENABLED(PIDTEMP) || ENABLED(PIDTEMPBED)
      #define PID_dT ((OVERSAMPLENR * float(ACTUAL_ADC_SAMPLES)) / (F_CPU / 64.0 / 256.0))
    #endif
    #if ENABLED(PIDTEMP)
      #if ENABLED(PID_PARAMS_PER_HOTEND) && HOTENDS > 1
        static float Kp[HOTENDS], Ki[HOTENDS], Kd[HOTENDS];
        #if ENABLED(PID_EXTRUSION_SCALING)
          static float Kc[HOTENDS];
        #endif
        #define PID_PARAM(param, h) Temperature::param[h]
      #else
        static float Kp, Ki, Kd;
        #if ENABLED(PID_EXTRUSION_SCALING)
          static float Kc;
        #endif
        #define PID_PARAM(param, h) Temperature::param
      #endif 
      #define scalePID_i(i)   ( (i) * PID_dT )
      #define unscalePID_i(i) ( (i) / PID_dT )
      #define scalePID_d(d)   ( (d) / PID_dT )
      #define unscalePID_d(d) ( (d) * PID_dT )
    #endif
    #if ENABLED(PIDTEMPBED)
      static float bedKp, bedKi, bedKd;
    #endif
    #if ENABLED(BABYSTEPPING)
      static volatile int babystepsTodo[3];
    #endif
    #if WATCH_HOTENDS
      static uint16_t watch_target_temp[HOTENDS];
      static millis_t watch_heater_next_ms[HOTENDS];
    #endif
    #if WATCH_THE_BED
      static uint16_t watch_target_bed_temp;
      static millis_t watch_bed_next_ms;
    #endif
    #if ENABLED(PREVENT_COLD_EXTRUSION)
      static bool allow_cold_extrude;
      static int16_t extrude_min_temp;
      static bool tooColdToExtrude(uint8_t e) {
        #if HOTENDS == 1
          UNUSED(e);
        #endif
        return allow_cold_extrude ? false : degHotend(HOTEND_INDEX) < extrude_min_temp;
      }
    #else
      static bool tooColdToExtrude(uint8_t e) { UNUSED(e); return false; }
    #endif
  private:
    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      static uint16_t redundant_temperature_raw;
      static float redundant_temperature;
    #endif
    static volatile bool temp_meas_ready;
    #if ENABLED(PIDTEMP)
      static float temp_iState[HOTENDS],
                   temp_dState[HOTENDS],
                   pTerm[HOTENDS],
                   iTerm[HOTENDS],
                   dTerm[HOTENDS];
      #if ENABLED(PID_EXTRUSION_SCALING)
        static float cTerm[HOTENDS];
        static long last_e_position;
        static long lpq[LPQ_MAX_LEN];
        static int lpq_ptr;
      #endif
      static float pid_error[HOTENDS];
      static bool pid_reset[HOTENDS];
    #endif
    #if ENABLED(PIDTEMPBED)
      static float temp_iState_bed,
                   temp_dState_bed,
                   pTerm_bed,
                   iTerm_bed,
                   dTerm_bed,
                   pid_error_bed;
    #else
      static millis_t next_bed_check_ms;
    #endif
    static uint16_t raw_temp_value[MAX_EXTRUDERS],
                    raw_temp_bed_value;
    static int16_t minttemp_raw[HOTENDS],
                   maxttemp_raw[HOTENDS],
                   minttemp[HOTENDS],
                   maxttemp[HOTENDS];
    #ifdef MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED
      static uint8_t consecutive_low_temperature_error[HOTENDS];
    #endif
    #ifdef MILLISECONDS_PREHEAT_TIME
      static millis_t preheat_end_time[HOTENDS];
    #endif
    #ifdef BED_MINTEMP
      static int16_t bed_minttemp_raw;
    #endif
    #ifdef BED_MAXTEMP
      static int16_t bed_maxttemp_raw;
    #endif
    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      static int8_t meas_shift_index;  
    #endif
    #if HAS_AUTO_FAN
      static millis_t next_auto_fan_check_ms;
    #endif
    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      static uint16_t current_raw_filwidth; 
    #endif
    #if ENABLED(PROBING_HEATERS_OFF)
      static bool paused;
    #endif
    #if HEATER_IDLE_HANDLER
      static millis_t heater_idle_timeout_ms[HOTENDS];
      static bool heater_idle_timeout_exceeded[HOTENDS];
      #if HAS_TEMP_BED
        static millis_t bed_idle_timeout_ms;
        static bool bed_idle_timeout_exceeded;
      #endif
    #endif
  public:
    #if ENABLED(ADC_KEYPAD)
      static uint32_t current_ADCKey_raw;
      static uint8_t ADCKey_count;
    #endif
    Temperature();
    void init();
    static float analog2temp(int raw, uint8_t e);
    static float analog2tempBed(int raw);
    static void isr();
    static void manage_heater() _O2; 
    #ifdef MILLISECONDS_PREHEAT_TIME
      static bool is_preheating(uint8_t e) {
        #if HOTENDS == 1
          UNUSED(e);
        #endif
        return preheat_end_time[HOTEND_INDEX] && PENDING(millis(), preheat_end_time[HOTEND_INDEX]);
      }
      static void start_preheat_time(uint8_t e) {
        #if HOTENDS == 1
          UNUSED(e);
        #endif
        preheat_end_time[HOTEND_INDEX] = millis() + MILLISECONDS_PREHEAT_TIME;
      }
      static void reset_preheat_time(uint8_t e) {
        #if HOTENDS == 1
          UNUSED(e);
        #endif
        preheat_end_time[HOTEND_INDEX] = 0;
      }
    #else
      #define is_preheating(n) (false)
    #endif
    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      static float analog2widthFil(); 
      static int widthFil_to_size_ratio(); 
    #endif
    static float degHotend(uint8_t e) {
      #if HOTENDS == 1
        UNUSED(e);
      #endif
      return current_temperature[HOTEND_INDEX];
    }
    static float degBed() { return current_temperature_bed; }
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      static int16_t rawHotendTemp(uint8_t e) {
        #if HOTENDS == 1
          UNUSED(e);
        #endif
        return current_temperature_raw[HOTEND_INDEX];
      }
      static int16_t rawBedTemp() { return current_temperature_bed_raw; }
    #endif
    static int16_t degTargetHotend(uint8_t e) {
      #if HOTENDS == 1
        UNUSED(e);
      #endif
      return target_temperature[HOTEND_INDEX];
    }
    static int16_t degTargetBed() { return target_temperature_bed; }
    #if WATCH_HOTENDS
      static void start_watching_heater(uint8_t e = 0);
    #endif
    #if WATCH_THE_BED
      static void start_watching_bed();
    #endif
    static void setTargetHotend(const int16_t celsius, uint8_t e) {
      #if HOTENDS == 1
        UNUSED(e);
      #endif
      #ifdef MILLISECONDS_PREHEAT_TIME
        if (celsius == 0)
          reset_preheat_time(HOTEND_INDEX);
        else if (target_temperature[HOTEND_INDEX] == 0)
          start_preheat_time(HOTEND_INDEX);
      #endif
      target_temperature[HOTEND_INDEX] = celsius;
      #if WATCH_HOTENDS
        start_watching_heater(HOTEND_INDEX);
      #endif
	  #if ENABLED(FYS_LCD_EVENT)
        GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V004);
	  #endif
    }
    static void setTargetBed(const int16_t celsius) {
      #if HAS_HEATER_BED
        target_temperature_bed =
          #ifdef BED_MAXTEMP
            min(celsius, BED_MAXTEMP)
          #else
            celsius
          #endif
        ;
        #if WATCH_THE_BED
          start_watching_bed();
        #endif
        #if ENABLED(FYS_LCD_EVENT)
          GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V004);
        #endif
      #endif
    }
    static bool isHeatingHotend(uint8_t e) {
      #if HOTENDS == 1
        UNUSED(e);
      #endif
      return target_temperature[HOTEND_INDEX] > current_temperature[HOTEND_INDEX];
    }
    static bool isHeatingBed() { return target_temperature_bed > current_temperature_bed; }
    static bool isCoolingHotend(uint8_t e) {
      #if HOTENDS == 1
        UNUSED(e);
      #endif
      return target_temperature[HOTEND_INDEX] < current_temperature[HOTEND_INDEX];
    }
    static bool isCoolingBed() { return target_temperature_bed < current_temperature_bed; }
    static int getHeaterPower(int heater);
    static void disable_all_heaters();
    #if HAS_PID_HEATING
      static void PID_autotune(float temp, int hotend, int ncycles, bool set_result=false);
    #endif
    static void updatePID();
    #if ENABLED(BABYSTEPPING)
      static void babystep_axis(const AxisEnum axis, const int distance) {
        if (axis_known_position[axis]) {
          #if IS_CORE
            #if ENABLED(BABYSTEP_XY)
              switch (axis) {
                case CORE_AXIS_1: 
                  babystepsTodo[CORE_AXIS_1] += distance * 2;
                  babystepsTodo[CORE_AXIS_2] += distance * 2;
                  break;
                case CORE_AXIS_2: 
                  babystepsTodo[CORE_AXIS_1] += CORESIGN(distance * 2);
                  babystepsTodo[CORE_AXIS_2] -= CORESIGN(distance * 2);
                  break;
                case NORMAL_AXIS: 
                  babystepsTodo[NORMAL_AXIS] += distance;
                  break;
              }
            #elif CORE_IS_XZ || CORE_IS_YZ
              babystepsTodo[CORE_AXIS_1] += CORESIGN(distance * 2);
              babystepsTodo[CORE_AXIS_2] -= CORESIGN(distance * 2);
            #else
              babystepsTodo[Z_AXIS] += distance;
            #endif
          #else
            babystepsTodo[axis] += distance;
          #endif
        }
        else
        {
        }
      }
      #if ENABLED(FYS_BABYSTEP_AXIS_FORCE)
        static void babystep_axis_force(const AxisEnum axis, const int distance) {
            #if IS_CORE
              #if ENABLED(BABYSTEP_XY)
                switch (axis) {
                  case CORE_AXIS_1: 
                    babystepsTodo[CORE_AXIS_1] += distance * 2;
                    babystepsTodo[CORE_AXIS_2] += distance * 2;
                    break;
                  case CORE_AXIS_2: 
                    babystepsTodo[CORE_AXIS_1] += CORESIGN(distance * 2);
                    babystepsTodo[CORE_AXIS_2] -= CORESIGN(distance * 2);
                    break;
                  case NORMAL_AXIS: 
                    babystepsTodo[NORMAL_AXIS] += distance;
                    break;
                }
              #elif CORE_IS_XZ || CORE_IS_YZ
                babystepsTodo[CORE_AXIS_1] += CORESIGN(distance * 2);
                babystepsTodo[CORE_AXIS_2] -= CORESIGN(distance * 2);
              #else
                babystepsTodo[Z_AXIS] += distance;
              #endif
            #else
              babystepsTodo[axis] += distance;
            #endif
        }
      #endif
    #endif 
    #if ENABLED(PROBING_HEATERS_OFF)
      static void pause(const bool p);
      static bool is_paused() { return paused; }
    #endif
    #if HEATER_IDLE_HANDLER
      static void start_heater_idle_timer(uint8_t e, millis_t timeout_ms) {
        #if HOTENDS == 1
          UNUSED(e);
        #endif
        heater_idle_timeout_ms[HOTEND_INDEX] = millis() + timeout_ms;
        heater_idle_timeout_exceeded[HOTEND_INDEX] = false;
      }
      static void reset_heater_idle_timer(uint8_t e) {
        #if HOTENDS == 1
          UNUSED(e);
        #endif
        heater_idle_timeout_ms[HOTEND_INDEX] = 0;
        heater_idle_timeout_exceeded[HOTEND_INDEX] = false;
        #if WATCH_HOTENDS
          start_watching_heater(HOTEND_INDEX);
        #endif
      }
      static bool is_heater_idle(uint8_t e) {
        #if HOTENDS == 1
          UNUSED(e);
        #endif
        return heater_idle_timeout_exceeded[HOTEND_INDEX];
      }
      #if HAS_TEMP_BED
        static void start_bed_idle_timer(millis_t timeout_ms) {
          bed_idle_timeout_ms = millis() + timeout_ms;
          bed_idle_timeout_exceeded = false;
        }
        static void reset_bed_idle_timer() {
          bed_idle_timeout_ms = 0;
          bed_idle_timeout_exceeded = false;
          #if WATCH_THE_BED
            start_watching_bed();
          #endif
        }
        static bool is_bed_idle() {
          return bed_idle_timeout_exceeded;
        }
      #endif
    #endif
  private:
    static void set_current_temp_raw();
    static void updateTemperaturesFromRawValues();
    #if ENABLED(HEATER_0_USES_MAX6675)
      static int read_max6675();
    #endif
    static void checkExtruderAutoFans();
    static float get_pid_output(const int8_t e);
    #if ENABLED(PIDTEMPBED)
      static float get_pid_output_bed();
    #endif
    static void _temp_error(const int8_t e, const char * const serial_msg, const char * const lcd_msg);
    static void min_temp_error(const int8_t e);
    static void max_temp_error(const int8_t e);
    #if ENABLED(THERMAL_PROTECTION_HOTENDS) || HAS_THERMALLY_PROTECTED_BED
      typedef enum TRState { TRInactive, TRFirstHeating, TRStable, TRRunaway } TRstate;
      static void thermal_runaway_protection(TRState* state, millis_t* timer, float temperature, float target_temperature, int heater_id, int period_seconds, int hysteresis_degc);
      #if ENABLED(THERMAL_PROTECTION_HOTENDS)
        static TRState thermal_runaway_state_machine[HOTENDS];
        static millis_t thermal_runaway_timer[HOTENDS];
      #endif
      #if HAS_THERMALLY_PROTECTED_BED
        static TRState thermal_runaway_bed_state_machine;
        static millis_t thermal_runaway_bed_timer;
      #endif
    #endif 
};
extern Temperature thermalManager;
#endif 
