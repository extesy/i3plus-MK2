#include "Marlin.h"
#include "temperature.h"
#include "thermistortables.h"
#include "ultralcd.h"
#include "planner.h"
#include "language.h"
#ifdef FYS_CONTROL_PROTOCOL
#include "fysControlProtocol.h"
#endif
#if ENABLED(HEATER_0_USES_MAX6675)
  #include "spi.h"
#endif
#if ENABLED(BABYSTEPPING)
  #include "stepper.h"
#endif
#if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
  #include "endstops.h"
#endif
#if ENABLED(USE_WATCHDOG)
  #include "watchdog.h"
#endif
#ifdef K1 
  #define K2 (1.0-K1)
#endif
#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  static void* heater_ttbl_map[2] = { (void*)HEATER_0_TEMPTABLE, (void*)HEATER_1_TEMPTABLE };
  static uint8_t heater_ttbllen_map[2] = { HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN };
#else
  static void* heater_ttbl_map[HOTENDS] = ARRAY_BY_HOTENDS((void*)HEATER_0_TEMPTABLE, (void*)HEATER_1_TEMPTABLE, (void*)HEATER_2_TEMPTABLE, (void*)HEATER_3_TEMPTABLE, (void*)HEATER_4_TEMPTABLE);
  static uint8_t heater_ttbllen_map[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN, HEATER_3_TEMPTABLE_LEN, HEATER_4_TEMPTABLE_LEN);
#endif
Temperature thermalManager;
float Temperature::current_temperature[HOTENDS] = { 0.0 },
      Temperature::current_temperature_bed = 0.0;
int16_t Temperature::current_temperature_raw[HOTENDS] = { 0 },
        Temperature::target_temperature[HOTENDS] = { 0 },
        Temperature::current_temperature_bed_raw = 0;
#if HAS_HEATER_BED
  int16_t Temperature::target_temperature_bed = 0;
#endif
#if ENABLED(PIDTEMP)
  #if ENABLED(PID_PARAMS_PER_HOTEND) && HOTENDS > 1
    float Temperature::Kp[HOTENDS] = ARRAY_BY_HOTENDS1(DEFAULT_Kp),
          Temperature::Ki[HOTENDS] = ARRAY_BY_HOTENDS1((DEFAULT_Ki) * (PID_dT)),
          Temperature::Kd[HOTENDS] = ARRAY_BY_HOTENDS1((DEFAULT_Kd) / (PID_dT));
    #if ENABLED(PID_EXTRUSION_SCALING)
      float Temperature::Kc[HOTENDS] = ARRAY_BY_HOTENDS1(DEFAULT_Kc);
    #endif
  #else
    float Temperature::Kp = DEFAULT_Kp,
          Temperature::Ki = (DEFAULT_Ki) * (PID_dT),
          Temperature::Kd = (DEFAULT_Kd) / (PID_dT);
    #if ENABLED(PID_EXTRUSION_SCALING)
      float Temperature::Kc = DEFAULT_Kc;
    #endif
  #endif
#endif
#if ENABLED(PIDTEMPBED)
  float Temperature::bedKp = DEFAULT_bedKp,
        Temperature::bedKi = ((DEFAULT_bedKi) * PID_dT),
        Temperature::bedKd = ((DEFAULT_bedKd) / PID_dT);
#endif
#if ENABLED(BABYSTEPPING)
  volatile int Temperature::babystepsTodo[XYZ] = { 0 };
#endif
#if WATCH_HOTENDS
  uint16_t Temperature::watch_target_temp[HOTENDS] = { 0 };
  millis_t Temperature::watch_heater_next_ms[HOTENDS] = { 0 };
#endif
#if WATCH_THE_BED
  uint16_t Temperature::watch_target_bed_temp = 0;
  millis_t Temperature::watch_bed_next_ms = 0;
#endif
#if ENABLED(PREVENT_COLD_EXTRUSION)
  bool Temperature::allow_cold_extrude = false;
  int16_t Temperature::extrude_min_temp = EXTRUDE_MINTEMP;
#endif
#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  uint16_t Temperature::redundant_temperature_raw = 0;
  float Temperature::redundant_temperature = 0.0;
#endif
volatile bool Temperature::temp_meas_ready = false;
#if ENABLED(PIDTEMP)
  float Temperature::temp_iState[HOTENDS] = { 0 },
        Temperature::temp_dState[HOTENDS] = { 0 },
        Temperature::pTerm[HOTENDS],
        Temperature::iTerm[HOTENDS],
        Temperature::dTerm[HOTENDS];
  #if ENABLED(PID_EXTRUSION_SCALING)
    float Temperature::cTerm[HOTENDS];
    long Temperature::last_e_position;
    long Temperature::lpq[LPQ_MAX_LEN];
    int Temperature::lpq_ptr = 0;
  #endif
  float Temperature::pid_error[HOTENDS];
  bool Temperature::pid_reset[HOTENDS];
#endif
#if ENABLED(PIDTEMPBED)
  float Temperature::temp_iState_bed = { 0 },
        Temperature::temp_dState_bed = { 0 },
        Temperature::pTerm_bed,
        Temperature::iTerm_bed,
        Temperature::dTerm_bed,
        Temperature::pid_error_bed;
#else
  millis_t Temperature::next_bed_check_ms;
#endif
uint16_t Temperature::raw_temp_value[MAX_EXTRUDERS] = { 0 },
         Temperature::raw_temp_bed_value = 0;
int16_t Temperature::minttemp_raw[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP, HEATER_3_RAW_LO_TEMP, HEATER_4_RAW_LO_TEMP),
        Temperature::maxttemp_raw[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP, HEATER_3_RAW_HI_TEMP, HEATER_4_RAW_HI_TEMP),
        Temperature::minttemp[HOTENDS] = { 0 },
        Temperature::maxttemp[HOTENDS] = ARRAY_BY_HOTENDS1(16383);
#ifdef MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED
  uint8_t Temperature::consecutive_low_temperature_error[HOTENDS] = { 0 };
#endif
#ifdef MILLISECONDS_PREHEAT_TIME
  millis_t Temperature::preheat_end_time[HOTENDS] = { 0 };
#endif
#ifdef BED_MINTEMP
  int16_t Temperature::bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP;
#endif
#ifdef BED_MAXTEMP
  int16_t Temperature::bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif
#if ENABLED(FILAMENT_WIDTH_SENSOR)
  int8_t Temperature::meas_shift_index;  
#endif
#if HAS_AUTO_FAN
  millis_t Temperature::next_auto_fan_check_ms = 0;
#endif
uint8_t Temperature::soft_pwm_amount[HOTENDS],
        Temperature::soft_pwm_amount_bed;
#if ENABLED(FAN_SOFT_PWM)
  uint8_t Temperature::soft_pwm_amount_fan[FAN_COUNT],
          Temperature::soft_pwm_count_fan[FAN_COUNT];
#endif
#if ENABLED(FILAMENT_WIDTH_SENSOR)
  uint16_t Temperature::current_raw_filwidth = 0; 
#endif
#if ENABLED(PROBING_HEATERS_OFF)
  bool Temperature::paused;
#endif
#if HEATER_IDLE_HANDLER
  millis_t Temperature::heater_idle_timeout_ms[HOTENDS] = { 0 };
  bool Temperature::heater_idle_timeout_exceeded[HOTENDS] = { false };
  #if HAS_TEMP_BED
    millis_t Temperature::bed_idle_timeout_ms = 0;
    bool Temperature::bed_idle_timeout_exceeded = false;
  #endif
#endif
#if ENABLED(ADC_KEYPAD)
  uint32_t Temperature::current_ADCKey_raw = 0;
  uint8_t Temperature::ADCKey_count = 0;
#endif
#if HAS_PID_HEATING
  void Temperature::PID_autotune(float temp, int hotend, int ncycles, bool set_result) {
    float input = 0.0;
    int cycles = 0;
    bool heating = true;
    millis_t temp_ms = millis(), t1 = temp_ms, t2 = temp_ms;
    long t_high = 0, t_low = 0;
    long bias, d;
    float Ku, Tu;
    float workKp = 0, workKi = 0, workKd = 0;
    float max = 0, min = 10000;
    #if HAS_AUTO_FAN
      next_auto_fan_check_ms = temp_ms + 2500UL;
    #endif
    if (hotend >=
        #if ENABLED(PIDTEMP)
          HOTENDS
        #else
          0
        #endif
      || hotend <
        #if ENABLED(PIDTEMPBED)
          -1
        #else
          0
        #endif
    ) {
      SERIAL_ECHOLN(MSG_PID_BAD_EXTRUDER_NUM);
      return;
    }
    SERIAL_ECHOLN(MSG_PID_AUTOTUNE_START);
    disable_all_heaters(); 
    #if HAS_PID_FOR_BOTH
      if (hotend < 0)
        soft_pwm_amount_bed = bias = d = (MAX_BED_POWER) >> 1;
      else
        soft_pwm_amount[hotend] = bias = d = (PID_MAX) >> 1;
    #elif ENABLED(PIDTEMP)
      soft_pwm_amount[hotend] = bias = d = (PID_MAX) >> 1;
    #else
      soft_pwm_amount_bed = bias = d = (MAX_BED_POWER) >> 1;
    #endif
    wait_for_heatup = true;
    while (wait_for_heatup) {
      millis_t ms = millis();
      if (temp_meas_ready) { 
        updateTemperaturesFromRawValues();
        input =
          #if HAS_PID_FOR_BOTH
            hotend < 0 ? current_temperature_bed : current_temperature[hotend]
          #elif ENABLED(PIDTEMP)
            current_temperature[hotend]
          #else
            current_temperature_bed
          #endif
        ;
        NOLESS(max, input);
        NOMORE(min, input);
        #if HAS_AUTO_FAN
          if (ELAPSED(ms, next_auto_fan_check_ms)) {
            checkExtruderAutoFans();
            next_auto_fan_check_ms = ms + 2500UL;
          }
        #endif
        if (heating && input > temp) {
          if (ELAPSED(ms, t2 + 5000UL)) {
            heating = false;
            #if HAS_PID_FOR_BOTH
              if (hotend < 0)
                soft_pwm_amount_bed = (bias - d) >> 1;
              else
                soft_pwm_amount[hotend] = (bias - d) >> 1;
            #elif ENABLED(PIDTEMP)
              soft_pwm_amount[hotend] = (bias - d) >> 1;
            #elif ENABLED(PIDTEMPBED)
              soft_pwm_amount_bed = (bias - d) >> 1;
            #endif
            t1 = ms;
            t_high = t1 - t2;
            max = temp;
          }
        }
        if (!heating && input < temp) {
          if (ELAPSED(ms, t1 + 5000UL)) {
            heating = true;
            t2 = ms;
            t_low = t2 - t1;
            if (cycles > 0) {
              long max_pow =
                #if HAS_PID_FOR_BOTH
                  hotend < 0 ? MAX_BED_POWER : PID_MAX
                #elif ENABLED(PIDTEMP)
                  PID_MAX
                #else
                  MAX_BED_POWER
                #endif
              ;
              bias += (d * (t_high - t_low)) / (t_low + t_high);
              bias = constrain(bias, 20, max_pow - 20);
              d = (bias > max_pow / 2) ? max_pow - 1 - bias : bias;
              SERIAL_PROTOCOLPAIR(MSG_BIAS, bias);
              SERIAL_PROTOCOLPAIR(MSG_D, d);
              SERIAL_PROTOCOLPAIR(MSG_T_MIN, min);
              SERIAL_PROTOCOLPAIR(MSG_T_MAX, max);
              if (cycles > 2) {
                Ku = (4.0 * d) / (M_PI * (max - min) * 0.5);
                Tu = ((float)(t_low + t_high) * 0.001);
                SERIAL_PROTOCOLPAIR(MSG_KU, Ku);
                SERIAL_PROTOCOLPAIR(MSG_TU, Tu);
                workKp = 0.6 * Ku;
                workKi = 2 * workKp / Tu;
                workKd = workKp * Tu * 0.125;
                SERIAL_PROTOCOLLNPGM("\n" MSG_CLASSIC_PID);
                SERIAL_PROTOCOLPAIR(MSG_KP, workKp);
                SERIAL_PROTOCOLPAIR(MSG_KI, workKi);
                SERIAL_PROTOCOLLNPAIR(MSG_KD, workKd);
              }
            }
            #if HAS_PID_FOR_BOTH
              if (hotend < 0)
                soft_pwm_amount_bed = (bias + d) >> 1;
              else
                soft_pwm_amount[hotend] = (bias + d) >> 1;
            #elif ENABLED(PIDTEMP)
              soft_pwm_amount[hotend] = (bias + d) >> 1;
            #else
              soft_pwm_amount_bed = (bias + d) >> 1;
            #endif
            cycles++;
            min = temp;
          }
        }
      }
      #define MAX_OVERSHOOT_PID_AUTOTUNE 30
      if (input > temp + MAX_OVERSHOOT_PID_AUTOTUNE) {
          SERIAL_PROTOCOLLNPGM(MSG_PID_TEMP_TOO_HIGH);
          #if ENABLED(FYS_LCD_EVENT)
            GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V05F);            
          #endif
          #if ENABLED(FYS_LCD_EXTRA_INFO)
            FunV006(MSG_PID_TEMP_TOO_HIGH);
          #endif
        return;
      }
      if (ELAPSED(ms, temp_ms + 2000UL)) {
        #if HAS_TEMP_HOTEND || HAS_TEMP_BED
          print_heaterstates();
          SERIAL_EOL();
        #endif
        temp_ms = ms;
      } 
      if (((ms - t1) + (ms - t2)) > (10L * 60L * 1000L * 2L)) {
        SERIAL_PROTOCOLLNPGM(MSG_PID_TIMEOUT);
        #if ENABLED(FYS_LCD_EVENT)
          GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V05F);
        #endif
        #if ENABLED(FYS_LCD_EXTRA_INFO)
          FunV006(MSG_PID_TIMEOUT);
        #endif
        return;
      }
      if (cycles > ncycles) {
        SERIAL_PROTOCOLLNPGM(MSG_PID_AUTOTUNE_FINISHED);
        #if HAS_PID_FOR_BOTH
          const char* estring = hotend < 0 ? "bed" : "";
          SERIAL_PROTOCOLPAIR("#define  DEFAULT_", estring); SERIAL_PROTOCOLPAIR("Kp ", workKp); SERIAL_EOL();
          SERIAL_PROTOCOLPAIR("#define  DEFAULT_", estring); SERIAL_PROTOCOLPAIR("Ki ", workKi); SERIAL_EOL();
          SERIAL_PROTOCOLPAIR("#define  DEFAULT_", estring); SERIAL_PROTOCOLPAIR("Kd ", workKd); SERIAL_EOL();
        #elif ENABLED(PIDTEMP)
          SERIAL_PROTOCOLPAIR("#define  DEFAULT_Kp ", workKp); SERIAL_EOL();
          SERIAL_PROTOCOLPAIR("#define  DEFAULT_Ki ", workKi); SERIAL_EOL();
          SERIAL_PROTOCOLPAIR("#define  DEFAULT_Kd ", workKd); SERIAL_EOL();
        #else
          SERIAL_PROTOCOLPAIR("#define  DEFAULT_bedKp ", workKp); SERIAL_EOL();
          SERIAL_PROTOCOLPAIR("#define  DEFAULT_bedKi ", workKi); SERIAL_EOL();
          SERIAL_PROTOCOLPAIR("#define  DEFAULT_bedKd ", workKd); SERIAL_EOL();
        #endif
        #define _SET_BED_PID() do { \
          bedKp = workKp; \
          bedKi = scalePID_i(workKi); \
          bedKd = scalePID_d(workKd); \
          updatePID(); }while(0)
        #define _SET_EXTRUDER_PID() do { \
          PID_PARAM(Kp, hotend) = workKp; \
          PID_PARAM(Ki, hotend) = scalePID_i(workKi); \
          PID_PARAM(Kd, hotend) = scalePID_d(workKd); \
          updatePID(); }while(0)
        if (set_result) {
          #if HAS_PID_FOR_BOTH
            if (hotend < 0)
              _SET_BED_PID();
            else
              _SET_EXTRUDER_PID();
          #elif ENABLED(PIDTEMP)
            _SET_EXTRUDER_PID();
          #else
            _SET_BED_PID();
          #endif
        }
        #if ENABLED(FYS_LCD_EVENT)
          GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V05F);
        #endif
        return;
      }
      lcd_update();
      #ifdef FYS_CONTROL_PROTOCOL
        fysControlProtocol();
      #endif
    }
    if (!wait_for_heatup) disable_all_heaters();
    #if ENABLED(FYS_LCD_EVENT)
      GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V05F);
    #endif
  }
#endif 
Temperature::Temperature() { }
void Temperature::updatePID() {
  #if ENABLED(PIDTEMP)
    #if ENABLED(PID_EXTRUSION_SCALING)
      last_e_position = 0;
    #endif
  #endif
}
int Temperature::getHeaterPower(int heater) {
  return heater < 0 ? soft_pwm_amount_bed : soft_pwm_amount[heater];
}
#if HAS_AUTO_FAN
  void Temperature::checkExtruderAutoFans() {
    static const int8_t fanPin[] PROGMEM = { E0_AUTO_FAN_PIN, E1_AUTO_FAN_PIN, E2_AUTO_FAN_PIN, E3_AUTO_FAN_PIN, E4_AUTO_FAN_PIN };
    static const uint8_t fanBit[] PROGMEM = {
                    0,
      AUTO_1_IS_0 ? 0 :               1,
      AUTO_2_IS_0 ? 0 : AUTO_2_IS_1 ? 1 :               2,
      AUTO_3_IS_0 ? 0 : AUTO_3_IS_1 ? 1 : AUTO_3_IS_2 ? 2 :               3,
      AUTO_4_IS_0 ? 0 : AUTO_4_IS_1 ? 1 : AUTO_4_IS_2 ? 2 : AUTO_4_IS_3 ? 3 : 4
    };
    uint8_t fanState = 0;
    HOTEND_LOOP()
      if (current_temperature[e] > EXTRUDER_AUTO_FAN_TEMPERATURE)
        SBI(fanState, pgm_read_byte(&fanBit[e]));
    uint8_t fanDone = 0;
    for (uint8_t f = 0; f < COUNT(fanPin); f++) {
      int8_t pin = pgm_read_byte(&fanPin[f]);
      const uint8_t bit = pgm_read_byte(&fanBit[f]);
      if (pin >= 0 && !TEST(fanDone, bit)) {
        uint8_t newFanSpeed = TEST(fanState, bit) ? EXTRUDER_AUTO_FAN_SPEED : 0;
        digitalWrite(pin, newFanSpeed);
        analogWrite(pin, newFanSpeed);
        SBI(fanDone, bit);
      }
    }
  }
#endif 
void Temperature::_temp_error(const int8_t e, const char * const serial_msg, const char * const lcd_msg) {
  static bool killed = false;
  if (IsRunning()) {
    SERIAL_ERROR_START();
    serialprintPGM(serial_msg);
    SERIAL_ERRORPGM(MSG_STOPPED_HEATER);
    #if ENABLED(FYS_POWER_STATUS)
      #if PIN_EXISTS(POW_BREAK_CHECK)||PIN_EXISTS(SHUTDOWN_CHECK)||PIN_EXISTS(PS_ON) 
      if (GLOBAL_var_V003 != MACRO_var_V00B)
      {
          if (e >= 0)
          {
              SERIAL_ERRORLN((int)e);
              dwin_popup(PSTR("Extruder heat fail.\nSystem stop.\nPlease restart machine."),1);
          }
          else
          {
              SERIAL_ERRORLNPGM(MSG_HEATER_BED);
              dwin_popup(PSTR("Bed heat fail.\nSystem stop.\nPlease restart machine."),1);
          }
      }
      #endif
    #endif
  }
  #if DISABLED(BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE)
    if (!killed) {
      Running = false;
      killed = true;
      #if ENABLED(FYS_NO_KILL_ONLY_TEMP_LCD_INFO)
        kill_temp(lcd_msg);
      #else
        kill(lcd_msg);
      #endif
    }
    else
      disable_all_heaters(); 
  #endif
}
void Temperature::max_temp_error(const int8_t e) {
  #if HAS_TEMP_BED
    _temp_error(e, PSTR(MSG_T_MAXTEMP), e >= 0 ? PSTR(MSG_ERR_MAXTEMP) : PSTR(MSG_ERR_MAXTEMP_BED));
  #else
    _temp_error(HOTEND_INDEX, PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP));
    #if HOTENDS == 1
      UNUSED(e);
    #endif
  #endif
}
void Temperature::min_temp_error(const int8_t e) {
  #ifndef FYS_IGNORE_MIN_TEMP
  #if HAS_TEMP_BED
    _temp_error(e, PSTR(MSG_T_MINTEMP), e >= 0 ? PSTR(MSG_ERR_MINTEMP) : PSTR(MSG_ERR_MINTEMP_BED));
  #else
    _temp_error(HOTEND_INDEX, PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP));
    #if HOTENDS == 1
      UNUSED(e);
    #endif
  #endif
  #endif
}
float Temperature::get_pid_output(const int8_t e) {
  #if HOTENDS == 1
    UNUSED(e);
    #define _HOTEND_TEST     true
  #else
    #define _HOTEND_TEST     e == active_extruder
  #endif
  float pid_output;
  #if ENABLED(PIDTEMP)
    #if DISABLED(PID_OPENLOOP)
      pid_error[HOTEND_INDEX] = target_temperature[HOTEND_INDEX] - current_temperature[HOTEND_INDEX];
      dTerm[HOTEND_INDEX] = K2 * PID_PARAM(Kd, HOTEND_INDEX) * (current_temperature[HOTEND_INDEX] - temp_dState[HOTEND_INDEX]) + K1 * dTerm[HOTEND_INDEX];
      temp_dState[HOTEND_INDEX] = current_temperature[HOTEND_INDEX];
      #if HEATER_IDLE_HANDLER
        if (heater_idle_timeout_exceeded[HOTEND_INDEX]) {
          pid_output = 0;
          pid_reset[HOTEND_INDEX] = true;
        }
        else
      #endif
      if (pid_error[HOTEND_INDEX] > PID_FUNCTIONAL_RANGE) {
        pid_output = BANG_MAX;
        pid_reset[HOTEND_INDEX] = true;
      }
      else if (pid_error[HOTEND_INDEX] < -(PID_FUNCTIONAL_RANGE) || target_temperature[HOTEND_INDEX] == 0
        #if HEATER_IDLE_HANDLER
          || heater_idle_timeout_exceeded[HOTEND_INDEX]
        #endif
        ) {
        pid_output = 0;
        pid_reset[HOTEND_INDEX] = true;
      }
      else {
        if (pid_reset[HOTEND_INDEX]) {
          temp_iState[HOTEND_INDEX] = 0.0;
          pid_reset[HOTEND_INDEX] = false;
        }
        pTerm[HOTEND_INDEX] = PID_PARAM(Kp, HOTEND_INDEX) * pid_error[HOTEND_INDEX];
        temp_iState[HOTEND_INDEX] += pid_error[HOTEND_INDEX];
        iTerm[HOTEND_INDEX] = PID_PARAM(Ki, HOTEND_INDEX) * temp_iState[HOTEND_INDEX];
        pid_output = pTerm[HOTEND_INDEX] + iTerm[HOTEND_INDEX] - dTerm[HOTEND_INDEX];
        #if ENABLED(PID_EXTRUSION_SCALING)
          cTerm[HOTEND_INDEX] = 0;
          if (_HOTEND_TEST) {
            long e_position = stepper.position(E_AXIS);
            if (e_position > last_e_position) {
              lpq[lpq_ptr] = e_position - last_e_position;
              last_e_position = e_position;
            }
            else {
              lpq[lpq_ptr] = 0;
            }
            if (++lpq_ptr >= lpq_len) lpq_ptr = 0;
            cTerm[HOTEND_INDEX] = (lpq[lpq_ptr] * planner.steps_to_mm[E_AXIS]) * PID_PARAM(Kc, HOTEND_INDEX);
            pid_output += cTerm[HOTEND_INDEX];
          }
        #endif 
        if (pid_output > PID_MAX) {
          if (pid_error[HOTEND_INDEX] > 0) temp_iState[HOTEND_INDEX] -= pid_error[HOTEND_INDEX]; 
          pid_output = PID_MAX;
        }
        else if (pid_output < 0) {
          if (pid_error[HOTEND_INDEX] < 0) temp_iState[HOTEND_INDEX] -= pid_error[HOTEND_INDEX]; 
          pid_output = 0;
        }
      }
    #else
      pid_output = constrain(target_temperature[HOTEND_INDEX], 0, PID_MAX);
    #endif 
    #if ENABLED(PID_DEBUG)
      SERIAL_ECHO_START();
      SERIAL_ECHOPAIR(MSG_PID_DEBUG, HOTEND_INDEX);
      SERIAL_ECHOPAIR(MSG_PID_DEBUG_INPUT, current_temperature[HOTEND_INDEX]);
      SERIAL_ECHOPAIR(MSG_PID_DEBUG_OUTPUT, pid_output);
      SERIAL_ECHOPAIR(MSG_PID_DEBUG_PTERM, pTerm[HOTEND_INDEX]);
      SERIAL_ECHOPAIR(MSG_PID_DEBUG_ITERM, iTerm[HOTEND_INDEX]);
      SERIAL_ECHOPAIR(MSG_PID_DEBUG_DTERM, dTerm[HOTEND_INDEX]);
      #if ENABLED(PID_EXTRUSION_SCALING)
        SERIAL_ECHOPAIR(MSG_PID_DEBUG_CTERM, cTerm[HOTEND_INDEX]);
      #endif
      SERIAL_EOL();
    #endif 
  #else 
    #if HEATER_IDLE_HANDLER
      if (heater_idle_timeout_exceeded[HOTEND_INDEX])
        pid_output = 0;
      else
    #endif
    pid_output = (current_temperature[HOTEND_INDEX] < target_temperature[HOTEND_INDEX]) ? PID_MAX : 0;
  #endif
  return pid_output;
}
#if ENABLED(PIDTEMPBED)
  float Temperature::get_pid_output_bed() {
    float pid_output;
    #if DISABLED(PID_OPENLOOP)
      pid_error_bed = target_temperature_bed - current_temperature_bed;
      pTerm_bed = bedKp * pid_error_bed;
      temp_iState_bed += pid_error_bed;
      iTerm_bed = bedKi * temp_iState_bed;
      dTerm_bed = K2 * bedKd * (current_temperature_bed - temp_dState_bed) + K1 * dTerm_bed;
      temp_dState_bed = current_temperature_bed;
      pid_output = pTerm_bed + iTerm_bed - dTerm_bed;
      if (pid_output > MAX_BED_POWER) {
        if (pid_error_bed > 0) temp_iState_bed -= pid_error_bed; 
        pid_output = MAX_BED_POWER;
      }
      else if (pid_output < 0) {
        if (pid_error_bed < 0) temp_iState_bed -= pid_error_bed; 
        pid_output = 0;
      }
    #else
      pid_output = constrain(target_temperature_bed, 0, MAX_BED_POWER);
    #endif 
    #if ENABLED(PID_BED_DEBUG)
      SERIAL_ECHO_START();
      SERIAL_ECHOPGM(" PID_BED_DEBUG ");
      SERIAL_ECHOPGM(": Input ");
      SERIAL_ECHO(current_temperature_bed);
      SERIAL_ECHOPGM(" Output ");
      SERIAL_ECHO(pid_output);
      SERIAL_ECHOPGM(" pTerm ");
      SERIAL_ECHO(pTerm_bed);
      SERIAL_ECHOPGM(" iTerm ");
      SERIAL_ECHO(iTerm_bed);
      SERIAL_ECHOPGM(" dTerm ");
      SERIAL_ECHOLN(dTerm_bed);
    #endif 
    return pid_output;
  }
#endif 
void Temperature::manage_heater() {
  if (!temp_meas_ready) return;
  updateTemperaturesFromRawValues(); 
  #if WATCH_HOTENDS || WATCH_THE_BED || DISABLED(PIDTEMPBED) || HAS_AUTO_FAN || HEATER_IDLE_HANDLER
    millis_t ms = millis();
  #endif
  #if ENABLED(HEATER_0_USES_MAX6675)
    if (current_temperature[0] > min(HEATER_0_MAXTEMP, MAX6675_TMAX - 1.0)) max_temp_error(0);
    if (current_temperature[0] < max(HEATER_0_MINTEMP, MAX6675_TMIN + .01)) min_temp_error(0);
  #elif defined(TEMP_LIMIT_MONITOR)
    static millis_t startTime = ms;
    if (ms > startTime + 1000)
    {
        for (uint8_t e = 0; e < HOTENDS; e++)
        {
            if (current_temperature[e] <= HEATER_0_MINTEMP) min_temp_error(e);
            else if (current_temperature[e] >= HEATER_0_MAXTEMP)max_temp_error(e);
        }
        #if HAS_TEMP_BED
        if(current_temperature_bed<=BED_MINTEMP)min_temp_error(-1);
        else if(current_temperature_bed>=BED_MAXTEMP)max_temp_error(-1);
        #endif
    }
  #endif
  HOTEND_LOOP() {
    #if HEATER_IDLE_HANDLER
      if (!heater_idle_timeout_exceeded[e] && heater_idle_timeout_ms[e] && ELAPSED(ms, heater_idle_timeout_ms[e]))
        heater_idle_timeout_exceeded[e] = true;
    #endif
    #if ENABLED(THERMAL_PROTECTION_HOTENDS)
      thermal_runaway_protection(&thermal_runaway_state_machine[e], &thermal_runaway_timer[e], current_temperature[e], target_temperature[e], e, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);
    #endif
    soft_pwm_amount[e] = (current_temperature[e] > minttemp[e] || is_preheating(e)) && current_temperature[e] < maxttemp[e] ? (int)get_pid_output(e) >> 1 : 0;
    #if WATCH_HOTENDS
      if (watch_heater_next_ms[e] && ELAPSED(ms, watch_heater_next_ms[e])) { 
          if (degHotend(e) < watch_target_temp[e])                             
          {
            #if ENABLED(FYS_LCD_EVENT)
              FunV006("Heating failed.\nSystem stop.");
              _temp_error(e, PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
            #endif
          }
        else                                                                 
          start_watching_heater(e);
      }
    #endif
    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      if (FABS(current_temperature[0] - redundant_temperature) > MAX_REDUNDANT_TEMP_SENSOR_DIFF)
        _temp_error(0, PSTR(MSG_REDUNDANCY), PSTR(MSG_ERR_REDUNDANT_TEMP));
    #endif
  } 
  #if HAS_AUTO_FAN
    if (ELAPSED(ms, next_auto_fan_check_ms)) { 
      checkExtruderAutoFans();
      next_auto_fan_check_ms = ms + 2500UL;
    }
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    if (filament_sensor) {
      meas_shift_index = filwidth_delay_index[0] - meas_delay_cm;
      if (meas_shift_index < 0) meas_shift_index += MAX_MEASUREMENT_DELAY + 1;  
      meas_shift_index = constrain(meas_shift_index, 0, MAX_MEASUREMENT_DELAY);
      const float vmroot = measurement_delay[meas_shift_index] * 0.01 + 1.0;
      volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM] = vmroot <= 0.1 ? 0.01 : sq(vmroot);
    }
  #endif 
  #if WATCH_THE_BED
    if (watch_bed_next_ms && ELAPSED(ms, watch_bed_next_ms)) {        
        if (degBed() < watch_target_bed_temp)                           
        {
          #if ENABLED(FYS_LCD_EVENT)
            FunV006("Heating failed.\nSystem stop.");
          #endif
          _temp_error(-1, PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
        }
      else                                                            
        start_watching_bed();
    }
  #endif 
  #if DISABLED(PIDTEMPBED)
    if (PENDING(ms, next_bed_check_ms)) return;
    next_bed_check_ms = ms + BED_CHECK_INTERVAL;
  #endif
  #if HAS_TEMP_BED
    #if HEATER_IDLE_HANDLER
      if (!bed_idle_timeout_exceeded && bed_idle_timeout_ms && ELAPSED(ms, bed_idle_timeout_ms))
        bed_idle_timeout_exceeded = true;
    #endif
    #if HAS_THERMALLY_PROTECTED_BED
      thermal_runaway_protection(&thermal_runaway_bed_state_machine, &thermal_runaway_bed_timer, current_temperature_bed, target_temperature_bed, -1, THERMAL_PROTECTION_BED_PERIOD, THERMAL_PROTECTION_BED_HYSTERESIS);
    #endif
    #if HEATER_IDLE_HANDLER
      if (bed_idle_timeout_exceeded)
      {
        soft_pwm_amount_bed = 0;
        #if DISABLED(PIDTEMPBED)
          WRITE_HEATER_BED(LOW);
        #endif
      }
      else
    #endif
    {
      #if ENABLED(PIDTEMPBED)
        soft_pwm_amount_bed = WITHIN(current_temperature_bed, BED_MINTEMP, BED_MAXTEMP) ? (int)get_pid_output_bed() >> 1 : 0;
      #elif ENABLED(BED_LIMIT_SWITCHING)
        if (WITHIN(current_temperature_bed, BED_MINTEMP, BED_MAXTEMP)) {
          if (current_temperature_bed >= target_temperature_bed + BED_HYSTERESIS)
            soft_pwm_amount_bed = 0;
          else if (current_temperature_bed <= target_temperature_bed - (BED_HYSTERESIS))
            soft_pwm_amount_bed = MAX_BED_POWER >> 1;
        }
        else {
          soft_pwm_amount_bed = 0;
          WRITE_HEATER_BED(LOW);
        }
      #else 
        if (WITHIN(current_temperature_bed, BED_MINTEMP, BED_MAXTEMP)) {
          soft_pwm_amount_bed = current_temperature_bed < target_temperature_bed ? MAX_BED_POWER >> 1 : 0;
        }
        else {
          soft_pwm_amount_bed = 0;
          WRITE_HEATER_BED(LOW);
        }
      #endif
    }
  #endif 
}
#define PGM_RD_W(x)   (short)pgm_read_word(&x)
float Temperature::analog2temp(int raw, uint8_t e) {
  #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
    if (e > HOTENDS)
  #else
    if (e >= HOTENDS)
  #endif
    {
      SERIAL_ERROR_START();
      SERIAL_ERROR((int)e);
      SERIAL_ERRORLNPGM(MSG_INVALID_EXTRUDER_NUM);
      #if ENABLED(FYS_LCD_EVENT)
        FunV006(MSG_INVALID_EXTRUDER_NUM);
      #endif
      kill(PSTR(MSG_KILLED));
      return 0.0;
    }
  #if ENABLED(HEATER_0_USES_MAX6675)
    if (e == 0) return 0.25 * raw;
  #endif
  if (heater_ttbl_map[e] != NULL) {
    float celsius = 0;
    uint8_t i;
    short(*tt)[][2] = (short(*)[][2])(heater_ttbl_map[e]);
    for (i = 1; i < heater_ttbllen_map[e]; i++) {
      if (PGM_RD_W((*tt)[i][0]) > raw) {
        celsius = PGM_RD_W((*tt)[i - 1][1]) +
                  (raw - PGM_RD_W((*tt)[i - 1][0])) *
                  (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i - 1][1])) /
                  (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i - 1][0]));
        break;
      }
    }
    if (i == heater_ttbllen_map[e]) celsius = PGM_RD_W((*tt)[i - 1][1]);
    return celsius;
  }
  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * (TEMP_SENSOR_AD595_GAIN)) + TEMP_SENSOR_AD595_OFFSET;
}
float Temperature::analog2tempBed(const int raw) {
  #if ENABLED(BED_USES_THERMISTOR)
    float celsius = 0;
    byte i;
    for (i = 1; i < BEDTEMPTABLE_LEN; i++) {
      if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw) {
        celsius  = PGM_RD_W(BEDTEMPTABLE[i - 1][1]) +
                   (raw - PGM_RD_W(BEDTEMPTABLE[i - 1][0])) *
                   (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i - 1][1])) /
                   (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i - 1][0]));
        break;
      }
    }
    if (i == BEDTEMPTABLE_LEN) celsius = PGM_RD_W(BEDTEMPTABLE[i - 1][1]);
    return celsius;
  #elif defined(BED_USES_AD595)
    return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * (TEMP_SENSOR_AD595_GAIN)) + TEMP_SENSOR_AD595_OFFSET;
  #else
    UNUSED(raw);
    return 0;
  #endif
}
void Temperature::updateTemperaturesFromRawValues() {
  #if ENABLED(HEATER_0_USES_MAX6675)
    current_temperature_raw[0] = read_max6675();
  #endif
    HOTEND_LOOP()
        current_temperature[e] = Temperature::analog2temp(current_temperature_raw[e], e);
  current_temperature_bed = Temperature::analog2tempBed(current_temperature_bed_raw);
  #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
    redundant_temperature = Temperature::analog2temp(redundant_temperature_raw, 1);
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    filament_width_meas = analog2widthFil();
  #endif
  #if ENABLED(USE_WATCHDOG)
    watchdog_reset();
  #endif
  CRITICAL_SECTION_START;
  temp_meas_ready = false;
  CRITICAL_SECTION_END;
}
#if ENABLED(FILAMENT_WIDTH_SENSOR)
  float Temperature::analog2widthFil() {
    return current_raw_filwidth * 5.0 * (1.0 / 16383.0);
  }
  int Temperature::widthFil_to_size_ratio() {
    float temp = filament_width_meas;
    if (temp < MEASURED_LOWER_LIMIT) temp = filament_width_nominal;  
    else NOMORE(temp, MEASURED_UPPER_LIMIT);
    return filament_width_nominal / temp * 100;
  }
#endif
#if ENABLED(HEATER_0_USES_MAX6675)
  #ifndef MAX6675_SCK_PIN
    #define MAX6675_SCK_PIN SCK_PIN
  #endif
  #ifndef MAX6675_DO_PIN
    #define MAX6675_DO_PIN MISO_PIN
  #endif
  SPI<MAX6675_DO_PIN, MOSI_PIN, MAX6675_SCK_PIN> max6675_spi;
#endif
void Temperature::init() {
  #if MB(RUMBA) && (TEMP_SENSOR_0 == -1 || TEMP_SENSOR_1 == -1 || TEMP_SENSOR_2 == -1 || TEMP_SENSOR_BED == -1)
    MCUCR = _BV(JTD);
    MCUCR = _BV(JTD);
  #endif
  HOTEND_LOOP() maxttemp[e] = maxttemp[0];
  #if ENABLED(PIDTEMP) && ENABLED(PID_EXTRUSION_SCALING)
    last_e_position = 0;
  #endif
  #if HAS_HEATER_0
    SET_OUTPUT(HEATER_0_PIN);
  #endif
  #if HAS_HEATER_1
    SET_OUTPUT(HEATER_1_PIN);
  #endif
  #if HAS_HEATER_2
    SET_OUTPUT(HEATER_2_PIN);
  #endif
  #if HAS_HEATER_3
    SET_OUTPUT(HEATER_3_PIN);
  #endif
  #if HAS_HEATER_4
    SET_OUTPUT(HEATER_3_PIN);
  #endif
  #if HAS_HEATER_BED
    SET_OUTPUT(HEATER_BED_PIN);
  #endif
  #if HAS_FAN0
    SET_OUTPUT(FAN_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(FAN_PIN, 1); 
    #endif
  #endif
  #if HAS_FAN1
    SET_OUTPUT(FAN1_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(FAN1_PIN, 1); 
    #endif
  #endif
  #if HAS_FAN2
    SET_OUTPUT(FAN2_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(FAN2_PIN, 1); 
    #endif
  #endif
  #if ENABLED(HEATER_0_USES_MAX6675)
    OUT_WRITE(SCK_PIN, LOW);
    OUT_WRITE(MOSI_PIN, HIGH);
    SET_INPUT_PULLUP(MISO_PIN);
    max6675_spi.init();
    OUT_WRITE(SS_PIN, HIGH);
    OUT_WRITE(MAX6675_SS, HIGH);
  #endif 
  #ifdef DIDR2
    #define ANALOG_SELECT(pin) do{ if (pin < 8) SBI(DIDR0, pin); else SBI(DIDR2, pin - 8); }while(0)
  #else
    #define ANALOG_SELECT(pin) do{ SBI(DIDR0, pin); }while(0)
  #endif
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIF) | 0x07;
  DIDR0 = 0;
  #ifdef DIDR2
    DIDR2 = 0;
  #endif
  #if HAS_TEMP_0
    ANALOG_SELECT(TEMP_0_PIN);
  #endif
  #if HAS_TEMP_1
    ANALOG_SELECT(TEMP_1_PIN);
  #endif
  #if HAS_TEMP_2
    ANALOG_SELECT(TEMP_2_PIN);
  #endif
  #if HAS_TEMP_3
    ANALOG_SELECT(TEMP_3_PIN);
  #endif
  #if HAS_TEMP_4
    ANALOG_SELECT(TEMP_4_PIN);
  #endif
  #if HAS_TEMP_BED
    ANALOG_SELECT(TEMP_BED_PIN);
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    ANALOG_SELECT(FILWIDTH_PIN);
  #endif
  #if HAS_AUTO_FAN_0
    #if E0_AUTO_FAN_PIN == FAN1_PIN
      SET_OUTPUT(E0_AUTO_FAN_PIN);
      #if ENABLED(FAST_PWM_FAN)
        setPwmFrequency(E0_AUTO_FAN_PIN, 1); 
      #endif
    #else
      SET_OUTPUT(E0_AUTO_FAN_PIN);
    #endif
  #endif
  #if HAS_AUTO_FAN_1 && !AUTO_1_IS_0
    #if E1_AUTO_FAN_PIN == FAN1_PIN
      SET_OUTPUT(E1_AUTO_FAN_PIN);
      #if ENABLED(FAST_PWM_FAN)
        setPwmFrequency(E1_AUTO_FAN_PIN, 1); 
      #endif
    #else
      SET_OUTPUT(E1_AUTO_FAN_PIN);
    #endif
  #endif
  #if HAS_AUTO_FAN_2 && !AUTO_2_IS_0 && !AUTO_2_IS_1
    #if E2_AUTO_FAN_PIN == FAN1_PIN
      SET_OUTPUT(E2_AUTO_FAN_PIN);
      #if ENABLED(FAST_PWM_FAN)
        setPwmFrequency(E2_AUTO_FAN_PIN, 1); 
      #endif
    #else
      SET_OUTPUT(E2_AUTO_FAN_PIN);
    #endif
  #endif
  #if HAS_AUTO_FAN_3 && !AUTO_3_IS_0 && !AUTO_3_IS_1 && !AUTO_3_IS_2
    #if E3_AUTO_FAN_PIN == FAN1_PIN
      SET_OUTPUT(E3_AUTO_FAN_PIN);
      #if ENABLED(FAST_PWM_FAN)
        setPwmFrequency(E3_AUTO_FAN_PIN, 1); 
      #endif
    #else
      SET_OUTPUT(E3_AUTO_FAN_PIN);
    #endif
  #endif
  #if HAS_AUTO_FAN_4 && !AUTO_4_IS_0 && !AUTO_4_IS_1 && !AUTO_4_IS_2 && !AUTO_4_IS_3
    #if E4_AUTO_FAN_PIN == FAN1_PIN
      SET_OUTPUT(E4_AUTO_FAN_PIN);
      #if ENABLED(FAST_PWM_FAN)
        setPwmFrequency(E4_AUTO_FAN_PIN, 1); 
      #endif
    #else
      SET_OUTPUT(E4_AUTO_FAN_PIN);
    #endif
  #endif
  OCR0B = 128;
  SBI(TIMSK0, OCIE0B);
  delay(250);
  #define TEMP_MIN_ROUTINE(NR) \
    minttemp[NR] = HEATER_ ##NR## _MINTEMP; \
    while (analog2temp(minttemp_raw[NR], NR) < HEATER_ ##NR## _MINTEMP) { \
      if (HEATER_ ##NR## _RAW_LO_TEMP < HEATER_ ##NR## _RAW_HI_TEMP) \
        minttemp_raw[NR] += OVERSAMPLENR; \
      else \
        minttemp_raw[NR] -= OVERSAMPLENR; \
    }
  #define TEMP_MAX_ROUTINE(NR) \
    maxttemp[NR] = HEATER_ ##NR## _MAXTEMP; \
    while (analog2temp(maxttemp_raw[NR], NR) > HEATER_ ##NR## _MAXTEMP) { \
      if (HEATER_ ##NR## _RAW_LO_TEMP < HEATER_ ##NR## _RAW_HI_TEMP) \
        maxttemp_raw[NR] -= OVERSAMPLENR; \
      else \
        maxttemp_raw[NR] += OVERSAMPLENR; \
    }
  #ifdef HEATER_0_MINTEMP
    TEMP_MIN_ROUTINE(0);
  #endif
  #ifdef HEATER_0_MAXTEMP
    TEMP_MAX_ROUTINE(0);
  #endif
  #if HOTENDS > 1
    #ifdef HEATER_1_MINTEMP
      TEMP_MIN_ROUTINE(1);
    #endif
    #ifdef HEATER_1_MAXTEMP
      TEMP_MAX_ROUTINE(1);
    #endif
    #if HOTENDS > 2
      #ifdef HEATER_2_MINTEMP
        TEMP_MIN_ROUTINE(2);
      #endif
      #ifdef HEATER_2_MAXTEMP
        TEMP_MAX_ROUTINE(2);
      #endif
      #if HOTENDS > 3
        #ifdef HEATER_3_MINTEMP
          TEMP_MIN_ROUTINE(3);
        #endif
        #ifdef HEATER_3_MAXTEMP
          TEMP_MAX_ROUTINE(3);
        #endif
        #if HOTENDS > 4
          #ifdef HEATER_4_MINTEMP
            TEMP_MIN_ROUTINE(4);
          #endif
          #ifdef HEATER_4_MAXTEMP
            TEMP_MAX_ROUTINE(4);
          #endif
        #endif 
      #endif 
    #endif 
  #endif 
  #ifdef BED_MINTEMP
    while (analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_minttemp_raw += OVERSAMPLENR;
      #else
        bed_minttemp_raw -= OVERSAMPLENR;
      #endif
    }
  #endif 
  #ifdef BED_MAXTEMP
    while (analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_maxttemp_raw -= OVERSAMPLENR;
      #else
        bed_maxttemp_raw += OVERSAMPLENR;
      #endif
    }
  #endif 
  #if ENABLED(PROBING_HEATERS_OFF)
    paused = false;
  #endif
}
#if WATCH_HOTENDS
  void Temperature::start_watching_heater(uint8_t e) {
    #if HOTENDS == 1
      UNUSED(e);
    #endif
    if (degHotend(HOTEND_INDEX) < degTargetHotend(HOTEND_INDEX) - (WATCH_TEMP_INCREASE + TEMP_HYSTERESIS + 1)) {
      watch_target_temp[HOTEND_INDEX] = degHotend(HOTEND_INDEX) + WATCH_TEMP_INCREASE;
      watch_heater_next_ms[HOTEND_INDEX] = millis() + (WATCH_TEMP_PERIOD) * 1000UL;
    }
    else
      watch_heater_next_ms[HOTEND_INDEX] = 0;
  }
#endif
#if WATCH_THE_BED
  void Temperature::start_watching_bed() {
    if (degBed() < degTargetBed() - (WATCH_BED_TEMP_INCREASE + TEMP_BED_HYSTERESIS + 1)) {
      watch_target_bed_temp = degBed() + WATCH_BED_TEMP_INCREASE;
      watch_bed_next_ms = millis() + (WATCH_BED_TEMP_PERIOD) * 1000UL;
    }
    else
      watch_bed_next_ms = 0;
  }
#endif
#if ENABLED(THERMAL_PROTECTION_HOTENDS) || HAS_THERMALLY_PROTECTED_BED
  #if ENABLED(THERMAL_PROTECTION_HOTENDS)
    Temperature::TRState Temperature::thermal_runaway_state_machine[HOTENDS] = { TRInactive };
    millis_t Temperature::thermal_runaway_timer[HOTENDS] = { 0 };
  #endif
  #if HAS_THERMALLY_PROTECTED_BED
    Temperature::TRState Temperature::thermal_runaway_bed_state_machine = TRInactive;
    millis_t Temperature::thermal_runaway_bed_timer;
  #endif
  void Temperature::thermal_runaway_protection(Temperature::TRState* state, millis_t* timer, float current, float target, int heater_id, int period_seconds, int hysteresis_degc) {
    static float tr_target_temperature[HOTENDS + 1] = { 0.0 };
    const int heater_index = heater_id >= 0 ? heater_id : HOTENDS;
    #if HEATER_IDLE_HANDLER
      if (heater_id >= 0 && heater_idle_timeout_exceeded[heater_id]) {
        *state = TRInactive;
        tr_target_temperature[heater_index] = 0;
      }
      #if HAS_TEMP_BED
        else if (heater_id < 0 && bed_idle_timeout_exceeded) {
          *state = TRInactive;
          tr_target_temperature[heater_index] = 0;
        }
      #endif
      else
    #endif
    if (tr_target_temperature[heater_index] != target) {
      tr_target_temperature[heater_index] = target;
      *state = target > 0 ? TRFirstHeating : TRInactive;
    }
    switch (*state) {
      case TRInactive: break;
      case TRFirstHeating:
        if (current < tr_target_temperature[heater_index]) break;
        *state = TRStable;
      case TRStable:
        if (current >= tr_target_temperature[heater_index] - hysteresis_degc) {
          *timer = millis() + period_seconds * 1000UL;
          break;
        }
        else if (PENDING(millis(), *timer)) break;
        *state = TRRunaway;
      case TRRunaway:
        _temp_error(heater_id, PSTR(MSG_T_THERMAL_RUNAWAY), PSTR(MSG_THERMAL_RUNAWAY));
    }
  }
#endif 
void Temperature::disable_all_heaters() {
  #if ENABLED(AUTOTEMP)
    planner.autotemp_enabled = false;
  #endif
  HOTEND_LOOP() setTargetHotend(0, e);
  setTargetBed(0);
  #if ENABLED(PROBING_HEATERS_OFF)
    pause(false);
  #endif
  print_job_timer.stop();
  #define DISABLE_HEATER(NR) { \
    setTargetHotend(0, NR); \
    soft_pwm_amount[NR] = 0; \
    WRITE_HEATER_ ##NR (LOW); \
  }
  #if HAS_TEMP_HOTEND
    DISABLE_HEATER(0);
    #if HOTENDS > 1
      DISABLE_HEATER(1);
      #if HOTENDS > 2
        DISABLE_HEATER(2);
        #if HOTENDS > 3
          DISABLE_HEATER(3);
          #if HOTENDS > 4
            DISABLE_HEATER(4);
          #endif 
        #endif 
      #endif 
    #endif 
  #endif
  #if HAS_TEMP_BED
    target_temperature_bed = 0;
    soft_pwm_amount_bed = 0;
    #if HAS_HEATER_BED
      WRITE_HEATER_BED(LOW);
    #endif
  #endif
}
#if ENABLED(PROBING_HEATERS_OFF)
  void Temperature::pause(const bool p) {
    if (p != paused) {
      paused = p;
      if (p) {
        HOTEND_LOOP() start_heater_idle_timer(e, 0); 
        #if HAS_TEMP_BED
          start_bed_idle_timer(0); 
        #endif
      }
      else {
        HOTEND_LOOP() reset_heater_idle_timer(e);
        #if HAS_TEMP_BED
          reset_bed_idle_timer();
        #endif
      }
    }
  }
#endif 
#if ENABLED(HEATER_0_USES_MAX6675)
  #define MAX6675_HEAT_INTERVAL 250u
  #if ENABLED(MAX6675_IS_MAX31855)
    uint32_t max6675_temp = 2000;
    #define MAX6675_ERROR_MASK 7
    #define MAX6675_DISCARD_BITS 18
    #define MAX6675_SPEED_BITS (_BV(SPR1)) 
  #else
    uint16_t max6675_temp = 2000;
    #define MAX6675_ERROR_MASK 4
    #define MAX6675_DISCARD_BITS 3
    #define MAX6675_SPEED_BITS (_BV(SPR0)) 
  #endif
  int Temperature::read_max6675() {
    static millis_t next_max6675_ms = 0;
    millis_t ms = millis();
    if (PENDING(ms, next_max6675_ms)) return (int)max6675_temp;
    next_max6675_ms = ms + MAX6675_HEAT_INTERVAL;
    CBI(
      #ifdef PRR
        PRR
      #elif defined(PRR0)
        PRR0
      #endif
        , PRSPI);
    SPCR = _BV(MSTR) | _BV(SPE) | MAX6675_SPEED_BITS;
    WRITE(MAX6675_SS, 0); 
    asm("nop");
    asm("nop");
    max6675_temp = 0;
    for (uint8_t i = sizeof(max6675_temp); i--;) {
      max6675_temp |= max6675_spi.receive();
      if (i > 0) max6675_temp <<= 8; 
    }
    WRITE(MAX6675_SS, 1); 
    if (max6675_temp & MAX6675_ERROR_MASK) {
      SERIAL_ERROR_START();
      SERIAL_ERRORPGM("Temp measurement error! ");
      #if MAX6675_ERROR_MASK == 7
        SERIAL_ERRORPGM("MAX31855 ");
        if (max6675_temp & 1)
          SERIAL_ERRORLNPGM("Open Circuit");
        else if (max6675_temp & 2)
          SERIAL_ERRORLNPGM("Short to GND");
        else if (max6675_temp & 4)
          SERIAL_ERRORLNPGM("Short to VCC");
      #else
        SERIAL_ERRORLNPGM("MAX6675");
      #endif
      max6675_temp = MAX6675_TMAX * 4; 
    }
    else
      max6675_temp >>= MAX6675_DISCARD_BITS;
      #if ENABLED(MAX6675_IS_MAX31855)
        if (max6675_temp & 0x00002000) max6675_temp |= 0xFFFFC000;
      #endif
    return (int)max6675_temp;
  }
#endif 
void Temperature::set_current_temp_raw() {
  #if HAS_TEMP_0 && DISABLED(HEATER_0_USES_MAX6675)
    current_temperature_raw[0] = raw_temp_value[0];
  #endif
  #if HAS_TEMP_1
    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      redundant_temperature_raw = raw_temp_value[1];
    #else
      current_temperature_raw[1] = raw_temp_value[1];
    #endif
    #if HAS_TEMP_2
      current_temperature_raw[2] = raw_temp_value[2];
      #if HAS_TEMP_3
        current_temperature_raw[3] = raw_temp_value[3];
        #if HAS_TEMP_4
          current_temperature_raw[4] = raw_temp_value[4];
        #endif
      #endif
    #endif
  #endif
  current_temperature_bed_raw = raw_temp_bed_value;
  temp_meas_ready = true;
}
#if ENABLED(PINS_DEBUGGING)
  void endstop_monitor() {
    static uint16_t old_endstop_bits_local = 0;
    static uint8_t local_LED_status = 0;
    uint16_t current_endstop_bits_local = 0;
    #if HAS_X_MIN
      if (READ(X_MIN_PIN)) SBI(current_endstop_bits_local, X_MIN);
    #endif
    #if HAS_X_MAX
      if (READ(X_MAX_PIN)) SBI(current_endstop_bits_local, X_MAX);
    #endif
    #if HAS_Y_MIN
      if (READ(Y_MIN_PIN)) SBI(current_endstop_bits_local, Y_MIN);
    #endif
    #if HAS_Y_MAX
      if (READ(Y_MAX_PIN)) SBI(current_endstop_bits_local, Y_MAX);
    #endif
    #if HAS_Z_MIN
      if (READ(Z_MIN_PIN)) SBI(current_endstop_bits_local, Z_MIN);
    #endif
    #if HAS_Z_MAX
      if (READ(Z_MAX_PIN)) SBI(current_endstop_bits_local, Z_MAX);
    #endif
    #if HAS_Z_MIN_PROBE_PIN
      if (READ(Z_MIN_PROBE_PIN)) SBI(current_endstop_bits_local, Z_MIN_PROBE);
    #endif
    #if HAS_Z2_MIN
      if (READ(Z2_MIN_PIN)) SBI(current_endstop_bits_local, Z2_MIN);
    #endif
    #if HAS_Z2_MAX
      if (READ(Z2_MAX_PIN)) SBI(current_endstop_bits_local, Z2_MAX);
    #endif
    uint16_t endstop_change = current_endstop_bits_local ^ old_endstop_bits_local;
    if (endstop_change) {
      #if HAS_X_MIN
        if (TEST(endstop_change, X_MIN)) SERIAL_PROTOCOLPAIR("X_MIN:", !!TEST(current_endstop_bits_local, X_MIN));
      #endif
      #if HAS_X_MAX
        if (TEST(endstop_change, X_MAX)) SERIAL_PROTOCOLPAIR("  X_MAX:", !!TEST(current_endstop_bits_local, X_MAX));
      #endif
      #if HAS_Y_MIN
        if (TEST(endstop_change, Y_MIN)) SERIAL_PROTOCOLPAIR("  Y_MIN:", !!TEST(current_endstop_bits_local, Y_MIN));
      #endif
      #if HAS_Y_MAX
        if (TEST(endstop_change, Y_MAX)) SERIAL_PROTOCOLPAIR("  Y_MAX:", !!TEST(current_endstop_bits_local, Y_MAX));
      #endif
      #if HAS_Z_MIN
        if (TEST(endstop_change, Z_MIN)) SERIAL_PROTOCOLPAIR("  Z_MIN:", !!TEST(current_endstop_bits_local, Z_MIN));
      #endif
      #if HAS_Z_MAX
        if (TEST(endstop_change, Z_MAX)) SERIAL_PROTOCOLPAIR("  Z_MAX:", !!TEST(current_endstop_bits_local, Z_MAX));
      #endif
      #if HAS_Z_MIN_PROBE_PIN
        if (TEST(endstop_change, Z_MIN_PROBE)) SERIAL_PROTOCOLPAIR("  PROBE:", !!TEST(current_endstop_bits_local, Z_MIN_PROBE));
      #endif
      #if HAS_Z2_MIN
        if (TEST(endstop_change, Z2_MIN)) SERIAL_PROTOCOLPAIR("  Z2_MIN:", !!TEST(current_endstop_bits_local, Z2_MIN));
      #endif
      #if HAS_Z2_MAX
        if (TEST(endstop_change, Z2_MAX)) SERIAL_PROTOCOLPAIR("  Z2_MAX:", !!TEST(current_endstop_bits_local, Z2_MAX));
      #endif
      SERIAL_PROTOCOLPGM("\n\n");
      analogWrite(LED_PIN, local_LED_status);
      local_LED_status ^= 255;
      old_endstop_bits_local = current_endstop_bits_local;
    }
  }
#endif 
ISR(TIMER0_COMPB_vect) { Temperature::isr(); }
volatile bool Temperature::in_temp_isr = false;
void Temperature::isr() {
  if (in_temp_isr) return;
  in_temp_isr = true;
  CBI(TIMSK0, OCIE0B); 
  sei();
  static int8_t temp_count = -1;
  static ADCSensorState adc_sensor_state = StartupDelay;
  static uint8_t pwm_count = _BV(SOFT_PWM_SCALE);
  uint8_t pwm_count_tmp = pwm_count;
  #if ENABLED(ADC_KEYPAD)
    static unsigned int raw_ADCKey_value = 0;
  #endif
  #if ENABLED(SLOW_PWM_HEATERS)
    static uint8_t slow_pwm_count = 0;
    #define ISR_STATICS(n) \
      static uint8_t soft_pwm_count_ ## n, \
                     state_heater_ ## n = 0, \
                     state_timer_heater_ ## n = 0
  #else
    #define ISR_STATICS(n) static uint8_t soft_pwm_count_ ## n = 0
  #endif
  ISR_STATICS(0);
  #if HOTENDS > 1
    ISR_STATICS(1);
    #if HOTENDS > 2
      ISR_STATICS(2);
      #if HOTENDS > 3
        ISR_STATICS(3);
        #if HOTENDS > 4
          ISR_STATICS(4);
        #endif 
      #endif 
    #endif 
  #endif 
  #if HAS_HEATER_BED
    ISR_STATICS(BED);
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    static unsigned long raw_filwidth_value = 0;
  #endif
  #if DISABLED(SLOW_PWM_HEATERS)
    constexpr uint8_t pwm_mask =
      #if ENABLED(SOFT_PWM_DITHER)
        _BV(SOFT_PWM_SCALE) - 1
      #else
        0
      #endif
    ;
    if (pwm_count_tmp >= 127) {
      pwm_count_tmp -= 127;
      soft_pwm_count_0 = (soft_pwm_count_0 & pwm_mask) + soft_pwm_amount[0];
      WRITE_HEATER_0(soft_pwm_count_0 > pwm_mask ? HIGH : LOW);
      #if HOTENDS > 1
        soft_pwm_count_1 = (soft_pwm_count_1 & pwm_mask) + soft_pwm_amount[1];
        WRITE_HEATER_1(soft_pwm_count_1 > pwm_mask ? HIGH : LOW);
        #if HOTENDS > 2
          soft_pwm_count_2 = (soft_pwm_count_2 & pwm_mask) + soft_pwm_amount[2];
          WRITE_HEATER_2(soft_pwm_count_2 > pwm_mask ? HIGH : LOW);
          #if HOTENDS > 3
            soft_pwm_count_3 = (soft_pwm_count_3 & pwm_mask) + soft_pwm_amount[3];
            WRITE_HEATER_3(soft_pwm_count_3 > pwm_mask ? HIGH : LOW);
            #if HOTENDS > 4
              soft_pwm_count_4 = (soft_pwm_count_4 & pwm_mask) + soft_pwm_amount[4];
              WRITE_HEATER_4(soft_pwm_count_4 > pwm_mask ? HIGH : LOW);
            #endif 
          #endif 
        #endif 
      #endif 
      #if HAS_HEATER_BED
        soft_pwm_count_BED = (soft_pwm_count_BED & pwm_mask) + soft_pwm_amount_bed;
        WRITE_HEATER_BED(soft_pwm_count_BED > pwm_mask ? HIGH : LOW);
      #endif
      #if ENABLED(FAN_SOFT_PWM)
        #if HAS_FAN0
          soft_pwm_count_fan[0] = (soft_pwm_count_fan[0] & pwm_mask) + soft_pwm_amount_fan[0] >> 1;
          WRITE_FAN(soft_pwm_count_fan[0] > pwm_mask ? HIGH : LOW);
        #endif
        #if HAS_FAN1
          soft_pwm_count_fan[1] = (soft_pwm_count_fan[1] & pwm_mask) + soft_pwm_amount_fan[1] >> 1;
          WRITE_FAN1(soft_pwm_count_fan[1] > pwm_mask ? HIGH : LOW);
        #endif
        #if HAS_FAN2
          soft_pwm_count_fan[2] = (soft_pwm_count_fan[2] & pwm_mask) + soft_pwm_amount_fan[2] >> 1;
          WRITE_FAN2(soft_pwm_count_fan[2] > pwm_mask ? HIGH : LOW);
        #endif
      #endif
    }
    else {
      if (soft_pwm_count_0 <= pwm_count_tmp) WRITE_HEATER_0(0);
      #if HOTENDS > 1
        if (soft_pwm_count_1 <= pwm_count_tmp) WRITE_HEATER_1(0);
        #if HOTENDS > 2
          if (soft_pwm_count_2 <= pwm_count_tmp) WRITE_HEATER_2(0);
          #if HOTENDS > 3
            if (soft_pwm_count_3 <= pwm_count_tmp) WRITE_HEATER_3(0);
            #if HOTENDS > 4
              if (soft_pwm_count_4 <= pwm_count_tmp) WRITE_HEATER_4(0);
            #endif 
          #endif 
        #endif 
      #endif 
      #if HAS_HEATER_BED
        if (soft_pwm_count_BED <= pwm_count_tmp) WRITE_HEATER_BED(0);
      #endif
      #if ENABLED(FAN_SOFT_PWM)
        #if HAS_FAN0
          if (soft_pwm_count_fan[0] <= pwm_count_tmp) WRITE_FAN(0);
        #endif
        #if HAS_FAN1
          if (soft_pwm_count_fan[1] <= pwm_count_tmp) WRITE_FAN1(0);
        #endif
        #if HAS_FAN2
          if (soft_pwm_count_fan[2] <= pwm_count_tmp) WRITE_FAN2(0);
        #endif
      #endif
    }
    pwm_count = pwm_count_tmp + _BV(SOFT_PWM_SCALE);
  #else 
    #ifndef MIN_STATE_TIME
      #define MIN_STATE_TIME 16 
    #endif
    #define _SLOW_PWM_ROUTINE(NR, src) \
      soft_pwm_ ##NR = src; \
      if (soft_pwm_ ##NR > 0) { \
        if (state_timer_heater_ ##NR == 0) { \
          if (state_heater_ ##NR == 0) state_timer_heater_ ##NR = MIN_STATE_TIME; \
          state_heater_ ##NR = 1; \
          WRITE_HEATER_ ##NR(1); \
        } \
      } \
      else { \
        if (state_timer_heater_ ##NR == 0) { \
          if (state_heater_ ##NR == 1) state_timer_heater_ ##NR = MIN_STATE_TIME; \
          state_heater_ ##NR = 0; \
          WRITE_HEATER_ ##NR(0); \
        } \
      }
    #define SLOW_PWM_ROUTINE(n) _SLOW_PWM_ROUTINE(n, soft_pwm_amount[n])
    #define PWM_OFF_ROUTINE(NR) \
      if (soft_pwm_ ##NR < slow_pwm_count) { \
        if (state_timer_heater_ ##NR == 0) { \
          if (state_heater_ ##NR == 1) state_timer_heater_ ##NR = MIN_STATE_TIME; \
          state_heater_ ##NR = 0; \
          WRITE_HEATER_ ##NR (0); \
        } \
      }
    if (slow_pwm_count == 0) {
      SLOW_PWM_ROUTINE(0);
      #if HOTENDS > 1
        SLOW_PWM_ROUTINE(1);
        #if HOTENDS > 2
          SLOW_PWM_ROUTINE(2);
          #if HOTENDS > 3
            SLOW_PWM_ROUTINE(3);
            #if HOTENDS > 4
              SLOW_PWM_ROUTINE(4);
            #endif 
          #endif 
        #endif 
      #endif 
      #if HAS_HEATER_BED
        _SLOW_PWM_ROUTINE(BED, soft_pwm_amount_bed); 
      #endif
    } 
    PWM_OFF_ROUTINE(0);
    #if HOTENDS > 1
      PWM_OFF_ROUTINE(1);
      #if HOTENDS > 2
        PWM_OFF_ROUTINE(2);
        #if HOTENDS > 3
          PWM_OFF_ROUTINE(3);
          #if HOTENDS > 4
            PWM_OFF_ROUTINE(4);
          #endif 
        #endif 
      #endif 
    #endif 
    #if HAS_HEATER_BED
      PWM_OFF_ROUTINE(BED); 
    #endif
    #if ENABLED(FAN_SOFT_PWM)
      if (pwm_count_tmp >= 127) {
        pwm_count_tmp = 0;
        #if HAS_FAN0
          soft_pwm_count_fan[0] = soft_pwm_amount_fan[0] >> 1;
          WRITE_FAN(soft_pwm_count_fan[0] > 0 ? HIGH : LOW);
        #endif
        #if HAS_FAN1
          soft_pwm_count_fan[1] = soft_pwm_amount_fan[1] >> 1;
          WRITE_FAN1(soft_pwm_count_fan[1] > 0 ? HIGH : LOW);
        #endif
        #if HAS_FAN2
          soft_pwm_count_fan[2] = soft_pwm_amount_fan[2] >> 1;
          WRITE_FAN2(soft_pwm_count_fan[2] > 0 ? HIGH : LOW);
        #endif
      }
      #if HAS_FAN0
        if (soft_pwm_count_fan[0] <= pwm_count_tmp) WRITE_FAN(0);
      #endif
      #if HAS_FAN1
        if (soft_pwm_count_fan[1] <= pwm_count_tmp) WRITE_FAN1(0);
      #endif
      #if HAS_FAN2
        if (soft_pwm_count_fan[2] <= pwm_count_tmp) WRITE_FAN2(0);
      #endif
    #endif 
    pwm_count = pwm_count_tmp + _BV(SOFT_PWM_SCALE);
    if (((pwm_count >> SOFT_PWM_SCALE) & 0x3F) == 0) {
      slow_pwm_count++;
      slow_pwm_count &= 0x7F;
      if (state_timer_heater_0 > 0) state_timer_heater_0--;
      #if HOTENDS > 1
        if (state_timer_heater_1 > 0) state_timer_heater_1--;
        #if HOTENDS > 2
          if (state_timer_heater_2 > 0) state_timer_heater_2--;
          #if HOTENDS > 3
            if (state_timer_heater_3 > 0) state_timer_heater_3--;
            #if HOTENDS > 4
              if (state_timer_heater_4 > 0) state_timer_heater_4--;
            #endif 
          #endif 
        #endif 
      #endif 
      #if HAS_HEATER_BED
        if (state_timer_heater_BED > 0) state_timer_heater_BED--;
      #endif
    } 
  #endif 
  static bool do_buttons;
  if ((do_buttons ^= true)) lcd_buttons_update();
  #define SET_ADMUX_ADCSRA(pin) ADMUX = _BV(REFS0) | (pin & 0x07); SBI(ADCSRA, ADSC)
  #ifdef MUX5
    #define START_ADC(pin) if (pin > 7) ADCSRB = _BV(MUX5); else ADCSRB = 0; SET_ADMUX_ADCSRA(pin)
  #else
    #define START_ADC(pin) ADCSRB = 0; SET_ADMUX_ADCSRA(pin)
  #endif
  switch (adc_sensor_state) {
    case SensorsReady: {
      constexpr int8_t extra_loops = MIN_ADC_ISR_LOOPS - (int8_t)SensorsReady;
      static uint8_t delay_count = 0;
      if (extra_loops > 0) {
        if (delay_count == 0) delay_count = extra_loops;   
        if (--delay_count)                                 
          adc_sensor_state = (ADCSensorState)(int(SensorsReady) - 1); 
        break;
      }
      else
        adc_sensor_state = (ADCSensorState)0; 
    }
    #if HAS_TEMP_0
      case PrepareTemp_0:
        START_ADC(TEMP_0_PIN);
        break;
      case MeasureTemp_0:
        raw_temp_value[0] += ADC;
        break;
    #endif
    #if HAS_TEMP_BED
      case PrepareTemp_BED:
        START_ADC(TEMP_BED_PIN);
        break;
      case MeasureTemp_BED:
        raw_temp_bed_value += ADC;
        break;
    #endif
    #if HAS_TEMP_1
      case PrepareTemp_1:
        START_ADC(TEMP_1_PIN);
        break;
      case MeasureTemp_1:
        raw_temp_value[1] += ADC;
        break;
    #endif
    #if HAS_TEMP_2
      case PrepareTemp_2:
        START_ADC(TEMP_2_PIN);
        break;
      case MeasureTemp_2:
        raw_temp_value[2] += ADC;
        break;
    #endif
    #if HAS_TEMP_3
      case PrepareTemp_3:
        START_ADC(TEMP_3_PIN);
        break;
      case MeasureTemp_3:
        raw_temp_value[3] += ADC;
        break;
    #endif
    #if HAS_TEMP_4
      case PrepareTemp_4:
        START_ADC(TEMP_4_PIN);
        break;
      case MeasureTemp_4:
        raw_temp_value[4] += ADC;
        break;
    #endif
    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      case Prepare_FILWIDTH:
        START_ADC(FILWIDTH_PIN);
      break;
      case Measure_FILWIDTH:
        if (ADC > 102) { 
          raw_filwidth_value -= (raw_filwidth_value >> 7); 
          raw_filwidth_value += ((unsigned long)ADC << 7); 
        }
      break;
    #endif
    #if ENABLED(ADC_KEYPAD)
      case Prepare_ADC_KEY:
        START_ADC(ADC_KEYPAD_PIN);
        break;
      case Measure_ADC_KEY:
        if (ADCKey_count < 16) {
          raw_ADCKey_value = ADC;
          if (raw_ADCKey_value > 900) {
            ADCKey_count = 0;
            current_ADCKey_raw = 0;
          }
          else {
            current_ADCKey_raw += raw_ADCKey_value;
            ADCKey_count++;
          }
        }
        break;
    #endif 
    case StartupDelay: break;
  } 
  if (!adc_sensor_state && ++temp_count >= OVERSAMPLENR) { 
    temp_count = 0;
    if (!temp_meas_ready) set_current_temp_raw();
    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      current_raw_filwidth = raw_filwidth_value >> 10;  
    #endif
    ZERO(raw_temp_value);
    raw_temp_bed_value = 0;
    #define TEMPDIR(N) ((HEATER_##N##_RAW_LO_TEMP) > (HEATER_##N##_RAW_HI_TEMP) ? -1 : 1)
    int constexpr temp_dir[] = {
      #if ENABLED(HEATER_0_USES_MAX6675)
         0
      #else
        TEMPDIR(0)
      #endif
      #if HOTENDS > 1
        , TEMPDIR(1)
        #if HOTENDS > 2
          , TEMPDIR(2)
          #if HOTENDS > 3
            , TEMPDIR(3)
            #if HOTENDS > 4
              , TEMPDIR(4)
            #endif 
          #endif 
        #endif 
      #endif 
    };
    for (uint8_t e = 0; e < COUNT(temp_dir); e++) {
      const int16_t tdir = temp_dir[e], rawtemp = current_temperature_raw[e] * tdir;
      if (rawtemp > maxttemp_raw[e] * tdir && target_temperature[e] > 0) max_temp_error(e);
      if (rawtemp < minttemp_raw[e] * tdir && !is_preheating(e) && target_temperature[e] > 0) {
        #ifdef MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED
          if (++consecutive_low_temperature_error[e] >= MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
        #endif
            min_temp_error(e);
      }
      #ifdef MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED
        else
          consecutive_low_temperature_error[e] = 0;
      #endif
    }
    #if HAS_TEMP_BED
      #if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
        #define GEBED <=
      #else
        #define GEBED >=
      #endif
      if (current_temperature_bed_raw GEBED bed_maxttemp_raw && target_temperature_bed > 0) max_temp_error(-1);
      if (bed_minttemp_raw GEBED current_temperature_bed_raw && target_temperature_bed > 0) min_temp_error(-1);
    #endif
  } 
  adc_sensor_state = (ADCSensorState)((int(adc_sensor_state) + 1) % int(StartupDelay));
  #if ENABLED(BABYSTEPPING)
    LOOP_XYZ(axis) {
      const int curTodo = babystepsTodo[axis]; 
      if (curTodo > 0) {
        stepper.babystep((AxisEnum)axis, true); 
        babystepsTodo[axis]--;
      }
      else if (curTodo < 0) {
        stepper.babystep((AxisEnum)axis, false);  
        babystepsTodo[axis]++;
      }
    }
  #endif 
  #if ENABLED(PINS_DEBUGGING)
    extern bool endstop_monitor_flag;
    static uint8_t endstop_monitor_count = 16;  
    if (endstop_monitor_flag) {
      endstop_monitor_count += _BV(1);  
      endstop_monitor_count &= 0x7F;
      if (!endstop_monitor_count) endstop_monitor();  
    }
  #endif
  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    extern volatile uint8_t e_hit;
    if (e_hit && ENDSTOPS_ENABLED) {
      endstops.update();  
      e_hit--;
    }
  #endif
  cli();
  in_temp_isr = false;
  SBI(TIMSK0, OCIE0B); 
}
