#ifndef MARLIN_H
#define MARLIN_H
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "MarlinConfig.h"
#ifdef DEBUG_GCODE_PARSER
  #include "gcode.h"
#endif
#include "enum.h"
#include "types.h"
#include "fastio.h"
#include "utility.h"
#include "serial.h"
#if ENABLED(PRINTCOUNTER)
  #include "printcounter.h"
#else
  #include "stopwatch.h"
#endif
void idle(
  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    bool no_stepper_sleep = false  
  #endif
);
  void report_current_position(); 
void manage_inactivity(bool ignore_stepper_queue = false);
#if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
  extern bool extruder_duplication_enabled;
#endif
#if HAS_X2_ENABLE
  #define  enable_X() do{ X_ENABLE_WRITE( X_ENABLE_ON); X2_ENABLE_WRITE( X_ENABLE_ON); }while(0)
  #define disable_X() do{ X_ENABLE_WRITE(!X_ENABLE_ON); X2_ENABLE_WRITE(!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }while(0)
#elif HAS_X_ENABLE
  #define  enable_X() X_ENABLE_WRITE( X_ENABLE_ON)
  #define disable_X() do{ X_ENABLE_WRITE(!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }while(0)
#else
  #define  enable_X() NOOP
  #define disable_X() NOOP
#endif
#if HAS_Y2_ENABLE
  #define  enable_Y() do{ Y_ENABLE_WRITE( Y_ENABLE_ON); Y2_ENABLE_WRITE(Y_ENABLE_ON); }while(0)
  #define disable_Y() do{ Y_ENABLE_WRITE(!Y_ENABLE_ON); Y2_ENABLE_WRITE(!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }while(0)
#elif HAS_Y_ENABLE
  #define  enable_Y() Y_ENABLE_WRITE( Y_ENABLE_ON)
  #define disable_Y() do{ Y_ENABLE_WRITE(!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }while(0)
#else
  #define  enable_Y() NOOP
  #define disable_Y() NOOP
#endif
#if HAS_Z2_ENABLE
  #define  enable_Z() do{ Z_ENABLE_WRITE( Z_ENABLE_ON); Z2_ENABLE_WRITE(Z_ENABLE_ON); }while(0)
  #define disable_Z() do{ Z_ENABLE_WRITE(!Z_ENABLE_ON); Z2_ENABLE_WRITE(!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }while(0)
#elif HAS_Z_ENABLE
  #define  enable_Z() Z_ENABLE_WRITE( Z_ENABLE_ON)
  #define disable_Z() do{ Z_ENABLE_WRITE(!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }while(0)
#else
  #define  enable_Z() NOOP
  #define disable_Z() NOOP
#endif
#if ENABLED(MIXING_EXTRUDER)
  #if MIXING_STEPPERS > 3
    #define  enable_E0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); E2_ENABLE_WRITE( E_ENABLE_ON); E3_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_E0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); E2_ENABLE_WRITE(!E_ENABLE_ON); E3_ENABLE_WRITE(!E_ENABLE_ON); }
  #elif MIXING_STEPPERS > 2
    #define  enable_E0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); E2_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_E0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); E2_ENABLE_WRITE(!E_ENABLE_ON); }
  #else
    #define  enable_E0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_E0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); }
  #endif
  #define  enable_E1() NOOP
  #define disable_E1() NOOP
  #define  enable_E2() NOOP
  #define disable_E2() NOOP
  #define  enable_E3() NOOP
  #define disable_E3() NOOP
  #define  enable_E4() NOOP
  #define disable_E4() NOOP
#else 
  #if HAS_E0_ENABLE
    #define  enable_E0() E0_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_E0() E0_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_E0() NOOP
    #define disable_E0() NOOP
  #endif
  #if E_STEPPERS > 1 && HAS_E1_ENABLE
    #define  enable_E1() E1_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_E1() E1_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_E1() NOOP
    #define disable_E1() NOOP
  #endif
  #if E_STEPPERS > 2 && HAS_E2_ENABLE
    #define  enable_E2() E2_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_E2() E2_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_E2() NOOP
    #define disable_E2() NOOP
  #endif
  #if E_STEPPERS > 3 && HAS_E3_ENABLE
    #define  enable_E3() E3_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_E3() E3_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_E3() NOOP
    #define disable_E3() NOOP
  #endif
  #if E_STEPPERS > 4 && HAS_E4_ENABLE
    #define  enable_E4() E4_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_E4() E4_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_E4() NOOP
    #define disable_E4() NOOP
  #endif
#endif 
#if ENABLED(G38_PROBE_TARGET)
  extern bool G38_move,        
              G38_endstop_hit; 
#endif
#ifdef FYS_WIFI_HLK
typedef enum
{
    WIFI_SWITCH_STAGE_INIT=0,
    WIFI_SWITCH_STAGE_NET_MODE=1,
    WIFI_SWITCH_STAGE_NET_MODE_END=2,
    WIFI_SWITCH_STAGE_WIFI_CONF=3,
    WIFI_SWITCH_STAGE_WIFI_CONF_END=4,
    WIFI_SWITCH_STAGE_WIFI_CONF1=5,
    WIFI_SWITCH_STAGE_WIFI_CONF2=6,
    WIFI_SWITCH_STAGE_DHCPD_IP=7,
    WIFI_SWITCH_STAGE_DHCPD_IP_END=8,
    WIFI_SWITCH_STAGE_DHCPD_IP_END2=18,
    WIFI_SWITCH_STAGE_DHCPD_IP1=9,
    WIFI_SWITCH_STAGE_DHCPD_IP2=10,
    WIFI_SWITCH_STAGE_WAN_IP=11,
    WIFI_SWITCH_STAGE_WAN_IP_END=12,
    WIFI_SWITCH_STAGE_WAN_IP_END2=19,
    WIFI_SWITCH_STAGE_WAN_IP1=13,
    WIFI_SWITCH_STAGE_WAN_IP2=14,
    WIFI_SWITCH_STAGE_OUT_TRANS=15,
    WIFI_SWITCH_STAGE_OUT_TRANS_END=16,
    WIFI_SWITCH_STAGE_END=17,
} EM_WIFI_SWITCH_STAGE;
extern EM_WIFI_SWITCH_STAGE wifiSwitchModeStage;
#endif
#define _AXIS(AXIS) AXIS ##_AXIS
void enable_all_steppers();
void disable_e_steppers();
void disable_all_steppers();
void FlushSerialRequestResend();
void ok_to_send();
void kill(const char*);
#if ENABLED(FYS_NO_KILL_ONLY_TEMP_LCD_INFO)
void kill_temp(const char*);
#endif
void quickstop_stepper();
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  void handle_filament_runout();
#endif
extern uint8_t marlin_debug_flags;
#define DEBUGGING(F) (marlin_debug_flags & (DEBUG_## F))
#ifdef FYS_POSITION_ERR
extern bool position_error;
#endif
#if ENABLED(FYS_PRINTING_STATE)
enum MACRO_var_V019 
{
    MACRO_var_V01A = 0,
    MACRO_var_V01D = 3,
    MACRO_var_V01E = 4,
    MACRO_var_V01F = 5,
    MACRO_var_V023 = 9,
};
extern MACRO_var_V019 GLOBAL_var_V007;
#endif
#if ENABLED(FYS_LOOP_EVENT)
enum MACRO_var_V01B
{
    MACRO_var_V01C=0,
    MACRO_var_V022=1,
    MACRO_var_V021=2,
    MACRO_var_V020=3,
    MACRO_var_V025=4,
    MACRO_VAR_V056=5,
};
extern MACRO_var_V01B GLOBAL_var_V00E;
#endif
#if defined(FYS_SAFE_PRINT_BREAK)
bool recordEnvironment();
bool recordEnvironmentPure(); 
#endif
#if ENABLED(FYS_SHUTDOWN_ONCE_PRINT_DONE)
  #if PIN_EXISTS(PS_ON)
    extern bool GLOBAL_var_V005;
  #endif
#endif
#ifdef FYS_ENERGY_CONSERVE_HEIGHT
extern float zEnergyHeight;
extern bool ifEnergyConserve;
#endif
#include "Sd2Card.h" 
#if defined(FYS_SAFE_PRINT_BREAK)||defined(mySERIAL_Nums)
union cmdPos_t
{
    uint8_t n8[4];
    uint32_t n32;
};
extern volatile cmdPos_t currentCmdSdPos;
extern float feedrate_mm_s;
#ifdef FYS_ENERGY_CONSERVE_HEIGHT
extern float recordBedTemperature;
#endif
#endif
#if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
extern char GLOBAL_var_V004[FILENAME_LENGTH + 1];
#endif
#ifdef FYS_ACTIVE_TIME_OVER
extern millis_t max_inactive_time;
#endif
#if ENABLED(FYS_G28_Z_UNMOVE_SWITCH)
    extern bool myNeedRetPos;
#endif
#if ENABLED(FYS_POWER_STATUS)
  #if PIN_EXISTS(POW_BREAK_CHECK)||PIN_EXISTS(SHUTDOWN_CHECK)||PIN_EXISTS(PS_ON)
    enum MACRO_var_V00E
    {
        MACRO_var_V009,                    
        MACRO_var_V00A,                 
        MACRO_var_V00B,                
        POWER_ON_SHUTDOWN_BTN_PRESSING,   
        POWER_DOWN_SHUTDOWN_BTN_PRESSING  
    };
    extern MACRO_var_V00E GLOBAL_var_V003;
  #endif
#endif
extern bool Running;
inline bool IsRunning() { return  Running; }
inline bool IsStopped() { return !Running; }
bool enqueue_and_echo_command(const char* cmd, bool say_ok=false); 
void enqueue_and_echo_commands_P(const char * const cmd);          
void clear_command_queue();
extern millis_t previous_cmd_ms;
inline void refresh_cmd_timeout() { previous_cmd_ms = millis(); }
#if ENABLED(FAST_PWM_FAN)
  void setPwmFrequency(uint8_t pin, int val);
#endif
extern int16_t feedrate_percentage;
#define MMM_TO_MMS(MM_M) ((MM_M)/60.0)
#define MMS_TO_MMM(MM_S) ((MM_S)*60.0)
#define MMS_SCALED(MM_S) ((MM_S)*feedrate_percentage*0.01)
extern bool axis_relative_modes[];
extern bool volumetric_enabled;
extern int16_t flow_percentage[EXTRUDERS]; 
extern float filament_size[EXTRUDERS]; 
extern float volumetric_multiplier[EXTRUDERS]; 
extern bool axis_known_position[XYZ];
extern bool axis_homed[XYZ];
extern volatile bool wait_for_heatup;
extern uint8_t commands_in_queue;
extern uint8_t cmd_queue_index_r, cmd_queue_index_w;
extern float destination[XYZE];
#if HAS_RESUME_CONTINUE
  extern volatile bool wait_for_user;
#endif
extern float current_position[NUM_AXIS];
#if HAS_WORKSPACE_OFFSET
  #if HAS_HOME_OFFSET
    extern float home_offset[XYZ];
  #endif
  #if HAS_POSITION_SHIFT
    extern float position_shift[XYZ];
  #endif
#endif
#if HAS_HOME_OFFSET && HAS_POSITION_SHIFT
  extern float workspace_offset[XYZ];
  #define WORKSPACE_OFFSET(AXIS) workspace_offset[AXIS]
#elif HAS_HOME_OFFSET
  #define WORKSPACE_OFFSET(AXIS) home_offset[AXIS]
#elif HAS_POSITION_SHIFT
  #define WORKSPACE_OFFSET(AXIS) position_shift[AXIS]
#else
  #define WORKSPACE_OFFSET(AXIS) 0
#endif
#define LOGICAL_POSITION(POS, AXIS) ((POS) + WORKSPACE_OFFSET(AXIS))
#define RAW_POSITION(POS, AXIS)     ((POS) - WORKSPACE_OFFSET(AXIS))
#if HAS_POSITION_SHIFT || DISABLED(DELTA)
  #define LOGICAL_X_POSITION(POS)   LOGICAL_POSITION(POS, X_AXIS)
  #define LOGICAL_Y_POSITION(POS)   LOGICAL_POSITION(POS, Y_AXIS)
  #define RAW_X_POSITION(POS)       RAW_POSITION(POS, X_AXIS)
  #define RAW_Y_POSITION(POS)       RAW_POSITION(POS, Y_AXIS)
#else
  #define LOGICAL_X_POSITION(POS)   (POS)
  #define LOGICAL_Y_POSITION(POS)   (POS)
  #define RAW_X_POSITION(POS)       (POS)
  #define RAW_Y_POSITION(POS)       (POS)
#endif
#define LOGICAL_Z_POSITION(POS)     LOGICAL_POSITION(POS, Z_AXIS)
#define RAW_Z_POSITION(POS)         RAW_POSITION(POS, Z_AXIS)
#define RAW_CURRENT_POSITION(A)     RAW_##A##_POSITION(current_position[A##_AXIS])
#if HOTENDS > 1
  extern float hotend_offset[XYZ][HOTENDS];
#endif
extern float soft_endstop_min[XYZ], soft_endstop_max[XYZ];
#if HAS_SOFTWARE_ENDSTOPS
  extern bool soft_endstops_enabled;
  void clamp_to_software_endstops(float target[XYZ]);
#else
  #define soft_endstops_enabled false
  #define clamp_to_software_endstops(x) NOOP
#endif
#if HAS_WORKSPACE_OFFSET || ENABLED(DUAL_X_CARRIAGE)
  void update_software_endstops(const AxisEnum axis);
#endif
#if IS_KINEMATIC
  extern float delta[ABC];
  void inverse_kinematics(const float logical[XYZ]);
#endif
#if ENABLED(DELTA)
  extern float endstop_adj[ABC],
               delta_radius,
               delta_diagonal_rod,
               delta_calibration_radius,
               delta_segments_per_second,
               delta_tower_angle_trim[2],
               delta_clip_start_height;
  void recalc_delta_settings(float radius, float diagonal_rod);
#elif IS_SCARA
  void forward_kinematics_SCARA(const float &a, const float &b);
#endif
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  extern int bilinear_grid_spacing[2], bilinear_start[2];
  extern float bilinear_grid_factor[2],
               z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
  float bilinear_z_offset(const float logical[XYZ]);
#endif
#if ENABLED(AUTO_BED_LEVELING_UBL)
  typedef struct { double A, B, D; } linear_fit;
  linear_fit* lsf_linear_fit(double x[], double y[], double z[], const int);
#endif
#if HAS_LEVELING
  bool leveling_is_valid();
  bool leveling_is_active();
  void set_bed_leveling_enabled(const bool enable=true);
  void reset_bed_level();
#endif
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
  void set_z_fade_height(const float zfh);
#endif
#if ENABLED(Z_DUAL_ENDSTOPS)
  extern float z_endstop_adj;
#endif
#if HAS_BED_PROBE
  extern float zprobe_zoffset;
  #if ENABLED(FYS_RECORD_ZOFFSET_LAST)
    extern float zprobe_zoffset_last; 
  #endif
  void refresh_zprobe_zoffset(const bool no_babystep=false);
  #define DEPLOY_PROBE() set_probe_deployed(true)
  #define STOW_PROBE() set_probe_deployed(false)
#else
  #define DEPLOY_PROBE()
  #define STOW_PROBE()
#endif
#if ENABLED(HOST_KEEPALIVE_FEATURE)
  extern MarlinBusyState busy_state;
  #define KEEPALIVE_STATE(n) do{ busy_state = n; }while(0)
#else
  #define KEEPALIVE_STATE(n) NOOP
#endif
#if FAN_COUNT > 0
  extern int16_t fanSpeeds[FAN_COUNT];
  #if ENABLED(PROBING_FANS_OFF)
    extern bool fans_paused;
    extern int16_t paused_fanSpeeds[FAN_COUNT];
  #endif
#endif
#if ENABLED(BARICUDA)
  extern int baricuda_valve_pressure;
  extern int baricuda_e_to_p_pressure;
#endif
#if ENABLED(FILAMENT_WIDTH_SENSOR)
  extern bool filament_sensor;         
  extern float filament_width_nominal, 
               filament_width_meas;    
  extern uint8_t meas_delay_cm,        
                 measurement_delay[];  
  extern int8_t filwidth_delay_index[2]; 
#endif
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  extern bool filament_ran_out; 
  extern bool filament_ran_out_still_printing;
#endif
#if ENABLED(ADVANCED_PAUSE_FEATURE)
  extern AdvancedPauseMenuResponse advanced_pause_menu_response;
#endif
#if ENABLED(PID_EXTRUSION_SCALING)
  extern int lpq_len;
#endif
#if ENABLED(FWRETRACT)
  extern bool autoretract_enabled;
  extern bool retracted[EXTRUDERS]; 
  extern float retract_length, retract_length_swap, retract_feedrate_mm_s, retract_zlift;
  extern float retract_recover_length, retract_recover_length_swap, retract_recover_feedrate_mm_s;
#endif
#if ENABLED(PRINTCOUNTER)
  extern PrintCounter print_job_timer;
#else
  extern Stopwatch print_job_timer;
#endif
extern uint8_t active_extruder;
#if HAS_TEMP_HOTEND || HAS_TEMP_BED
  void print_heaterstates();
#endif
#if ENABLED(MIXING_EXTRUDER)
  extern float mixing_factor[MIXING_STEPPERS];
#endif
void calculate_volumetric_multipliers();
void do_blocking_move_to(const float &x, const float &y, const float &z, const float &fr_mm_s=0.0);
void do_blocking_move_to_x(const float &x, const float &fr_mm_s=0.0);
void do_blocking_move_to_z(const float &z, const float &fr_mm_s=0.0);
void do_blocking_move_to_xy(const float &x, const float &y, const float &fr_mm_s=0.0);
#if ENABLED(Z_PROBE_ALLEN_KEY) || ENABLED(Z_PROBE_SLED) || HAS_PROBING_PROCEDURE || HOTENDS > 1 || ENABLED(NOZZLE_CLEAN_FEATURE) || ENABLED(NOZZLE_PARK_FEATURE)
  bool axis_unhomed_error(const bool x=true, const bool y=true, const bool z=true);
#endif
#if IS_KINEMATIC 
  #if IS_SCARA
    extern const float L1, L2;
  #endif
  inline bool position_is_reachable_raw_xy(const float &rx, const float &ry) {
    #if ENABLED(DELTA)
      return HYPOT2(rx, ry) <= sq(DELTA_PRINTABLE_RADIUS);
    #elif IS_SCARA
      #if MIDDLE_DEAD_ZONE_R > 0
        const float R2 = HYPOT2(rx - SCARA_OFFSET_X, ry - SCARA_OFFSET_Y);
        return R2 >= sq(float(MIDDLE_DEAD_ZONE_R)) && R2 <= sq(L1 + L2);
      #else
        return HYPOT2(rx - SCARA_OFFSET_X, ry - SCARA_OFFSET_Y) <= sq(L1 + L2);
      #endif
    #else 
    #endif
  }
  inline bool position_is_reachable_by_probe_raw_xy(const float &rx, const float &ry) {
    return position_is_reachable_raw_xy(rx, ry)
        && position_is_reachable_raw_xy(rx - X_PROBE_OFFSET_FROM_EXTRUDER, ry - Y_PROBE_OFFSET_FROM_EXTRUDER);
  }
#else 
  inline bool position_is_reachable_raw_xy(const float &rx, const float &ry) {
      return WITHIN(rx, X_MIN_POS - 0.001, X_MAX_POS + 0.001)
          && WITHIN(ry, Y_MIN_POS - 0.001, Y_MAX_POS + 0.001);
  }
  inline bool position_is_reachable_by_probe_raw_xy(const float &rx, const float &ry) {
      return WITHIN(rx, MIN_PROBE_X - 0.001, MAX_PROBE_X + 0.001)
          && WITHIN(ry, MIN_PROBE_Y - 0.001, MAX_PROBE_Y + 0.001);
  }
#endif 
FORCE_INLINE bool position_is_reachable_by_probe_xy(const float &lx, const float &ly) {
  return position_is_reachable_by_probe_raw_xy(RAW_X_POSITION(lx), RAW_Y_POSITION(ly));
}
FORCE_INLINE bool position_is_reachable_xy(const float &lx, const float &ly) {
  return position_is_reachable_raw_xy(RAW_X_POSITION(lx), RAW_Y_POSITION(ly));
}
#if ENABLED(FYS_HOME_FUNCTION)
void FunV007();
void homeZ();
#endif
void gcode_G28(const bool always_home_all);
void do_probe_raise(const float z_raise);
#endif 
