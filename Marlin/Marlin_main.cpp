#include "Marlin.h"
#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "endstops.h"
#include "temperature.h"
#include "cardreader.h"
#include "configuration_store.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "nozzle.h"
#include "duration_t.h"
#include "types.h"
#include "gcode.h"
#ifdef FYS_CONTROL_PROTOCOL
#include "fysControlProtocol.h"
#endif
#if HAS_ABL
  #include "vector_3.h"
  #if ENABLED(AUTO_BED_LEVELING_LINEAR)
    #include "qr_solve.h"
  #endif
#elif ENABLED(MESH_BED_LEVELING)
  #include "mesh_bed_leveling.h"
#endif
#if ENABLED(BEZIER_CURVE_SUPPORT)
  #include "planner_bezier.h"
#endif
#if HAS_BUZZER && DISABLED(LCD_USE_I2C_BUZZER)
  #include "buzzer.h"
#endif
#if ENABLED(USE_WATCHDOG)
  #include "watchdog.h"
#endif
#if ENABLED(BLINKM)
  #include "blinkm.h"
  #include "Wire.h"
#endif
#if ENABLED(PCA9632)
  #include "pca9632.h"
#endif
#if HAS_SERVOS
  #include "servo.h"
#endif
#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif
#if ENABLED(DAC_STEPPER_CURRENT)
  #include "stepper_dac.h"
#endif
#if ENABLED(EXPERIMENTAL_I2CBUS)
  #include "twibus.h"
#endif
#if ENABLED(I2C_POSITION_ENCODERS)
  #include "I2CPositionEncoder.h"
#endif
#if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
  #include "endstop_interrupts.h"
#endif
#if ENABLED(M100_FREE_MEMORY_WATCHER)
  void gcode_M100();
  void M100_dump_routine(const char * const title, const char *start, const char *end);
#endif
#if ENABLED(SDSUPPORT)
  CardReader card;
#endif
#if ENABLED(EXPERIMENTAL_I2CBUS)
  TWIBus i2c;
#endif
#if ENABLED(G38_PROBE_TARGET)
  bool G38_move = false,
       G38_endstop_hit = false;
#endif
#if ENABLED(FYS_FILAMENT_RUNOUT_ADV)||ENABLED(FYS_FILAMENT_RUNOUT_ADV2)
bool filStateOld=false;
#endif
#if ENABLED(FYS_G28_Z_UNMOVE_SWITCH)
bool myNeedRetPos = false;
#endif
#ifdef FYS_POSITION_ERR
bool position_error = false;
#endif
#ifdef FYS_RESUME_PRINT_NO_HOME
bool g_resumePrintNoHome = false;
#endif
#if ENABLED(FYS_PRINTING_STATE)
MACRO_var_V019 GLOBAL_var_V007 = MACRO_var_V01A;
#endif
#if ENABLED(FYS_LOOP_EVENT)
MACRO_var_V01B GLOBAL_var_V00E = MACRO_var_V01C;
#endif
#ifdef FYS_ENERGY_CONSERVE_HEIGHT
bool ifEnergyConserve = false;
float zEnergyHeight = FYS_ENERGY_CONSERVE_HEIGHT;
float recordBedTemperature = 0;
#endif
#if ENABLED(AUTO_BED_LEVELING_UBL)
  #include "ubl.h"
  extern bool defer_return_to_status;
  unified_bed_leveling ubl;
  #define UBL_MESH_VALID !( ( ubl.z_values[0][0] == ubl.z_values[0][1] && ubl.z_values[0][1] == ubl.z_values[0][2] \
                           && ubl.z_values[1][0] == ubl.z_values[1][1] && ubl.z_values[1][1] == ubl.z_values[1][2] \
                           && ubl.z_values[2][0] == ubl.z_values[2][1] && ubl.z_values[2][1] == ubl.z_values[2][2] \
                           && ubl.z_values[0][0] == 0 && ubl.z_values[1][0] == 0 && ubl.z_values[2][0] == 0 )  \
                           || isnan(ubl.z_values[0][0]))
#endif
bool Running = true;
uint8_t marlin_debug_flags = DEBUG_NONE; 
float current_position[XYZE] = { 0.0 };
float destination[XYZE] = { 0.0 };
bool axis_homed[XYZ] = { false }, axis_known_position[XYZ] = { false };
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;
uint8_t commands_in_queue = 0; 
uint8_t cmd_queue_index_r = 0, 
               cmd_queue_index_w = 0; 
#if defined(FYS_SAFE_PRINT_BREAK)||defined(mySERIAL_Nums)
#define     CMDPOS_SPECIAL_LEN      6
#define     CMDPOS_SD_CMD_SIGN      0x00 
#define     CMDPOS_SD_POS           0x01
#define     CMDPOS_SERIAL_NUM       0x05
char command_queue[BUFSIZE][MAX_CMD_SIZE];
volatile cmdPos_t currentCmdSdPos;
#elif ENABLED(M100_FREE_MEMORY_WATCHER)
  char command_queue[BUFSIZE][MAX_CMD_SIZE];  
#else                                         
static char command_queue[BUFSIZE][MAX_CMD_SIZE];
#endif
static const char *injected_commands_P = NULL;
#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  TempUnit input_temp_units = TEMPUNIT_C;
#endif
const float homing_feedrate_mm_s[] PROGMEM = {
  #if ENABLED(DELTA)
    MMM_TO_MMS(HOMING_FEEDRATE_Z), MMM_TO_MMS(HOMING_FEEDRATE_Z),
  #else
    MMM_TO_MMS(HOMING_FEEDRATE_XY), MMM_TO_MMS(HOMING_FEEDRATE_XY),
  #endif
  MMM_TO_MMS(HOMING_FEEDRATE_Z), 0
};
FORCE_INLINE float homing_feedrate(const AxisEnum a) { return pgm_read_float(&homing_feedrate_mm_s[a]); }
float feedrate_mm_s = MMM_TO_MMS(1500.0);
static float saved_feedrate_mm_s;
int16_t feedrate_percentage = 100, saved_feedrate_percentage,
    flow_percentage[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(100);
bool axis_relative_modes[] = AXIS_RELATIVE_MODES,
     volumetric_enabled =
        #if ENABLED(VOLUMETRIC_DEFAULT_ON)
          true
        #else
          false
        #endif
      ;
float filament_size[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(DEFAULT_NOMINAL_FILAMENT_DIA),
      volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(1.0);
#if HAS_WORKSPACE_OFFSET
  #if HAS_POSITION_SHIFT
    float position_shift[XYZ] = { 0 };
  #endif
  #if HAS_HOME_OFFSET
    float home_offset[XYZ] = { 0 };
  #endif
  #if HAS_HOME_OFFSET && HAS_POSITION_SHIFT
    float workspace_offset[XYZ] = { 0 };
  #endif
#endif
#if HAS_SOFTWARE_ENDSTOPS
  bool soft_endstops_enabled = true;
#endif
float soft_endstop_min[XYZ] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
      soft_endstop_max[XYZ] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
#if FAN_COUNT > 0
  int16_t fanSpeeds[FAN_COUNT] = { 0 };
  #if ENABLED(PROBING_FANS_OFF)
    bool fans_paused = false;
    int16_t paused_fanSpeeds[FAN_COUNT] = { 0 };
  #endif
#endif
uint8_t active_extruder = 0;
static bool relative_mode = false;
volatile bool wait_for_heatup = true;
#if HAS_RESUME_CONTINUE
  volatile bool wait_for_user = false;
#endif
const char axis_codes[XYZE] = { 'X', 'Y', 'Z', 'E' };
#ifdef mySERIAL_Nums
static int serial_count[mySERIAL_Nums] = { 0 };
static char *serial_line_buffer[mySERIAL_Nums];
#else
static int serial_count = 0;
#endif
millis_t previous_cmd_ms = 0;
#ifdef FYS_ACTIVE_TIME_OVER
millis_t max_inactive_time = (FYS_ACTIVE_TIME_OVER)* 1000UL;
#else
static millis_t max_inactive_time = 0;
#endif
static millis_t stepper_inactive_time = (DEFAULT_STEPPER_DEACTIVE_TIME) * 1000UL;
#if ENABLED(PRINTCOUNTER)
  PrintCounter print_job_timer = PrintCounter();
#else
  Stopwatch print_job_timer = Stopwatch();
#endif
#if ENABLED(LCD_USE_I2C_BUZZER)
  #define BUZZ(d,f) lcd_buzz(d, f)
#elif PIN_EXISTS(BEEPER)
  Buzzer buzzer;
  #define BUZZ(d,f) buzzer.tone(d, f)
#else
  #define BUZZ(d,f) NOOP
#endif
static uint8_t target_extruder;
#if HAS_BED_PROBE
  float zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
  #if ENABLED(FYS_RECORD_ZOFFSET_LAST)
    float zprobe_zoffset_last = 0; 
  #endif
#endif
#if HAS_ABL
  float xy_probe_feedrate_mm_s = MMM_TO_MMS(XY_PROBE_SPEED);
  #define XY_PROBE_FEEDRATE_MM_S xy_probe_feedrate_mm_s
#elif defined(XY_PROBE_SPEED)
  #define XY_PROBE_FEEDRATE_MM_S MMM_TO_MMS(XY_PROBE_SPEED)
#else
  #define XY_PROBE_FEEDRATE_MM_S PLANNER_XY_FEEDRATE()
#endif
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #if ENABLED(DELTA)
    #define ADJUST_DELTA(V) \
      if (planner.abl_enabled) { \
        const float zadj = bilinear_z_offset(V); \
        delta[A_AXIS] += zadj; \
        delta[B_AXIS] += zadj; \
        delta[C_AXIS] += zadj; \
      }
  #else
    #define ADJUST_DELTA(V) if (planner.abl_enabled) { delta[Z_AXIS] += bilinear_z_offset(V); }
  #endif
#elif IS_KINEMATIC
  #define ADJUST_DELTA(V) NOOP
#endif
#if ENABLED(Z_DUAL_ENDSTOPS)
  float z_endstop_adj =
    #ifdef Z_DUAL_ENDSTOPS_ADJUSTMENT
      Z_DUAL_ENDSTOPS_ADJUSTMENT
    #else
      0
    #endif
  ;
#endif
#if HOTENDS > 1
  float hotend_offset[XYZ][HOTENDS];
#endif
#if HAS_Z_SERVO_ENDSTOP
  const int z_servo_angle[2] = Z_SERVO_ANGLES;
#endif
#if ENABLED(BARICUDA)
  int baricuda_valve_pressure = 0;
  int baricuda_e_to_p_pressure = 0;
#endif
#if ENABLED(FWRETRACT)
  bool autoretract_enabled = false;
  bool retracted[EXTRUDERS] = { false };
  bool retracted_swap[EXTRUDERS] = { false };
  float retract_length = RETRACT_LENGTH;
  float retract_length_swap = RETRACT_LENGTH_SWAP;
  float retract_feedrate_mm_s = RETRACT_FEEDRATE;
  float retract_zlift = RETRACT_ZLIFT;
  float retract_recover_length = RETRACT_RECOVER_LENGTH;
  float retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
  float retract_recover_feedrate_mm_s = RETRACT_RECOVER_FEEDRATE;
#endif 
#if HAS_POWER_SWITCH
  bool powersupply_on =
    #if ENABLED(PS_DEFAULT_OFF)
      false
    #else
      true
    #endif
  ;
#endif
#if ENABLED(DELTA)
  float delta[ABC],
        endstop_adj[ABC] = { 0 };
  float delta_radius,
        delta_tower_angle_trim[2],
        delta_tower[ABC][2],
        delta_diagonal_rod,
        delta_calibration_radius,
        delta_diagonal_rod_2_tower[ABC],
        delta_segments_per_second,
        delta_clip_start_height = Z_MAX_POS;
  float delta_safe_distance_from_top();
#endif
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  int bilinear_grid_spacing[2], bilinear_start[2]; 
  float bilinear_grid_factor[2],
        z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
#endif
#if IS_SCARA
  const float L1 = SCARA_LINKAGE_1, L2 = SCARA_LINKAGE_2,
              L1_2 = sq(float(L1)), L1_2_2 = 2.0 * L1_2,
              L2_2 = sq(float(L2));
  float delta_segments_per_second = SCARA_SEGMENTS_PER_SECOND,
        delta[ABC];
#endif
float cartes[XYZ] = { 0 }; 
#if ENABLED(FILAMENT_WIDTH_SENSOR)
  bool filament_sensor = false;                                 
  float filament_width_nominal = DEFAULT_NOMINAL_FILAMENT_DIA,  
        filament_width_meas = DEFAULT_MEASURED_FILAMENT_DIA;    
  uint8_t meas_delay_cm = MEASUREMENT_DELAY_CM,                 
          measurement_delay[MAX_MEASUREMENT_DELAY + 1];         
  int8_t filwidth_delay_index[2] = { 0, -1 };                   
#endif
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  bool filament_ran_out = false; 
  #if ENABLED(FYS_MIX_FIL_RANOUT_SAFE_PRINT_BREAK)
  bool filament_ran_out_still_printing = false; 
  #endif
#endif
#if ENABLED(ADVANCED_PAUSE_FEATURE)
  AdvancedPauseMenuResponse advanced_pause_menu_response;
#endif
#if ENABLED(MIXING_EXTRUDER)
  float mixing_factor[MIXING_STEPPERS]; 
  #if MIXING_VIRTUAL_TOOLS > 1
    float mixing_virtual_tool_mix[MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];
  #endif
#endif
static bool send_ok[BUFSIZE];
#ifdef FYS_WIFI_HLK
EM_WIFI_SWITCH_STAGE wifiSwitchModeStage = WIFI_SWITCH_STAGE_END; 
#endif
#if HAS_SERVOS
  Servo servo[NUM_SERVOS];
  #define MOVE_SERVO(I, P) servo[I].move(P)
  #if HAS_Z_SERVO_ENDSTOP
    #define DEPLOY_Z_SERVO() MOVE_SERVO(Z_ENDSTOP_SERVO_NR, z_servo_angle[0])
    #define STOW_Z_SERVO() MOVE_SERVO(Z_ENDSTOP_SERVO_NR, z_servo_angle[1])
  #endif
#endif
#ifdef CHDK
  millis_t chdkHigh = 0;
  bool chdkActive = false;
#endif
#ifdef AUTOMATIC_CURRENT_CONTROL
  bool auto_current_control = 0;
#endif
#if ENABLED(PID_EXTRUSION_SCALING)
  int lpq_len = 20;
#endif
#if ENABLED(HOST_KEEPALIVE_FEATURE)
  MarlinBusyState busy_state = NOT_BUSY;
  static millis_t next_busy_signal_ms = 0;
  uint8_t host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
#else
  #define host_keepalive() NOOP
#endif
#if ENABLED(I2C_POSITION_ENCODERS)
  I2CPositionEncodersMgr I2CPEM;
  uint8_t blockBufferIndexRef = 0;
  millis_t lastUpdateMillis;
#endif
#if ENABLED(CNC_WORKSPACE_PLANES)
  static WorkspacePlane workspace_plane = PLANE_XY;
#endif
#ifdef FYS_CONTROL_PROTOCOL
  DevStatus devStatus;
#endif
FORCE_INLINE float pgm_read_any(const float *p) { return pgm_read_float_near(p); }
FORCE_INLINE signed char pgm_read_any(const signed char *p) { return pgm_read_byte_near(p); }
#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  static const PROGMEM type array##_P[XYZ] = { X_##CONFIG, Y_##CONFIG, Z_##CONFIG }; \
  static inline type array(AxisEnum axis) { return pgm_read_any(&array##_P[axis]); } \
  typedef void __void_##CONFIG##__
XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,   MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,   MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,  HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,     MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,   HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir, HOME_DIR);
void stop();
void get_available_commands();
void process_next_command();
void prepare_move_to_destination();
void get_cartesian_from_steppers();
void set_current_from_steppers_for_axis(const AxisEnum axis);
#if ENABLED(ARC_SUPPORT)
  void plan_arc(float target[XYZE], float* offset, uint8_t clockwise);
#endif
#if ENABLED(BEZIER_CURVE_SUPPORT)
  void plan_cubic_move(const float offset[4]);
#endif
void tool_change(const uint8_t tmp_extruder, const float fr_mm_s=0.0, bool no_move=false);
void report_current_position();
void report_current_position_detail();
#if ENABLED(DEBUG_LEVELING_FEATURE)
  void print_xyz(const char* prefix, const char* suffix, const float x, const float y, const float z) {
    serialprintPGM(prefix);
    SERIAL_CHAR('(');
    SERIAL_ECHO(x);
    SERIAL_ECHOPAIR(", ", y);
    SERIAL_ECHOPAIR(", ", z);
    SERIAL_CHAR(')');
    if (suffix) serialprintPGM(suffix); else SERIAL_EOL();
  }
  void print_xyz(const char* prefix, const char* suffix, const float xyz[]) {
    print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
  }
  #if HAS_ABL
    void print_xyz(const char* prefix, const char* suffix, const vector_3 &xyz) {
      print_xyz(prefix, suffix, xyz.x, xyz.y, xyz.z);
    }
  #endif
  #define DEBUG_POS(SUFFIX,VAR) do { \
    print_xyz(PSTR("  " STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR); }while(0)
#endif
void sync_plan_position() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position", current_position);
  #endif
  planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
inline void sync_plan_position_e() { planner.set_e_position_mm(current_position[E_AXIS]); }
#if IS_KINEMATIC
  inline void sync_plan_position_kinematic() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position_kinematic", current_position);
    #endif
    planner.set_position_mm_kinematic(current_position);
  }
  #define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position_kinematic()
#else
  #define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()
#endif
#if ENABLED(SDSUPPORT)
  #include "SdFatUtil.h"
  int freeMemory() { return SdFatUtil::FreeRam(); }
#else
extern "C" {
  extern char __bss_end;
  extern char __heap_start;
  extern void* __brkval;
  int freeMemory() {
    int free_memory;
    if ((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
  }
}
#endif 
#if ENABLED(DIGIPOT_I2C)
  extern void digipot_i2c_set_current(uint8_t channel, float current);
  extern void digipot_i2c_init();
#endif
static bool drain_injected_commands_P() {
  if (injected_commands_P != NULL) {
    size_t i = 0;
    char c, cmd[30];
    strncpy_P(cmd, injected_commands_P, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';
    while ((c = cmd[i]) && c != '\n') i++; 
    cmd[i] = '\0';
    if (enqueue_and_echo_command(cmd))     
      injected_commands_P = c ? injected_commands_P + i + 1 : NULL; 
  }
  return (injected_commands_P != NULL);    
}
void enqueue_and_echo_commands_P(const char * const pgcode) {
  injected_commands_P = pgcode;
  drain_injected_commands_P(); 
}
void clear_command_queue() {
  cmd_queue_index_r = cmd_queue_index_w;
  commands_in_queue = 0;
}
inline void _commit_command(bool say_ok) {
    send_ok[cmd_queue_index_w] = say_ok;
    if (++cmd_queue_index_w >= BUFSIZE) cmd_queue_index_w = 0;
    commands_in_queue++;
}
inline bool _enqueuecommand(const char* cmd, bool say_ok=false) {
  if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;
  strcpy(command_queue[cmd_queue_index_w] 
    #if defined(FYS_SAFE_PRINT_BREAK) || defined(mySERIAL_Nums)
      + CMDPOS_SPECIAL_LEN
    #endif
      , cmd);
  _commit_command(say_ok);
  return true;
}
bool enqueue_and_echo_command(const char* cmd, bool say_ok) {
  if (_enqueuecommand(cmd, say_ok)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOPAIR(MSG_ENQUEUEING, cmd);
    SERIAL_CHAR('"');
    SERIAL_EOL();
    return true;
  }
  return false;
}
void setup_killpin() {
  #if HAS_KILL
    SET_INPUT_PULLUP(KILL_PIN);
  #endif
}
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  void setup_filrunoutpin() {
    #if ENABLED(ENDSTOPPULLUP_FIL_RUNOUT)
      SET_INPUT_PULLUP(FIL_RUNOUT_PIN);
    #else
      SET_INPUT(FIL_RUNOUT_PIN);
    #endif
    #if ENABLED(FYS_FILAMENT_RUNOUT_ADV)
    {
      filStateOld = READ(FIL_RUNOUT_PIN);
    }
    #endif
  }
#endif
void setup_powerhold() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if HAS_POWER_SWITCH
    #if ENABLED(PS_DEFAULT_OFF)
      OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
      OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
    #endif
  #endif
}
void suicide() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}
void servo_init() {
  #if NUM_SERVOS >= 1 && HAS_SERVO_0
    servo[0].attach(SERVO0_PIN);
    servo[0].detach(); 
  #endif
  #if NUM_SERVOS >= 2 && HAS_SERVO_1
    servo[1].attach(SERVO1_PIN);
    servo[1].detach();
  #endif
  #if NUM_SERVOS >= 3 && HAS_SERVO_2
    servo[2].attach(SERVO2_PIN);
    servo[2].detach();
  #endif
  #if NUM_SERVOS >= 4 && HAS_SERVO_3
    servo[3].attach(SERVO3_PIN);
    servo[3].detach();
  #endif
  #if HAS_Z_SERVO_ENDSTOP
    STOW_Z_SERVO();
  #endif
}
#if HAS_STEPPER_RESET
  void disableStepperDrivers() {
    OUT_WRITE(STEPPER_RESET_PIN, LOW);  
  }
  void enableStepperDrivers() { SET_INPUT(STEPPER_RESET_PIN); }  
#endif
#if ENABLED(EXPERIMENTAL_I2CBUS) && I2C_SLAVE_ADDRESS > 0
  void i2c_on_receive(int bytes) { 
    i2c.receive(bytes);
  }
  void i2c_on_request() {          
    i2c.reply("Hello World!\n");
  }
#endif
#if HAS_COLOR_LEDS
  void set_led_color(
    const uint8_t r, const uint8_t g, const uint8_t b
      #if ENABLED(RGBW_LED)
        , const uint8_t w=0
      #endif
  ) {
    #if ENABLED(BLINKM)
      SendColors(r, g, b);
    #endif
    #if ENABLED(RGB_LED) || ENABLED(RGBW_LED)
      WRITE(RGB_LED_R_PIN, r ? HIGH : LOW);
      WRITE(RGB_LED_G_PIN, g ? HIGH : LOW);
      WRITE(RGB_LED_B_PIN, b ? HIGH : LOW);
      analogWrite(RGB_LED_R_PIN, r);
      analogWrite(RGB_LED_G_PIN, g);
      analogWrite(RGB_LED_B_PIN, b);
      #if ENABLED(RGBW_LED)
        WRITE(RGB_LED_W_PIN, w ? HIGH : LOW);
        analogWrite(RGB_LED_W_PIN, w);
      #endif
    #endif
    #if ENABLED(PCA9632)
      PCA9632_SetColor(r, g, b);
    #endif
  }
#endif 
void gcode_line_error(const char* err, bool doFlush = true) {
#ifdef mySERIAL_Nums
    char t = MYSERIAL.activeTxSerial;
    MYSERIAL.setTxActiveSerial(MYSERIAL.activeRxSerial);
#endif
  SERIAL_ERROR_START();
  serialprintPGM(err);
  SERIAL_ERRORLN(gcode_LastN);
  if (doFlush) FlushSerialRequestResend();
#ifdef mySERIAL_Nums
  MYSERIAL.setTxActiveSerial(t);
  serial_count[MYSERIAL.activeRxSerial] = 0;
#else
  serial_count = 0;
#endif
}
inline void get_serial_commands() {
#ifdef mySERIAL_Nums
    static bool serial_comment_mode[mySERIAL_Nums] = { false };
  {
      char i;
      for (i = 0; i < mySERIAL_Nums; i++)
      {
          if (MYSERIAL.activeRxSerial >= mySERIAL_Nums)MYSERIAL.activeRxSerial = 0;
          MYSERIAL.setRxActiveSerial(MYSERIAL.activeRxSerial);
          if (MYSERIAL.available()>0 && MYSERIAL.sCmd[MYSERIAL.activeRxSerial])break;
          MYSERIAL.activeRxSerial++;
      }
      if (i == mySERIAL_Nums)
      {
          return;
      }
  }
#else
    static char serial_line_buffer[MAX_CMD_SIZE];
    static bool serial_comment_mode = false;
#endif
  #if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
    static millis_t last_command_time = 0;
    const millis_t ms = millis();
    if (commands_in_queue == 0 && !MYSERIAL.available() && ELAPSED(ms, last_command_time + NO_TIMEOUTS)) {
      SERIAL_ECHOLNPGM(MSG_WAIT);
      last_command_time = ms;
    }
  #endif
  while (commands_in_queue < BUFSIZE && MYSERIAL.available() > 0) {
    char serial_char = MYSERIAL.read();
    if (serial_char == '\n' || serial_char == '\r') {
    #ifdef mySERIAL_Nums
      serial_comment_mode[MYSERIAL.activeRxSerial] = false; 
      if (!serial_count[MYSERIAL.activeRxSerial]) continue; 
      serial_line_buffer[MYSERIAL.activeRxSerial][serial_count[MYSERIAL.activeRxSerial]] = 0; 
      serial_count[MYSERIAL.activeRxSerial] = 0; 
      char* command = serial_line_buffer[MYSERIAL.activeRxSerial] + 1;
    #else
      serial_comment_mode = false; 
      if (!serial_count) continue; 
      serial_line_buffer[serial_count] = 0; 
      serial_count = 0; 
      char* command = serial_line_buffer;
    #endif
      while (*command == ' ') command++; 
      char *npos = (*command == 'N') ? command : NULL, 
           *apos = strchr(command, '*');
      if (npos) {
        bool M110 = strstr_P(command, PSTR("M110")) != NULL;
        if (M110) {
          char* n2pos = strchr(command + 4, 'N');
          if (n2pos) npos = n2pos;
        }
        gcode_N = strtol(npos + 1, NULL, 10);
        if (gcode_N != gcode_LastN + 1 && !M110) {
          gcode_line_error(PSTR(MSG_ERR_LINE_NO));
          return;
        }
        if (apos) {
          byte checksum = 0, count = 0;
          while (command[count] != '*') checksum ^= command[count++];
          if (strtol(apos + 1, NULL, 10) != checksum) {
            gcode_line_error(PSTR(MSG_ERR_CHECKSUM_MISMATCH));
            return;
          }
        }
        else {
          gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM));
          return;
        }
        gcode_LastN = gcode_N;
      }
      else if (apos) { 
        gcode_line_error(PSTR(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM), false);
        return;
      }
      if (IsStopped()) {
        char* gpos = strchr(command, 'G');
        if (gpos) {
          const int codenum = strtol(gpos + 1, NULL, 10);
          switch (codenum) {
            case 0:
            case 1:
            case 2:
            case 3:
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
              break;
          }
        }
      }
      #if DISABLED(EMERGENCY_PARSER)
        if (strcmp(command, "M108") == 0) {
          wait_for_heatup = false;
        }
        if (strcmp(command, "M112") == 0) kill(PSTR(MSG_KILLED));
        if (strcmp(command, "M410") == 0) { quickstop_stepper(); }
      #endif
      #if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
        last_command_time = ms;
      #endif
    #ifdef FYS_WIFI_HLK
      if( MYSERIAL.activeRxSerial == 1 )
      {
        if(wifiSwitchModeStage==WIFI_SWITCH_STAGE_INIT)
        {
            if( 0==strncmp(command,"at+wifi_conf",12) )
            {
                wifiSwitchModeStage = WIFI_SWITCH_STAGE_WIFI_CONF;
            }
            return;
        }
        if(wifiSwitchModeStage==WIFI_SWITCH_STAGE_WIFI_CONF)
        {
            wifiSwitchModeStage=WIFI_SWITCH_STAGE_WIFI_CONF_END;
            char* pos = strchr(command, ',');
            char ssid[30]="SSID:";
            strncat(ssid,command,pos-command);
            SERIAL_ECHOLN(ssid);
            lcd_setstatus(ssid);
            char tAs = MYSERIAL.activeTxSerial;
            MYSERIAL.setTxActiveSerial(1);
            SERIAL_ECHOLNPGM("at+netmode=?\r");
            MYSERIAL.setTxActiveSerial(tAs);
            return;
        }
        if(wifiSwitchModeStage==WIFI_SWITCH_STAGE_WIFI_CONF_END)
        {
            if( 0==strncmp(command,"at+netmode",10) )
            {
                wifiSwitchModeStage = WIFI_SWITCH_STAGE_NET_MODE;
            }
            return;
        }
        if(wifiSwitchModeStage==WIFI_SWITCH_STAGE_NET_MODE)
        {
            if(strcmp(command, "0") == 0 || strcmp(command, "3") == 0)
            {
                char tAs = MYSERIAL.activeTxSerial;
                MYSERIAL.setTxActiveSerial(1);
                SERIAL_ECHOLNPGM("at+dhcpd_ip=?\r");
                MYSERIAL.setTxActiveSerial(tAs);
                wifiSwitchModeStage = WIFI_SWITCH_STAGE_DHCPD_IP;
            }
            else if(strcmp(command, "2") == 0)
            {
                char tAs = MYSERIAL.activeTxSerial;
                MYSERIAL.setTxActiveSerial(1);
                SERIAL_ECHOLNPGM("at+net_wanip=?\r");
                MYSERIAL.setTxActiveSerial(tAs);
                wifiSwitchModeStage = WIFI_SWITCH_STAGE_WAN_IP;
            }
            return;
        }
        if(wifiSwitchModeStage==WIFI_SWITCH_STAGE_DHCPD_IP)
        {
            if( 0==strncmp(command,"at+dhcpd_ip",11) )
            {
                wifiSwitchModeStage = WIFI_SWITCH_STAGE_DHCPD_IP_END;
            }
            return;
        }
        if( wifiSwitchModeStage==WIFI_SWITCH_STAGE_DHCPD_IP_END)
        {
            wifiSwitchModeStage=WIFI_SWITCH_STAGE_OUT_TRANS;
            char* pos = strrchr(command, ',');
            char ip[30]="";
            strncat(ip,pos+1,strlen(command)-(pos-command+1));
            lcd_setstatus(ip);
            char tAs = MYSERIAL.activeTxSerial;
            MYSERIAL.setTxActiveSerial(1);
            SERIAL_ECHOLNPGM("at+out_trans=0\r");
            MYSERIAL.setTxActiveSerial(tAs);
            return;
        }
        if(wifiSwitchModeStage==WIFI_SWITCH_STAGE_WAN_IP)
        {
            if( 0==strncmp(command,"at+net_wanip",12) )
            {
                wifiSwitchModeStage = WIFI_SWITCH_STAGE_WAN_IP_END;
            }
            return;
        }
        if( wifiSwitchModeStage==WIFI_SWITCH_STAGE_WAN_IP_END)
        {
            wifiSwitchModeStage=WIFI_SWITCH_STAGE_OUT_TRANS;
            char* pos = strchr(command, ',');
            char ip[30]="";
            strncpy(ip,command,pos-command);
            lcd_setstatus(ip);
            char tAs = MYSERIAL.activeTxSerial;
            MYSERIAL.setTxActiveSerial(1);
            SERIAL_ECHOLNPGM("at+out_trans=0\r");
            MYSERIAL.setTxActiveSerial(tAs);
            return;
        }
        if(wifiSwitchModeStage==WIFI_SWITCH_STAGE_OUT_TRANS)
        {
            if( 0==strncmp(command,"at+out_trans",12) )
            {
                wifiSwitchModeStage = WIFI_SWITCH_STAGE_OUT_TRANS_END;
            }
            return;
        }
        if(wifiSwitchModeStage==WIFI_SWITCH_STAGE_OUT_TRANS_END)
        {
            if( 0==strncmp(command,"ok",2) )
            {
                wifiSwitchModeStage = WIFI_SWITCH_STAGE_END;
            }
            return;
        }
    }
    #endif
    #ifdef mySERIAL_Nums
        command_queue[cmd_queue_index_w][CMDPOS_SD_CMD_SIGN] = 0x00;
        command_queue[cmd_queue_index_w][CMDPOS_SERIAL_NUM]=serial_line_buffer[MYSERIAL.activeRxSerial][0];
        _enqueuecommand(serial_line_buffer[MYSERIAL.activeRxSerial] + 1, true);
    #else
      _enqueuecommand(serial_line_buffer, true);
    #endif
    }
    #ifdef mySERIAL_Nums
    else if (serial_count[MYSERIAL.activeRxSerial] >= MAX_CMD_SIZE - 1) {
    }
    #else
    else if (serial_count >= MAX_CMD_SIZE - 1) {
    }
    #endif
    else if (serial_char == '\\') {  
      if (MYSERIAL.available() > 0) {
        serial_char = MYSERIAL.read();
        #ifdef mySERIAL_Nums
        if (!serial_comment_mode[MYSERIAL.activeRxSerial])
        {
            if (serial_count[MYSERIAL.activeRxSerial] == 0)
            {
                serial_line_buffer[MYSERIAL.activeRxSerial][0] = MYSERIAL.activeRxSerial;
                serial_count[MYSERIAL.activeRxSerial] = 1;
            }
            serial_line_buffer[MYSERIAL.activeRxSerial][serial_count[MYSERIAL.activeRxSerial]++] = serial_char;
        }
        #else
        if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
        #endif
      }
    }
    else { 
        #ifdef mySERIAL_Nums
        if (serial_char == ';') serial_comment_mode[MYSERIAL.activeRxSerial] = true;
        if (!serial_comment_mode[MYSERIAL.activeRxSerial])
        {
            if (serial_count[MYSERIAL.activeRxSerial] == 0)
            {
                serial_line_buffer[MYSERIAL.activeRxSerial][0] = MYSERIAL.activeRxSerial;
                serial_count[MYSERIAL.activeRxSerial] = 1;
            }
            serial_line_buffer[MYSERIAL.activeRxSerial][serial_count[MYSERIAL.activeRxSerial]++] = serial_char;
        }
        #else
        if (serial_char == ';') serial_comment_mode = true;
        if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
        #endif
    }
  } 
}
#if ENABLED(SDSUPPORT)
    #if defined(FYS_SAFE_PRINT_BREAK)||defined(mySERIAL_Nums)
    inline void pushSdPosToCommands()
    {
        static uint32_t lastSdPos = 0;
        command_queue[cmd_queue_index_w][CMDPOS_SD_CMD_SIGN] = 0x01;
        currentCmdSdPos.n32 = lastSdPos;
        for (char i = 0; i < 4; i++)
            command_queue[cmd_queue_index_w][CMDPOS_SD_POS + i] = currentCmdSdPos.n8[i];
        lastSdPos = card.getFilePos();
    }
    #endif
  inline void get_sdcard_commands() {
    static bool stop_buffering = false,
                sd_comment_mode = false;
    if (!card.sdprinting)
        return;
    if (commands_in_queue == 0) stop_buffering = false;
    uint16_t sd_count = 0;
    bool card_eof = card.eof();
    while (commands_in_queue < BUFSIZE && !card_eof && !stop_buffering) {
      const int16_t n = card.get();
      char sd_char = (char)n;
      card_eof = card.eof();
      if (card_eof || n == -1
          || sd_char == '\n' || sd_char == '\r'
          || ((sd_char == '#' || sd_char == ':') && !sd_comment_mode)
      ) {
        if (card_eof) {
          SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
          card.printingHasFinished();
          #if ENABLED(PRINTER_EVENT_LEDS)
            LCD_MESSAGEPGM(MSG_INFO_COMPLETED_PRINTS);
            set_led_color(0, 255, 0); 
            #if HAS_RESUME_CONTINUE
              enqueue_and_echo_commands_P(PSTR("M0")); 
            #else
              safe_delay(1000);
            #endif
            set_led_color(0, 0, 0);   
          #endif
          card.checkautostart(true);
          #if ENABLED(FYS_LOOP_EVENT)
            GLOBAL_var_V00E = MACRO_var_V025;
          #endif
          #if ENABLED(FYS_LCD_EVENT)
            GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_VAR_V058);
          #endif
        }
        else if (n == -1) {
          SERIAL_ERROR_START();
          SERIAL_ECHOLNPGM(MSG_SD_ERR_READ);
        }
        if (sd_char == '#') stop_buffering = true;
        sd_comment_mode = false; 
        if (!sd_count) continue; 
        command_queue[cmd_queue_index_w][
            #if defined(FYS_SAFE_PRINT_BREAK)||defined(mySERIAL_Nums) 
            CMDPOS_SPECIAL_LEN +
            #endif
            sd_count
        ] = '\0'; 
        sd_count = 0; 
        #if defined(FYS_SAFE_PRINT_BREAK)||defined(mySERIAL_Nums)
        pushSdPosToCommands();
        cli();
        _commit_command(false);
        sei();
        #else
        _commit_command(false);
        #endif
      }
      else if (sd_count >= MAX_CMD_SIZE - 1) {
      }
      else {
        if (sd_char == ';') sd_comment_mode = true;
        if (!sd_comment_mode) 
        {
            command_queue[cmd_queue_index_w][
                #if defined(FYS_SAFE_PRINT_BREAK)||defined(mySERIAL_Nums)
                CMDPOS_SPECIAL_LEN +    
                #endif
                sd_count++
            ] = sd_char;
        }
      }
    }
  }
#endif 
void get_available_commands() {
  if (drain_injected_commands_P()) return;
  get_serial_commands();
  #if ENABLED(SDSUPPORT)
    get_sdcard_commands();
  #endif
}
bool get_target_extruder_from_command(const uint16_t code) {
  if (parser.seenval('T')) {
    const int8_t e = parser.value_byte();
    if (e >= EXTRUDERS) {
      SERIAL_ECHO_START();
      SERIAL_CHAR('M');
      SERIAL_ECHO(code);
      SERIAL_ECHOLNPAIR(" " MSG_INVALID_EXTRUDER " ", e);
      return true;
    }
    target_extruder = e;
  }
  else
    target_extruder = active_extruder;
  return false;
}
#if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
  bool extruder_duplication_enabled = false; 
#endif
#if ENABLED(DUAL_X_CARRIAGE)
  static DualXMode dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
  static float x_home_pos(const int extruder) {
    if (extruder == 0)
      return LOGICAL_X_POSITION(base_home_pos(X_AXIS));
    else
      return LOGICAL_X_POSITION(hotend_offset[X_AXIS][1] > 0 ? hotend_offset[X_AXIS][1] : X2_HOME_POS);
  }
  static int x_home_dir(const int extruder) { return extruder ? X2_HOME_DIR : X_HOME_DIR; }
  static float inactive_extruder_x_pos = X2_MAX_POS; 
  static bool active_extruder_parked = false;        
  static float raised_parked_position[XYZE];         
  static millis_t delayed_move_time = 0;             
  static float duplicate_extruder_x_offset = DEFAULT_DUPLICATION_X_OFFSET; 
  static int16_t duplicate_extruder_temp_offset = 0; 
#endif 
#if HAS_WORKSPACE_OFFSET || ENABLED(DUAL_X_CARRIAGE)
  void update_software_endstops(const AxisEnum axis) {
    const float offs = 0.0
      #if HAS_HOME_OFFSET
        + home_offset[axis]
      #endif
      #if HAS_POSITION_SHIFT
        + position_shift[axis]
      #endif
    ;
    #if HAS_HOME_OFFSET && HAS_POSITION_SHIFT
      workspace_offset[axis] = offs;
    #endif
    #if ENABLED(DUAL_X_CARRIAGE)
      if (axis == X_AXIS) {
        float dual_max_x = max(hotend_offset[X_AXIS][1], X2_MAX_POS);
        if (active_extruder != 0) {
          soft_endstop_min[X_AXIS] = X2_MIN_POS + offs;
          soft_endstop_max[X_AXIS] = dual_max_x + offs;
        }
        else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
          soft_endstop_min[X_AXIS] = base_min_pos(X_AXIS) + offs;
          soft_endstop_max[X_AXIS] = min(base_max_pos(X_AXIS), dual_max_x - duplicate_extruder_x_offset) + offs;
        }
        else {
          soft_endstop_min[axis] = base_min_pos(axis) + offs;
          soft_endstop_max[axis] = base_max_pos(axis) + offs;
        }
      }
    #else
      soft_endstop_min[axis] = base_min_pos(axis) + offs;
      soft_endstop_max[axis] = base_max_pos(axis) + offs;
    #endif
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPAIR("For ", axis_codes[axis]);
        #if HAS_HOME_OFFSET
          SERIAL_ECHOPAIR(" axis:\n home_offset = ", home_offset[axis]);
        #endif
        #if HAS_POSITION_SHIFT
          SERIAL_ECHOPAIR("\n position_shift = ", position_shift[axis]);
        #endif
        SERIAL_ECHOPAIR("\n soft_endstop_min = ", soft_endstop_min[axis]);
        SERIAL_ECHOLNPAIR("\n soft_endstop_max = ", soft_endstop_max[axis]);
      }
    #endif
    #if ENABLED(DELTA)
      if (axis == Z_AXIS)
        delta_clip_start_height = soft_endstop_max[axis] - delta_safe_distance_from_top();
    #endif
  }
#endif 
#if HAS_M206_COMMAND
  static void set_home_offset(const AxisEnum axis, const float v) {
    current_position[axis] += v - home_offset[axis];
    home_offset[axis] = v;
    update_software_endstops(axis);
  }
#endif 
static void set_axis_is_at_home(const AxisEnum axis) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR(">>> set_axis_is_at_home(", axis_codes[axis]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif
  axis_known_position[axis] = axis_homed[axis] = true;
  #if HAS_POSITION_SHIFT
    position_shift[axis] = 0;
    update_software_endstops(axis);
  #endif
  #if ENABLED(DUAL_X_CARRIAGE)
    if (axis == X_AXIS && (active_extruder == 1 || dual_x_carriage_mode == DXC_DUPLICATION_MODE)) {
      current_position[X_AXIS] = x_home_pos(active_extruder);
      return;
    }
  #endif
  #if ENABLED(MORGAN_SCARA)
    if (axis == X_AXIS || axis == Y_AXIS) {
      float homeposition[XYZ];
      LOOP_XYZ(i) homeposition[i] = LOGICAL_POSITION(base_home_pos((AxisEnum)i), i);
      inverse_kinematics(homeposition);
      forward_kinematics_SCARA(delta[A_AXIS], delta[B_AXIS]);
      current_position[axis] = LOGICAL_POSITION(cartes[axis], axis);
      soft_endstop_min[axis] = base_min_pos(axis); 
      soft_endstop_max[axis] = base_max_pos(axis); 
    }
    else
  #endif
  {
    current_position[axis] = LOGICAL_POSITION(base_home_pos(axis), axis);
  }
  #if HAS_BED_PROBE && Z_HOME_DIR < 0
    if (axis == Z_AXIS) {
      #if HOMING_Z_WITH_PROBE
        current_position[Z_AXIS] -= zprobe_zoffset;
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_ECHOLNPGM("*** Z HOMED WITH PROBE (Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN) ***");
            SERIAL_ECHOLNPAIR("> zprobe_zoffset = ", zprobe_zoffset);
          }
        #endif
      #elif ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("*** Z HOMED TO ENDSTOP (Z_MIN_PROBE_ENDSTOP) ***");
      #endif
    }
  #endif
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      #if HAS_HOME_OFFSET
        SERIAL_ECHOPAIR("> home_offset[", axis_codes[axis]);
        SERIAL_ECHOLNPAIR("] = ", home_offset[axis]);
      #endif
      DEBUG_POS("", current_position);
      SERIAL_ECHOPAIR("<<< set_axis_is_at_home(", axis_codes[axis]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif
  #if ENABLED(I2C_POSITION_ENCODERS)
    I2CPEM.homed(axis);
  #endif
}
inline float get_homing_bump_feedrate(const AxisEnum axis) {
  static const uint8_t homing_bump_divisor[] PROGMEM = HOMING_BUMP_DIVISOR;
  uint8_t hbd = pgm_read_byte(&homing_bump_divisor[axis]);
  if (hbd < 1) {
    hbd = 10;
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM("Warning: Homing Bump Divisor < 1");
  }
  return homing_feedrate(axis) / hbd;
}
inline void line_to_current_position() {
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate_mm_s, active_extruder);
}
inline void line_to_destination(const float fr_mm_s) {
  planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, active_extruder);
}
inline void line_to_destination() { line_to_destination(feedrate_mm_s); }
inline void set_current_to_destination() { COPY(current_position, destination); }
inline void set_destination_to_current() { COPY(destination, current_position); }
#if IS_KINEMATIC
  void prepare_uninterpolated_move_to_destination(const float fr_mm_s=0.0) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("prepare_uninterpolated_move_to_destination", destination);
    #endif
    refresh_cmd_timeout();
    #if UBL_DELTA
      ubl.prepare_segmented_line_to(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s));
    #else
      if ( current_position[X_AXIS] == destination[X_AXIS]
        && current_position[Y_AXIS] == destination[Y_AXIS]
        && current_position[Z_AXIS] == destination[Z_AXIS]
        && current_position[E_AXIS] == destination[E_AXIS]
      ) return;
      planner.buffer_line_kinematic(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s), active_extruder);
    #endif
    set_current_to_destination();
  }
#endif 
void do_blocking_move_to(const float &lx, const float &ly, const float &lz, const float &fr_mm_s) {
  const float old_feedrate_mm_s = feedrate_mm_s;
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) print_xyz(PSTR(">>> do_blocking_move_to"), NULL, lx, ly, lz);
  #endif
  #if ENABLED(DELTA)
    if (!position_is_reachable_xy(lx, ly)) return;
    feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;
    set_destination_to_current();          
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("set_destination_to_current", destination);
    #endif
    if (current_position[Z_AXIS] > delta_clip_start_height) {
      if (lz > delta_clip_start_height) {   
        destination[X_AXIS] = lx;           
        destination[Y_AXIS] = ly;
        destination[Z_AXIS] = lz;
        prepare_uninterpolated_move_to_destination(); 
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("danger zone move", current_position);
        #endif
        return;
      }
      else {
        destination[Z_AXIS] = delta_clip_start_height;
        prepare_uninterpolated_move_to_destination(); 
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("zone border move", current_position);
        #endif
      }
    }
    if (lz > current_position[Z_AXIS]) {    
      destination[Z_AXIS] = lz;
      prepare_uninterpolated_move_to_destination();   
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("z raise move", current_position);
      #endif
    }
    destination[X_AXIS] = lx;
    destination[Y_AXIS] = ly;
    prepare_move_to_destination();         
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("xy move", current_position);
    #endif
    if (lz < current_position[Z_AXIS]) {    
      destination[Z_AXIS] = lz;
      prepare_uninterpolated_move_to_destination();   
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("z lower move", current_position);
      #endif
    }
  #elif IS_SCARA
    if (!position_is_reachable_xy(lx, ly)) return;
    set_destination_to_current();
    if (destination[Z_AXIS] < lz) {
      destination[Z_AXIS] = lz;
      prepare_uninterpolated_move_to_destination(fr_mm_s ? fr_mm_s : homing_feedrate(Z_AXIS));
    }
    destination[X_AXIS] = lx;
    destination[Y_AXIS] = ly;
    prepare_uninterpolated_move_to_destination(fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S);
    if (destination[Z_AXIS] > lz) {
      destination[Z_AXIS] = lz;
      prepare_uninterpolated_move_to_destination(fr_mm_s ? fr_mm_s : homing_feedrate(Z_AXIS));
    }
  #else
    if (current_position[Z_AXIS] < lz) {
      feedrate_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate(Z_AXIS);
      current_position[Z_AXIS] = lz;
      line_to_current_position();
    }
    feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;
    current_position[X_AXIS] = lx;
    current_position[Y_AXIS] = ly;
    line_to_current_position();
    if (current_position[Z_AXIS] > lz) {
      feedrate_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate(Z_AXIS);
      current_position[Z_AXIS] = lz;
      line_to_current_position();
    }
  #endif
  stepper.synchronize();
  feedrate_mm_s = old_feedrate_mm_s;
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< do_blocking_move_to");
  #endif
}
void do_blocking_move_to_x(const float &lx, const float &fr_mm_s) {
  do_blocking_move_to(lx, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
}
void do_blocking_move_to_z(const float &lz, const float &fr_mm_s) {
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], lz, fr_mm_s);
}
void do_blocking_move_to_xy(const float &lx, const float &ly, const float &fr_mm_s) {
  do_blocking_move_to(lx, ly, current_position[Z_AXIS], fr_mm_s);
}
static void setup_for_endstop_or_probe_move() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("setup_for_endstop_or_probe_move", current_position);
  #endif
  saved_feedrate_mm_s = feedrate_mm_s;
  saved_feedrate_percentage = feedrate_percentage;
  feedrate_percentage = 100;
  refresh_cmd_timeout();
}
static void clean_up_after_endstop_or_probe_move() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("clean_up_after_endstop_or_probe_move", current_position);
  #endif
  feedrate_mm_s = saved_feedrate_mm_s;
  feedrate_percentage = saved_feedrate_percentage;
  refresh_cmd_timeout();
}
#if HAS_PROBING_PROCEDURE || HOTENDS > 1 || ENABLED(Z_PROBE_ALLEN_KEY) || ENABLED(Z_PROBE_SLED) || ENABLED(NOZZLE_CLEAN_FEATURE) || ENABLED(NOZZLE_PARK_FEATURE) || ENABLED(DELTA_AUTO_CALIBRATION)
  bool axis_unhomed_error(const bool x, const bool y, const bool z) {
    #if ENABLED(HOME_AFTER_DEACTIVATE)
      const bool xx = x && !axis_known_position[X_AXIS],
                 yy = y && !axis_known_position[Y_AXIS],
                 zz = z && !axis_known_position[Z_AXIS];
    #else
      const bool xx = x && !axis_homed[X_AXIS],
                 yy = y && !axis_homed[Y_AXIS],
                 zz = z && !axis_homed[Z_AXIS];
    #endif
    if (xx || yy || zz) {
      SERIAL_ECHO_START();
      SERIAL_ECHOPGM(MSG_HOME " ");
      if (xx) SERIAL_ECHOPGM(MSG_X);
      if (yy) SERIAL_ECHOPGM(MSG_Y);
      if (zz) SERIAL_ECHOPGM(MSG_Z);
      SERIAL_ECHOLNPGM(" " MSG_FIRST);
      #if ENABLED(FYS_ULTRA_LCD_WANHAO_ONEPLUS)
      #elif ENABLED(ULTRA_LCD)
        lcd_status_printf_P(0, PSTR(MSG_HOME " %s%s%s " MSG_FIRST), xx ? MSG_X : "", yy ? MSG_Y : "", zz ? MSG_Z : "");
      #endif
      return true;
    }
    return false;
  }
#endif
#if ENABLED(Z_PROBE_SLED)
  #ifndef SLED_DOCKING_OFFSET
    #define SLED_DOCKING_OFFSET 0
  #endif
  static void dock_sled(bool stow) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPAIR("dock_sled(", stow);
        SERIAL_CHAR(')');
        SERIAL_EOL();
      }
    #endif
    do_blocking_move_to_x(X_MAX_POS + SLED_DOCKING_OFFSET - ((stow) ? 1 : 0));
    #if HAS_SOLENOID_1 && DISABLED(EXT_SOLENOID)
      WRITE(SOL1_PIN, !stow); 
    #endif
  }
#elif ENABLED(Z_PROBE_ALLEN_KEY)
  FORCE_INLINE void do_blocking_move_to(const float logical[XYZ], const float &fr_mm_s) {
    do_blocking_move_to(logical[X_AXIS], logical[Y_AXIS], logical[Z_AXIS], fr_mm_s);
  }
  void run_deploy_moves_script() {
    #if defined(Z_PROBE_ALLEN_KEY_DEPLOY_1_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_1_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_1_Z)
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_X
        #define Z_PROBE_ALLEN_KEY_DEPLOY_1_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_Y
        #define Z_PROBE_ALLEN_KEY_DEPLOY_1_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_Z
        #define Z_PROBE_ALLEN_KEY_DEPLOY_1_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE 0.0
      #endif
      const float deploy_1[] = { Z_PROBE_ALLEN_KEY_DEPLOY_1_X, Z_PROBE_ALLEN_KEY_DEPLOY_1_Y, Z_PROBE_ALLEN_KEY_DEPLOY_1_Z };
      do_blocking_move_to(deploy_1, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE));
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_DEPLOY_2_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_2_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_2_Z)
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_X
        #define Z_PROBE_ALLEN_KEY_DEPLOY_2_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_Y
        #define Z_PROBE_ALLEN_KEY_DEPLOY_2_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_Z
        #define Z_PROBE_ALLEN_KEY_DEPLOY_2_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE 0.0
      #endif
      const float deploy_2[] = { Z_PROBE_ALLEN_KEY_DEPLOY_2_X, Z_PROBE_ALLEN_KEY_DEPLOY_2_Y, Z_PROBE_ALLEN_KEY_DEPLOY_2_Z };
      do_blocking_move_to(deploy_2, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE));
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_DEPLOY_3_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_3_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_3_Z)
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_X
        #define Z_PROBE_ALLEN_KEY_DEPLOY_3_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_Y
        #define Z_PROBE_ALLEN_KEY_DEPLOY_3_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_Z
        #define Z_PROBE_ALLEN_KEY_DEPLOY_3_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE 0.0
      #endif
      const float deploy_3[] = { Z_PROBE_ALLEN_KEY_DEPLOY_3_X, Z_PROBE_ALLEN_KEY_DEPLOY_3_Y, Z_PROBE_ALLEN_KEY_DEPLOY_3_Z };
      do_blocking_move_to(deploy_3, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE));
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_DEPLOY_4_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_4_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_4_Z)
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_X
        #define Z_PROBE_ALLEN_KEY_DEPLOY_4_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_Y
        #define Z_PROBE_ALLEN_KEY_DEPLOY_4_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_Z
        #define Z_PROBE_ALLEN_KEY_DEPLOY_4_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE 0.0
      #endif
      const float deploy_4[] = { Z_PROBE_ALLEN_KEY_DEPLOY_4_X, Z_PROBE_ALLEN_KEY_DEPLOY_4_Y, Z_PROBE_ALLEN_KEY_DEPLOY_4_Z };
      do_blocking_move_to(deploy_4, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE));
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_DEPLOY_5_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_5_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_5_Z)
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_X
        #define Z_PROBE_ALLEN_KEY_DEPLOY_5_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_Y
        #define Z_PROBE_ALLEN_KEY_DEPLOY_5_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_Z
        #define Z_PROBE_ALLEN_KEY_DEPLOY_5_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE 0.0
      #endif
      const float deploy_5[] = { Z_PROBE_ALLEN_KEY_DEPLOY_5_X, Z_PROBE_ALLEN_KEY_DEPLOY_5_Y, Z_PROBE_ALLEN_KEY_DEPLOY_5_Z };
      do_blocking_move_to(deploy_5, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE));
    #endif
  }
  void run_stow_moves_script() {
    #if defined(Z_PROBE_ALLEN_KEY_STOW_1_X) || defined(Z_PROBE_ALLEN_KEY_STOW_1_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_1_Z)
      #ifndef Z_PROBE_ALLEN_KEY_STOW_1_X
        #define Z_PROBE_ALLEN_KEY_STOW_1_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_1_Y
        #define Z_PROBE_ALLEN_KEY_STOW_1_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_1_Z
        #define Z_PROBE_ALLEN_KEY_STOW_1_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE 0.0
      #endif
      const float stow_1[] = { Z_PROBE_ALLEN_KEY_STOW_1_X, Z_PROBE_ALLEN_KEY_STOW_1_Y, Z_PROBE_ALLEN_KEY_STOW_1_Z };
      do_blocking_move_to(stow_1, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE));
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_STOW_2_X) || defined(Z_PROBE_ALLEN_KEY_STOW_2_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_2_Z)
      #ifndef Z_PROBE_ALLEN_KEY_STOW_2_X
        #define Z_PROBE_ALLEN_KEY_STOW_2_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_2_Y
        #define Z_PROBE_ALLEN_KEY_STOW_2_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_2_Z
        #define Z_PROBE_ALLEN_KEY_STOW_2_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE 0.0
      #endif
      const float stow_2[] = { Z_PROBE_ALLEN_KEY_STOW_2_X, Z_PROBE_ALLEN_KEY_STOW_2_Y, Z_PROBE_ALLEN_KEY_STOW_2_Z };
      do_blocking_move_to(stow_2, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE));
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_STOW_3_X) || defined(Z_PROBE_ALLEN_KEY_STOW_3_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_3_Z)
      #ifndef Z_PROBE_ALLEN_KEY_STOW_3_X
        #define Z_PROBE_ALLEN_KEY_STOW_3_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_3_Y
        #define Z_PROBE_ALLEN_KEY_STOW_3_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_3_Z
        #define Z_PROBE_ALLEN_KEY_STOW_3_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE 0.0
      #endif
      const float stow_3[] = { Z_PROBE_ALLEN_KEY_STOW_3_X, Z_PROBE_ALLEN_KEY_STOW_3_Y, Z_PROBE_ALLEN_KEY_STOW_3_Z };
      do_blocking_move_to(stow_3, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE));
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_STOW_4_X) || defined(Z_PROBE_ALLEN_KEY_STOW_4_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_4_Z)
      #ifndef Z_PROBE_ALLEN_KEY_STOW_4_X
        #define Z_PROBE_ALLEN_KEY_STOW_4_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_4_Y
        #define Z_PROBE_ALLEN_KEY_STOW_4_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_4_Z
        #define Z_PROBE_ALLEN_KEY_STOW_4_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE 0.0
      #endif
      const float stow_4[] = { Z_PROBE_ALLEN_KEY_STOW_4_X, Z_PROBE_ALLEN_KEY_STOW_4_Y, Z_PROBE_ALLEN_KEY_STOW_4_Z };
      do_blocking_move_to(stow_4, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE));
    #endif
    #if defined(Z_PROBE_ALLEN_KEY_STOW_5_X) || defined(Z_PROBE_ALLEN_KEY_STOW_5_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_5_Z)
      #ifndef Z_PROBE_ALLEN_KEY_STOW_5_X
        #define Z_PROBE_ALLEN_KEY_STOW_5_X current_position[X_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_5_Y
        #define Z_PROBE_ALLEN_KEY_STOW_5_Y current_position[Y_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_5_Z
        #define Z_PROBE_ALLEN_KEY_STOW_5_Z current_position[Z_AXIS]
      #endif
      #ifndef Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE 0.0
      #endif
      const float stow_5[] = { Z_PROBE_ALLEN_KEY_STOW_5_X, Z_PROBE_ALLEN_KEY_STOW_5_Y, Z_PROBE_ALLEN_KEY_STOW_5_Z };
      do_blocking_move_to(stow_5, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE));
    #endif
  }
#endif
#if ENABLED(PROBING_FANS_OFF)
  void fans_pause(const bool p) {
    if (p != fans_paused) {
      fans_paused = p;
      if (p)
        for (uint8_t x = 0; x < FAN_COUNT; x++) {
          paused_fanSpeeds[x] = fanSpeeds[x];
          fanSpeeds[x] = 0;
        }
      else
        for (uint8_t x = 0; x < FAN_COUNT; x++)
          fanSpeeds[x] = paused_fanSpeeds[x];
    }
  }
#endif 
#if HAS_BED_PROBE
  #if ENABLED(PROBE_IS_TRIGGERED_WHEN_STOWED_TEST)
    #if ENABLED(Z_MIN_PROBE_ENDSTOP)
      #define _TRIGGERED_WHEN_STOWED_TEST (READ(Z_MIN_PROBE_PIN) != Z_MIN_PROBE_ENDSTOP_INVERTING)
    #else
      #define _TRIGGERED_WHEN_STOWED_TEST (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING)
    #endif
  #endif
  #if QUIET_PROBING
    void probing_pause(const bool p) {
      #if ENABLED(PROBING_HEATERS_OFF)
        thermalManager.pause(p);
      #endif
      #if ENABLED(PROBING_FANS_OFF)
        fans_pause(p);
      #endif
      if (p) safe_delay(25);
    }
  #endif 
  #if ENABLED(BLTOUCH)
    void bltouch_command(int angle) {
      servo[Z_ENDSTOP_SERVO_NR].move(angle);  
      safe_delay(BLTOUCH_DELAY);
    }
    void set_bltouch_deployed(const bool deploy) {
      if (deploy && TEST_BLTOUCH()) {      
        bltouch_command(BLTOUCH_RESET);    
        bltouch_command(BLTOUCH_DEPLOY);   
        bltouch_command(BLTOUCH_STOW);     
        safe_delay(1500);                  
        if (TEST_BLTOUCH()) {              
          SERIAL_ERROR_START();
          SERIAL_ERRORLNPGM(MSG_STOP_BLTOUCH);
          #if ENABLED(FYS_LCD_EXTRA_INFO)
          FunV006("Bltouch error.");
          #endif
          stop();                          
        }
      }
      bltouch_command(deploy ? BLTOUCH_DEPLOY : BLTOUCH_STOW);
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
          SERIAL_ECHOPAIR("set_bltouch_deployed(", deploy);
          SERIAL_CHAR(')');
          SERIAL_EOL();
        }
      #endif
    }
  #endif 
#if HAS_BED_PROBE
  void do_probe_raise(const float z_raise) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPAIR("do_probe_raise(", z_raise);
        SERIAL_CHAR(')');
        SERIAL_EOL();
      }
    #endif
    float z_dest = LOGICAL_Z_POSITION(z_raise);
    if (zprobe_zoffset < 0) z_dest -= zprobe_zoffset;  
    #if ENABLED(DELTA)
      z_dest -= home_offset[Z_AXIS]; 
    #endif
    if (z_dest > current_position[Z_AXIS])
      do_blocking_move_to_z(z_dest);
  }
#endif 
  bool set_probe_deployed(bool deploy) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        DEBUG_POS("set_probe_deployed", current_position);
        SERIAL_ECHOLNPAIR("deploy: ", deploy);
      }
    #endif
    if (endstops.z_probe_enabled == deploy) return false;
    do_probe_raise(_Z_CLEARANCE_DEPLOY_PROBE);
    #if ENABLED(BLTOUCH)
      if (deploy && TEST_BLTOUCH()) {      
        bltouch_command(BLTOUCH_RESET);    
        bltouch_command(BLTOUCH_DEPLOY);   
        bltouch_command(BLTOUCH_STOW);     
        safe_delay(1500);                  
        if (TEST_BLTOUCH()) {              
          SERIAL_ERROR_START();
          SERIAL_ERRORLNPGM(MSG_STOP_BLTOUCH);
          FunV006("Bltouch error.");
          stop();                          
          return true;
        }
      }
    #elif ENABLED(Z_PROBE_SLED) || ENABLED(Z_PROBE_ALLEN_KEY)
      #if ENABLED(Z_PROBE_SLED)
        #define _AUE_ARGS true, false, false
      #else
        #define _AUE_ARGS
      #endif
      if (axis_unhomed_error(_AUE_ARGS)) {
        SERIAL_ERROR_START();
        SERIAL_ERRORLNPGM(MSG_STOP_UNHOMED);
        stop();
        return true;
      }
    #endif
    const float oldXpos = current_position[X_AXIS],
                oldYpos = current_position[Y_AXIS];
    #ifdef _TRIGGERED_WHEN_STOWED_TEST
      if (_TRIGGERED_WHEN_STOWED_TEST == deploy) {     
        if (!deploy) endstops.enable_z_probe(false); 
    #endif
        #if ENABLED(SOLENOID_PROBE)
          #if HAS_SOLENOID_1
            WRITE(SOL1_PIN, deploy);
          #endif
        #elif ENABLED(Z_PROBE_SLED)
          dock_sled(!deploy);
        #elif HAS_Z_SERVO_ENDSTOP && DISABLED(BLTOUCH)
          servo[Z_ENDSTOP_SERVO_NR].move(z_servo_angle[deploy ? 0 : 1]);
        #elif ENABLED(Z_PROBE_ALLEN_KEY)
          deploy ? run_deploy_moves_script() : run_stow_moves_script();
        #endif
    #ifdef _TRIGGERED_WHEN_STOWED_TEST
      } 
      if (_TRIGGERED_WHEN_STOWED_TEST == deploy) { 
        if (IsRunning()) {
          SERIAL_ERROR_START();
          SERIAL_ERRORLNPGM("Z-Probe failed");
          LCD_ALERTMESSAGEPGM("Err: ZPROBE");
        }
        stop();
        return true;
      } 
    #endif
    do_blocking_move_to(oldXpos, oldYpos, current_position[Z_AXIS]); 
    endstops.enable_z_probe(deploy);
    return false;
  }
  static void do_probe_move(float z, float fr_mm_m) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS(">>> do_probe_move", current_position);
    #endif
    #if ENABLED(BLTOUCH)
      set_bltouch_deployed(true);
    #endif
    #if QUIET_PROBING
      probing_pause(true);
    #endif
    do_blocking_move_to_z(LOGICAL_Z_POSITION(z), MMM_TO_MMS(fr_mm_m));
    #if QUIET_PROBING
      probing_pause(false);
    #endif
    #if ENABLED(BLTOUCH)
      set_bltouch_deployed(false);
    #endif
    endstops.hit_on_purpose();
    set_current_from_steppers_for_axis(Z_AXIS);
    SYNC_PLAN_POSITION_KINEMATIC();
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("<<< do_probe_move", current_position);
    #endif
  }
  static float run_z_probe() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS(">>> run_z_probe", current_position);
    #endif
    refresh_cmd_timeout();
    #if ENABLED(PROBE_DOUBLE_TOUCH)
      do_probe_move(-(Z_MAX_LENGTH) - 10, Z_PROBE_SPEED_FAST);
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        float first_probe_z = current_position[Z_AXIS];
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR("1st Probe Z:", first_probe_z);
      #endif
      do_blocking_move_to_z(current_position[Z_AXIS] + home_bump_mm(Z_AXIS), MMM_TO_MMS(Z_PROBE_SPEED_FAST));
    #else
      float z = LOGICAL_Z_POSITION(Z_CLEARANCE_BETWEEN_PROBES);
      if (zprobe_zoffset < 0) z -= zprobe_zoffset;
      #if ENABLED(DELTA)
        z -= home_offset[Z_AXIS]; 
      #endif
      if (z < current_position[Z_AXIS])
        do_blocking_move_to_z(z, MMM_TO_MMS(Z_PROBE_SPEED_FAST));
    #endif
    do_probe_move(-(Z_MAX_LENGTH) - 10, Z_PROBE_SPEED_SLOW);
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("<<< run_z_probe", current_position);
    #endif
    #if ENABLED(PROBE_DOUBLE_TOUCH) && ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPAIR("2nd Probe Z:", current_position[Z_AXIS]);
        SERIAL_ECHOLNPAIR(" Discrepancy:", first_probe_z - current_position[Z_AXIS]);
      }
    #endif
     return RAW_CURRENT_POSITION(Z) + zprobe_zoffset
      #if ENABLED(DELTA)
        + home_offset[Z_AXIS] 
      #endif
    ;
  }
  float probe_pt(const float &lx, const float &ly, const bool stow, const uint8_t verbose_level, const bool printable=true) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPAIR(">>> probe_pt(", lx);
        SERIAL_ECHOPAIR(", ", ly);
        SERIAL_ECHOPAIR(", ", stow ? "" : "no ");
        SERIAL_ECHOLNPGM("stow)");
        DEBUG_POS("", current_position);
      }
    #endif
    const float nx = lx - (X_PROBE_OFFSET_FROM_EXTRUDER), ny = ly - (Y_PROBE_OFFSET_FROM_EXTRUDER);
    if (printable)
      if (!position_is_reachable_by_probe_xy(lx, ly)) return NAN;
    else
      if (!position_is_reachable_xy(nx, ny)) return NAN;
    const float old_feedrate_mm_s = feedrate_mm_s;
    #if ENABLED(DELTA)
      if (current_position[Z_AXIS] > delta_clip_start_height)
        do_blocking_move_to_z(delta_clip_start_height);
    #endif
    do_probe_raise(Z_CLEARANCE_BETWEEN_PROBES);
    feedrate_mm_s = XY_PROBE_FEEDRATE_MM_S;
    do_blocking_move_to_xy(nx, ny);
    if (DEPLOY_PROBE()) return NAN;
    const float measured_z = run_z_probe();
    if (!stow)
      do_probe_raise(Z_CLEARANCE_BETWEEN_PROBES);
    else
      if (STOW_PROBE()) return NAN;
    if (verbose_level > 2) {
      SERIAL_PROTOCOLPGM("Bed X: ");
      SERIAL_PROTOCOL_F(lx, 3);
      SERIAL_PROTOCOLPGM(" Y: ");
      SERIAL_PROTOCOL_F(ly, 3);
      SERIAL_PROTOCOLPGM(" Z: ");
      SERIAL_PROTOCOL_F(measured_z, 3);
      SERIAL_EOL();
    }
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< probe_pt");
    #endif
    feedrate_mm_s = old_feedrate_mm_s;
    return measured_z;
  }
#endif 
#if HAS_LEVELING
  bool leveling_is_valid() {
    return
      #if ENABLED(MESH_BED_LEVELING)
        mbl.has_mesh()
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        !!bilinear_grid_spacing[X_AXIS]
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        true
      #else 
        true
      #endif
    ;
  }
  bool leveling_is_active() {
    return
      #if ENABLED(MESH_BED_LEVELING)
        mbl.active()
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        ubl.state.active
      #else
        planner.abl_enabled
      #endif
    ;
  }
  void set_bed_leveling_enabled(const bool enable) {
    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      const bool can_change = (!enable || leveling_is_valid());
    #else
      constexpr bool can_change = true;
    #endif
    if (can_change && enable != leveling_is_active()) {
      #if ENABLED(MESH_BED_LEVELING)
        if (!enable)
          planner.apply_leveling(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
        const bool enabling = enable && leveling_is_valid();
        mbl.set_active(enabling);
        if (enabling) planner.unapply_leveling(current_position);
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        #if PLANNER_LEVELING
          if (ubl.state.active) {                       
            planner.apply_leveling(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
            ubl.state.active = false;                   
          }
          else {                                        
            ubl.state.active = true;                    
            planner.unapply_leveling(current_position);
          }
        #else
          ubl.state.active = enable;                    
        #endif
      #else 
        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          const float reset[XYZ] = { -9999.999, -9999.999, 0 };
          (void)bilinear_z_offset(reset);
        #endif
        planner.abl_enabled = enable;
        if (!enable)
          set_current_from_steppers_for_axis(
            #if ABL_PLANAR
              ALL_AXES
            #else
              Z_AXIS
            #endif
          );
        else
          planner.unapply_leveling(current_position);
      #endif 
    }
  }
  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    void set_z_fade_height(const float zfh) {
      const bool level_active = leveling_is_active();
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        if (level_active)
          set_bed_leveling_enabled(false);  
        planner.z_fade_height = zfh;
        planner.inverse_z_fade_height = RECIPROCAL(zfh);
        if (level_active)
          set_bed_leveling_enabled(true);  
      #else
        planner.z_fade_height = zfh;
        planner.inverse_z_fade_height = RECIPROCAL(zfh);
        if (level_active) {
          set_current_from_steppers_for_axis(
            #if ABL_PLANAR
              ALL_AXES
            #else
              Z_AXIS
            #endif
          );
        }
      #endif
    }
  #endif 
  void reset_bed_level() {
    set_bed_leveling_enabled(false);
    #if ENABLED(MESH_BED_LEVELING)
      if (leveling_is_valid()) {
        mbl.reset();
        mbl.set_has_mesh(false);
      }
    #else
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("reset_bed_level");
      #endif
      #if ABL_PLANAR
        planner.bed_level_matrix.set_to_identity();
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        bilinear_start[X_AXIS] = bilinear_start[Y_AXIS] =
        bilinear_grid_spacing[X_AXIS] = bilinear_grid_spacing[Y_AXIS] = 0;
        for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
          for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
            z_values[x][y] = NAN;
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        ubl.reset();
      #endif
    #endif
  }
#endif 
#if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)
  static void print_2d_array(const uint8_t sx, const uint8_t sy, const uint8_t precision, float (*fn)(const uint8_t, const uint8_t)) {
    #ifndef SCAD_MESH_OUTPUT
      for (uint8_t x = 0; x < sx; x++) {
        for (uint8_t i = 0; i < precision + 2 + (x < 10 ? 1 : 0); i++)
          SERIAL_PROTOCOLCHAR(' ');
        SERIAL_PROTOCOL((int)x);
      }
      SERIAL_EOL();
    #endif
    #ifdef SCAD_MESH_OUTPUT
      SERIAL_PROTOCOLLNPGM("measured_z = ["); 
    #endif
    for (uint8_t y = 0; y < sy; y++) {
      #ifdef SCAD_MESH_OUTPUT
        SERIAL_PROTOCOLPGM(" [");           
      #else
        if (y < 10) SERIAL_PROTOCOLCHAR(' ');
        SERIAL_PROTOCOL((int)y);
      #endif
      for (uint8_t x = 0; x < sx; x++) {
        SERIAL_PROTOCOLCHAR(' ');
        const float offset = fn(x, y);
        if (!isnan(offset)) {
          if (offset >= 0) SERIAL_PROTOCOLCHAR('+');
          SERIAL_PROTOCOL_F(offset, precision);
        }
        else {
          #ifdef SCAD_MESH_OUTPUT
            for (uint8_t i = 3; i < precision + 3; i++)
              SERIAL_PROTOCOLCHAR(' ');
            SERIAL_PROTOCOLPGM("NAN");
          #else
            for (uint8_t i = 0; i < precision + 3; i++)
              SERIAL_PROTOCOLCHAR(i ? '=' : ' ');
          #endif
        }
        #ifdef SCAD_MESH_OUTPUT
          if (x < sx - 1) SERIAL_PROTOCOLCHAR(',');
        #endif
      }
      #ifdef SCAD_MESH_OUTPUT
        SERIAL_PROTOCOLCHAR(' ');
        SERIAL_PROTOCOLCHAR(']');                     
        if (y < sy - 1) SERIAL_PROTOCOLCHAR(',');
      #endif
      SERIAL_EOL();
    }
    #ifdef SCAD_MESH_OUTPUT
      SERIAL_PROTOCOLPGM("];");                       
    #endif
    SERIAL_EOL();
  }
#endif
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  static void extrapolate_one_point(const uint8_t x, const uint8_t y, const int8_t xdir, const int8_t ydir) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPGM("Extrapolate [");
        if (x < 10) SERIAL_CHAR(' ');
        SERIAL_ECHO((int)x);
        SERIAL_CHAR(xdir ? (xdir > 0 ? '+' : '-') : ' ');
        SERIAL_CHAR(' ');
        if (y < 10) SERIAL_CHAR(' ');
        SERIAL_ECHO((int)y);
        SERIAL_CHAR(ydir ? (ydir > 0 ? '+' : '-') : ' ');
        SERIAL_CHAR(']');
      }
    #endif
    if (!isnan(z_values[x][y])) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM(" (done)");
      #endif
      return;  
    }
    SERIAL_EOL();
    const uint8_t x1 = x + xdir, y1 = y + ydir, x2 = x1 + xdir, y2 = y1 + ydir;
    float a1 = z_values[x1][y ], a2 = z_values[x2][y ],
          b1 = z_values[x ][y1], b2 = z_values[x ][y2],
          c1 = z_values[x1][y1], c2 = z_values[x2][y2];
    if (isnan(a2)) a2 = 0.0; if (isnan(a1)) a1 = a2;
    if (isnan(b2)) b2 = 0.0; if (isnan(b1)) b1 = b2;
    if (isnan(c2)) c2 = 0.0; if (isnan(c1)) c1 = c2;
    const float a = 2 * a1 - a2, b = 2 * b1 - b2, c = 2 * c1 - c2;
    z_values[x][y] = (a + b + c) / 3.0;
  }
  #if ENABLED(EXTRAPOLATE_FROM_EDGE)
    #if GRID_MAX_POINTS_X < GRID_MAX_POINTS_Y
      #define HALF_IN_X
    #elif GRID_MAX_POINTS_Y < GRID_MAX_POINTS_X
      #define HALF_IN_Y
    #endif
  #endif
  static void extrapolate_unprobed_bed_level() {
    #ifdef HALF_IN_X
      constexpr uint8_t ctrx2 = 0, xlen = GRID_MAX_POINTS_X - 1;
    #else
      constexpr uint8_t ctrx1 = (GRID_MAX_POINTS_X - 1) / 2, 
                        ctrx2 = (GRID_MAX_POINTS_X) / 2,     
                        xlen = ctrx1;
    #endif
    #ifdef HALF_IN_Y
      constexpr uint8_t ctry2 = 0, ylen = GRID_MAX_POINTS_Y - 1;
    #else
      constexpr uint8_t ctry1 = (GRID_MAX_POINTS_Y - 1) / 2, 
                        ctry2 = (GRID_MAX_POINTS_Y) / 2,     
                        ylen = ctry1;
    #endif
    for (uint8_t xo = 0; xo <= xlen; xo++)
      for (uint8_t yo = 0; yo <= ylen; yo++) {
        uint8_t x2 = ctrx2 + xo, y2 = ctry2 + yo;
        #ifndef HALF_IN_X
          const uint8_t x1 = ctrx1 - xo;
        #endif
        #ifndef HALF_IN_Y
          const uint8_t y1 = ctry1 - yo;
          #ifndef HALF_IN_X
            extrapolate_one_point(x1, y1, +1, +1);   
          #endif
          extrapolate_one_point(x2, y1, -1, +1);     
        #endif
        #ifndef HALF_IN_X
          extrapolate_one_point(x1, y2, +1, -1);     
        #endif
        extrapolate_one_point(x2, y2, -1, -1);       
      }
  }
  static void print_bilinear_leveling_grid() {
    SERIAL_ECHOLNPGM("Bilinear Leveling Grid:");
    print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 3,
      [](const uint8_t ix, const uint8_t iy) { return z_values[ix][iy]; }
    );
  }
  #if ENABLED(ABL_BILINEAR_SUBDIVISION)
    #define ABL_GRID_POINTS_VIRT_X (GRID_MAX_POINTS_X - 1) * (BILINEAR_SUBDIVISIONS) + 1
    #define ABL_GRID_POINTS_VIRT_Y (GRID_MAX_POINTS_Y - 1) * (BILINEAR_SUBDIVISIONS) + 1
    #define ABL_TEMP_POINTS_X (GRID_MAX_POINTS_X + 2)
    #define ABL_TEMP_POINTS_Y (GRID_MAX_POINTS_Y + 2)
    float z_values_virt[ABL_GRID_POINTS_VIRT_X][ABL_GRID_POINTS_VIRT_Y];
    int bilinear_grid_spacing_virt[2] = { 0 };
    float bilinear_grid_factor_virt[2] = { 0 };
    static void bed_level_virt_print() {
      SERIAL_ECHOLNPGM("Subdivided with CATMULL ROM Leveling Grid:");
      print_2d_array(ABL_GRID_POINTS_VIRT_X, ABL_GRID_POINTS_VIRT_Y, 5,
        [](const uint8_t ix, const uint8_t iy) { return z_values_virt[ix][iy]; }
      );
    }
    #define LINEAR_EXTRAPOLATION(E, I) ((E) * 2 - (I))
    float bed_level_virt_coord(const uint8_t x, const uint8_t y) {
      uint8_t ep = 0, ip = 1;
      if (!x || x == ABL_TEMP_POINTS_X - 1) {
        if (x) {
          ep = GRID_MAX_POINTS_X - 1;
          ip = GRID_MAX_POINTS_X - 2;
        }
        if (WITHIN(y, 1, ABL_TEMP_POINTS_Y - 2))
          return LINEAR_EXTRAPOLATION(
            z_values[ep][y - 1],
            z_values[ip][y - 1]
          );
        else
          return LINEAR_EXTRAPOLATION(
            bed_level_virt_coord(ep + 1, y),
            bed_level_virt_coord(ip + 1, y)
          );
      }
      if (!y || y == ABL_TEMP_POINTS_Y - 1) {
        if (y) {
          ep = GRID_MAX_POINTS_Y - 1;
          ip = GRID_MAX_POINTS_Y - 2;
        }
        if (WITHIN(x, 1, ABL_TEMP_POINTS_X - 2))
          return LINEAR_EXTRAPOLATION(
            z_values[x - 1][ep],
            z_values[x - 1][ip]
          );
        else
          return LINEAR_EXTRAPOLATION(
            bed_level_virt_coord(x, ep + 1),
            bed_level_virt_coord(x, ip + 1)
          );
      }
      return z_values[x - 1][y - 1];
    }
    static float bed_level_virt_cmr(const float p[4], const uint8_t i, const float t) {
      return (
          p[i-1] * -t * sq(1 - t)
        + p[i]   * (2 - 5 * sq(t) + 3 * t * sq(t))
        + p[i+1] * t * (1 + 4 * t - 3 * sq(t))
        - p[i+2] * sq(t) * (1 - t)
      ) * 0.5;
    }
    static float bed_level_virt_2cmr(const uint8_t x, const uint8_t y, const float &tx, const float &ty) {
      float row[4], column[4];
      for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4; j++) {
          column[j] = bed_level_virt_coord(i + x - 1, j + y - 1);
        }
        row[i] = bed_level_virt_cmr(column, 1, ty);
      }
      return bed_level_virt_cmr(row, 1, tx);
    }
    void bed_level_virt_interpolate() {
      bilinear_grid_spacing_virt[X_AXIS] = bilinear_grid_spacing[X_AXIS] / (BILINEAR_SUBDIVISIONS);
      bilinear_grid_spacing_virt[Y_AXIS] = bilinear_grid_spacing[Y_AXIS] / (BILINEAR_SUBDIVISIONS);
      bilinear_grid_factor_virt[X_AXIS] = RECIPROCAL(bilinear_grid_spacing_virt[X_AXIS]);
      bilinear_grid_factor_virt[Y_AXIS] = RECIPROCAL(bilinear_grid_spacing_virt[Y_AXIS]);
      for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
        for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
          for (uint8_t ty = 0; ty < BILINEAR_SUBDIVISIONS; ty++)
            for (uint8_t tx = 0; tx < BILINEAR_SUBDIVISIONS; tx++) {
              if ((ty && y == GRID_MAX_POINTS_Y - 1) || (tx && x == GRID_MAX_POINTS_X - 1))
                continue;
              z_values_virt[x * (BILINEAR_SUBDIVISIONS) + tx][y * (BILINEAR_SUBDIVISIONS) + ty] =
                bed_level_virt_2cmr(
                  x + 1,
                  y + 1,
                  (float)tx / (BILINEAR_SUBDIVISIONS),
                  (float)ty / (BILINEAR_SUBDIVISIONS)
                );
            }
    }
  #endif 
  void refresh_bed_level() {
    bilinear_grid_factor[X_AXIS] = RECIPROCAL(bilinear_grid_spacing[X_AXIS]);
    bilinear_grid_factor[Y_AXIS] = RECIPROCAL(bilinear_grid_spacing[Y_AXIS]);
    #if ENABLED(ABL_BILINEAR_SUBDIVISION)
      bed_level_virt_interpolate();
    #endif
  }
#endif 
static void do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s=0.0) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR(">>> do_homing_move(", axis_codes[axis]);
      SERIAL_ECHOPAIR(", ", distance);
      SERIAL_ECHOPAIR(", ", fr_mm_s);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif
  #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
    const bool deploy_bltouch = (axis == Z_AXIS && distance < 0);
    if (deploy_bltouch) set_bltouch_deployed(true);
  #endif
  #if QUIET_PROBING
    if (axis == Z_AXIS) probing_pause(true);
  #endif
  current_position[axis] = 0;
  #if IS_SCARA
    SYNC_PLAN_POSITION_KINEMATIC();
    current_position[axis] = distance;
    inverse_kinematics(current_position);
    planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate(axis), active_extruder);
  #else
    sync_plan_position();
    current_position[axis] = distance;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate(axis), active_extruder);
  #endif
  stepper.synchronize();
  #if QUIET_PROBING
    if (axis == Z_AXIS) probing_pause(false);
  #endif
  #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
    if (deploy_bltouch) set_bltouch_deployed(false);
  #endif
  endstops.hit_on_purpose();
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR("<<< do_homing_move(", axis_codes[axis]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif
}
#if ENABLED(SENSORLESS_HOMING)
  void tmc2130_sensorless_homing(TMC2130Stepper &st, bool enable=true) {
    #if ENABLED(STEALTHCHOP)
      if (enable) {
        st.coolstep_min_speed(1024UL * 1024UL - 1UL);
        st.stealthChop(0);
      }
      else {
        st.coolstep_min_speed(0);
        st.stealthChop(1);
      }
    #endif
    st.diag1_stall(enable ? 1 : 0);
  }
#endif
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)
static void homeaxis(const AxisEnum axis) {
  #if IS_SCARA
    if (axis != Z_AXIS) { BUZZ(100, 880); return; }
  #else
    #define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;
  #endif
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR(">>> homeaxis(", axis_codes[axis]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif
  const int axis_home_dir =
    #if ENABLED(DUAL_X_CARRIAGE)
      (axis == X_AXIS) ? x_home_dir(active_extruder) :
    #endif
    home_dir(axis);
  #if HOMING_Z_WITH_PROBE
  if (axis == Z_AXIS && DEPLOY_PROBE()) return;
  #endif
  #if ENABLED(Z_DUAL_ENDSTOPS)
    if (axis == Z_AXIS) stepper.set_homing_flag(true);
  #endif
  #if ENABLED(SENSORLESS_HOMING)
    #if ENABLED(X_IS_TMC2130)
      if (axis == X_AXIS) tmc2130_sensorless_homing(stepperX);
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (axis == Y_AXIS) tmc2130_sensorless_homing(stepperY);
    #endif
  #endif
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Home 1 Fast:");
  #endif
    do_homing_move(axis, 600 * axis_home_dir);
#if !ENABLED(FYS_G28_NO_Z_BUMP)
  float bump = axis_home_dir * (
    #if HOMING_Z_WITH_PROBE
      (axis == Z_AXIS) ? max(Z_CLEARANCE_BETWEEN_PROBES, home_bump_mm(Z_AXIS)) :
    #endif
    home_bump_mm(axis)
  );
  if (bump) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Move Away:");
    #endif
    do_homing_move(axis, -bump);
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Home 2 Slow:");
    #endif
    do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
  }
#endif
  #if ENABLED(Z_DUAL_ENDSTOPS)
    if (axis == Z_AXIS) {
      float adj = FABS(z_endstop_adj);
      bool lockZ1;
      if (axis_home_dir > 0) {
        adj = -adj;
        lockZ1 = (z_endstop_adj > 0);
      }
      else
        lockZ1 = (z_endstop_adj < 0);
      if (lockZ1) stepper.set_z_lock(true); else stepper.set_z2_lock(true);
      do_homing_move(axis, adj);
      if (lockZ1) stepper.set_z_lock(false); else stepper.set_z2_lock(false);
      stepper.set_homing_flag(false);
    } 
  #endif
  #if IS_SCARA
    set_axis_is_at_home(axis);
    SYNC_PLAN_POSITION_KINEMATIC();
  #elif ENABLED(DELTA)
    if (endstop_adj[axis] * Z_HOME_DIR <= 0) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("endstop_adj:");
      #endif
      do_homing_move(axis, endstop_adj[axis] - 0.1);
    }
  #else
    set_axis_is_at_home(axis);
    sync_plan_position();
    destination[axis] = current_position[axis];
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("> AFTER set_axis_is_at_home", current_position);
    #endif
  #endif
  #if ENABLED(SENSORLESS_HOMING)
    #if ENABLED(X_IS_TMC2130)
      if (axis == X_AXIS) tmc2130_sensorless_homing(stepperX, false);
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (axis == Y_AXIS) tmc2130_sensorless_homing(stepperY, false);
    #endif
  #endif
  #if HOMING_Z_WITH_PROBE
    if (axis == Z_AXIS && STOW_PROBE()) return;
  #endif
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR("<<< homeaxis(", axis_codes[axis]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif
} 
#if ENABLED(FWRETRACT)
  void retract(const bool retracting, const bool swapping = false) {
    static float hop_height;
    if (retracting == retracted[active_extruder]) return;
    const float old_feedrate_mm_s = feedrate_mm_s;
    set_destination_to_current();
    if (retracting) {
      feedrate_mm_s = retract_feedrate_mm_s;
      current_position[E_AXIS] += (swapping ? retract_length_swap : retract_length) / volumetric_multiplier[active_extruder];
      sync_plan_position_e();
      prepare_move_to_destination();
      if (retract_zlift > 0.01) {
        hop_height = current_position[Z_AXIS];
        current_position[Z_AXIS] -= retract_zlift;
        SYNC_PLAN_POSITION_KINEMATIC();
        prepare_move_to_destination();
      }
    }
    else {
      if (retract_zlift > 0.01 && hop_height <= current_position[Z_AXIS]) {
        current_position[Z_AXIS] += retract_zlift;
        SYNC_PLAN_POSITION_KINEMATIC();
        prepare_move_to_destination();
      }
      feedrate_mm_s = retract_recover_feedrate_mm_s;
      const float move_e = swapping ? retract_length_swap + retract_recover_length_swap : retract_length + retract_recover_length;
      current_position[E_AXIS] -= move_e / volumetric_multiplier[active_extruder];
      sync_plan_position_e();
      prepare_move_to_destination();
    }
    feedrate_mm_s = old_feedrate_mm_s;
    retracted[active_extruder] = retracting;
  } 
#endif 
#if ENABLED(MIXING_EXTRUDER)
  void normalize_mix() {
    float mix_total = 0.0;
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++) mix_total += RECIPROCAL(mixing_factor[i]);
    if (!NEAR(mix_total, 1.0)) {
      SERIAL_PROTOCOLLNPGM("Warning: Mix factors must add up to 1.0. Scaling.");
      for (uint8_t i = 0; i < MIXING_STEPPERS; i++) mixing_factor[i] *= mix_total;
    }
  }
  #if ENABLED(DIRECT_MIXING_IN_G1)
    void gcode_get_mix() {
      const char* mixing_codes = "ABCDHI";
      byte mix_bits = 0;
      for (uint8_t i = 0; i < MIXING_STEPPERS; i++) {
        if (parser.seenval(mixing_codes[i])) {
          SBI(mix_bits, i);
          float v = parser.value_float();
          NOLESS(v, 0.0);
          mixing_factor[i] = RECIPROCAL(v);
        }
      }
      if (mix_bits) {
        for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
          if (!TEST(mix_bits, i)) mixing_factor[i] = 0.0;
        normalize_mix();
      }
    }
  #endif
#endif
void gcode_get_destination() {
  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i]))
      destination[i] = parser.value_axis_units((AxisEnum)i) + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
    else
      destination[i] = current_position[i];
  }
  if (parser.linearval('F') > 0.0)
    feedrate_mm_s = MMM_TO_MMS(parser.value_feedrate());
  #if ENABLED(PRINTCOUNTER)
    if (!DEBUGGING(DRYRUN))
      print_job_timer.incFilamentUsed(destination[E_AXIS] - current_position[E_AXIS]);
  #endif
  #if ENABLED(MIXING_EXTRUDER) && ENABLED(DIRECT_MIXING_IN_G1)
    gcode_get_mix();
  #endif
}
#if ENABLED(HOST_KEEPALIVE_FEATURE)
  void host_keepalive() {
    const millis_t ms = millis();
    if (host_keepalive_interval && busy_state != NOT_BUSY) {
      if (PENDING(ms, next_busy_signal_ms)) return;
      switch (busy_state) {
        case IN_HANDLER:
        case IN_PROCESS:
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_BUSY_PROCESSING);
          break;
        case PAUSED_FOR_USER:
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_USER);
          break;
        case PAUSED_FOR_INPUT:
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_INPUT);
          break;
        default:
          break;
      }
    }
    next_busy_signal_ms = ms + host_keepalive_interval * 1000UL;
  }
#endif 
inline void gcode_G0_G1(
  #if IS_SCARA
    bool fast_move=false
  #endif
) {
  if (IsRunning()) {
    gcode_get_destination(); 
    #if ENABLED(FWRETRACT)
      if (autoretract_enabled && !(parser.seen('X') || parser.seen('Y') || parser.seen('Z')) && parser.seen('E')) {
        const float echange = destination[E_AXIS] - current_position[E_AXIS];
        if ((echange < -MIN_RETRACT && !retracted[active_extruder]) || (echange > MIN_RETRACT && retracted[active_extruder])) {
          current_position[E_AXIS] = destination[E_AXIS]; 
          sync_plan_position_e();  
          retract(!retracted[active_extruder]);
          return;
        }
      }
    #endif 
    #if IS_SCARA
      fast_move ? prepare_uninterpolated_move_to_destination() : prepare_move_to_destination();
    #else
      prepare_move_to_destination();
    #endif
  }
}
#if ENABLED(ARC_SUPPORT)
  inline void gcode_G2_G3(bool clockwise) {
    if (IsRunning()) {
      #if ENABLED(SF_ARC_FIX)
        const bool relative_mode_backup = relative_mode;
        relative_mode = true;
      #endif
      gcode_get_destination();
      #if ENABLED(SF_ARC_FIX)
        relative_mode = relative_mode_backup;
      #endif
      float arc_offset[2] = { 0.0, 0.0 };
      if (parser.seenval('R')) {
        const float r = parser.value_linear_units(),
                    p1 = current_position[X_AXIS], q1 = current_position[Y_AXIS],
                    p2 = destination[X_AXIS], q2 = destination[Y_AXIS];
        if (r && (p2 != p1 || q2 != q1)) {
          const float e = clockwise ^ (r < 0) ? -1 : 1,           
                      dx = p2 - p1, dy = q2 - q1,                 
                      d = HYPOT(dx, dy),                          
                      h = SQRT(sq(r) - sq(d * 0.5)),              
                      mx = (p1 + p2) * 0.5, my = (q1 + q2) * 0.5, 
                      sx = -dy / d, sy = dx / d,                  
                      cx = mx + e * h * sx, cy = my + e * h * sy; 
          arc_offset[0] = cx - p1;
          arc_offset[1] = cy - q1;
        }
      }
      else {
        if (parser.seenval('I')) arc_offset[0] = parser.value_linear_units();
        if (parser.seenval('J')) arc_offset[1] = parser.value_linear_units();
      }
      if (arc_offset[0] || arc_offset[1]) {
        #if ENABLED(ARC_P_CIRCLES)
          int8_t circles_to_do = parser.byteval('P');
          if (!WITHIN(circles_to_do, 0, 100)) {
            SERIAL_ERROR_START();
            SERIAL_ERRORLNPGM(MSG_ERR_ARC_ARGS);
          }
          while (circles_to_do--)
            plan_arc(current_position, arc_offset, clockwise);
        #endif
        plan_arc(destination, arc_offset, clockwise);
        refresh_cmd_timeout();
      }
      else {
        SERIAL_ERROR_START();
        SERIAL_ERRORLNPGM(MSG_ERR_ARC_ARGS);
      }
    }
  }
#endif 
inline void gcode_G4() {
  millis_t dwell_ms = 0;
  if (parser.seenval('P')) dwell_ms = parser.value_millis(); 
  if (parser.seenval('S')) dwell_ms = parser.value_millis_from_seconds(); 
  stepper.synchronize();
  refresh_cmd_timeout();
  dwell_ms += previous_cmd_ms;  
  if (!lcd_hasstatus()) LCD_MESSAGEPGM(MSG_DWELL);
  while (PENDING(millis(), dwell_ms)) idle();
}
#if ENABLED(BEZIER_CURVE_SUPPORT)
  inline void gcode_G5() {
    if (IsRunning()) {
      gcode_get_destination();
      const float offset[] = {
        parser.linearval('I'),
        parser.linearval('J'),
        parser.linearval('P'),
        parser.linearval('Q')
      };
      plan_cubic_move(offset);
    }
  }
#endif 
#if ENABLED(FWRETRACT)
  inline void gcode_G10_G11(bool doRetract=false) {
    #if EXTRUDERS > 1
      if (doRetract)
        retracted_swap[active_extruder] = parser.boolval('S'); 
    #endif
    retract(doRetract
     #if EXTRUDERS > 1
      , retracted_swap[active_extruder]
     #endif
    );
  }
#endif 
#if ENABLED(NOZZLE_CLEAN_FEATURE)
  inline void gcode_G12() {
    if (axis_unhomed_error()) return;
    const uint8_t pattern = parser.ushortval('P', 0),
                  strokes = parser.ushortval('S', NOZZLE_CLEAN_STROKES),
                  objects = parser.ushortval('T', NOZZLE_CLEAN_TRIANGLES);
    const float radius = parser.floatval('R', NOZZLE_CLEAN_CIRCLE_RADIUS);
    Nozzle::clean(pattern, strokes, radius, objects);
  }
#endif
#if ENABLED(CNC_WORKSPACE_PLANES)
  void report_workspace_plane() {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM("Workspace Plane ");
    serialprintPGM(workspace_plane == PLANE_YZ ? PSTR("YZ\n") : workspace_plane == PLANE_ZX ? PSTR("ZX\n") : PSTR("XY\n"));
  }
  inline void gcode_G17() { workspace_plane = PLANE_XY; }
  inline void gcode_G18() { workspace_plane = PLANE_ZX; }
  inline void gcode_G19() { workspace_plane = PLANE_YZ; }
#endif 
#if ENABLED(INCH_MODE_SUPPORT)
  inline void gcode_G20() { parser.set_input_linear_units(LINEARUNIT_INCH); }
  inline void gcode_G21() { parser.set_input_linear_units(LINEARUNIT_MM); }
#endif
#if ENABLED(NOZZLE_PARK_FEATURE)
  inline void gcode_G27() {
    if (axis_unhomed_error()) return;
    Nozzle::park(parser.ushortval('P'));
  }
#endif 
#if ENABLED(QUICK_HOME)
  static void quick_home_xy() {
    current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;
    sync_plan_position();
    const int x_axis_home_dir =
      #if ENABLED(DUAL_X_CARRIAGE)
        x_home_dir(active_extruder)
      #else
        home_dir(X_AXIS)
      #endif
    ;
    const float mlx = max_length(X_AXIS),
                mly = max_length(Y_AXIS),
                mlratio = mlx > mly ? mly / mlx : mlx / mly,
                fr_mm_s = min(homing_feedrate(X_AXIS), homing_feedrate(Y_AXIS)) * SQRT(sq(mlratio) + 1.0);
    do_blocking_move_to_xy(1.5 * mlx * x_axis_home_dir, 1.5 * mly * home_dir(Y_AXIS), fr_mm_s);
    endstops.hit_on_purpose(); 
    current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;
  }
#endif 
#if ENABLED(DEBUG_LEVELING_FEATURE)
  void log_machine_info() {
    SERIAL_ECHOPGM("Machine Type: ");
    #if ENABLED(DELTA)
      SERIAL_ECHOLNPGM("Delta");
    #elif IS_SCARA
      SERIAL_ECHOLNPGM("SCARA");
    #elif IS_CORE
      SERIAL_ECHOLNPGM("Core");
    #else
      SERIAL_ECHOLNPGM("Cartesian");
    #endif
    SERIAL_ECHOPGM("Probe: ");
    #if ENABLED(PROBE_MANUALLY)
      SERIAL_ECHOLNPGM("PROBE_MANUALLY");
    #elif ENABLED(FIX_MOUNTED_PROBE)
      SERIAL_ECHOLNPGM("FIX_MOUNTED_PROBE");
    #elif ENABLED(BLTOUCH)
      SERIAL_ECHOLNPGM("BLTOUCH");
    #elif HAS_Z_SERVO_ENDSTOP
      SERIAL_ECHOLNPGM("SERVO PROBE");
    #elif ENABLED(Z_PROBE_SLED)
      SERIAL_ECHOLNPGM("Z_PROBE_SLED");
    #elif ENABLED(Z_PROBE_ALLEN_KEY)
      SERIAL_ECHOLNPGM("Z_PROBE_ALLEN_KEY");
    #else
      SERIAL_ECHOLNPGM("NONE");
    #endif
    #if HAS_BED_PROBE
      SERIAL_ECHOPAIR("Probe Offset X:", X_PROBE_OFFSET_FROM_EXTRUDER);
      SERIAL_ECHOPAIR(" Y:", Y_PROBE_OFFSET_FROM_EXTRUDER);
      SERIAL_ECHOPAIR(" Z:", zprobe_zoffset);
      #if X_PROBE_OFFSET_FROM_EXTRUDER > 0
        SERIAL_ECHOPGM(" (Right");
      #elif X_PROBE_OFFSET_FROM_EXTRUDER < 0
        SERIAL_ECHOPGM(" (Left");
      #elif Y_PROBE_OFFSET_FROM_EXTRUDER != 0
        SERIAL_ECHOPGM(" (Middle");
      #else
        SERIAL_ECHOPGM(" (Aligned With");
      #endif
      #if Y_PROBE_OFFSET_FROM_EXTRUDER > 0
        SERIAL_ECHOPGM("-Back");
      #elif Y_PROBE_OFFSET_FROM_EXTRUDER < 0
        SERIAL_ECHOPGM("-Front");
      #elif X_PROBE_OFFSET_FROM_EXTRUDER != 0
        SERIAL_ECHOPGM("-Center");
      #endif
      if (zprobe_zoffset < 0)
        SERIAL_ECHOPGM(" & Below");
      else if (zprobe_zoffset > 0)
        SERIAL_ECHOPGM(" & Above");
      else
        SERIAL_ECHOPGM(" & Same Z as");
      SERIAL_ECHOLNPGM(" Nozzle)");
    #endif
    #if HAS_ABL
      SERIAL_ECHOPGM("Auto Bed Leveling: ");
      #if ENABLED(AUTO_BED_LEVELING_LINEAR)
        SERIAL_ECHOPGM("LINEAR");
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        SERIAL_ECHOPGM("BILINEAR");
      #elif ENABLED(AUTO_BED_LEVELING_3POINT)
        SERIAL_ECHOPGM("3POINT");
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        SERIAL_ECHOPGM("UBL");
      #endif
      if (leveling_is_active()) {
        SERIAL_ECHOLNPGM(" (enabled)");
        #if ABL_PLANAR
          const float diff[XYZ] = {
            stepper.get_axis_position_mm(X_AXIS) - current_position[X_AXIS],
            stepper.get_axis_position_mm(Y_AXIS) - current_position[Y_AXIS],
            stepper.get_axis_position_mm(Z_AXIS) - current_position[Z_AXIS]
          };
          SERIAL_ECHOPGM("ABL Adjustment X");
          if (diff[X_AXIS] > 0) SERIAL_CHAR('+');
          SERIAL_ECHO(diff[X_AXIS]);
          SERIAL_ECHOPGM(" Y");
          if (diff[Y_AXIS] > 0) SERIAL_CHAR('+');
          SERIAL_ECHO(diff[Y_AXIS]);
          SERIAL_ECHOPGM(" Z");
          if (diff[Z_AXIS] > 0) SERIAL_CHAR('+');
          SERIAL_ECHO(diff[Z_AXIS]);
        #elif ENABLED(AUTO_BED_LEVELING_UBL)
          SERIAL_ECHOPAIR("UBL Adjustment Z", stepper.get_axis_position_mm(Z_AXIS) - current_position[Z_AXIS]);
        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
          SERIAL_ECHOPAIR("ABL Adjustment Z", bilinear_z_offset(current_position));
        #endif
      }
      else
        SERIAL_ECHOLNPGM(" (disabled)");
      SERIAL_EOL();
    #elif ENABLED(MESH_BED_LEVELING)
      SERIAL_ECHOPGM("Mesh Bed Leveling");
      if (leveling_is_active()) {
        float lz = current_position[Z_AXIS];
        planner.apply_leveling(current_position[X_AXIS], current_position[Y_AXIS], lz);
        SERIAL_ECHOLNPGM(" (enabled)");
        SERIAL_ECHOPAIR("MBL Adjustment Z", lz);
      }
      else
        SERIAL_ECHOPGM(" (disabled)");
      SERIAL_EOL();
    #endif 
  }
#endif 
#if ENABLED(DELTA)
  inline void home_delta() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS(">>> home_delta", current_position);
    #endif
    ZERO(current_position);
    sync_plan_position();
    current_position[X_AXIS] = current_position[Y_AXIS] = current_position[Z_AXIS] = (Z_MAX_LENGTH + 10);
    feedrate_mm_s = homing_feedrate(X_AXIS);
    line_to_current_position();
    stepper.synchronize();
    endstops.hit_on_purpose(); 
    HOMEAXIS(A);
    HOMEAXIS(B);
    HOMEAXIS(C);
    LOOP_XYZ(i) set_axis_is_at_home((AxisEnum)i);
    SYNC_PLAN_POSITION_KINEMATIC();
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("<<< home_delta", current_position);
    #endif
  }
#endif 
#if ENABLED(Z_SAFE_HOMING)
inline void home_z_safely() {
    if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
      LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
      SERIAL_ECHO_START();
      SERIAL_ECHOLNPGM(MSG_ERR_Z_HOMING);
      return;
    }
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Z_SAFE_HOMING >>>");
    #endif
    SYNC_PLAN_POSITION_KINEMATIC();
    destination[X_AXIS] = LOGICAL_X_POSITION(Z_SAFE_HOMING_X_POINT);
    destination[Y_AXIS] = LOGICAL_Y_POSITION(Z_SAFE_HOMING_Y_POINT);
    destination[Z_AXIS] = current_position[Z_AXIS]; 
    #if HOMING_Z_WITH_PROBE
      destination[X_AXIS] -= X_PROBE_OFFSET_FROM_EXTRUDER;
      destination[Y_AXIS] -= Y_PROBE_OFFSET_FROM_EXTRUDER;
    #endif
    if (position_is_reachable_xy(destination[X_AXIS], destination[Y_AXIS])) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("Z_SAFE_HOMING", destination);
      #endif
      #if ENABLED(DUAL_X_CARRIAGE)
        active_extruder_parked = false;
      #endif
      do_blocking_move_to_xy(destination[X_AXIS], destination[Y_AXIS]);
      HOMEAXIS(Z);
    }
    else {
      LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
      SERIAL_ECHO_START();
      SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
    }
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< Z_SAFE_HOMING");
    #endif
  }
#endif 
#if ENABLED(PROBE_MANUALLY)
  bool g29_in_progress = false;
#else
  constexpr bool g29_in_progress = false;
#endif
void gcode_G28(const bool always_home_all) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOLNPGM(">>> gcode_G28");
      log_machine_info();
    }
  #endif
  #if ENABLED(FYS_PRINTING_STATE)
    GLOBAL_var_V007 = MACRO_var_V01F;
  #endif
  stepper.synchronize();
  #if ENABLED(PROBE_MANUALLY)
    g29_in_progress = false;
  #endif
  #if HAS_LEVELING
    #if ENABLED(AUTO_BED_LEVELING_UBL)
      const bool ubl_state_at_entry = leveling_is_active();
    #endif
	  #if ENABLED(FYS_RESTORE_LEVELING_AFTER_G28)
      char rememberEnable = planner.abl_enabled;
    #endif
      set_bed_leveling_enabled(false);
  #endif
  #if ENABLED(CNC_WORKSPACE_PLANES)
    workspace_plane = PLANE_XY;
  #endif
  #if HOTENDS > 1
    const uint8_t old_tool_index = active_extruder;
    tool_change(0, 0, true);
  #endif
  #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
    extruder_duplication_enabled = false;
  #endif
  setup_for_endstop_or_probe_move();
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("> endstops.enable(true)");
  #endif
  endstops.enable(true); 
  #if ENABLED(DELTA)
    home_delta();
    UNUSED(always_home_all);
  #else 
    const bool homeX = always_home_all || parser.seen('X'),
               homeY = always_home_all || parser.seen('Y'),
               homeZ = always_home_all || parser.seen('Z'),
               home_all = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);
    set_destination_to_current();
    SYNC_PLAN_POSITION_KINEMATIC(); 
    #if ENABLED(FYS_G28_Z_UNMOVE_SWITCH)
    if (!myNeedRetPos)
    {
    #endif
    #if Z_HOME_DIR > 0  
      if (home_all || homeZ) {
        HOMEAXIS(Z);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> HOMEAXIS(Z)", current_position);
        #endif
      }
    #else
      if (home_all || homeX || homeY) {
        destination[Z_AXIS] = LOGICAL_Z_POSITION(Z_HOMING_HEIGHT);
        if (destination[Z_AXIS] > current_position[Z_AXIS]) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING))
              SERIAL_ECHOLNPAIR("Raise Z (before homing) to ", destination[Z_AXIS]);
          #endif
          do_blocking_move_to_z(destination[Z_AXIS]);
        }
      }
    #endif
    #if ENABLED(FYS_G28_Z_UNMOVE_SWITCH) 
      }
    #endif
    #if ENABLED(QUICK_HOME)
      if (home_all || (homeX && homeY)) quick_home_xy();
    #endif
    #if ENABLED(HOME_Y_BEFORE_X)
      if (home_all || homeY) {
        HOMEAXIS(Y);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
        #endif
      }
    #endif
    if (home_all || homeX) {
      #if ENABLED(DUAL_X_CARRIAGE)
        active_extruder = 1;
        HOMEAXIS(X);
        inactive_extruder_x_pos = RAW_X_POSITION(current_position[X_AXIS]);
        active_extruder = 0;
        HOMEAXIS(X);
        COPY(raised_parked_position, current_position);
        delayed_move_time = 0;
        active_extruder_parked = true;
      #else
        HOMEAXIS(X);
      #endif
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("> homeX", current_position);
      #endif
    }
    #if DISABLED(HOME_Y_BEFORE_X)
      if (home_all || homeY) {
        HOMEAXIS(Y);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
        #endif
      }
    #endif
    #if Z_HOME_DIR < 0
      if (home_all || homeZ) {
        #if ENABLED(Z_SAFE_HOMING)
          home_z_safely();
        #else
          HOMEAXIS(Z);
        #endif
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> (home_all || homeZ) > final", current_position);
        #endif
      } 
    #endif 
    SYNC_PLAN_POSITION_KINEMATIC();
  #endif 
  endstops.not_homing();
  #if ENABLED(DELTA) && ENABLED(DELTA_HOME_TO_SAFE_ZONE)
    do_blocking_move_to_z(delta_clip_start_height);
  #endif
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    set_bed_leveling_enabled(ubl_state_at_entry);
  #endif
  #if HAS_LEVELING
	#if ENABLED(FYS_RESTORE_LEVELING_AFTER_G28)
     planner.abl_enabled = rememberEnable;
  	#endif
  #endif
  clean_up_after_endstop_or_probe_move();
  #if HOTENDS > 1
    tool_change(old_tool_index, 0, true);
  #endif
  lcd_refresh();
  report_current_position();
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< gcode_G28");
  #endif
  #if ENABLED(FYS_PRINTING_STATE)
    GLOBAL_var_V007 = MACRO_var_V01A;
  #endif
} 
void home_all_axes() { gcode_G28(true); }
#if HAS_PROBING_PROCEDURE
  void out_of_range_error(const char* p_edge) {
    SERIAL_PROTOCOLPGM("?Probe ");
    serialprintPGM(p_edge);
    SERIAL_PROTOCOLLNPGM(" position out of range.");
  }
#endif
#if ENABLED(MESH_BED_LEVELING) || ENABLED(PROBE_MANUALLY)
  #if ENABLED(PROBE_MANUALLY) && ENABLED(LCD_BED_LEVELING)
    extern bool lcd_wait_for_move;
  #endif
  inline void _manual_goto_xy(const float &x, const float &y) {
    const float old_feedrate_mm_s = feedrate_mm_s;
    #if MANUAL_PROBE_HEIGHT > 0
      feedrate_mm_s = homing_feedrate(Z_AXIS);
      current_position[Z_AXIS] = LOGICAL_Z_POSITION(Z_MIN_POS) + MANUAL_PROBE_HEIGHT;
      line_to_current_position();
    #endif
    feedrate_mm_s = MMM_TO_MMS(XY_PROBE_SPEED);
    current_position[X_AXIS] = LOGICAL_X_POSITION(x);
    current_position[Y_AXIS] = LOGICAL_Y_POSITION(y);
    line_to_current_position();
    #if MANUAL_PROBE_HEIGHT > 0
      feedrate_mm_s = homing_feedrate(Z_AXIS);
      current_position[Z_AXIS] = LOGICAL_Z_POSITION(Z_MIN_POS); 
      line_to_current_position();
    #endif
    feedrate_mm_s = old_feedrate_mm_s;
    stepper.synchronize();
    #if ENABLED(PROBE_MANUALLY) && ENABLED(LCD_BED_LEVELING)
      lcd_wait_for_move = false;
    #endif
  }
#endif
#if ENABLED(MESH_BED_LEVELING)
  void echo_not_entered() { SERIAL_PROTOCOLLNPGM(" not entered."); }
  void mbl_mesh_report() {
    SERIAL_PROTOCOLLNPGM("Num X,Y: " STRINGIFY(GRID_MAX_POINTS_X) "," STRINGIFY(GRID_MAX_POINTS_Y));
    SERIAL_PROTOCOLPGM("Z offset: "); SERIAL_PROTOCOL_F(mbl.z_offset, 5);
    SERIAL_PROTOCOLLNPGM("\nMeasured points:");
    print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 5,
      [](const uint8_t ix, const uint8_t iy) { return mbl.z_values[ix][iy]; }
    );
  }
  void mesh_probing_done() {
    mbl.set_has_mesh(true);
    home_all_axes();
    set_bed_leveling_enabled(true);
    #if ENABLED(MESH_G28_REST_ORIGIN)
      current_position[Z_AXIS] = LOGICAL_Z_POSITION(Z_MIN_POS);
      set_destination_to_current();
      line_to_destination(homing_feedrate(Z_AXIS));
      stepper.synchronize();
    #endif
  }
  inline void gcode_G29() {
    static int mbl_probe_index = -1;
    #if HAS_SOFTWARE_ENDSTOPS
      static bool enable_soft_endstops;
    #endif
    const MeshLevelingState state = (MeshLevelingState)parser.byteval('S', (int8_t)MeshReport);
    if (!WITHIN(state, 0, 5)) {
      SERIAL_PROTOCOLLNPGM("S out of range (0-5).");
      return;
    }
    int8_t px, py;
    switch (state) {
      case MeshReport:
        if (leveling_is_valid()) {
          SERIAL_PROTOCOLLNPAIR("State: ", leveling_is_active() ? MSG_ON : MSG_OFF);
          mbl_mesh_report();
        }
        else
          SERIAL_PROTOCOLLNPGM("Mesh bed leveling has no data.");
        break;
      case MeshStart:
        mbl.reset();
        mbl_probe_index = 0;
        enqueue_and_echo_commands_P(PSTR("G28\nG29 S2"));
        break;
      case MeshNext:
        if (mbl_probe_index < 0) {
          SERIAL_PROTOCOLLNPGM("Start mesh probing with \"G29 S1\" first.");
          return;
        }
        if (mbl_probe_index == 0) {
          #if HAS_SOFTWARE_ENDSTOPS
            enable_soft_endstops = soft_endstops_enabled;
          #endif
        }
        else {
          mbl.set_zigzag_z(mbl_probe_index - 1, current_position[Z_AXIS]);
          #if HAS_SOFTWARE_ENDSTOPS
            soft_endstops_enabled = enable_soft_endstops;
          #endif
        }
        if (mbl_probe_index < GRID_MAX_POINTS) {
          mbl.zigzag(mbl_probe_index, px, py);
          _manual_goto_xy(mbl.index_to_xpos[px], mbl.index_to_ypos[py]);
          #if HAS_SOFTWARE_ENDSTOPS
            soft_endstops_enabled = false;
          #endif
          mbl_probe_index++;
        }
        else {
          current_position[Z_AXIS] = LOGICAL_Z_POSITION(Z_MIN_POS) + MANUAL_PROBE_HEIGHT;
          line_to_current_position();
          stepper.synchronize();
          mbl_probe_index = -1;
          SERIAL_PROTOCOLLNPGM("Mesh probing done.");
          BUZZ(100, 659);
          BUZZ(100, 698);
          mesh_probing_done();
        }
        break;
      case MeshSet:
        if (parser.seenval('X')) {
          px = parser.value_int() - 1;
          if (!WITHIN(px, 0, GRID_MAX_POINTS_X - 1)) {
            SERIAL_PROTOCOLLNPGM("X out of range (1-" STRINGIFY(GRID_MAX_POINTS_X) ").");
            return;
          }
        }
        else {
          SERIAL_CHAR('X'); echo_not_entered();
          return;
        }
        if (parser.seenval('Y')) {
          py = parser.value_int() - 1;
          if (!WITHIN(py, 0, GRID_MAX_POINTS_Y - 1)) {
            SERIAL_PROTOCOLLNPGM("Y out of range (1-" STRINGIFY(GRID_MAX_POINTS_Y) ").");
            return;
          }
        }
        else {
          SERIAL_CHAR('Y'); echo_not_entered();
          return;
        }
        if (parser.seenval('Z')) {
          mbl.z_values[px][py] = parser.value_linear_units();
        }
        else {
          SERIAL_CHAR('Z'); echo_not_entered();
          return;
        }
        break;
      case MeshSetZOffset:
        if (parser.seenval('Z')) {
          mbl.z_offset = parser.value_linear_units();
        }
        else {
          SERIAL_CHAR('Z'); echo_not_entered();
          return;
        }
        break;
      case MeshReset:
        reset_bed_level();
        break;
    } 
    #if ENABLED(FYS_SAVE_SETTING_AFTER_G29)
      settings.save();
    #endif
    report_current_position();
  }
#elif HAS_ABL && DISABLED(AUTO_BED_LEVELING_UBL)
  #if ABL_GRID
    #if ENABLED(PROBE_Y_FIRST)
      #define PR_OUTER_VAR xCount
      #define PR_OUTER_END abl_grid_points_x
      #define PR_INNER_VAR yCount
      #define PR_INNER_END abl_grid_points_y
    #else
      #define PR_OUTER_VAR yCount
      #define PR_OUTER_END abl_grid_points_y
      #define PR_INNER_VAR xCount
      #define PR_INNER_END abl_grid_points_x
    #endif
  #endif
  inline void gcode_G29() {
    #if ENABLED(FYS_PRINTING_STATE)
      GLOBAL_var_V007 = MACRO_var_V023;
    #endif
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      const bool query = parser.seen('Q');
      const uint8_t old_debug_flags = marlin_debug_flags;
      if (query) marlin_debug_flags |= DEBUG_LEVELING;
      if (DEBUGGING(LEVELING)) {
        DEBUG_POS(">>> gcode_G29", current_position);
        log_machine_info();
      }
      marlin_debug_flags = old_debug_flags;
      #if DISABLED(PROBE_MANUALLY)
        if (query) 
    {
      #if ENABLED(FYS_LCD_EVENT)
        GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
      #endif
      #if ENABLED(FYS_PRINTING_STATE)
        GLOBAL_var_V007 = MACRO_var_V01A;
      #endif
      return;
    }
      #endif
    #endif
    #if ENABLED(PROBE_MANUALLY)
      const bool seenA = parser.seen('A'), seenQ = parser.seen('Q'), no_action = seenA || seenQ;
    #endif
    #if ENABLED(DEBUG_LEVELING_FEATURE) && DISABLED(PROBE_MANUALLY)
      const bool faux = parser.boolval('C');
    #elif ENABLED(PROBE_MANUALLY)
      const bool faux = no_action;
    #else
      bool constexpr faux = false;
    #endif
    if (axis_unhomed_error())
    {
      #if ENABLED(FYS_LCD_EVENT)
        GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
      #endif
      #if ENABLED(FYS_PRINTING_STATE)
        GLOBAL_var_V007 = MACRO_var_V01A;
      #endif
      return;
    }
    #if ENABLED(PROBE_MANUALLY)
      #define ABL_VAR static
    #else
      #define ABL_VAR
    #endif
    ABL_VAR int verbose_level;
    ABL_VAR float xProbe, yProbe, measured_z;
    ABL_VAR bool dryrun, abl_should_enable;
    #if ENABLED(PROBE_MANUALLY) || ENABLED(AUTO_BED_LEVELING_LINEAR)
      ABL_VAR int abl_probe_index;
    #endif
    #if HAS_SOFTWARE_ENDSTOPS && ENABLED(PROBE_MANUALLY)
      ABL_VAR bool enable_soft_endstops = true;
    #endif
    #if ABL_GRID
      #if ENABLED(PROBE_MANUALLY)
        ABL_VAR uint8_t PR_OUTER_VAR;
        ABL_VAR  int8_t PR_INNER_VAR;
      #endif
      ABL_VAR int left_probe_bed_position, right_probe_bed_position, front_probe_bed_position, back_probe_bed_position;
      ABL_VAR float xGridSpacing, yGridSpacing;
      #if ENABLED(AUTO_BED_LEVELING_LINEAR)
        ABL_VAR uint8_t abl_grid_points_x = GRID_MAX_POINTS_X,
                        abl_grid_points_y = GRID_MAX_POINTS_Y;
        ABL_VAR bool do_topography_map;
      #else 
        uint8_t constexpr abl_grid_points_x = GRID_MAX_POINTS_X,
                          abl_grid_points_y = GRID_MAX_POINTS_Y;
      #endif
      #if ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(PROBE_MANUALLY)
        #if ENABLED(AUTO_BED_LEVELING_LINEAR)
          ABL_VAR int abl2;
        #else 
          int constexpr abl2 = GRID_MAX_POINTS;
        #endif
      #endif
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        ABL_VAR float zoffset;
      #elif ENABLED(AUTO_BED_LEVELING_LINEAR)
        ABL_VAR int indexIntoAB[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
        ABL_VAR float eqnAMatrix[GRID_MAX_POINTS * 3], 
                     eqnBVector[GRID_MAX_POINTS],     
                     mean;
      #endif
    #elif ENABLED(AUTO_BED_LEVELING_3POINT)
      int constexpr abl2 = 3;
      ABL_VAR vector_3 points[3] = {
        vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, 0),
        vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, 0),
        vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, 0)
      };
    #endif 
    if (!g29_in_progress) {
      #if ENABLED(PROBE_MANUALLY) || ENABLED(AUTO_BED_LEVELING_LINEAR)
        abl_probe_index = -1;
      #endif
      abl_should_enable = leveling_is_active();
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (parser.seen('W')) {
          if (!leveling_is_valid()) {
            SERIAL_ERROR_START();
            SERIAL_ERRORLNPGM("No bilinear grid");
            #if ENABLED(FYS_LCD_EVENT)
              GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
            #endif
            #if ENABLED(FYS_PRINTING_STATE)
              GLOBAL_var_V007 = MACRO_var_V01A;
            #endif
            return;
          }
          const float z = parser.floatval('Z', RAW_CURRENT_POSITION(Z));
          if (!WITHIN(z, -10, 10)) {
            SERIAL_ERROR_START();
            SERIAL_ERRORLNPGM("Bad Z value");
            #if ENABLED(FYS_LCD_EVENT)
              GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
            #endif
            #if ENABLED(FYS_PRINTING_STATE)
              GLOBAL_var_V007 = MACRO_var_V01A;
            #endif
            return;
          }
          const float x = parser.floatval('X', NAN),
                      y = parser.floatval('Y', NAN);
          int8_t i = parser.byteval('I', -1),
                 j = parser.byteval('J', -1);
          if (!isnan(x) && !isnan(y)) {
            i = (x - LOGICAL_X_POSITION(bilinear_start[X_AXIS]) + 0.5 * xGridSpacing) / xGridSpacing;
            j = (y - LOGICAL_Y_POSITION(bilinear_start[Y_AXIS]) + 0.5 * yGridSpacing) / yGridSpacing;
            i = constrain(i, 0, GRID_MAX_POINTS_X - 1);
            j = constrain(j, 0, GRID_MAX_POINTS_Y - 1);
          }
          if (WITHIN(i, 0, GRID_MAX_POINTS_X - 1) && WITHIN(j, 0, GRID_MAX_POINTS_Y)) {
            set_bed_leveling_enabled(false);
            z_values[i][j] = z;
            #if ENABLED(ABL_BILINEAR_SUBDIVISION)
              bed_level_virt_interpolate();
            #endif
            set_bed_leveling_enabled(abl_should_enable);
          }
          #if ENABLED(FYS_LCD_EVENT)
            GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
          #endif
          #if ENABLED(FYS_PRINTING_STATE)
            GLOBAL_var_V007 = MACRO_var_V01A;
          #endif
          return;
        } 
      #endif
      #if HAS_LEVELING
        if (parser.seen('J')) {
          reset_bed_level();
          #if ENABLED(FYS_LCD_EVENT)
            GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
          #endif
          #if ENABLED(FYS_PRINTING_STATE)
            GLOBAL_var_V007 = MACRO_var_V01A;
          #endif
          return;
        }
      #endif
      verbose_level = parser.intval('V');
      if (!WITHIN(verbose_level, 0, 4)) {
        SERIAL_PROTOCOLLNPGM("?(V)erbose level is implausible (0-4).");
        #if ENABLED(FYS_LCD_EVENT)
          GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
        #endif
        #if ENABLED(FYS_PRINTING_STATE)
          GLOBAL_var_V007 = MACRO_var_V01A;
        #endif
        return;
      }
      dryrun = parser.boolval('D')
        #if ENABLED(PROBE_MANUALLY)
          || no_action
        #endif
      ;
      #if ENABLED(AUTO_BED_LEVELING_LINEAR)
        do_topography_map = verbose_level > 2 || parser.boolval('T');
        abl_grid_points_x = parser.intval('X', GRID_MAX_POINTS_X);
        abl_grid_points_y = parser.intval('Y', GRID_MAX_POINTS_Y);
        if (parser.seenval('P')) abl_grid_points_x = abl_grid_points_y = parser.value_int();
        if (abl_grid_points_x < 2 || abl_grid_points_y < 2) {
          SERIAL_PROTOCOLLNPGM("?Number of probe points is implausible (2 minimum).");
          #if ENABLED(FYS_LCD_EVENT)
            GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
          #endif
          #if ENABLED(FYS_PRINTING_STATE)
            GLOBAL_var_V007 = MACRO_var_V01A;
          #endif
          return;
        }
        abl2 = abl_grid_points_x * abl_grid_points_y;
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        zoffset = parser.linearval('Z');
      #endif
      #if ABL_GRID
        xy_probe_feedrate_mm_s = MMM_TO_MMS(parser.linearval('S', XY_PROBE_SPEED));
        left_probe_bed_position = (int)parser.linearval('L', LOGICAL_X_POSITION(LEFT_PROBE_BED_POSITION));
        right_probe_bed_position = (int)parser.linearval('R', LOGICAL_X_POSITION(RIGHT_PROBE_BED_POSITION));
        front_probe_bed_position = (int)parser.linearval('F', LOGICAL_Y_POSITION(FRONT_PROBE_BED_POSITION));
        back_probe_bed_position = (int)parser.linearval('B', LOGICAL_Y_POSITION(BACK_PROBE_BED_POSITION));
        const bool left_out_l = left_probe_bed_position < LOGICAL_X_POSITION(MIN_PROBE_X),
                   left_out = left_out_l || left_probe_bed_position > right_probe_bed_position - (MIN_PROBE_EDGE),
                   right_out_r = right_probe_bed_position > LOGICAL_X_POSITION(MAX_PROBE_X),
                   right_out = right_out_r || right_probe_bed_position < left_probe_bed_position + MIN_PROBE_EDGE,
                   front_out_f = front_probe_bed_position < LOGICAL_Y_POSITION(MIN_PROBE_Y),
                   front_out = front_out_f || front_probe_bed_position > back_probe_bed_position - (MIN_PROBE_EDGE),
                   back_out_b = back_probe_bed_position > LOGICAL_Y_POSITION(MAX_PROBE_Y),
                   back_out = back_out_b || back_probe_bed_position < front_probe_bed_position + MIN_PROBE_EDGE;
        if (left_out || right_out || front_out || back_out) {
          if (left_out) {
            out_of_range_error(PSTR("(L)eft"));
            left_probe_bed_position = left_out_l ? LOGICAL_X_POSITION(MIN_PROBE_X) : right_probe_bed_position - (MIN_PROBE_EDGE);
          }
          if (right_out) {
            out_of_range_error(PSTR("(R)ight"));
            right_probe_bed_position = right_out_r ? LOGICAL_Y_POSITION(MAX_PROBE_X) : left_probe_bed_position + MIN_PROBE_EDGE;
          }
          if (front_out) {
            out_of_range_error(PSTR("(F)ront"));
            front_probe_bed_position = front_out_f ? LOGICAL_Y_POSITION(MIN_PROBE_Y) : back_probe_bed_position - (MIN_PROBE_EDGE);
          }
          if (back_out) {
            out_of_range_error(PSTR("(B)ack"));
            back_probe_bed_position = back_out_b ? LOGICAL_Y_POSITION(MAX_PROBE_Y) : front_probe_bed_position + MIN_PROBE_EDGE;
          }
          #if ENABLED(FYS_LCD_EVENT)
            GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
          #endif
          #if ENABLED(FYS_PRINTING_STATE)
            GLOBAL_var_V007 = MACRO_var_V01A;
          #endif
          return;
        }
        xGridSpacing = (right_probe_bed_position - left_probe_bed_position) / (abl_grid_points_x - 1);
        yGridSpacing = (back_probe_bed_position - front_probe_bed_position) / (abl_grid_points_y - 1);
      #endif 
      if (verbose_level > 0) {
        SERIAL_PROTOCOLLNPGM("G29 Auto Bed Leveling");
        if (dryrun) SERIAL_PROTOCOLLNPGM("Running in DRY-RUN mode");
      }
      stepper.synchronize();
      planner.abl_enabled = false;
      if (!dryrun) {
        set_current_from_steppers_for_axis(ALL_AXES);
        SYNC_PLAN_POSITION_KINEMATIC();
      }
      if (!faux) setup_for_endstop_or_probe_move();
      #if HAS_BED_PROBE
      if (DEPLOY_PROBE()) {
          planner.abl_enabled = abl_should_enable;
          #if ENABLED(FYS_LCD_EVENT)
            GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
          #endif
          #if ENABLED(FYS_PRINTING_STATE)
            GLOBAL_var_V007 = MACRO_var_V01A;
          #endif
          return;
      }
      #endif
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if ( xGridSpacing != bilinear_grid_spacing[X_AXIS]
          || yGridSpacing != bilinear_grid_spacing[Y_AXIS]
          || left_probe_bed_position != LOGICAL_X_POSITION(bilinear_start[X_AXIS])
          || front_probe_bed_position != LOGICAL_Y_POSITION(bilinear_start[Y_AXIS])
        ) {
          if (dryrun) {
            planner.abl_enabled = abl_should_enable;
          }
          reset_bed_level();
          bilinear_grid_spacing[X_AXIS] = xGridSpacing;
          bilinear_grid_spacing[Y_AXIS] = yGridSpacing;
          bilinear_start[X_AXIS] = RAW_X_POSITION(left_probe_bed_position);
          bilinear_start[Y_AXIS] = RAW_Y_POSITION(front_probe_bed_position);
          abl_should_enable = false;
        }
      #elif ENABLED(AUTO_BED_LEVELING_LINEAR)
        mean = 0.0;
      #endif 
      #if ENABLED(AUTO_BED_LEVELING_3POINT)
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("> 3-point Leveling");
        #endif
        points[0].z = points[1].z = points[2].z = 0;
      #endif 
    } 
    #if ENABLED(PROBE_MANUALLY)
      if (!no_action) {
        ++abl_probe_index;
        g29_in_progress = true;
      }
      if (seenA && g29_in_progress) {
        SERIAL_PROTOCOLLNPGM("Manual G29 aborted");
        #if HAS_SOFTWARE_ENDSTOPS
          soft_endstops_enabled = enable_soft_endstops;
        #endif
        planner.abl_enabled = abl_should_enable;
        g29_in_progress = false;
        #if ENABLED(LCD_BED_LEVELING)
          lcd_wait_for_move = false;
        #endif
      }
      if (verbose_level || seenQ) {
        SERIAL_PROTOCOLPGM("Manual G29 ");
        if (g29_in_progress) {
          SERIAL_PROTOCOLPAIR("point ", min(abl_probe_index + 1, abl2));
          SERIAL_PROTOCOLLNPAIR(" of ", abl2);
        }
        else
          SERIAL_PROTOCOLLNPGM("idle");
      }
      if (no_action) 
      {
        #if ENABLED(FYS_LCD_EVENT)
          GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
        #endif
        #if ENABLED(FYS_PRINTING_STATE)
          GLOBAL_var_V007 = MACRO_var_V01A;
        #endif
        return;
      }
      if (abl_probe_index == 0) {
        #if HAS_SOFTWARE_ENDSTOPS
          enable_soft_endstops = soft_endstops_enabled;
        #endif
      }
      else {
        measured_z = current_position[Z_AXIS];
        #if ENABLED(AUTO_BED_LEVELING_LINEAR)
          mean += measured_z;
          eqnBVector[abl_probe_index] = measured_z;
          eqnAMatrix[abl_probe_index + 0 * abl2] = xProbe;
          eqnAMatrix[abl_probe_index + 1 * abl2] = yProbe;
          eqnAMatrix[abl_probe_index + 2 * abl2] = 1;
        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
          z_values[xCount][yCount] = measured_z + zoffset;
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_PROTOCOLPAIR("Save X", xCount);
              SERIAL_PROTOCOLPAIR(" Y", yCount);
              SERIAL_PROTOCOLLNPAIR(" Z", measured_z + zoffset);
            }
          #endif
        #elif ENABLED(AUTO_BED_LEVELING_3POINT)
          points[abl_probe_index].z = measured_z;
        #endif
      }
      #if ABL_GRID
        while (abl_probe_index < abl2) {
          PR_OUTER_VAR = abl_probe_index / PR_INNER_END;
          PR_INNER_VAR = abl_probe_index - (PR_OUTER_VAR * PR_INNER_END);
          bool zig = (PR_OUTER_VAR & 1); 
          if (zig) PR_INNER_VAR = (PR_INNER_END - 1) - PR_INNER_VAR;
          const float xBase = xCount * xGridSpacing + left_probe_bed_position,
                      yBase = yCount * yGridSpacing + front_probe_bed_position;
          xProbe = FLOOR(xBase + (xBase < 0 ? 0 : 0.5));
          yProbe = FLOOR(yBase + (yBase < 0 ? 0 : 0.5));
          #if ENABLED(AUTO_BED_LEVELING_LINEAR)
            indexIntoAB[xCount][yCount] = abl_probe_index;
          #endif
          if (position_is_reachable_xy(xProbe, yProbe)) break;
          ++abl_probe_index;
        }
        if (abl_probe_index < abl2) {
          _manual_goto_xy(xProbe, yProbe); 
          #if HAS_SOFTWARE_ENDSTOPS
            soft_endstops_enabled = false;
          #endif
          #if ENABLED(FYS_LCD_EVENT)
            GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
          #endif
          #if ENABLED(FYS_PRINTING_STATE)
            GLOBAL_var_V007 = MACRO_var_V01A;
          #endif
          return;
        }
        else {
          SERIAL_PROTOCOLLNPGM("Grid probing done.");
          #if HAS_SOFTWARE_ENDSTOPS
            soft_endstops_enabled = enable_soft_endstops;
          #endif
        }
      #elif ENABLED(AUTO_BED_LEVELING_3POINT)
        if (abl_probe_index < 3) {
          xProbe = LOGICAL_X_POSITION(points[abl_probe_index].x);
          yProbe = LOGICAL_Y_POSITION(points[abl_probe_index].y);
          #if HAS_SOFTWARE_ENDSTOPS
            soft_endstops_enabled = false;
          #endif
          #if ENABLED(FYS_LCD_EVENT)
            GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
          #endif
          #if ENABLED(FYS_PRINTING_STATE)
            GLOBAL_var_V007 = MACRO_var_V01A;
          #endif
          return;
        }
        else {
          SERIAL_PROTOCOLLNPGM("3-point probing done.");
          #if HAS_SOFTWARE_ENDSTOPS
            soft_endstops_enabled = enable_soft_endstops;
          #endif
          if (!dryrun) {
            vector_3 planeNormal = vector_3::cross(points[0] - points[1], points[2] - points[1]).get_normal();
            if (planeNormal.z < 0) {
              planeNormal.x *= -1;
              planeNormal.y *= -1;
              planeNormal.z *= -1;
            }
            planner.bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
            abl_should_enable = false;
          }
        }
      #endif 
    #else 
      const bool stow_probe_after_each = parser.boolval('E');
      #if ABL_GRID
        bool zig = PR_OUTER_END & 1;  
        for (uint8_t PR_OUTER_VAR = 0; PR_OUTER_VAR < PR_OUTER_END; PR_OUTER_VAR++) {
          int8_t inStart, inStop, inInc;
          if (zig) { 
            inStart = 0;
            inStop = PR_INNER_END;
            inInc = 1;
          }
          else {     
            inStart = PR_INNER_END - 1;
            inStop = -1;
            inInc = -1;
          }
          zig ^= true; 
          for (int8_t PR_INNER_VAR = inStart; PR_INNER_VAR != inStop; PR_INNER_VAR += inInc) {
            float xBase = left_probe_bed_position + xGridSpacing * xCount,
                  yBase = front_probe_bed_position + yGridSpacing * yCount;
            xProbe = FLOOR(xBase + (xBase < 0 ? 0 : 0.5));
            yProbe = FLOOR(yBase + (yBase < 0 ? 0 : 0.5));
            #if ENABLED(AUTO_BED_LEVELING_LINEAR)
              indexIntoAB[xCount][yCount] = ++abl_probe_index; 
            #endif
            #if IS_KINEMATIC 
              if (!position_is_reachable_by_probe_xy(xProbe, yProbe)) continue;
            #endif
            measured_z = faux ? 0.001 * random(-100, 101) : probe_pt(xProbe, yProbe, stow_probe_after_each, verbose_level);
        if (isnan(measured_z)) {
              planner.abl_enabled = abl_should_enable;
              #if ENABLED(FYS_LCD_EVENT)
                GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
              #endif
              #if ENABLED(FYS_PRINTING_STATE)
                GLOBAL_var_V007 = MACRO_var_V01A;
              #endif
              return;
            }
            #if ENABLED(AUTO_BED_LEVELING_LINEAR)
              mean += measured_z;
              eqnBVector[abl_probe_index] = measured_z;
              eqnAMatrix[abl_probe_index + 0 * abl2] = xProbe;
              eqnAMatrix[abl_probe_index + 1 * abl2] = yProbe;
              eqnAMatrix[abl_probe_index + 2 * abl2] = 1;
            #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
              z_values[xCount][yCount] = measured_z + zoffset;
            #endif
            abl_should_enable = false;
            idle();
          } 
        } 
      #elif ENABLED(AUTO_BED_LEVELING_3POINT)
        for (uint8_t i = 0; i < 3; ++i) {
          xProbe = LOGICAL_X_POSITION(points[i].x);
          yProbe = LOGICAL_Y_POSITION(points[i].y);
          measured_z = faux ? 0.001 * random(-100, 101) : probe_pt(xProbe, yProbe, stow_probe_after_each, verbose_level);
          if (isnan(measured_z)) {
            planner.abl_enabled = abl_should_enable;
            #if ENABLED(FYS_LCD_EVENT)
              GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
            #endif
            #if ENABLED(FYS_PRINTING_STATE)
              GLOBAL_var_V007 = MACRO_var_V01A;
            #endif
            return;
          }
          points[i].z = measured_z;
        }
        if (!dryrun) {
          vector_3 planeNormal = vector_3::cross(points[0] - points[1], points[2] - points[1]).get_normal();
          if (planeNormal.z < 0) {
            planeNormal.x *= -1;
            planeNormal.y *= -1;
            planeNormal.z *= -1;
          }
          planner.bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
          abl_should_enable = false;
        }
      #endif 
      if (STOW_PROBE()) {
        planner.abl_enabled = abl_should_enable;
        #if ENABLED(FYS_LCD_EVENT)
          GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
        #endif
        #if ENABLED(FYS_PRINTING_STATE)
          GLOBAL_var_V007 = MACRO_var_V01A;
        #endif
        return;
      }
    #endif 
    if (!faux) clean_up_after_endstop_or_probe_move();
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("> probing complete", current_position);
    #endif
    #if ENABLED(PROBE_MANUALLY)
      g29_in_progress = false;
      #if ENABLED(LCD_BED_LEVELING)
        lcd_wait_for_move = false;
      #endif
    #endif
    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      if (!dryrun) extrapolate_unprobed_bed_level();
      print_bilinear_leveling_grid();
      refresh_bed_level();
      #if ENABLED(ABL_BILINEAR_SUBDIVISION)
        bed_level_virt_print();
      #endif
    #elif ENABLED(AUTO_BED_LEVELING_LINEAR)
      float plane_equation_coefficients[3];
      qr_solve(plane_equation_coefficients, abl2, 3, eqnAMatrix, eqnBVector);
      mean /= abl2;
      if (verbose_level) {
        SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
        SERIAL_PROTOCOL_F(plane_equation_coefficients[0], 8);
        SERIAL_PROTOCOLPGM(" b: ");
        SERIAL_PROTOCOL_F(plane_equation_coefficients[1], 8);
        SERIAL_PROTOCOLPGM(" d: ");
        SERIAL_PROTOCOL_F(plane_equation_coefficients[2], 8);
        SERIAL_EOL();
        if (verbose_level > 2) {
          SERIAL_PROTOCOLPGM("Mean of sampled points: ");
          SERIAL_PROTOCOL_F(mean, 8);
          SERIAL_EOL();
        }
      }
      if (!dryrun) {
        planner.bed_level_matrix = matrix_3x3::create_look_at(
          vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1)
        );
      }
      if (do_topography_map) {
        SERIAL_PROTOCOLLNPGM("\nBed Height Topography:\n"
                               "   +--- BACK --+\n"
                               "   |           |\n"
                               " L |    (+)    | R\n"
                               " E |           | I\n"
                               " F | (-) N (+) | G\n"
                               " T |           | H\n"
                               "   |    (-)    | T\n"
                               "   |           |\n"
                               "   O-- FRONT --+\n"
                               " (0,0)");
        float min_diff = 999;
        for (int8_t yy = abl_grid_points_y - 1; yy >= 0; yy--) {
          for (uint8_t xx = 0; xx < abl_grid_points_x; xx++) {
            int ind = indexIntoAB[xx][yy];
            float diff = eqnBVector[ind] - mean,
                  x_tmp = eqnAMatrix[ind + 0 * abl2],
                  y_tmp = eqnAMatrix[ind + 1 * abl2],
                  z_tmp = 0;
            apply_rotation_xyz(planner.bed_level_matrix, x_tmp, y_tmp, z_tmp);
            NOMORE(min_diff, eqnBVector[ind] - z_tmp);
            if (diff >= 0.0)
              SERIAL_PROTOCOLPGM(" +");   
            else
              SERIAL_PROTOCOLCHAR(' ');
            SERIAL_PROTOCOL_F(diff, 5);
          } 
          SERIAL_EOL();
        } 
        SERIAL_EOL();
        if (verbose_level > 3) {
          SERIAL_PROTOCOLLNPGM("\nCorrected Bed Height vs. Bed Topology:");
          for (int8_t yy = abl_grid_points_y - 1; yy >= 0; yy--) {
            for (uint8_t xx = 0; xx < abl_grid_points_x; xx++) {
              int ind = indexIntoAB[xx][yy];
              float x_tmp = eqnAMatrix[ind + 0 * abl2],
                    y_tmp = eqnAMatrix[ind + 1 * abl2],
                    z_tmp = 0;
              apply_rotation_xyz(planner.bed_level_matrix, x_tmp, y_tmp, z_tmp);
              float diff = eqnBVector[ind] - z_tmp - min_diff;
              if (diff >= 0.0)
                SERIAL_PROTOCOLPGM(" +");
              else
                SERIAL_PROTOCOLCHAR(' ');
              SERIAL_PROTOCOL_F(diff, 5);
            } 
            SERIAL_EOL();
          } 
          SERIAL_EOL();
        }
      } 
    #endif 
    #if ABL_PLANAR
      if (verbose_level > 0)
        planner.bed_level_matrix.debug(PSTR("\n\nBed Level Correction Matrix:"));
      if (!dryrun) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("G29 uncorrected XYZ", current_position);
        #endif
        float converted[XYZ];
        COPY(converted, current_position);
        planner.abl_enabled = true;
        planner.unapply_leveling(converted); 
        planner.abl_enabled = false;
        if ( NEAR(current_position[X_AXIS], xProbe - (X_PROBE_OFFSET_FROM_EXTRUDER))
          && NEAR(current_position[Y_AXIS], yProbe - (Y_PROBE_OFFSET_FROM_EXTRUDER))
        ) {
          const float simple_z = current_position[Z_AXIS] - measured_z;
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOPAIR("Z from Probe:", simple_z);
              SERIAL_ECHOPAIR("  Matrix:", converted[Z_AXIS]);
              SERIAL_ECHOLNPAIR("  Discrepancy:", simple_z - converted[Z_AXIS]);
            }
          #endif
          converted[Z_AXIS] = simple_z;
        }
        COPY(current_position, converted);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("G29 corrected XYZ", current_position);
        #endif
      }
    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
      if (!dryrun) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR("G29 uncorrected Z:", current_position[Z_AXIS]);
        #endif
        current_position[Z_AXIS] -= bilinear_z_offset(current_position);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR(" corrected Z:", current_position[Z_AXIS]);
        #endif
      }
    #endif 
    #ifdef Z_PROBE_END_SCRIPT
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR("Z Probe End Script: ", Z_PROBE_END_SCRIPT);
      #endif
      enqueue_and_echo_commands_P(PSTR(Z_PROBE_END_SCRIPT));
      stepper.synchronize();
    #endif
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< gcode_G29");
    #endif
    report_current_position();
    KEEPALIVE_STATE(IN_HANDLER);
    planner.abl_enabled = dryrun ? abl_should_enable : true;
    #if ENABLED(FYS_SAVE_SETTING_AFTER_G29)
      settings.save();
      SERIAL_ECHOLNPGM("Martrix save ok.");
    #endif
    #if ABL_PLANAR 
    for (char i = 0; i < 9;i++) MYSERIAL.println(planner.bed_level_matrix.matrix[i]);
    #endif
    if (planner.abl_enabled)
      SYNC_PLAN_POSITION_KINEMATIC();
    #if ENABLED(FYS_LCD_EVENT)
      GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V024);
    #endif
    #if ENABLED(FYS_PRINTING_STATE)
      GLOBAL_var_V007 = MACRO_var_V01A;
    #endif
  } 
#endif 
#if HAS_BED_PROBE
  inline void gcode_G30() {
    const float xpos = parser.linearval('X', current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER),
                ypos = parser.linearval('Y', current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER);
    if (!position_is_reachable_by_probe_xy(xpos, ypos)) return;
    #if HAS_LEVELING
      set_bed_leveling_enabled(false);
    #endif
    setup_for_endstop_or_probe_move();
    const float measured_z = probe_pt(xpos, ypos, parser.boolval('S', true), 1);
    if (!isnan(measured_z)) {
      SERIAL_PROTOCOLPAIR("Bed X: ", FIXFLOAT(xpos));
      SERIAL_PROTOCOLPAIR(" Y: ", FIXFLOAT(ypos));
      SERIAL_PROTOCOLLNPAIR(" Z: ", FIXFLOAT(measured_z));
    }
    clean_up_after_endstop_or_probe_move();
    report_current_position();
  }
  #if ENABLED(Z_PROBE_SLED)
    inline void gcode_G31() { DEPLOY_PROBE(); }
    inline void gcode_G32() { STOW_PROBE(); }
  #endif 
#endif 
#if PROBE_SELECTED
  #if ENABLED(DELTA_AUTO_CALIBRATION)
    void print_signed_float(const char * const prefix, const float &f) {
      SERIAL_PROTOCOLPGM("  ");
      serialprintPGM(prefix);
      SERIAL_PROTOCOLCHAR(':');
      if (f >= 0) SERIAL_CHAR('+');
      SERIAL_PROTOCOL_F(f, 2);
    }
    inline void gcode_G33() {
      const int8_t probe_points = parser.intval('P', DELTA_CALIBRATION_DEFAULT_POINTS);
      if (!WITHIN(probe_points, 1, 7)) {
        SERIAL_PROTOCOLLNPGM("?(P)oints is implausible (1 to 7).");
        return;
      }
      const int8_t verbose_level = parser.byteval('V', 1);
      if (!WITHIN(verbose_level, 0, 2)) {
        SERIAL_PROTOCOLLNPGM("?(V)erbose level is implausible (0-2).");
        return;
      }
      const float calibration_precision = parser.floatval('C');
      if (calibration_precision < 0) {
        SERIAL_PROTOCOLLNPGM("?(C)alibration precision is implausible (>0).");
        return;
      }
      const int8_t force_iterations = parser.intval('F', 0);
      if (!WITHIN(force_iterations, 0, 30)) {
        SERIAL_PROTOCOLLNPGM("?(F)orce iteration is implausible (0-30).");
        return;
      }
      const bool towers_set           = parser.boolval('T', true),
                 stow_after_each      = parser.boolval('E'),
                 _1p_calibration      = probe_points == 1,
                 _4p_calibration      = probe_points == 2,
                 _4p_towers_points    = _4p_calibration && towers_set,
                 _4p_opposite_points  = _4p_calibration && !towers_set,
                 _7p_calibration      = probe_points >= 3,
                 _7p_half_circle      = probe_points == 3,
                 _7p_double_circle    = probe_points == 5,
                 _7p_triple_circle    = probe_points == 6,
                 _7p_quadruple_circle = probe_points == 7,
                 _7p_multi_circle     = _7p_double_circle || _7p_triple_circle || _7p_quadruple_circle,
                 _7p_intermed_points  = _7p_calibration && !_7p_half_circle;
      const static char save_message[] PROGMEM = "Save with M500 and/or copy to Configuration.h";
      const float dx = (X_PROBE_OFFSET_FROM_EXTRUDER),
                  dy = (Y_PROBE_OFFSET_FROM_EXTRUDER);
      int8_t iterations = 0;
      float test_precision,
            zero_std_dev = (verbose_level ? 999.0 : 0.0), 
            zero_std_dev_old = zero_std_dev,
            zero_std_dev_min = zero_std_dev,
            e_old[XYZ] = {
              endstop_adj[A_AXIS],
              endstop_adj[B_AXIS],
              endstop_adj[C_AXIS]
            },
            dr_old = delta_radius,
            zh_old = home_offset[Z_AXIS],
            alpha_old = delta_tower_angle_trim[A_AXIS],
            beta_old = delta_tower_angle_trim[B_AXIS];
      if (!_1p_calibration) {  
        const float circles = (_7p_quadruple_circle ? 1.5 :
                               _7p_triple_circle    ? 1.0 :
                               _7p_double_circle    ? 0.5 : 0),
                    r = (1 + circles * 0.1) * delta_calibration_radius;
        for (uint8_t axis = 1; axis < 13; ++axis) {
          const float a = RADIANS(180 + 30 * axis);
          if (!position_is_reachable_xy(cos(a) * r, sin(a) * r)) {
            SERIAL_PROTOCOLLNPGM("?(M665 B)ed radius is implausible.");
            return;
          }
        }
      }
      SERIAL_PROTOCOLLNPGM("G33 Auto Calibrate");
      stepper.synchronize();
      #if HAS_LEVELING
        reset_bed_level(); 
      #endif
      #if HOTENDS > 1
        const uint8_t old_tool_index = active_extruder;
        tool_change(0, 0, true);
      #endif
      setup_for_endstop_or_probe_move();
      DEPLOY_PROBE();
      endstops.enable(true);
      home_delta();
      endstops.not_homing();
      SERIAL_PROTOCOLPGM("Checking... AC");
      if (verbose_level == 0) SERIAL_PROTOCOLPGM(" (DRY-RUN)");
      SERIAL_EOL();
      LCD_MESSAGEPGM("Checking... AC"); 
      SERIAL_PROTOCOLPAIR(".Height:", DELTA_HEIGHT + home_offset[Z_AXIS]);
      if (!_1p_calibration) {
        print_signed_float(PSTR("  Ex"), endstop_adj[A_AXIS]);
        print_signed_float(PSTR("Ey"), endstop_adj[B_AXIS]);
        print_signed_float(PSTR("Ez"), endstop_adj[C_AXIS]);
        SERIAL_PROTOCOLPAIR("    Radius:", delta_radius);
      }
      SERIAL_EOL();
      if (_7p_calibration && towers_set) {
        SERIAL_PROTOCOLPGM(".Tower angle :  ");
        print_signed_float(PSTR("Tx"), delta_tower_angle_trim[A_AXIS]);
        print_signed_float(PSTR("Ty"), delta_tower_angle_trim[B_AXIS]);
        SERIAL_PROTOCOLPGM("  Tz:+0.00");
        SERIAL_EOL();
      }
      #if DISABLED(PROBE_MANUALLY)
        home_offset[Z_AXIS] -= probe_pt(dx, dy, stow_after_each, 1, false); 
      #endif
      do {
        float z_at_pt[13] = { 0.0 };
        test_precision = zero_std_dev_old != 999.0 ? (zero_std_dev + zero_std_dev_old) / 2 : zero_std_dev;
        iterations++;
        if (!_7p_half_circle && !_7p_triple_circle) { 
          #if ENABLED(PROBE_MANUALLY)
            z_at_pt[0] += lcd_probe_pt(0, 0);
          #else
            z_at_pt[0] += probe_pt(dx, dy, stow_after_each, 1, false);
          #endif
        }
        if (_7p_calibration) { 
          for (int8_t axis = _7p_multi_circle ? 11 : 9; axis > 0; axis -= _7p_multi_circle ? 2 : 4) {
            const float a = RADIANS(180 + 30 * axis), r = delta_calibration_radius * 0.1;
            #if ENABLED(PROBE_MANUALLY)
              z_at_pt[0] += lcd_probe_pt(cos(a) * r, sin(a) * r);
            #else
              z_at_pt[0] += probe_pt(cos(a) * r + dx, sin(a) * r + dy, stow_after_each, 1, false);
            #endif
          }
          z_at_pt[0] /= float(_7p_double_circle ? 7 : probe_points);
        }
        if (!_1p_calibration) {  
          bool zig_zag = true;
          const uint8_t start = _4p_opposite_points ? 3 : 1,
                         step = _4p_calibration ? 4 : _7p_half_circle ? 2 : 1;
          for (uint8_t axis = start; axis < 13; axis += step) {
            const float zigadd = (zig_zag ? 0.5 : 0.0),
                        offset_circles = _7p_quadruple_circle ? zigadd + 1.0 :
                                         _7p_triple_circle    ? zigadd + 0.5 :
                                         _7p_double_circle    ? zigadd : 0;
            for (float circles = -offset_circles ; circles <= offset_circles; circles++) {
              const float a = RADIANS(180 + 30 * axis),
                          r = delta_calibration_radius * (1 + circles * (zig_zag ? 0.1 : -0.1));
              #if ENABLED(PROBE_MANUALLY)
                z_at_pt[axis] += lcd_probe_pt(cos(a) * r, sin(a) * r);
              #else
                z_at_pt[axis] += probe_pt(cos(a) * r + dx, sin(a) * r + dy, stow_after_each, 1, false);
              #endif
            }
            zig_zag = !zig_zag;
            z_at_pt[axis] /= (2 * offset_circles + 1);
          }
        }
        if (_7p_intermed_points) 
          for (uint8_t axis = 1; axis < 13; axis += 2)
            z_at_pt[axis] = (z_at_pt[axis] + (z_at_pt[axis + 1] + z_at_pt[(axis + 10) % 12 + 1]) / 2.0) / 2.0;
        float S1 = z_at_pt[0],
              S2 = sq(z_at_pt[0]);
        int16_t N = 1;
        if (!_1p_calibration) 
          for (uint8_t axis = (_4p_opposite_points ? 3 : 1); axis < 13; axis += (_4p_calibration ? 4 : 2)) {
            S1 += z_at_pt[axis];
            S2 += sq(z_at_pt[axis]);
            N++;
          }
        zero_std_dev_old = zero_std_dev;
        NOMORE(zero_std_dev_min, zero_std_dev);
        zero_std_dev = round(sqrt(S2 / N) * 1000.0) / 1000.0 + 0.00001;
        if ((zero_std_dev < test_precision && zero_std_dev > calibration_precision) || iterations <= force_iterations) {
          if (zero_std_dev < zero_std_dev_min) {
            COPY(e_old, endstop_adj);
            dr_old = delta_radius;
            zh_old = home_offset[Z_AXIS];
            alpha_old = delta_tower_angle_trim[A_AXIS];
            beta_old = delta_tower_angle_trim[B_AXIS];
          }
          float e_delta[XYZ] = { 0.0 }, r_delta = 0.0, t_alpha = 0.0, t_beta = 0.0;
          const float r_diff = delta_radius - delta_calibration_radius,
                      h_factor = 1.00 + r_diff * 0.001,                          
                      r_factor = -(1.75 + 0.005 * r_diff + 0.001 * sq(r_diff)),  
                      a_factor = 100.0 / delta_calibration_radius;               
          #define ZP(N,I) ((N) * z_at_pt[I])
          #define Z1000(I) ZP(1.00, I)
          #define Z1050(I) ZP(h_factor, I)
          #define Z0700(I) ZP(h_factor * 2.0 / 3.00, I)
          #define Z0350(I) ZP(h_factor / 3.00, I)
          #define Z0175(I) ZP(h_factor / 6.00, I)
          #define Z2250(I) ZP(r_factor, I)
          #define Z0750(I) ZP(r_factor / 3.00, I)
          #define Z0375(I) ZP(r_factor / 6.00, I)
          #define Z0444(I) ZP(a_factor * 4.0 / 9.0, I)
          #define Z0888(I) ZP(a_factor * 8.0 / 9.0, I)
          #if ENABLED(PROBE_MANUALLY)
            test_precision = 0.00; 
          #endif
          switch (probe_points) {
            case 1:
              test_precision = 0.00; 
              LOOP_XYZ(i) e_delta[i] = Z1000(0);
              break;
            case 2:
              if (towers_set) {
                e_delta[X_AXIS] = Z1050(0) + Z0700(1) - Z0350(5) - Z0350(9);
                e_delta[Y_AXIS] = Z1050(0) - Z0350(1) + Z0700(5) - Z0350(9);
                e_delta[Z_AXIS] = Z1050(0) - Z0350(1) - Z0350(5) + Z0700(9);
                r_delta         = Z2250(0) - Z0750(1) - Z0750(5) - Z0750(9);
              }
              else {
                e_delta[X_AXIS] = Z1050(0) - Z0700(7) + Z0350(11) + Z0350(3);
                e_delta[Y_AXIS] = Z1050(0) + Z0350(7) - Z0700(11) + Z0350(3);
                e_delta[Z_AXIS] = Z1050(0) + Z0350(7) + Z0350(11) - Z0700(3);
                r_delta         = Z2250(0) - Z0750(7) - Z0750(11) - Z0750(3);
              }
              break;
            default:
              e_delta[X_AXIS] = Z1050(0) + Z0350(1) - Z0175(5) - Z0175(9) - Z0350(7) + Z0175(11) + Z0175(3);
              e_delta[Y_AXIS] = Z1050(0) - Z0175(1) + Z0350(5) - Z0175(9) + Z0175(7) - Z0350(11) + Z0175(3);
              e_delta[Z_AXIS] = Z1050(0) - Z0175(1) - Z0175(5) + Z0350(9) + Z0175(7) + Z0175(11) - Z0350(3);
              r_delta         = Z2250(0) - Z0375(1) - Z0375(5) - Z0375(9) - Z0375(7) - Z0375(11) - Z0375(3);
              if (towers_set) {
                t_alpha = Z0444(1) - Z0888(5) + Z0444(9) + Z0444(7) - Z0888(11) + Z0444(3);
                t_beta  = Z0888(1) - Z0444(5) - Z0444(9) + Z0888(7) - Z0444(11) - Z0444(3);
              }
              break;
          }
          LOOP_XYZ(axis) endstop_adj[axis] += e_delta[axis];
          delta_radius += r_delta;
          delta_tower_angle_trim[A_AXIS] += t_alpha;
          delta_tower_angle_trim[B_AXIS] += t_beta;
          const float z_temp = MAX3(endstop_adj[A_AXIS], endstop_adj[B_AXIS], endstop_adj[C_AXIS]);
          home_offset[Z_AXIS] -= z_temp;
          LOOP_XYZ(i) endstop_adj[i] -= z_temp;
          recalc_delta_settings(delta_radius, delta_diagonal_rod);
        }
        else if (zero_std_dev >= test_precision) {   
          COPY(endstop_adj, e_old);
          delta_radius = dr_old;
          home_offset[Z_AXIS] = zh_old;
          delta_tower_angle_trim[A_AXIS] = alpha_old;
          delta_tower_angle_trim[B_AXIS] = beta_old;
          recalc_delta_settings(delta_radius, delta_diagonal_rod);
        }
        if (verbose_level != 1) {
          SERIAL_PROTOCOLPGM(".    ");
          print_signed_float(PSTR("c"), z_at_pt[0]);
          if (_4p_towers_points || _7p_calibration) {
            print_signed_float(PSTR("   x"), z_at_pt[1]);
            print_signed_float(PSTR(" y"), z_at_pt[5]);
            print_signed_float(PSTR(" z"), z_at_pt[9]);
          }
          if (!_4p_opposite_points) SERIAL_EOL();
          if ((_4p_opposite_points) || _7p_calibration) {
            if (_7p_calibration) {
              SERIAL_CHAR('.');
              SERIAL_PROTOCOL_SP(13);
            }
            print_signed_float(PSTR("  yz"), z_at_pt[7]);
            print_signed_float(PSTR("zx"), z_at_pt[11]);
            print_signed_float(PSTR("xy"), z_at_pt[3]);
            SERIAL_EOL();
          }
        }
        if (verbose_level != 0) {                                    
          if ((zero_std_dev >= test_precision || zero_std_dev <= calibration_precision) && iterations > force_iterations) {  
            SERIAL_PROTOCOLPGM("Calibration OK");
            SERIAL_PROTOCOL_SP(36);
            #if DISABLED(PROBE_MANUALLY)
              if (zero_std_dev >= test_precision && !_1p_calibration)
                SERIAL_PROTOCOLPGM("rolling back.");
              else
            #endif
              {
                SERIAL_PROTOCOLPGM("std dev:");
                SERIAL_PROTOCOL_F(zero_std_dev, 3);
              }
            SERIAL_EOL();
            LCD_MESSAGEPGM("Calibration OK"); 
          }
          else {                                                     
            char mess[15] = "No convergence";
            if (iterations < 31)
              sprintf_P(mess, PSTR("Iteration : %02i"), (int)iterations);
            SERIAL_PROTOCOL(mess);
            SERIAL_PROTOCOL_SP(36);
            SERIAL_PROTOCOLPGM("std dev:");
            SERIAL_PROTOCOL_F(zero_std_dev, 3);
            SERIAL_EOL();
            lcd_setstatus(mess);
          }
          SERIAL_PROTOCOLPAIR(".Height:", DELTA_HEIGHT + home_offset[Z_AXIS]);
          if (!_1p_calibration) {
            print_signed_float(PSTR("  Ex"), endstop_adj[A_AXIS]);
            print_signed_float(PSTR("Ey"), endstop_adj[B_AXIS]);
            print_signed_float(PSTR("Ez"), endstop_adj[C_AXIS]);
            SERIAL_PROTOCOLPAIR("    Radius:", delta_radius);
          }
          SERIAL_EOL();
          if (_7p_calibration && towers_set) {
            SERIAL_PROTOCOLPGM(".Tower angle :  ");
            print_signed_float(PSTR("Tx"), delta_tower_angle_trim[A_AXIS]);
            print_signed_float(PSTR("Ty"), delta_tower_angle_trim[B_AXIS]);
            SERIAL_PROTOCOLPGM("  Tz:+0.00");
            SERIAL_EOL();
          }
          if ((zero_std_dev >= test_precision || zero_std_dev <= calibration_precision) && iterations > force_iterations)
            serialprintPGM(save_message);
            SERIAL_EOL();
        }
        else {                                                       
          SERIAL_PROTOCOLPGM("End DRY-RUN");
          SERIAL_PROTOCOL_SP(39);
          SERIAL_PROTOCOLPGM("std dev:");
          SERIAL_PROTOCOL_F(zero_std_dev, 3);
          SERIAL_EOL();
        }
        endstops.enable(true);
        home_delta();
        endstops.not_homing();
      }
      while ((zero_std_dev < test_precision && zero_std_dev > calibration_precision && iterations < 31) || iterations <= force_iterations);
      #if ENABLED(DELTA_HOME_TO_SAFE_ZONE)
        do_blocking_move_to_z(delta_clip_start_height);
      #endif
      STOW_PROBE();
      clean_up_after_endstop_or_probe_move();
      #if HOTENDS > 1
        tool_change(old_tool_index, 0, true);
      #endif
    }
  #endif 
#endif 
#if ENABLED(G38_PROBE_TARGET)
  static bool G38_run_probe() {
    bool G38_pass_fail = false;
    float retract_mm[XYZ];
    LOOP_XYZ(i) {
      float dist = destination[i] - current_position[i];
      retract_mm[i] = FABS(dist) < G38_MINIMUM_MOVE ? 0 : home_bump_mm((AxisEnum)i) * (dist > 0 ? -1 : 1);
    }
    stepper.synchronize();  
    endstops.enable(true);
    G38_move = true;
    G38_endstop_hit = false;
    prepare_move_to_destination();
    stepper.synchronize();
    G38_move = false;
    endstops.hit_on_purpose();
    set_current_from_steppers_for_axis(ALL_AXES);
    SYNC_PLAN_POSITION_KINEMATIC();
    if (G38_endstop_hit) {
      G38_pass_fail = true;
      #if ENABLED(PROBE_DOUBLE_TOUCH)
        set_destination_to_current();
        LOOP_XYZ(i) destination[i] += retract_mm[i];
        endstops.enable(false);
        prepare_move_to_destination();
        stepper.synchronize();
        feedrate_mm_s /= 4;
        LOOP_XYZ(i) destination[i] -= retract_mm[i] * 2;
        endstops.enable(true);
        G38_move = true;
        prepare_move_to_destination();
        stepper.synchronize();
        G38_move = false;
        set_current_from_steppers_for_axis(ALL_AXES);
        SYNC_PLAN_POSITION_KINEMATIC();
      #endif
    }
    endstops.hit_on_purpose();
    endstops.not_homing();
    return G38_pass_fail;
  }
  inline void gcode_G38(bool is_38_2) {
    gcode_get_destination();
    setup_for_endstop_or_probe_move();
    LOOP_XYZ(i)
      if (FABS(destination[i] - current_position[i]) >= G38_MINIMUM_MOVE) {
        if (!parser.seenval('F')) feedrate_mm_s = homing_feedrate(i);
        if (!G38_run_probe() && is_38_2) {
          SERIAL_ERROR_START();
          SERIAL_ERRORLNPGM("Failed to reach target");
        }
        break;
      }
    clean_up_after_endstop_or_probe_move();
  }
#endif 
#if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(MESH_BED_LEVELING)
  inline void gcode_G42() {
    if (IsRunning()) {
      const bool hasI = parser.seenval('I');
      const int8_t ix = hasI ? parser.value_int() : 0;
      const bool hasJ = parser.seenval('J');
      const int8_t iy = hasJ ? parser.value_int() : 0;
      if ((hasI && !WITHIN(ix, 0, GRID_MAX_POINTS_X - 1)) || (hasJ && !WITHIN(iy, 0, GRID_MAX_POINTS_Y - 1))) {
        SERIAL_ECHOLNPGM(MSG_ERR_MESH_XY);
        return;
      }
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        #define _GET_MESH_X(I) bilinear_start[X_AXIS] + I * bilinear_grid_spacing[X_AXIS]
        #define _GET_MESH_Y(J) bilinear_start[Y_AXIS] + J * bilinear_grid_spacing[Y_AXIS]
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        #define _GET_MESH_X(I) ubl.mesh_index_to_xpos(I)
        #define _GET_MESH_Y(J) ubl.mesh_index_to_ypos(J)
      #elif ENABLED(MESH_BED_LEVELING)
        #define _GET_MESH_X(I) mbl.index_to_xpos[I]
        #define _GET_MESH_Y(J) mbl.index_to_ypos[J]
      #endif
      set_destination_to_current();
      if (hasI) destination[X_AXIS] = LOGICAL_X_POSITION(_GET_MESH_X(ix));
      if (hasJ) destination[Y_AXIS] = LOGICAL_Y_POSITION(_GET_MESH_Y(iy));
      if (parser.boolval('P')) {
        if (hasI) destination[X_AXIS] -= X_PROBE_OFFSET_FROM_EXTRUDER;
        if (hasJ) destination[Y_AXIS] -= Y_PROBE_OFFSET_FROM_EXTRUDER;
      }
      const float fval = parser.linearval('F');
      if (fval > 0.0) feedrate_mm_s = MMM_TO_MMS(fval);
      #if IS_SCARA
        prepare_uninterpolated_move_to_destination();
      #else
        prepare_move_to_destination();
      #endif
    }
  }
#endif 
inline void gcode_G92() {
  bool didXYZ = false,
       didE = parser.seenval('E');
  if (!didE) stepper.synchronize();
  LOOP_XYZE(i) {
    if (parser.seenval(axis_codes[i])) {
      #if IS_SCARA
        current_position[i] = parser.value_axis_units((AxisEnum)i);
        if (i != E_AXIS) didXYZ = true;
      #else
        #if HAS_POSITION_SHIFT
          const float p = current_position[i];
        #endif
        const float v = parser.value_axis_units((AxisEnum)i);
        current_position[i] = v;
        if (i != E_AXIS) {
          didXYZ = true;
          #if HAS_POSITION_SHIFT
            position_shift[i] += v - p; 
            update_software_endstops((AxisEnum)i);
            #if ENABLED(I2C_POSITION_ENCODERS)
              I2CPEM.encoders[I2CPEM.idx_from_axis((AxisEnum)i)].set_axis_offset(position_shift[i]);
            #endif
          #endif
        }
      #endif
    }
  }
  if (didXYZ)
    SYNC_PLAN_POSITION_KINEMATIC();
  else if (didE)
    sync_plan_position_e();
  report_current_position();
}
#if HAS_RESUME_CONTINUE
  inline void gcode_M0_M1() {
    const char * const args = parser.string_arg;
    millis_t ms = 0;
    bool hasP = false, hasS = false;
    if (parser.seenval('P')) {
      ms = parser.value_millis(); 
      hasP = ms > 0;
    }
    if (parser.seenval('S')) {
      ms = parser.value_millis_from_seconds(); 
      hasS = ms > 0;
    }
    #if ENABLED(ULTIPANEL)
      if (!hasP && !hasS && args && *args)
        lcd_setstatus(args, true);
      else {
        LCD_MESSAGEPGM(MSG_USERWAIT);
        #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
          dontExpireStatus();
        #endif
      }
    #else
      if (!hasP && !hasS && args && *args) {
        SERIAL_ECHO_START();
        SERIAL_ECHOLN(args);
      }
    #endif
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    wait_for_user = true;
    stepper.synchronize();
    refresh_cmd_timeout();
    if (ms > 0) {
      ms += previous_cmd_ms;  
      while (PENDING(millis(), ms) && wait_for_user) idle();
    }
    else {
      #if ENABLED(ULTIPANEL)
        if (lcd_detected()) {
          while (wait_for_user) idle();
          IS_SD_PRINTING ? LCD_MESSAGEPGM(MSG_RESUMING) : LCD_MESSAGEPGM(WELCOME_MSG);
        }
      #else
        while (wait_for_user) idle();
      #endif
    }
    wait_for_user = false;
    KEEPALIVE_STATE(IN_HANDLER);
  }
#endif 
#if ENABLED(SPINDLE_LASER_ENABLE)
  inline void delay_for_power_up() {
    refresh_cmd_timeout();
    while (PENDING(millis(), SPINDLE_LASER_POWERUP_DELAY + previous_cmd_ms)) idle();
  }
  inline void delay_for_power_down() {
    refresh_cmd_timeout();
    while (PENDING(millis(), SPINDLE_LASER_POWERDOWN_DELAY + previous_cmd_ms + 1)) idle();
  }
  inline void ocr_val_mode() {
    uint8_t spindle_laser_power = parser.value_byte();
    WRITE(SPINDLE_LASER_ENABLE_PIN, SPINDLE_LASER_ENABLE_INVERT); 
    if (SPINDLE_LASER_PWM_INVERT) spindle_laser_power = 255 - spindle_laser_power;
    analogWrite(SPINDLE_LASER_PWM_PIN, spindle_laser_power);
  }
  inline void gcode_M3_M4(bool is_M3) {
    stepper.synchronize();   
    #if SPINDLE_DIR_CHANGE
      const bool rotation_dir = (is_M3 && !SPINDLE_INVERT_DIR || !is_M3 && SPINDLE_INVERT_DIR) ? HIGH : LOW;
      if (SPINDLE_STOP_ON_DIR_CHANGE \
         && READ(SPINDLE_LASER_ENABLE_PIN) == SPINDLE_LASER_ENABLE_INVERT \
         && READ(SPINDLE_DIR_PIN) != rotation_dir
      ) {
        WRITE(SPINDLE_LASER_ENABLE_PIN, !SPINDLE_LASER_ENABLE_INVERT);  
        delay_for_power_down();
      }
      digitalWrite(SPINDLE_DIR_PIN, rotation_dir);
    #endif
    #if ENABLED(SPINDLE_LASER_PWM)
      if (parser.seen('O')) ocr_val_mode();
      else {
        const float spindle_laser_power = parser.floatval('S');
        if (spindle_laser_power == 0) {
          WRITE(SPINDLE_LASER_ENABLE_PIN, !SPINDLE_LASER_ENABLE_INVERT);                                    
          delay_for_power_down();
        }
        else {
          int16_t ocr_val = (spindle_laser_power - (SPEED_POWER_INTERCEPT)) * (1.0 / (SPEED_POWER_SLOPE));  
          NOMORE(ocr_val, 255);                                                                             
          if (spindle_laser_power <= SPEED_POWER_MIN)
            ocr_val = (SPEED_POWER_MIN - (SPEED_POWER_INTERCEPT)) * (1.0 / (SPEED_POWER_SLOPE));            
          if (spindle_laser_power >= SPEED_POWER_MAX)
            ocr_val = (SPEED_POWER_MAX - (SPEED_POWER_INTERCEPT)) * (1.0 / (SPEED_POWER_SLOPE));            
          if (SPINDLE_LASER_PWM_INVERT) ocr_val = 255 - ocr_val;
          WRITE(SPINDLE_LASER_ENABLE_PIN, SPINDLE_LASER_ENABLE_INVERT);                                     
          analogWrite(SPINDLE_LASER_PWM_PIN, ocr_val & 0xFF);                                               
          delay_for_power_up();
        }
      }
    #else
      WRITE(SPINDLE_LASER_ENABLE_PIN, SPINDLE_LASER_ENABLE_INVERT); 
      delay_for_power_up();
    #endif
  }
  inline void gcode_M5() {
    stepper.synchronize();
    WRITE(SPINDLE_LASER_ENABLE_PIN, !SPINDLE_LASER_ENABLE_INVERT);
    delay_for_power_down();
  }
#endif 
inline void gcode_M17() {
  LCD_MESSAGEPGM(MSG_NO_MOVE);
  enable_all_steppers();
}
#if IS_KINEMATIC
  #define RUNPLAN(RATE_MM_S) planner.buffer_line_kinematic(destination, RATE_MM_S, active_extruder)
#else
  #define RUNPLAN(RATE_MM_S) line_to_destination(RATE_MM_S)
#endif
#if ENABLED(FYS_HOME_FUNCTION)
void FunV007()
{
    stepper.synchronize();
    setup_for_endstop_or_probe_move();
    endstops.enable(true); 
    HOMEAXIS(X);
    HOMEAXIS(Y);
    endstops.not_homing();
    clean_up_after_endstop_or_probe_move();
}
void homeZ()
{
    stepper.synchronize();
    setup_for_endstop_or_probe_move();
    HOMEAXIS(Z);
    clean_up_after_endstop_or_probe_move();
}
#endif
#if ENABLED(ADVANCED_PAUSE_FEATURE)
static float resume_position[XYZE];
static bool move_away_flag = false;
#if ENABLED(SDSUPPORT)
static bool sd_print_paused = false;
#endif
  static void filament_change_beep(const int8_t max_beep_count, const bool init=false) {
    static millis_t next_buzz = 0;
    static int8_t runout_beep = 0;
    if (init) next_buzz = runout_beep = 0;
    const millis_t ms = millis();
    if (ELAPSED(ms, next_buzz)) {
      if (max_beep_count < 0 || runout_beep < max_beep_count + 5) { 
        next_buzz = ms + ((max_beep_count < 0 || runout_beep < max_beep_count) ? 2500 : 400);
        BUZZ(300, 2000);
        runout_beep++;
      }
    }
  }
  #ifdef FYS_PRINT_BREAK_HEATUP_FIRST
  static void ensure_safe_temperature_print_break() {
    bool heaters_heating = true;
    wait_for_heatup = true;    
    while (wait_for_heatup && heaters_heating) {
      idle();
      heaters_heating = false;
      HOTEND_LOOP() {
        if(thermalManager.degTargetHotend(e) < EXTRUDE_MINTEMP) thermalManager.setTargetHotend(EXTRUDE_MINTEMP,e);
        else thermalManager.setTargetHotend(thermalManager.degTargetHotend(e),e);
        if (thermalManager.degTargetHotend(e) && abs(thermalManager.degHotend(e) - thermalManager.degTargetHotend(e)) > 1
            && thermalManager.degHotend(e) < EXTRUDE_MINTEMP) {
          heaters_heating = true;
          #if ENABLED(ULTIPANEL)
            lcd_advanced_pause_show_message(ADVANCED_PRINT_BREAK_MESSAGE_HEAT_NOZZLE);
          #elif defined(FYSTLCD_V1)
          dwin_popup(PSTR("     Heating nozzle\n     Please wait..."), 1);
          SERIAL_ERRORLNPGM("Print break heat up first!");
          #endif
          break;
        }
        else 
        {
          #if ENABLED(ULTIPANEL)
            lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_STATUS);
          #elif defined(FYSTLCD_V1)
            lcd_setPage(FTPAGE(PRINT));
          #endif
          SERIAL_ERRORLNPGM("Print break return to print page!");
        }
      }
    }
    wait_for_heatup = false; 
  }
  #endif
  static void ensure_safe_temperature() {
    bool heaters_heating = true;
    wait_for_heatup = true;    
    while (wait_for_heatup && heaters_heating) {
      idle();
      heaters_heating = false;
      HOTEND_LOOP() {
        if(thermalManager.degTargetHotend(e) < EXTRUDE_MINTEMP) thermalManager.setTargetHotend(210,e);
        else thermalManager.setTargetHotend(thermalManager.degTargetHotend(e),e);
        if (thermalManager.degTargetHotend(e) && abs(thermalManager.degHotend(e) - thermalManager.degTargetHotend(e)) > TEMP_HYSTERESIS) {
          heaters_heating = true;
          #if ENABLED(ULTIPANEL)
            lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT);
          #elif defined(FYSTLCD_V1)
            dwin_popup(PSTR("     Filament runout1!\n     Heating nozzle\n     Please wait..."), 1);          
          #endif
          SERIAL_ERRORLNPGM("Filament runout1!");
          break;
        }
      }
    }
  }
  static bool pause_print(const float &retract, const float &z_lift, const float &x_pos, const float &y_pos,
                          const float &unload_length = 0 , const int8_t max_beep_count = 0, const bool show_lcd = false
  ) {
    if (move_away_flag) return false; 
    if (!DEBUGGING(DRYRUN) && (unload_length != 0 || retract != 0)) {
      #if ENABLED(PREVENT_COLD_EXTRUSION)
        if (!thermalManager.allow_cold_extrude &&
            thermalManager.degTargetHotend(active_extruder) < thermalManager.extrude_min_temp) {
          SERIAL_ERROR_START();
          SERIAL_ERRORLNPGM(MSG_TOO_COLD_FOR_M600);
          return false;
        }
      #endif
      ensure_safe_temperature(); 
    }
    move_away_flag = true;
    #if ENABLED(SDSUPPORT)
      if (card.sdprinting) {
        card.pauseSDPrint();
        sd_print_paused = true;
      }
    #endif
    print_job_timer.pause();
    if (show_lcd) {
      #if ENABLED(ULTIPANEL)
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INIT);
      #elif defined(FYSTLCD_V1)
      dwin_popup(PSTR("       Filament runout2!\nWait for start of filament change."),1);
      #endif
			SERIAL_ERRORLNPGM("Filament runout2!");
    }
    stepper.synchronize();
    COPY(resume_position, current_position);
    #if ENABLED(FYS_MIX_FIL_RANOUT_SAFE_PRINT_BREAK)
    #if ENABLED(FILAMENT_RUNOUT_SENSOR)
      #if defined(FYS_SAFE_PRINT_BREAK)
      if(filament_ran_out) 
      {
        cli();
        recordEnvironmentPure();
        sei();
        filament_ran_out_still_printing = false;
      }
      #endif
    #endif
    #endif
    if (retract) {
      set_destination_to_current();
      destination[E_AXIS] += retract;
      RUNPLAN(PAUSE_PARK_RETRACT_FEEDRATE);
      stepper.synchronize();
    }
    if (z_lift > 0)
      do_blocking_move_to_z(current_position[Z_AXIS] + z_lift, PAUSE_PARK_Z_FEEDRATE);
    do_blocking_move_to_xy(x_pos, y_pos, PAUSE_PARK_XY_FEEDRATE);
    if (unload_length != 0) {
      if (show_lcd) {
        #if ENABLED(ULTIPANEL)
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_UNLOAD);
          idle();
        #elif defined(FYSTLCD_V1)
          dwin_popup(PSTR("       Filament runout3!\nWait for filament unload."),1);
          idle();
        #endif
				SERIAL_ERRORLNPGM("Filament runout3!");
      }
      set_destination_to_current();
      destination[E_AXIS] += unload_length;
      RUNPLAN(FILAMENT_CHANGE_UNLOAD_FEEDRATE);
      stepper.synchronize();
    }
    if (show_lcd) {
      #if ENABLED(ULTIPANEL)
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
      #elif defined(FYSTLCD_V1)
        dwin_popup(PSTR("       Filament runout4!\nInsert filament and press button to continue.\n Waiting for changing filament."));
      #endif
    }
    #if HAS_BUZZER
      filament_change_beep(max_beep_count, true);
    #endif
      SERIAL_ECHOLNPGM("EXE LCD INSERT FILAMENT1");
    idle();
    SERIAL_ECHOLNPGM("EXE LCD INSERT FILAMENT2");
    #if E0_ENABLE_PIN != X_ENABLE_PIN && E1_ENABLE_PIN != Y_ENABLE_PIN
      disable_e_steppers();
      safe_delay(100);
    #endif
    const millis_t nozzle_timeout = (millis_t)(PAUSE_PARK_NOZZLE_TIMEOUT) * 1000UL;
    HOTEND_LOOP()
      thermalManager.start_heater_idle_timer(e, nozzle_timeout);
   SERIAL_ECHOLNPGM("PAUSE PRINT DONE");
    return true;
  }
  #if ENABLED(FYS_MIX_FIL_RANOUT_SAFE_PRINT_BREAK)
  static bool FunV05D() {
      #if ENABLED(ULTIPANEL)
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
      #elif defined(FYSTLCD_V1)
        dwin_popup(PSTR("       Filament runout4!\nInsert filament and press button to continue.\n Waiting for changing filament."));
      #endif
	    SERIAL_ERRORLNPGM("Filament runout4!");
      SERIAL_ECHOLNPGM("EXE LCD INSERT FILAMENT1");
    idle();
    SERIAL_ECHOLNPGM("EXE LCD INSERT FILAMENT2");
    #if E0_ENABLE_PIN != X_ENABLE_PIN && E1_ENABLE_PIN != Y_ENABLE_PIN
      disable_e_steppers();
      safe_delay(100);
    #endif
    const millis_t nozzle_timeout = (millis_t)(PAUSE_PARK_NOZZLE_TIMEOUT) * 1000UL;
    HOTEND_LOOP()
      thermalManager.start_heater_idle_timer(e, nozzle_timeout);
    SERIAL_ECHOLNPGM("PAUSE FILAMENT DONE");
    return true;
  }
  #endif
  static void wait_for_filament_reload(const int8_t max_beep_count = 0) {
    bool nozzle_timed_out = false;
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    wait_for_user = true;    
    SERIAL_ECHOLNPGM("EXE WAIT FOR FILAMENT RELOAD");
    while (wait_for_user) { 
      #if HAS_BUZZER
        filament_change_beep(max_beep_count);
      #endif
      if (!nozzle_timed_out)
        HOTEND_LOOP()
          nozzle_timed_out |= thermalManager.is_heater_idle(e);
      if (nozzle_timed_out) {
        #if ENABLED(ULTIPANEL)
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_CLICK_TO_HEAT_NOZZLE);
        #elif defined(FYSTLCD_V1)
          dwin_popup(PSTR("       Filament runout5!\nThe nozzle has cool down,press button to reheat the nozzle."));
        #endif
	      SERIAL_ERRORLNPGM("Filament runout5!");
        while (wait_for_user) idle(true);
        HOTEND_LOOP() thermalManager.reset_heater_idle_timer(e);
        ensure_safe_temperature();
        #if ENABLED(ULTIPANEL)
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
        #elif defined(FYSTLCD_V1)
        dwin_popup(PSTR("       Filament runout6!\nInsert filament and press button to continue.\n Waiting for changing filament."));
        #endif
				SERIAL_ERRORLNPGM("Filament runout6!");
        const millis_t nozzle_timeout = (millis_t)(PAUSE_PARK_NOZZLE_TIMEOUT) * 1000UL;
        HOTEND_LOOP()
          thermalManager.start_heater_idle_timer(e, nozzle_timeout);
        wait_for_user = true; 
        nozzle_timed_out = false;
        #if HAS_BUZZER
          filament_change_beep(max_beep_count, true);
        #endif
      }
      idle(true);
    }
    KEEPALIVE_STATE(IN_HANDLER);
    SERIAL_ECHOLNPGM("WAIT FOR RELOAD DONE");
  }
  static void resume_print(const float &load_length = 0, const float &initial_extrude_length = 0, const int8_t max_beep_count = 0) {
    bool nozzle_timed_out = false;
    SERIAL_ECHOLNPGM("EXE RESUME PRINT");
    if (!move_away_flag) return;
    HOTEND_LOOP() {
      nozzle_timed_out |= thermalManager.is_heater_idle(e);
      thermalManager.reset_heater_idle_timer(e);
    }
    if (nozzle_timed_out) ensure_safe_temperature();
    #if HAS_BUZZER
      filament_change_beep(max_beep_count, true);
    #endif
    set_destination_to_current();
    if (load_length != 0) {
      #if ENABLED(ULTIPANEL)
        if (nozzle_timed_out)
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
       #elif defined(FYSTLCD_V1)
        if (nozzle_timed_out)
            dwin_popup(PSTR("       Filament runout7!\nInsert filament and press button to continue.\n Waiting for changing filament."));
      #endif
			SERIAL_ERRORLNPGM("Filament runout7!");
      KEEPALIVE_STATE(PAUSED_FOR_USER);
      wait_for_user = true;    
      while (wait_for_user && nozzle_timed_out) {
        #if HAS_BUZZER
          filament_change_beep(max_beep_count);
        #endif
        idle(true);
      }
      KEEPALIVE_STATE(IN_HANDLER);
      #if ENABLED(ULTIPANEL)
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_LOAD);
      #elif defined(FYSTLCD_V1)
      dwin_popup(PSTR("       Filament runout8!\nWait for filament load."), 1);
      #endif
      SERIAL_ERRORLNPGM("Filament runout8!");
      destination[E_AXIS] += load_length;
      RUNPLAN(FILAMENT_CHANGE_LOAD_FEEDRATE);
      stepper.synchronize();
    }
    #if  ADVANCED_PAUSE_EXTRUDE_LENGTH > 0
      float extrude_length = initial_extrude_length;
        SERIAL_ECHOLNPGM("EXE RESUME WAIT WAIT WAIT");
      do {
        if (extrude_length > 0) {
          #if ENABLED(ULTIPANEL)
            lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_EXTRUDE);
          #elif defined(FYSTLCD_V1)
          dwin_popup(PSTR("       Filament runout9!\nWait for filament extrude."), 1);
          #endif
          SERIAL_ERRORLNPGM("Filament runout9!");
          destination[E_AXIS] += extrude_length;
          RUNPLAN(ADVANCED_PAUSE_EXTRUDE_FEEDRATE);
          stepper.synchronize();
        }
        KEEPALIVE_STATE(PAUSED_FOR_USER);
        wait_for_user = false;
        #if ENABLED(ULTIPANEL)
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_OPTION);
        #elif defined(FYSTLCD_V1)
        dwin_popup(PSTR(MSG_FILAMENT_CHANGE_OPTION_HEADER), 2, 1);
        #endif
					SERIAL_ERRORLNPGM("Filament runout10:resume option!");
        while (advanced_pause_menu_response == ADVANCED_PAUSE_RESPONSE_WAIT_FOR) idle(true);
        KEEPALIVE_STATE(IN_HANDLER);
        extrude_length = ADVANCED_PAUSE_EXTRUDE_LENGTH;
      } while (advanced_pause_menu_response == ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE);
    #endif
    #if ENABLED(ULTIPANEL)
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_RESUME);
    #elif defined(FYSTLCD_V1)
      dwin_popup(PSTR("   Filament runout11!\nWait for print to resume."),1);
    #endif
		SERIAL_ERRORLNPGM("Filament runout11!");
    destination[E_AXIS] = current_position[E_AXIS] = resume_position[E_AXIS];
    planner.set_e_position_mm(current_position[E_AXIS]);
    do_blocking_move_to_xy(resume_position[X_AXIS], resume_position[Y_AXIS], PAUSE_PARK_XY_FEEDRATE);
    do_blocking_move_to_z(resume_position[Z_AXIS], PAUSE_PARK_Z_FEEDRATE);
    #if ENABLED(FILAMENT_RUNOUT_SENSOR)
      filament_ran_out = false;
    #endif
    #if ENABLED(ULTIPANEL)
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_STATUS);
    #elif defined(FYSTLCD_V1)
      FunV052();
    #endif
		SERIAL_ERRORLNPGM("Filament runout12:resume done!");
    #if ENABLED(SDSUPPORT)
      if (sd_print_paused) {
        card.startFileprint();
        sd_print_paused = false;
      }
    #endif
    move_away_flag = false;
  }
  #if ENABLED(FYS_MIX_FIL_RANOUT_SAFE_PRINT_BREAK)
  static void FunV05E(const float &load_length = 0, const float &initial_extrude_length = 0, const int8_t max_beep_count = 0) {
    bool nozzle_timed_out = false;
    SERIAL_ECHOLNPGM("EXE RESUME PRINT RELOAD FILAMENT");
    HOTEND_LOOP() {
      nozzle_timed_out |= thermalManager.is_heater_idle(e);
      thermalManager.reset_heater_idle_timer(e);
    }
    if (nozzle_timed_out) ensure_safe_temperature();
    #if HAS_BUZZER
      filament_change_beep(max_beep_count, true);
    #endif
    set_destination_to_current();
    if (load_length != 0) {
      #if ENABLED(ULTIPANEL)
        if (nozzle_timed_out)
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
       #elif defined(FYSTLCD_V1)
        if (nozzle_timed_out)
            dwin_popup(PSTR("       Filament runout7!\nInsert filament and press button to continue.\n Waiting for changing filament."));
      #endif
			SERIAL_ERRORLNPGM("Filament runout7!");
      KEEPALIVE_STATE(PAUSED_FOR_USER);
      wait_for_user = true;    
      while (wait_for_user && nozzle_timed_out) {
        #if HAS_BUZZER
          filament_change_beep(max_beep_count);
        #endif
        idle(true);
      }
      KEEPALIVE_STATE(IN_HANDLER);
      #if ENABLED(ULTIPANEL)
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_LOAD);
      #elif defined(FYSTLCD_V1)
      dwin_popup(PSTR("       Filament runout8!\nWait for filament load."), 1);
      #endif
			SERIAL_ERRORLNPGM("Filament runout8!");
      destination[E_AXIS] += load_length;
      RUNPLAN(FILAMENT_CHANGE_LOAD_FEEDRATE);
      stepper.synchronize();
    }
    #if  ADVANCED_PAUSE_EXTRUDE_LENGTH > 0
      float extrude_length = initial_extrude_length;
        SERIAL_ECHOLNPGM("EXE RESUME RELOAD FILAMENT WAIT WAIT WAIT");
      do {
          if (extrude_length > 0) {
          #if ENABLED(ULTIPANEL)
            lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_EXTRUDE);
          #elif defined(FYSTLCD_V1)
          dwin_popup(PSTR("       Filament runout9!\nWait for filament extrude."), 1);
          #endif
  				SERIAL_ERRORLNPGM("Filament runout9!");
          destination[E_AXIS] += extrude_length;
          RUNPLAN(ADVANCED_PAUSE_EXTRUDE_FEEDRATE);
          stepper.synchronize();
        }
        KEEPALIVE_STATE(PAUSED_FOR_USER);
        wait_for_user = false;
        #if ENABLED(ULTIPANEL)
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_OPTION);
        #elif defined(FYSTLCD_V1)
        dwin_popup(PSTR(MSG_FILAMENT_CHANGE_OPTION_HEADER), 2, 1);
        SERIAL_ECHOLNPGM("Filament runout10!\r\n");
        #endif
				SERIAL_ERRORLNPGM("Filament runout10:resume option!");
        while (advanced_pause_menu_response == ADVANCED_PAUSE_RESPONSE_WAIT_FOR) idle(true);
        KEEPALIVE_STATE(IN_HANDLER);
        extrude_length = ADVANCED_PAUSE_EXTRUDE_LENGTH;
      } while (advanced_pause_menu_response == ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE);
    #endif
    #if ENABLED(ULTIPANEL)
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_RESUME);
    #elif defined(FYSTLCD_V1)
      dwin_popup(PSTR("   Filament runout11!\nWait for print to resume."),1);
    #endif
		SERIAL_ERRORLNPGM("Filament runout11!");
    #if ENABLED(FILAMENT_RUNOUT_SENSOR)
      filament_ran_out = false;
    #endif
    #if ENABLED(ULTIPANEL)
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_STATUS);
    #elif defined(FYSTLCD_V1)
      FunV052();
    #endif
		SERIAL_ERRORLNPGM("Filament runout12:resume done!");
    move_away_flag = false;
  }
  #endif
#endif 
#if ENABLED(SDSUPPORT)
  inline void gcode_M20() {
    SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
    card.ls();
    SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
  }
  inline void gcode_M21() { card.initsd(); }
  inline void gcode_M22() { card.release(); }
  inline void gcode_M23() { 
      card.openFile(parser.string_arg, true);
      #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
      strcpy(GLOBAL_var_V004, card.filename);
      #endif
  }
  inline void gcode_M24() {
    #if ENABLED(PARK_HEAD_ON_PAUSE)
      resume_print();
    #endif
    card.startFileprint();
    print_job_timer.start();
  }
  inline void gcode_M25() {
    card.pauseSDPrint();
    print_job_timer.pause();
    #if ENABLED(PARK_HEAD_ON_PAUSE)
      enqueue_and_echo_commands_P(PSTR("M125")); 
    #endif
  }
  inline void gcode_M26() {
    if (card.cardOK && parser.seenval('S'))
      card.setIndex(parser.value_long());
  }
  inline void gcode_M27() { card.getStatus(); }
  inline void gcode_M28() { card.openFile(parser.string_arg, false); }
  inline void gcode_M29() {
  }
  inline void gcode_M30() {
    if (card.cardOK) {
      card.closefile();
      card.removeFile(parser.string_arg);
    }
  }
#endif 
inline void gcode_M31() {
  char buffer[21];
  duration_t elapsed = print_job_timer.duration();
  elapsed.toString(buffer);
  lcd_setstatus(buffer);
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPAIR("Print time: ", buffer);
}
#if ENABLED(SDSUPPORT)
  inline void gcode_M32() {
    if (card.sdprinting)
      stepper.synchronize();
    char* namestartpos = parser.string_arg;
    const bool call_procedure = parser.boolval('P');
    if (card.cardOK) {
      card.openFile(namestartpos, true, call_procedure);
      if (parser.seenval('S'))
        card.setIndex(parser.value_long());
      #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
      strcpy(GLOBAL_var_V004, card.filename);
      #endif
      card.startFileprint();
      if (!call_procedure) print_job_timer.start();
    }
  }
  #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
    inline void gcode_M33() {
      card.printLongPath(parser.string_arg);
    }
  #endif
  #if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_GCODE)
    inline void gcode_M34() {
      if (parser.seen('S')) card.setSortOn(parser.value_bool());
      if (parser.seenval('F')) {
        const int v = parser.value_long();
        card.setSortFolders(v < 0 ? -1 : v > 0 ? 1 : 0);
      }
    }
  #endif 
  inline void gcode_M928() {
    card.openLogFile(parser.string_arg);
  }
#endif 
static bool pin_is_protected(const int8_t pin) {
  static const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS;
  for (uint8_t i = 0; i < COUNT(sensitive_pins); i++)
    if (pin == (int8_t)pgm_read_byte(&sensitive_pins[i])) return true;
  return false;
}
inline void gcode_M42() {
  if (!parser.seenval('S')) return;
  const byte pin_status = parser.value_byte();
  const int pin_number = parser.intval('P', LED_PIN);
  if (pin_number < 0) return;
  if (pin_is_protected(pin_number)) {
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_ERR_PROTECTED_PIN);
    return;
  }
  pinMode(pin_number, OUTPUT);
  digitalWrite(pin_number, pin_status);
  analogWrite(pin_number, pin_status);
  #if FAN_COUNT > 0
    switch (pin_number) {
      #if HAS_FAN0
        case FAN_PIN: fanSpeeds[0] = pin_status; break;
      #endif
      #if HAS_FAN1
        case FAN1_PIN: fanSpeeds[1] = pin_status; break;
      #endif
      #if HAS_FAN2
        case FAN2_PIN: fanSpeeds[2] = pin_status; break;
      #endif
    }
  #endif
}
#if ENABLED(PINS_DEBUGGING)
  #include "pinsDebug.h"
  inline void toggle_pins() {
    const bool I_flag = parser.boolval('I');
    const int repeat = parser.intval('R', 1),
              start = parser.intval('S'),
              end = parser.intval('E', NUM_DIGITAL_PINS - 1),
              wait = parser.intval('W', 500);
    for (uint8_t pin = start; pin <= end; pin++) {
      if (!I_flag && pin_is_protected(pin)) {
        report_pin_state_extended(pin, I_flag, true, "Untouched ");
        SERIAL_EOL();
      }
      else {
        report_pin_state_extended(pin, I_flag, true, "Pulsing   ");
        #if AVR_AT90USB1286_FAMILY 
          if (pin == 46) {
            SET_OUTPUT(46);
            for (int16_t j = 0; j < repeat; j++) {
              WRITE(46, 0); safe_delay(wait);
              WRITE(46, 1); safe_delay(wait);
              WRITE(46, 0); safe_delay(wait);
            }
          }
          else if (pin == 47) {
            SET_OUTPUT(47);
            for (int16_t j = 0; j < repeat; j++) {
              WRITE(47, 0); safe_delay(wait);
              WRITE(47, 1); safe_delay(wait);
              WRITE(47, 0); safe_delay(wait);
            }
          }
          else
        #endif
        {
          pinMode(pin, OUTPUT);
          for (int16_t j = 0; j < repeat; j++) {
            digitalWrite(pin, 0); safe_delay(wait);
            digitalWrite(pin, 1); safe_delay(wait);
            digitalWrite(pin, 0); safe_delay(wait);
          }
        }
      }
      SERIAL_EOL();
    }
    SERIAL_ECHOLNPGM("Done.");
  } 
  inline void servo_probe_test() {
    #if !(NUM_SERVOS > 0 && HAS_SERVO_0)
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM("SERVO not setup");
    #elif !HAS_Z_SERVO_ENDSTOP
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM("Z_ENDSTOP_SERVO_NR not setup");
    #else
      const uint8_t probe_index = parser.byteval('P', Z_ENDSTOP_SERVO_NR);
      SERIAL_PROTOCOLLNPGM("Servo probe test");
      SERIAL_PROTOCOLLNPAIR(".  using index:  ", probe_index);
      SERIAL_PROTOCOLLNPAIR(".  deploy angle: ", z_servo_angle[0]);
      SERIAL_PROTOCOLLNPAIR(".  stow angle:   ", z_servo_angle[1]);
      bool probe_inverting;
      #if ENABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN)
        #define PROBE_TEST_PIN Z_MIN_PIN
        SERIAL_PROTOCOLLNPAIR(". probe uses Z_MIN pin: ", PROBE_TEST_PIN);
        SERIAL_PROTOCOLLNPGM(". uses Z_MIN_ENDSTOP_INVERTING (ignores Z_MIN_PROBE_ENDSTOP_INVERTING)");
        SERIAL_PROTOCOLPGM(". Z_MIN_ENDSTOP_INVERTING: ");
        #if Z_MIN_ENDSTOP_INVERTING
          SERIAL_PROTOCOLLNPGM("true");
        #else
          SERIAL_PROTOCOLLNPGM("false");
        #endif
        probe_inverting = Z_MIN_ENDSTOP_INVERTING;
      #elif ENABLED(Z_MIN_PROBE_ENDSTOP)
        #define PROBE_TEST_PIN Z_MIN_PROBE_PIN
        SERIAL_PROTOCOLLNPAIR(". probe uses Z_MIN_PROBE_PIN: ", PROBE_TEST_PIN);
        SERIAL_PROTOCOLLNPGM(". uses Z_MIN_PROBE_ENDSTOP_INVERTING (ignores Z_MIN_ENDSTOP_INVERTING)");
        SERIAL_PROTOCOLPGM(". Z_MIN_PROBE_ENDSTOP_INVERTING: ");
        #if Z_MIN_PROBE_ENDSTOP_INVERTING
          SERIAL_PROTOCOLLNPGM("true");
        #else
          SERIAL_PROTOCOLLNPGM("false");
        #endif
        probe_inverting = Z_MIN_PROBE_ENDSTOP_INVERTING;
      #endif
      SERIAL_PROTOCOLLNPGM(". deploy & stow 4 times");
      SET_INPUT_PULLUP(PROBE_TEST_PIN);
      bool deploy_state, stow_state;
      for (uint8_t i = 0; i < 4; i++) {
        servo[probe_index].move(z_servo_angle[0]); 
        safe_delay(500);
        deploy_state = digitalRead(PROBE_TEST_PIN);
        servo[probe_index].move(z_servo_angle[1]); 
        safe_delay(500);
        stow_state = digitalRead(PROBE_TEST_PIN);
      }
      if (probe_inverting != deploy_state) SERIAL_PROTOCOLLNPGM("WARNING - INVERTING setting probably backwards");
      refresh_cmd_timeout();
      if (deploy_state != stow_state) {
        SERIAL_PROTOCOLLNPGM("BLTouch clone detected");
        if (deploy_state) {
          SERIAL_PROTOCOLLNPGM(".  DEPLOYED state: HIGH (logic 1)");
          SERIAL_PROTOCOLLNPGM(".  STOWED (triggered) state: LOW (logic 0)");
        }
        else {
          SERIAL_PROTOCOLLNPGM(".  DEPLOYED state: LOW (logic 0)");
          SERIAL_PROTOCOLLNPGM(".  STOWED (triggered) state: HIGH (logic 1)");
        }
        #if ENABLED(BLTOUCH)
          SERIAL_PROTOCOLLNPGM("ERROR: BLTOUCH enabled - set this device up as a Z Servo Probe with inverting as true.");
        #endif
      }
      else {                                           
        servo[probe_index].move(z_servo_angle[0]);     
        safe_delay(500);
        SERIAL_PROTOCOLLNPGM("please trigger probe");
        uint16_t probe_counter = 0;
        for (uint16_t j = 0; j < 500 * 30 && probe_counter == 0 ; j++) {
          safe_delay(2);
          if (0 == j % (500 * 1)) 
            refresh_cmd_timeout();
          if (deploy_state != digitalRead(PROBE_TEST_PIN)) { 
            for (probe_counter = 1; probe_counter < 50 && deploy_state != digitalRead(PROBE_TEST_PIN); ++probe_counter)
              safe_delay(2);
            if (probe_counter == 50)
              SERIAL_PROTOCOLLNPGM("Z Servo Probe detected"); 
            else if (probe_counter >= 2)
              SERIAL_PROTOCOLLNPAIR("BLTouch compatible probe detected - pulse width (+/- 4mS): ", probe_counter * 2); 
            else
              SERIAL_PROTOCOLLNPGM("noise detected - please re-run test"); 
            servo[probe_index].move(z_servo_angle[1]); 
          }  
        } 
        if (probe_counter == 0) SERIAL_PROTOCOLLNPGM("trigger not detected");
      } 
    #endif
  } 
  inline void gcode_M43() {
    if (parser.seen('T')) {   
      toggle_pins();
      return;
    }
    if (parser.seen('E')) {
      endstop_monitor_flag = parser.value_bool();
      SERIAL_PROTOCOLPGM("endstop monitor ");
      SERIAL_PROTOCOL(endstop_monitor_flag ? "en" : "dis");
      SERIAL_PROTOCOLLNPGM("abled");
      return;
    }
    if (parser.seen('S')) {
      servo_probe_test();
      return;
    }
    const uint8_t first_pin = parser.byteval('P'),
                  last_pin = parser.seenval('P') ? first_pin : NUM_DIGITAL_PINS - 1;
    if (first_pin > last_pin) return;
    const bool ignore_protection = parser.boolval('I');
    if (parser.boolval('W')) {
      SERIAL_PROTOCOLLNPGM("Watching pins");
      byte pin_state[last_pin - first_pin + 1];
      for (int8_t pin = first_pin; pin <= last_pin; pin++) {
        if (pin_is_protected(pin) && !ignore_protection) continue;
        pinMode(pin, INPUT_PULLUP);
        delay(1);
            pin_state[pin - first_pin] = digitalRead(pin);
      }
      #if HAS_RESUME_CONTINUE
        wait_for_user = true;
        KEEPALIVE_STATE(PAUSED_FOR_USER);
      #endif
      for (;;) {
        for (int8_t pin = first_pin; pin <= last_pin; pin++) {
          if (pin_is_protected(pin) && !ignore_protection) continue;
          const byte val =
              digitalRead(pin);
          if (val != pin_state[pin - first_pin]) {
            report_pin_state_extended(pin, ignore_protection, false);
            pin_state[pin - first_pin] = val;
          }
        }
        #if HAS_RESUME_CONTINUE
          if (!wait_for_user) {
            KEEPALIVE_STATE(IN_HANDLER);
            break;
          }
        #endif
        safe_delay(200);
      }
      return;
    }
    for (uint8_t pin = first_pin; pin <= last_pin; pin++)
      report_pin_state_extended(pin, ignore_protection, true);
  }
#endif 
#if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
  inline void gcode_M48() {
    if (axis_unhomed_error()) return;
    const int8_t verbose_level = parser.byteval('V', 1);
    if (!WITHIN(verbose_level, 0, 4)) {
      SERIAL_PROTOCOLLNPGM("?(V)erbose level is implausible (0-4).");
      return;
    }
    if (verbose_level > 0)
      SERIAL_PROTOCOLLNPGM("M48 Z-Probe Repeatability Test");
    const int8_t n_samples = parser.byteval('P', 10);
    if (!WITHIN(n_samples, 4, 50)) {
      SERIAL_PROTOCOLLNPGM("?Sample size not plausible (4-50).");
      return;
    }
    const bool stow_probe_after_each = parser.boolval('E');
    float X_current = current_position[X_AXIS],
          Y_current = current_position[Y_AXIS];
    const float X_probe_location = parser.linearval('X', X_current + X_PROBE_OFFSET_FROM_EXTRUDER),
                Y_probe_location = parser.linearval('Y', Y_current + Y_PROBE_OFFSET_FROM_EXTRUDER);
    #if DISABLED(DELTA)
      if (!WITHIN(X_probe_location, LOGICAL_X_POSITION(MIN_PROBE_X), LOGICAL_X_POSITION(MAX_PROBE_X))) {
        out_of_range_error(PSTR("X"));
        return;
      }
      if (!WITHIN(Y_probe_location, LOGICAL_Y_POSITION(MIN_PROBE_Y), LOGICAL_Y_POSITION(MAX_PROBE_Y))) {
        out_of_range_error(PSTR("Y"));
        return;
      }
    #else
      if (!position_is_reachable_by_probe_xy(X_probe_location, Y_probe_location)) {
        SERIAL_PROTOCOLLNPGM("? (X,Y) location outside of probeable radius.");
        return;
      }
    #endif
    bool seen_L = parser.seen('L');
    uint8_t n_legs = seen_L ? parser.value_byte() : 0;
    if (n_legs > 15) {
      SERIAL_PROTOCOLLNPGM("?Number of legs in movement not plausible (0-15).");
      return;
    }
    if (n_legs == 1) n_legs = 2;
    const bool schizoid_flag = parser.boolval('S');
    if (schizoid_flag && !seen_L) n_legs = 7;
    if (verbose_level > 2)
      SERIAL_PROTOCOLLNPGM("Positioning the probe...");
    #if HAS_LEVELING
      const bool was_enabled = leveling_is_active();
      set_bed_leveling_enabled(false);
    #endif
    setup_for_endstop_or_probe_move();
    const float t = probe_pt(X_probe_location, Y_probe_location, stow_probe_after_each, verbose_level);
    if (isnan(t)) return;
    randomSeed(millis());
    double mean = 0.0, sigma = 0.0, min = 99999.9, max = -99999.9, sample_set[n_samples];
    for (uint8_t n = 0; n < n_samples; n++) {
      if (n_legs) {
        int dir = (random(0, 10) > 5.0) ? -1 : 1;  
        float angle = random(0.0, 360.0),
              radius = random(
                #if ENABLED(DELTA)
                  DELTA_PROBEABLE_RADIUS / 8, DELTA_PROBEABLE_RADIUS / 3
                #else
                  5, X_MAX_LENGTH / 8
                #endif
              );
        if (verbose_level > 3) {
          SERIAL_ECHOPAIR("Starting radius: ", radius);
          SERIAL_ECHOPAIR("   angle: ", angle);
          SERIAL_ECHOPGM(" Direction: ");
          if (dir > 0) SERIAL_ECHOPGM("Counter-");
          SERIAL_ECHOLNPGM("Clockwise");
        }
        for (uint8_t l = 0; l < n_legs - 1; l++) {
          double delta_angle;
          if (schizoid_flag)
            delta_angle = dir * 2.0 * 72.0;
          else
            delta_angle = dir * (float) random(25, 45);
          angle += delta_angle;
          while (angle > 360.0)   
            angle -= 360.0;       
          while (angle < 0.0)     
            angle += 360.0;       
          X_current = X_probe_location - (X_PROBE_OFFSET_FROM_EXTRUDER) + cos(RADIANS(angle)) * radius;
          Y_current = Y_probe_location - (Y_PROBE_OFFSET_FROM_EXTRUDER) + sin(RADIANS(angle)) * radius;
          #if DISABLED(DELTA)
            X_current = constrain(X_current, X_MIN_POS, X_MAX_POS);
            Y_current = constrain(Y_current, Y_MIN_POS, Y_MAX_POS);
          #else
            while (!position_is_reachable_by_probe_xy(X_current, Y_current)) {
              X_current *= 0.8;
              Y_current *= 0.8;
              if (verbose_level > 3) {
                SERIAL_ECHOPAIR("Pulling point towards center:", X_current);
                SERIAL_ECHOLNPAIR(", ", Y_current);
              }
            }
          #endif
          if (verbose_level > 3) {
            SERIAL_PROTOCOLPGM("Going to:");
            SERIAL_ECHOPAIR(" X", X_current);
            SERIAL_ECHOPAIR(" Y", Y_current);
            SERIAL_ECHOLNPAIR(" Z", current_position[Z_AXIS]);
          }
          do_blocking_move_to_xy(X_current, Y_current);
        } 
      } 
      sample_set[n] = probe_pt(X_probe_location, Y_probe_location, stow_probe_after_each, 0);
      double sum = 0.0;
      for (uint8_t j = 0; j <= n; j++) sum += sample_set[j];
      mean = sum / (n + 1);
      NOMORE(min, sample_set[n]);
      NOLESS(max, sample_set[n]);
      sum = 0.0;
      for (uint8_t j = 0; j <= n; j++)
        sum += sq(sample_set[j] - mean);
      sigma = SQRT(sum / (n + 1));
      if (verbose_level > 0) {
        if (verbose_level > 1) {
          SERIAL_PROTOCOL(n + 1);
          SERIAL_PROTOCOLPGM(" of ");
          SERIAL_PROTOCOL((int)n_samples);
          SERIAL_PROTOCOLPGM(": z: ");
          SERIAL_PROTOCOL_F(sample_set[n], 3);
          if (verbose_level > 2) {
            SERIAL_PROTOCOLPGM(" mean: ");
            SERIAL_PROTOCOL_F(mean, 4);
            SERIAL_PROTOCOLPGM(" sigma: ");
            SERIAL_PROTOCOL_F(sigma, 6);
            SERIAL_PROTOCOLPGM(" min: ");
            SERIAL_PROTOCOL_F(min, 3);
            SERIAL_PROTOCOLPGM(" max: ");
            SERIAL_PROTOCOL_F(max, 3);
            SERIAL_PROTOCOLPGM(" range: ");
            SERIAL_PROTOCOL_F(max-min, 3);
          }
          SERIAL_EOL();
        }
      }
    } 
    if (STOW_PROBE()) return;
    SERIAL_PROTOCOLPGM("Finished!");
    SERIAL_EOL();
    if (verbose_level > 0) {
      SERIAL_PROTOCOLPGM("Mean: ");
      SERIAL_PROTOCOL_F(mean, 6);
      SERIAL_PROTOCOLPGM(" Min: ");
      SERIAL_PROTOCOL_F(min, 3);
      SERIAL_PROTOCOLPGM(" Max: ");
      SERIAL_PROTOCOL_F(max, 3);
      SERIAL_PROTOCOLPGM(" Range: ");
      SERIAL_PROTOCOL_F(max-min, 3);
      SERIAL_EOL();
    }
    SERIAL_PROTOCOLPGM("Standard Deviation: ");
    SERIAL_PROTOCOL_F(sigma, 6);
    SERIAL_EOL();
    SERIAL_EOL();
    clean_up_after_endstop_or_probe_move();
    #if HAS_LEVELING
      set_bed_leveling_enabled(was_enabled);
    #endif
    report_current_position();
  }
#endif 
#if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(UBL_G26_MESH_VALIDATION)
  inline void gcode_M49() {
    ubl.g26_debug_flag ^= true;
    SERIAL_PROTOCOLPGM("UBL Debug Flag turned ");
    serialprintPGM(ubl.g26_debug_flag ? PSTR("on.") : PSTR("off."));
  }
#endif 
inline void gcode_M75() { print_job_timer.start(); }
inline void gcode_M76() { print_job_timer.pause(); }
inline void gcode_M77() { print_job_timer.stop(); }
#if ENABLED(PRINTCOUNTER)
  inline void gcode_M78() {
    if (parser.intval('S') == 78)
      print_job_timer.initStats();
    else
      print_job_timer.showStats();
  }
#endif
inline void gcode_M104() {
  if (get_target_extruder_from_command(104)) return;
  if (DEBUGGING(DRYRUN)) return;
  #if ENABLED(SINGLENOZZLE)
    if (target_extruder != active_extruder) return;
  #endif
  if (parser.seenval('S')) {
    const int16_t temp = parser.value_celsius();
    thermalManager.setTargetHotend(temp, target_extruder);
    #if ENABLED(DUAL_X_CARRIAGE)
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        thermalManager.setTargetHotend(temp ? temp + duplicate_extruder_temp_offset : 0, 1);
    #endif
    #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
      if (parser.value_celsius() <= (EXTRUDE_MINTEMP) / 2) {
        print_job_timer.stop();
        LCD_MESSAGEPGM(WELCOME_MSG);
      }
    #endif
    if (parser.value_celsius() > thermalManager.degHotend(target_extruder))
        #if ENABLED(FYS_ULTRA_LCD_WANHAO_ONEPLUS)
        LCD_MESSAGEPGM(MSG_HEATING);
        #else 
        lcd_status_printf_P(0, PSTR("E%i %s"), target_extruder + 1, MSG_HEATING);
        #endif
  }
  #if ENABLED(AUTOTEMP)
    planner.autotemp_M104_M109();
  #endif
}
#if HAS_TEMP_HOTEND || HAS_TEMP_BED
  void print_heater_state(const float &c, const float &t,
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      const float r,
    #endif
    const int8_t e=-2
  ) {
    SERIAL_PROTOCOLCHAR(' ');
    SERIAL_PROTOCOLCHAR(
      #if HAS_TEMP_BED && HAS_TEMP_HOTEND
        e == -1 ? 'B' : 'T'
      #elif HAS_TEMP_HOTEND
        'T'
      #else
        'B'
      #endif
    );
    #if HOTENDS > 1
      if (e >= 0) SERIAL_PROTOCOLCHAR('0' + e);
    #endif
    SERIAL_PROTOCOLCHAR(':');
    SERIAL_PROTOCOL(c);
    SERIAL_PROTOCOLPAIR(" /" , t);
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      SERIAL_PROTOCOLPAIR(" (", r / OVERSAMPLENR);
      SERIAL_PROTOCOLCHAR(')');
    #endif
  }
  void print_heaterstates() {
    #if HAS_TEMP_HOTEND
      print_heater_state(thermalManager.degHotend(target_extruder), thermalManager.degTargetHotend(target_extruder)
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          , thermalManager.rawHotendTemp(target_extruder)
        #endif
      );
    #endif
    #if HAS_TEMP_BED
      print_heater_state(thermalManager.degBed(), thermalManager.degTargetBed(),
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          thermalManager.rawBedTemp(),
        #endif
        -1 
      );
    #endif
    #if HOTENDS > 1
      HOTEND_LOOP() print_heater_state(thermalManager.degHotend(e), thermalManager.degTargetHotend(e),
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          thermalManager.rawHotendTemp(e),
        #endif
        e
      );
    #endif
    SERIAL_PROTOCOLPGM(" @:");
    SERIAL_PROTOCOL(thermalManager.getHeaterPower(target_extruder));
    #if HAS_TEMP_BED
      SERIAL_PROTOCOLPGM(" B@:");
      SERIAL_PROTOCOL(thermalManager.getHeaterPower(-1));
    #endif
    #if HOTENDS > 1
      HOTEND_LOOP() {
        SERIAL_PROTOCOLPAIR(" @", e);
        SERIAL_PROTOCOLCHAR(':');
        SERIAL_PROTOCOL(thermalManager.getHeaterPower(e));
      }
    #endif
  }
#endif
inline void gcode_M105() {
  if (get_target_extruder_from_command(105)) return;
  #if HAS_TEMP_HOTEND || HAS_TEMP_BED
    SERIAL_PROTOCOLPGM(MSG_OK);
    print_heaterstates();
  #else 
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
  #endif
  SERIAL_EOL();
}
#if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
  static uint8_t auto_report_temp_interval;
  static millis_t next_temp_report_ms;
  inline void gcode_M155() {
    if (parser.seenval('S')) {
      auto_report_temp_interval = parser.value_byte();
      NOMORE(auto_report_temp_interval, 60);
      next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
    }
  }
  inline void auto_report_temperatures() {
    if (auto_report_temp_interval && ELAPSED(millis(), next_temp_report_ms)) {
      next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
      print_heaterstates();
      SERIAL_EOL();
    }
  }
#endif 
#if FAN_COUNT > 0
  inline void gcode_M106() {
    uint16_t s = parser.ushortval('S', 255);
    NOMORE(s, 255);
    const uint8_t p = parser.byteval('P', 0);
    if (p < FAN_COUNT) fanSpeeds[p] = s;
  }
  inline void gcode_M107() {
    const uint16_t p = parser.ushortval('P');
    if (p < FAN_COUNT) fanSpeeds[p] = 0;
  }
#endif 
#if DISABLED(EMERGENCY_PARSER)
  inline void gcode_M108() { wait_for_heatup = false; }
  inline void gcode_M112() { kill(PSTR(MSG_KILLED)); }
  inline void gcode_M410() { quickstop_stepper(); }
#endif
#ifndef MIN_COOLING_SLOPE_DEG
  #define MIN_COOLING_SLOPE_DEG 1.50
#endif
#ifndef MIN_COOLING_SLOPE_TIME
  #define MIN_COOLING_SLOPE_TIME 60
#endif
#ifdef  FYS_SAFE_PRINT_BREAK
void FunV00A()
{
  #if ENABLED(FYS_PRINTING_STATE)
    GLOBAL_var_V007 = MACRO_var_V01D;
  #endif
    print_job_timer.pause();
    #if TEMP_RESIDENCY_TIME > 0
    millis_t residency_start_ms = 0;
    #define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_RESIDENCY_TIME) * 1000UL))
    #else
    #define TEMP_CONDITIONS (wants_to_cool ? thermalManager.isCoolingHotend(target_extruder) : thermalManager.isHeatingHotend(target_extruder))
    #endif
    float target_temp = -1.0, old_temp = 9999.0;
    bool wants_to_cool = false;
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;
    KEEPALIVE_STATE(NOT_BUSY);
    #if ENABLED(PRINTER_EVENT_LEDS)
    const float start_temp = thermalManager.degHotend(target_extruder);
    uint8_t old_blue = 0;
    #endif
    do {
    if (target_temp != thermalManager.degTargetHotend(target_extruder)) {
        wants_to_cool = thermalManager.isCoolingHotend(target_extruder);
        target_temp = thermalManager.degTargetHotend(target_extruder);
        if (wants_to_cool) break;
    }
    now = millis();
    if (ELAPSED(now, next_temp_ms)) { 
        next_temp_ms = now + 1000UL;
        print_heaterstates();
        #if TEMP_RESIDENCY_TIME > 0
        SERIAL_PROTOCOLPGM(" W:");
        if (residency_start_ms)
            SERIAL_PROTOCOL(long((((TEMP_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL));
        else
            SERIAL_PROTOCOLCHAR('?');
        #endif
        SERIAL_EOL();
    }
    idle();
    refresh_cmd_timeout(); 
    const float temp = thermalManager.degHotend(target_extruder);
    #if ENABLED(PRINTER_EVENT_LEDS)
        if (!wants_to_cool) {
        const uint8_t blue = map(constrain(temp, start_temp, target_temp), start_temp, target_temp, 255, 0);
        if (blue != old_blue) set_led_color(255, 0, (old_blue = blue));
        }
    #endif
    #if TEMP_RESIDENCY_TIME > 0
        const float temp_diff = FABS(target_temp - temp);
        if (!residency_start_ms) {
        if (temp_diff < TEMP_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_HYSTERESIS) {
        residency_start_ms = now;
        }
    #endif
        if (wants_to_cool) 
        {
            if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
            if (old_temp - temp < MIN_COOLING_SLOPE_DEG) break;
            next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME;
            old_temp = temp;
            }
        }
    } while (wait_for_heatup && TEMP_CONDITIONS);
    if (print_job_timer.isPaused())print_job_timer.start();
 }
#endif
inline void gcode_M109() {
  if (get_target_extruder_from_command(109)) return;
  if (DEBUGGING(DRYRUN)) return;
  #if ENABLED(SINGLENOZZLE)
    if (target_extruder != active_extruder) return;
  #endif
  const bool no_wait_for_cooling = parser.seenval('S');
  if (no_wait_for_cooling || parser.seenval('R')) {
    const int16_t temp = parser.value_celsius();
    thermalManager.setTargetHotend(temp, target_extruder);
    #if ENABLED(DUAL_X_CARRIAGE)
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        thermalManager.setTargetHotend(temp ? temp + duplicate_extruder_temp_offset : 0, 1);
    #endif
    #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
      if (parser.value_celsius() <= (EXTRUDE_MINTEMP) / 2) {
        print_job_timer.stop();
        LCD_MESSAGEPGM(WELCOME_MSG);
      }
      else
        print_job_timer.start();
    #endif
    if (thermalManager.isHeatingHotend(target_extruder)) 
        #if ENABLED(FYS_ULTRA_LCD_WANHAO_ONEPLUS)
        LCD_MESSAGEPGM(MSG_HEATING);
        #else 
        lcd_status_printf_P(0, PSTR("E%i %s"), target_extruder + 1, MSG_HEATING);
        #endif
  }
  else return;
  #if ENABLED(AUTOTEMP)
    planner.autotemp_M104_M109();
  #endif
#ifdef FYS_SAFE_PRINT_BREAK
  FunV00A();
#else
  #if ENABLED(FYS_PRINTING_STATE)
    GLOBAL_var_V007 = MACRO_var_V01D;
  #endif
  #if TEMP_RESIDENCY_TIME > 0
    millis_t residency_start_ms = 0;
    #define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_RESIDENCY_TIME) * 1000UL))
  #else
    #define TEMP_CONDITIONS (wants_to_cool ? thermalManager.isCoolingHotend(target_extruder) : thermalManager.isHeatingHotend(target_extruder))
  #endif
  float target_temp = -1.0, old_temp = 9999.0;
  bool wants_to_cool = false;
  wait_for_heatup = true;
  millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;
  KEEPALIVE_STATE(NOT_BUSY);
  #if ENABLED(PRINTER_EVENT_LEDS)
    const float start_temp = thermalManager.degHotend(target_extruder);
    uint8_t old_blue = 0;
  #endif
  do {
    if (target_temp != thermalManager.degTargetHotend(target_extruder)) {
      wants_to_cool = thermalManager.isCoolingHotend(target_extruder);
      target_temp = thermalManager.degTargetHotend(target_extruder);
      if (no_wait_for_cooling && wants_to_cool) break;
    }
    now = millis();
    if (ELAPSED(now, next_temp_ms)) { 
      next_temp_ms = now + 1000UL;
      print_heaterstates();
      #if TEMP_RESIDENCY_TIME > 0
        SERIAL_PROTOCOLPGM(" W:");
        if (residency_start_ms)
          SERIAL_PROTOCOL(long((((TEMP_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL));
        else
          SERIAL_PROTOCOLCHAR('?');
      #endif
      SERIAL_EOL();
    }
    idle();
    refresh_cmd_timeout(); 
    const float temp = thermalManager.degHotend(target_extruder);
    #if ENABLED(PRINTER_EVENT_LEDS)
      if (!wants_to_cool) {
        const uint8_t blue = map(constrain(temp, start_temp, target_temp), start_temp, target_temp, 255, 0);
        if (blue != old_blue) set_led_color(255, 0, (old_blue = blue));
      }
    #endif
    #if TEMP_RESIDENCY_TIME > 0
      const float temp_diff = FABS(target_temp - temp);
      if (!residency_start_ms) {
        if (temp_diff < TEMP_WINDOW) residency_start_ms = now;
      }
      else if (temp_diff > TEMP_HYSTERESIS) {
        residency_start_ms = now;
      }
    #endif
    if (wants_to_cool) {
      if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
        if (old_temp - temp < MIN_COOLING_SLOPE_DEG) break;
        next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME;
        old_temp = temp;
      }
    }
  } while (wait_for_heatup && TEMP_CONDITIONS);
#endif
  if (wait_for_heatup) {
    LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
    #if ENABLED(PRINTER_EVENT_LEDS)
      #if ENABLED(RGBW_LED)
        set_led_color(0, 0, 0, 255);  
      #else
        set_led_color(255, 255, 255); 
      #endif
    #endif
  }
  KEEPALIVE_STATE(IN_HANDLER);
  #if ENABLED(FYS_PRINTING_STATE)
    GLOBAL_var_V007 = MACRO_var_V01A;
  #endif
}
#if HAS_TEMP_BED
  #ifndef MIN_COOLING_SLOPE_DEG_BED
    #define MIN_COOLING_SLOPE_DEG_BED 1.50
  #endif
  #ifndef MIN_COOLING_SLOPE_TIME_BED
    #define MIN_COOLING_SLOPE_TIME_BED 60
  #endif
  #if ENABLED(FYS_SAFE_PRINT_BREAK)
    void FunV00C()
    {
      #if ENABLED(FYS_PRINTING_STATE)
        GLOBAL_var_V007 = MACRO_var_V01E;
      #endif
        print_job_timer.pause();
        #if TEMP_BED_RESIDENCY_TIME > 0
          millis_t residency_start_ms = 0;
          #define TEMP_BED_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_BED_RESIDENCY_TIME) * 1000UL))
        #else
          #define TEMP_BED_CONDITIONS (wants_to_cool ? thermalManager.isCoolingBed() : thermalManager.isHeatingBed())
        #endif
        float target_temp = -1.0, old_temp = 9999.0;
        bool wants_to_cool = false;
        wait_for_heatup = true;
        millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;
        KEEPALIVE_STATE(NOT_BUSY);
        target_extruder = active_extruder; 
        #if ENABLED(PRINTER_EVENT_LEDS)
          const float start_temp = thermalManager.degBed();
          uint8_t old_red = 255;
        #endif
        do {
          if (target_temp != thermalManager.degTargetBed()) {
            wants_to_cool = thermalManager.isCoolingBed();
            target_temp = thermalManager.degTargetBed();
            if (wants_to_cool) break;
          }
          now = millis();
          if (ELAPSED(now, next_temp_ms)) { 
            next_temp_ms = now + 1000UL;
            print_heaterstates();
            #if TEMP_BED_RESIDENCY_TIME > 0
              SERIAL_PROTOCOLPGM(" W:");
              if (residency_start_ms)
                SERIAL_PROTOCOL(long((((TEMP_BED_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL));
              else
                SERIAL_PROTOCOLCHAR('?');
            #endif
            SERIAL_EOL();
          }
          idle();
          refresh_cmd_timeout(); 
          const float temp = thermalManager.degBed();
          #if ENABLED(PRINTER_EVENT_LEDS)
            if (!wants_to_cool) {
              const uint8_t red = map(constrain(temp, start_temp, target_temp), start_temp, target_temp, 0, 255);
              if (red != old_red) set_led_color((old_red = red), 0, 255);
            }
          #endif
          #if TEMP_BED_RESIDENCY_TIME > 0
            const float temp_diff = FABS(target_temp - temp);
            if (!residency_start_ms) {
              if (temp_diff < TEMP_BED_WINDOW) residency_start_ms = now;
            }
            else if (temp_diff > TEMP_BED_HYSTERESIS) {
              residency_start_ms = now;
            }
          #endif 
          if (wants_to_cool) {
            if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
              if (old_temp - temp < MIN_COOLING_SLOPE_DEG_BED) break;
              next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME_BED;
              old_temp = temp;
            }
          }
        } while (wait_for_heatup && TEMP_BED_CONDITIONS);
        if(print_job_timer.isPaused())print_job_timer.start();
    }
  #endif
  inline void gcode_M190() {
    if (DEBUGGING(DRYRUN))return;
    LCD_MESSAGEPGM(MSG_BED_HEATING);
    const bool no_wait_for_cooling = parser.seenval('S');
    if (no_wait_for_cooling || parser.seenval('R')) {
      thermalManager.setTargetBed(parser.value_celsius());
      #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
        if (parser.value_celsius() > BED_MINTEMP)
          print_job_timer.start();
      #endif
    }
    else return;
  #ifdef FYS_SAFE_PRINT_BREAK
    FunV00C();
  #else
    #if ENABLED(FYS_PRINTING_STATE)
      GLOBAL_var_V007 = MACRO_var_V01E;
    #endif
    #if TEMP_BED_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      #define TEMP_BED_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_BED_RESIDENCY_TIME) * 1000UL))
    #else
      #define TEMP_BED_CONDITIONS (wants_to_cool ? thermalManager.isCoolingBed() : thermalManager.isHeatingBed())
    #endif
    float target_temp = -1.0, old_temp = 9999.0;
    bool wants_to_cool = false;
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;
    KEEPALIVE_STATE(NOT_BUSY);
    target_extruder = active_extruder; 
    #if ENABLED(PRINTER_EVENT_LEDS)
      const float start_temp = thermalManager.degBed();
      uint8_t old_red = 255;
    #endif
    do {
      if (target_temp != thermalManager.degTargetBed()) {
        wants_to_cool = thermalManager.isCoolingBed();
        target_temp = thermalManager.degTargetBed();
        if (no_wait_for_cooling && wants_to_cool) break;
      }
      now = millis();
      if (ELAPSED(now, next_temp_ms)) { 
        next_temp_ms = now + 1000UL;
        print_heaterstates();
        #if TEMP_BED_RESIDENCY_TIME > 0
          SERIAL_PROTOCOLPGM(" W:");
          if (residency_start_ms)
            SERIAL_PROTOCOL(long((((TEMP_BED_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL));
          else
            SERIAL_PROTOCOLCHAR('?');
        #endif
        SERIAL_EOL();
      }
      idle();
      refresh_cmd_timeout(); 
      const float temp = thermalManager.degBed();
      #if ENABLED(PRINTER_EVENT_LEDS)
        if (!wants_to_cool) {
          const uint8_t red = map(constrain(temp, start_temp, target_temp), start_temp, target_temp, 0, 255);
          if (red != old_red) set_led_color((old_red = red), 0, 255);
        }
      #endif
      #if TEMP_BED_RESIDENCY_TIME > 0
        const float temp_diff = FABS(target_temp - temp);
        if (!residency_start_ms) {
          if (temp_diff < TEMP_BED_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_BED_HYSTERESIS) {
          residency_start_ms = now;
        }
      #endif 
      if (wants_to_cool) {
        if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
          if (old_temp - temp < MIN_COOLING_SLOPE_DEG_BED) break;
          next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME_BED;
          old_temp = temp;
        }
      }
    } while (wait_for_heatup && TEMP_BED_CONDITIONS);
  #endif 
    if (wait_for_heatup) LCD_MESSAGEPGM(MSG_BED_DONE);
    KEEPALIVE_STATE(IN_HANDLER);
    #if ENABLED(FYS_PRINTING_STATE)
      GLOBAL_var_V007 = MACRO_var_V01A;
    #endif
  }
#endif 
inline void gcode_M110() {
  if (parser.seenval('N')) gcode_LastN = parser.value_long();
}
inline void gcode_M111() {
  marlin_debug_flags = parser.byteval('S', (uint8_t)DEBUG_NONE);
  const static char str_debug_1[] PROGMEM = MSG_DEBUG_ECHO;
  const static char str_debug_2[] PROGMEM = MSG_DEBUG_INFO;
  const static char str_debug_4[] PROGMEM = MSG_DEBUG_ERRORS;
  const static char str_debug_8[] PROGMEM = MSG_DEBUG_DRYRUN;
  const static char str_debug_16[] PROGMEM = MSG_DEBUG_COMMUNICATION;
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    const static char str_debug_32[] PROGMEM = MSG_DEBUG_LEVELING;
  #endif
  const static char* const debug_strings[] PROGMEM = {
    str_debug_1, str_debug_2, str_debug_4, str_debug_8, str_debug_16
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      , str_debug_32
    #endif
  };
  SERIAL_ECHO_START();
  SERIAL_ECHOPGM(MSG_DEBUG_PREFIX);
  if (marlin_debug_flags) {
    uint8_t comma = 0;
    for (uint8_t i = 0; i < COUNT(debug_strings); i++) {
      if (TEST(marlin_debug_flags, i)) {
        if (comma++) SERIAL_CHAR(',');
        serialprintPGM((char*)pgm_read_word(&debug_strings[i]));
      }
    }
  }
  else {
    SERIAL_ECHOPGM(MSG_DEBUG_OFF);
  }
  SERIAL_EOL();
}
#if ENABLED(HOST_KEEPALIVE_FEATURE)
  inline void gcode_M113() {
    if (parser.seenval('S')) {
      host_keepalive_interval = parser.value_byte();
      NOMORE(host_keepalive_interval, 60);
    }
    else {
      SERIAL_ECHO_START();
      SERIAL_ECHOLNPAIR("M113 S", (unsigned long)host_keepalive_interval);
    }
  }
#endif
#if ENABLED(BARICUDA)
  #if HAS_HEATER_1
    inline void gcode_M126() { baricuda_valve_pressure = parser.byteval('S', 255); }
    inline void gcode_M127() { baricuda_valve_pressure = 0; }
  #endif
  #if HAS_HEATER_2
    inline void gcode_M128() { baricuda_e_to_p_pressure = parser.byteval('S', 255); }
    inline void gcode_M129() { baricuda_e_to_p_pressure = 0; }
  #endif
#endif 
inline void gcode_M140() {
  if (DEBUGGING(DRYRUN)) return;
  if (parser.seenval('S')) thermalManager.setTargetBed(parser.value_celsius());
}
  inline void gcode_M145() {
    const uint8_t material = (uint8_t)parser.intval('S');
    if (material >= COUNT(lcd_preheat_hotend_temp)) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM(MSG_ERR_MATERIAL_INDEX);
    }
    else {
      int v;
      if (parser.seenval('H')) {
        v = parser.value_int();
        lcd_preheat_hotend_temp[material] = constrain(v, EXTRUDE_MINTEMP, HEATER_0_MAXTEMP - 15);
      }
      if (parser.seenval('F')) {
        v = parser.value_int();
        lcd_preheat_fan_speed[material] = constrain(v, 0, 255);
      }
      #if TEMP_SENSOR_BED != 0
        if (parser.seenval('B')) {
          v = parser.value_int();
          lcd_preheat_bed_temp[material] = constrain(v, BED_MINTEMP, BED_MAXTEMP - 15);
        }
      #endif
    }
  }
#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  inline void gcode_M149() {
         if (parser.seenval('C')) parser.set_input_temp_units(TEMPUNIT_C);
    else if (parser.seenval('K')) parser.set_input_temp_units(TEMPUNIT_K);
    else if (parser.seenval('F')) parser.set_input_temp_units(TEMPUNIT_F);
  }
#endif
#if HAS_POWER_SWITCH
  inline void gcode_M80() {
    if (parser.seen('S')) {
      serialprintPGM(powersupply_on ? PSTR("PS:1\n") : PSTR("PS:0\n"));
      return;
    }
    OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE); 
    #if HAS_SUICIDE
      OUT_WRITE(SUICIDE_PIN, HIGH);
    #endif
    #if ENABLED(HAVE_TMC2130)
      delay(100);
      tmc2130_init(); 
    #endif
    powersupply_on = true;
    #if ENABLED(ULTIPANEL)
      LCD_MESSAGEPGM(WELCOME_MSG);
    #endif
  }
#endif 
inline void gcode_M81() {
  thermalManager.disable_all_heaters();
  stepper.finish_and_disable();
  #if FAN_COUNT > 0
    for (uint8_t i = 0; i < FAN_COUNT; i++) fanSpeeds[i] = 0;
    #if ENABLED(PROBING_FANS_OFF)
      fans_paused = false;
      ZERO(paused_fanSpeeds);
    #endif
  #endif
  safe_delay(1000); 
  #if HAS_SUICIDE
    stepper.synchronize();
    suicide();
  #elif HAS_POWER_SWITCH
    OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    powersupply_on = false;
  #endif
  #if ENABLED(ULTIPANEL)
    LCD_MESSAGEPGM(MACHINE_NAME " " MSG_OFF ".");
  #endif
}
inline void gcode_M82() { axis_relative_modes[E_AXIS] = false; }
inline void gcode_M83() { axis_relative_modes[E_AXIS] = true; }
inline void gcode_M18_M84() {
  if (parser.seenval('S')) {
    stepper_inactive_time = parser.value_millis_from_seconds();
  }
  else {
    bool all_axis = !((parser.seen('X')) || (parser.seen('Y')) || (parser.seen('Z')) || (parser.seen('E')));
    if (all_axis) {
      stepper.finish_and_disable();
    }
    else {
      stepper.synchronize();
      if (parser.seen('X')) disable_X();
      if (parser.seen('Y')) disable_Y();
      if (parser.seen('Z')) disable_Z();
      #if E0_ENABLE_PIN != X_ENABLE_PIN && E1_ENABLE_PIN != Y_ENABLE_PIN 
        if (parser.seen('E')) disable_e_steppers();
      #endif
    }
    #if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(ULTRA_LCD)  
      ubl_lcd_map_control = defer_return_to_status = false;
    #endif
  }
}
inline void gcode_M85() {
  if (parser.seen('S')) max_inactive_time = parser.value_millis_from_seconds();
}
#if ENABLED(DISTINCT_E_FACTORS)
  #define GET_TARGET_EXTRUDER(CMD) if (get_target_extruder_from_command(CMD)) return
  #define TARGET_EXTRUDER target_extruder
#else
  #define GET_TARGET_EXTRUDER(CMD) NOOP
  #define TARGET_EXTRUDER 0
#endif
inline void gcode_M92() {
  GET_TARGET_EXTRUDER(92);
  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      if (i == E_AXIS) {
        const float value = parser.value_per_axis_unit((AxisEnum)(E_AXIS + TARGET_EXTRUDER));
        if (value < 20.0) {
          float factor = planner.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] / value; 
          planner.max_jerk[E_AXIS] *= factor;
          planner.max_feedrate_mm_s[E_AXIS + TARGET_EXTRUDER] *= factor;
          planner.max_acceleration_steps_per_s2[E_AXIS + TARGET_EXTRUDER] *= factor;
        }
        planner.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] = value;
      }
      else {
        planner.axis_steps_per_mm[i] = parser.value_per_axis_unit((AxisEnum)i);
      }
    }
  }
  planner.refresh_positioning();
}
void report_current_position() {
  SERIAL_PROTOCOLPGM("X:");
  SERIAL_PROTOCOL(current_position[X_AXIS]);
  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(current_position[Y_AXIS]);
  SERIAL_PROTOCOLPGM(" Z:");
  SERIAL_PROTOCOL(current_position[Z_AXIS]);
  SERIAL_PROTOCOLPGM(" E:");
  SERIAL_PROTOCOL(current_position[E_AXIS]);
  stepper.report_positions();
  #if IS_SCARA
    SERIAL_PROTOCOLPAIR("SCARA Theta:", stepper.get_axis_position_degrees(A_AXIS));
    SERIAL_PROTOCOLLNPAIR("   Psi+Theta:", stepper.get_axis_position_degrees(B_AXIS));
    SERIAL_EOL();
  #endif
}
#ifdef M114_DETAIL
  void report_xyze(const float pos[XYZE], const uint8_t n = 4, const uint8_t precision = 3) {
    char str[12];
    for (uint8_t i = 0; i < n; i++) {
      SERIAL_CHAR(' ');
      SERIAL_CHAR(axis_codes[i]);
      SERIAL_CHAR(':');
      SERIAL_PROTOCOL(dtostrf(pos[i], 8, precision, str));
    }
    SERIAL_EOL();
  }
  inline void report_xyz(const float pos[XYZ]) { report_xyze(pos, 3); }
  void report_current_position_detail() {
    SERIAL_PROTOCOLPGM("\nLogical:");
    report_xyze(current_position);
    SERIAL_PROTOCOLPGM("Raw:    ");
    const float raw[XYZ] = { RAW_X_POSITION(current_position[X_AXIS]), RAW_Y_POSITION(current_position[Y_AXIS]), RAW_Z_POSITION(current_position[Z_AXIS]) };
    report_xyz(raw);
    SERIAL_PROTOCOLPGM("Leveled:");
    float leveled[XYZ] = { current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] };
    planner.apply_leveling(leveled);
    report_xyz(leveled);
    SERIAL_PROTOCOLPGM("UnLevel:");
    float unleveled[XYZ] = { leveled[X_AXIS], leveled[Y_AXIS], leveled[Z_AXIS] };
    planner.unapply_leveling(unleveled);
    report_xyz(unleveled);
    SERIAL_PROTOCOLPGM("Leveled:");
    planner.apply_leveling(unleveled);
    report_xyz(unleveled);
    SERIAL_PROTOCOLPGM("UnLevel:");
    planner.unapply_leveling(unleveled);
    report_xyz(unleveled);
    #if IS_KINEMATIC
      #if IS_SCARA
        SERIAL_PROTOCOLPGM("ScaraK: ");
      #else
        SERIAL_PROTOCOLPGM("DeltaK: ");
      #endif
      inverse_kinematics(leveled);  
      report_xyz(delta);
    #endif
    SERIAL_PROTOCOLPGM("Stepper:");
    const float step_count[XYZE] = { stepper.position(X_AXIS), stepper.position(Y_AXIS), stepper.position(Z_AXIS), stepper.position(E_AXIS) };
    report_xyze(step_count, 4, 0);
    #if IS_SCARA
      const float deg[XYZ] = {
        stepper.get_axis_position_degrees(A_AXIS),
        stepper.get_axis_position_degrees(B_AXIS)
      };
      SERIAL_PROTOCOLPGM("Degrees:");
      report_xyze(deg, 2);
    #endif
    SERIAL_PROTOCOLPGM("FromStp:");
    get_cartesian_from_steppers();  
    const float from_steppers[XYZE] = { cartes[X_AXIS], cartes[Y_AXIS], cartes[Z_AXIS], stepper.get_axis_position_mm(E_AXIS) };
    report_xyze(from_steppers);
    const float diff[XYZE] = {
      from_steppers[X_AXIS] - leveled[X_AXIS],
      from_steppers[Y_AXIS] - leveled[Y_AXIS],
      from_steppers[Z_AXIS] - leveled[Z_AXIS],
      from_steppers[E_AXIS] - current_position[E_AXIS]
    };
    SERIAL_PROTOCOLPGM("Differ: ");
    report_xyze(diff);
  }
#endif 
inline void gcode_M114() {
  #ifdef M114_DETAIL
    if (parser.seen('D')) {
      report_current_position_detail();
      return;
    }
  #endif
  stepper.synchronize();
  report_current_position();
}
inline void gcode_M115() {
  SERIAL_PROTOCOLLNPGM(MSG_M115_REPORT);
  #if ENABLED(EXTENDED_CAPABILITIES_REPORT)
    #if ENABLED(EEPROM_SETTINGS)
      SERIAL_PROTOCOLLNPGM("Cap:EEPROM:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:EEPROM:0");
    #endif
    #if ENABLED(AUTO_REPORT_TEMPERATURES)
      SERIAL_PROTOCOLLNPGM("Cap:AUTOREPORT_TEMP:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:AUTOREPORT_TEMP:0");
    #endif
    SERIAL_PROTOCOLLNPGM("Cap:PROGRESS:0");
    SERIAL_PROTOCOLLNPGM("Cap:PRINT_JOB:1");
    #if HAS_ABL
      SERIAL_PROTOCOLLNPGM("Cap:AUTOLEVEL:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:AUTOLEVEL:0");
    #endif
    #if HAS_BED_PROBE
      SERIAL_PROTOCOLLNPGM("Cap:Z_PROBE:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:Z_PROBE:0");
    #endif
    #if HAS_LEVELING
      SERIAL_PROTOCOLLNPGM("Cap:LEVELING_DATA:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:LEVELING_DATA:0");
    #endif
    #if HAS_POWER_SWITCH
      SERIAL_PROTOCOLLNPGM("Cap:SOFTWARE_POWER:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:SOFTWARE_POWER:0");
    #endif
    #if HAS_CASE_LIGHT
      SERIAL_PROTOCOLLNPGM("Cap:TOGGLE_LIGHTS:1");
      if (USEABLE_HARDWARE_PWM(CASE_LIGHT_PIN)) {
        SERIAL_PROTOCOLLNPGM("Cap:CASE_LIGHT_BRIGHTNESS:1");
      }
      else
        SERIAL_PROTOCOLLNPGM("Cap:CASE_LIGHT_BRIGHTNESS:0");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:TOGGLE_LIGHTS:0");
      SERIAL_PROTOCOLLNPGM("Cap:CASE_LIGHT_BRIGHTNESS:0");
    #endif
    #if ENABLED(EMERGENCY_PARSER)
      SERIAL_PROTOCOLLNPGM("Cap:EMERGENCY_PARSER:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:EMERGENCY_PARSER:0");
    #endif
  #endif 
}
inline void gcode_M117() { lcd_setstatus(parser.string_arg); }
inline void gcode_M118() {
  if (parser.boolval('E')) SERIAL_ECHO_START();
  if (parser.boolval('A')) SERIAL_ECHOPGM("// ");
  SERIAL_ECHOLN(parser.string_arg);
}
/**
 * M119: Output endstop states to serial output
 */
inline void gcode_M119() { endstops.M119(); }
/**
 * M120: Enable endstops and set non-homing endstop state to "enabled"
 */
inline void gcode_M120() { endstops.enable_globally(true); }
/**
 * M121: Disable endstops and set non-homing endstop state to "disabled"
 */
inline void gcode_M121() { endstops.enable_globally(false); }
#if ENABLED(PARK_HEAD_ON_PAUSE)
  /**
   * M125: Store current position and move to filament change position.
   *       Called on pause (by M25) to prevent material leaking onto the
   *       object. On resume (M24) the head will be moved back and the
   *       print will resume.
   *
   *       If Marlin is compiled without SD Card support, M125 can be
   *       used directly to pause the print and move to park position,
   *       resuming with a button click or M108.
   *
   *    L = override retract length
   *    X = override X
   *    Y = override Y
   *    Z = override Z raise
   */
  inline void gcode_M125() {
    // Initial retract before move to filament change position
    const float retract = parser.seen('L') ? parser.value_axis_units(E_AXIS) : 0
      #if defined(PAUSE_PARK_RETRACT_LENGTH) && PAUSE_PARK_RETRACT_LENGTH > 0
        - (PAUSE_PARK_RETRACT_LENGTH)
      #endif
    ;
    // Lift Z axis
    const float z_lift = parser.linearval('Z')
      #if PAUSE_PARK_Z_ADD > 0
        + PAUSE_PARK_Z_ADD
      #endif
    ;
    // Move XY axes to filament change position or given position
    const float x_pos = parser.linearval('X')
      #ifdef PAUSE_PARK_X_POS
        + PAUSE_PARK_X_POS
      #endif
      #if HOTENDS > 1 && DISABLED(DUAL_X_CARRIAGE)
        + (active_extruder ? hotend_offset[X_AXIS][active_extruder] : 0)
      #endif
    ;
    const float y_pos = parser.linearval('Y')
      #ifdef PAUSE_PARK_Y_POS
        + PAUSE_PARK_Y_POS
      #endif
      #if HOTENDS > 1 && DISABLED(DUAL_X_CARRIAGE)
        + (active_extruder ? hotend_offset[Y_AXIS][active_extruder] : 0)
      #endif
    ;
    const bool job_running = print_job_timer.isRunning();
    if (pause_print(retract, z_lift, x_pos, y_pos)) {
      #if DISABLED(SDSUPPORT)
        // Wait for lcd click or M108
        wait_for_filament_reload();
        // Return to print position and continue
        resume_print();
        if (job_running) print_job_timer.start();
      #endif
    }
  }
#endif // PARK_HEAD_ON_PAUSE
#if HAS_COLOR_LEDS
  /**
   * M150: Set Status LED Color - Use R-U-B-W for R-G-B-W
   *
   * Always sets all 3 or 4 components. If a component is left out, set to 0.
   *
   * Examples:
   *
   *   M150 R255       ; Turn LED red
   *   M150 R255 U127  ; Turn LED orange (PWM only)
   *   M150            ; Turn LED off
   *   M150 R U B      ; Turn LED white
   *   M150 W          ; Turn LED white using a white LED
   *
   */
  inline void gcode_M150() {
    set_led_color(
      parser.seen('R') ? (parser.has_value() ? parser.value_byte() : 255) : 0,
      parser.seen('U') ? (parser.has_value() ? parser.value_byte() : 255) : 0,
      parser.seen('B') ? (parser.has_value() ? parser.value_byte() : 255) : 0
      #if ENABLED(RGBW_LED)
        , parser.seen('W') ? (parser.has_value() ? parser.value_byte() : 255) : 0
      #endif
    );
  }
#endif // HAS_COLOR_LEDS
/**
 * M200: Set filament diameter and set E axis units to cubic units
 *
 *    T<extruder> - Optional extruder number. Current extruder if omitted.
 *    D<linear> - Diameter of the filament. Use "D0" to switch back to linear units on the E axis.
 */
inline void gcode_M200() {
  if (get_target_extruder_from_command(200)) return;
  if (parser.seen('D')) {
    // setting any extruder filament size disables volumetric on the assumption that
    // slicers either generate in extruder values as cubic mm or as as filament feeds
    // for all extruders
    volumetric_enabled = (parser.value_linear_units() != 0.0);
    if (volumetric_enabled) {
      filament_size[target_extruder] = parser.value_linear_units();
      // make sure all extruders have some sane value for the filament size
      for (uint8_t i = 0; i < COUNT(filament_size); i++)
        if (! filament_size[i]) filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
    }
  }
  calculate_volumetric_multipliers();
}
/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M201() {
  GET_TARGET_EXTRUDER(201);
  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      planner.max_acceleration_mm_per_s2[a] = parser.value_axis_units((AxisEnum)a);
    }
  }
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  planner.reset_acceleration_rates();
}
#if 0 // Not used for Sprinter/grbl gen6
  inline void gcode_M202() {
    LOOP_XYZE(i) {
      if (parser.seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = parser.value_axis_units((AxisEnum)i) * planner.axis_steps_per_mm[i];
    }
  }
#endif
/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in units/sec
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M203() {
  GET_TARGET_EXTRUDER(203);
  LOOP_XYZE(i)
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      planner.max_feedrate_mm_s[a] = parser.value_axis_units((AxisEnum)a);
    }
}
/**
 * M204: Set Accelerations in units/sec^2 (M204 P1200 R3000 T3000)
 *
 *    P = Printing moves
 *    R = Retract only (no X, Y, Z) moves
 *    T = Travel (non printing) moves
 *
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
 */
inline void gcode_M204() {
  if (parser.seen('S')) {  // Kept for legacy compatibility. Should NOT BE USED for new developments.
    planner.travel_acceleration = planner.acceleration = parser.value_linear_units();
    SERIAL_ECHOLNPAIR("Setting Print and Travel Acceleration: ", planner.acceleration);
  }
  if (parser.seen('P')) {
    planner.acceleration = parser.value_linear_units();
    SERIAL_ECHOLNPAIR("Setting Print Acceleration: ", planner.acceleration);
  }
  if (parser.seen('R')) {
    planner.retract_acceleration = parser.value_linear_units();
    SERIAL_ECHOLNPAIR("Setting Retract Acceleration: ", planner.retract_acceleration);
  }
  if (parser.seen('T')) {
    planner.travel_acceleration = parser.value_linear_units();
    SERIAL_ECHOLNPAIR("Setting Travel Acceleration: ", planner.travel_acceleration);
  }
}
/**
 * M205: Set Advanced Settings
 *
 *    S = Min Feed Rate (units/s)
 *    T = Min Travel Feed Rate (units/s)
 *    B = Min Segment Time (¦Ìs)
 *    X = Max X Jerk (units/sec^2)
 *    Y = Max Y Jerk (units/sec^2)
 *    Z = Max Z Jerk (units/sec^2)
 *    E = Max E Jerk (units/sec^2)
 */
inline void gcode_M205() {
  if (parser.seen('S')) planner.min_feedrate_mm_s = parser.value_linear_units();
  if (parser.seen('T')) planner.min_travel_feedrate_mm_s = parser.value_linear_units();
  if (parser.seen('B')) planner.min_segment_time = parser.value_millis();
  if (parser.seen('X')) planner.max_jerk[X_AXIS] = parser.value_linear_units();
  if (parser.seen('Y')) planner.max_jerk[Y_AXIS] = parser.value_linear_units();
  if (parser.seen('Z')) planner.max_jerk[Z_AXIS] = parser.value_linear_units();
  if (parser.seen('E')) planner.max_jerk[E_AXIS] = parser.value_linear_units();
}
#if HAS_M206_COMMAND
  /**
   * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
   *
   * *** @thinkyhead: I recommend deprecating M206 for SCARA in favor of M665.
   * ***              M206 for SCARA will remain enabled in 1.1.x for compatibility.
   * ***              In the next 1.2 release, it will simply be disabled by default.
   */
  inline void gcode_M206() {
    LOOP_XYZ(i)
      if (parser.seen(axis_codes[i]))
        set_home_offset((AxisEnum)i, parser.value_linear_units());
    #if ENABLED(MORGAN_SCARA)
      if (parser.seen('T')) set_home_offset(A_AXIS, parser.value_linear_units()); 
      if (parser.seen('P')) set_home_offset(B_AXIS, parser.value_linear_units()); 
    #endif
    SYNC_PLAN_POSITION_KINEMATIC();
    report_current_position();
  }
#endif 
#if ENABLED(DELTA)
  inline void gcode_M665() {
    if (parser.seen('H')) {
      home_offset[Z_AXIS] = parser.value_linear_units() - DELTA_HEIGHT;
      update_software_endstops(Z_AXIS);
    }
    if (parser.seen('L')) delta_diagonal_rod             = parser.value_linear_units();
    if (parser.seen('R')) delta_radius                   = parser.value_linear_units();
    if (parser.seen('S')) delta_segments_per_second      = parser.value_float();
    if (parser.seen('B')) delta_calibration_radius       = parser.value_float();
    if (parser.seen('X')) delta_tower_angle_trim[A_AXIS] = parser.value_float();
    if (parser.seen('Y')) delta_tower_angle_trim[B_AXIS] = parser.value_float();
    if (parser.seen('Z')) { 
      delta_tower_angle_trim[A_AXIS] -= parser.value_float();
      delta_tower_angle_trim[B_AXIS] -= parser.value_float();
    }
    recalc_delta_settings(delta_radius, delta_diagonal_rod);
  }
  inline void gcode_M666() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOLNPGM(">>> gcode_M666");
      }
    #endif
    LOOP_XYZ(i) {
      if (parser.seen(axis_codes[i])) {
        endstop_adj[i] = parser.value_linear_units();
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_ECHOPAIR("endstop_adj[", axis_codes[i]);
            SERIAL_ECHOLNPAIR("] = ", endstop_adj[i]);
          }
        #endif
      }
    }
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOLNPGM("<<< gcode_M666");
      }
    #endif
    const float z_temp = MAX3(endstop_adj[A_AXIS], endstop_adj[B_AXIS], endstop_adj[C_AXIS]);
    home_offset[Z_AXIS] -= z_temp;
    LOOP_XYZ(i) endstop_adj[i] -= z_temp;
  }
#elif IS_SCARA
  inline void gcode_M665() {
    if (parser.seen('S')) delta_segments_per_second = parser.value_float();
    const bool hasA = parser.seen('A'), hasP = parser.seen('P'), hasX = parser.seen('X');
    const uint8_t sumAPX = hasA + hasP + hasX;
    if (sumAPX == 1)
      home_offset[A_AXIS] = parser.value_float();
    else if (sumAPX > 1) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM("Only one of A, P, or X is allowed.");
      return;
    }
    const bool hasB = parser.seen('B'), hasT = parser.seen('T'), hasY = parser.seen('Y');
    const uint8_t sumBTY = hasB + hasT + hasY;
    if (sumBTY == 1)
      home_offset[B_AXIS] = parser.value_float();
    else if (sumBTY > 1) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM("Only one of B, T, or Y is allowed.");
      return;
    }
  }
#elif ENABLED(Z_DUAL_ENDSTOPS) 
  inline void gcode_M666() {
    if (parser.seen('Z')) z_endstop_adj = parser.value_linear_units();
    SERIAL_ECHOLNPAIR("Z Endstop Adjustment set to (mm):", z_endstop_adj);
  }
#endif 
#if ENABLED(FWRETRACT)
  inline void gcode_M207() {
    if (parser.seen('S')) retract_length = parser.value_axis_units(E_AXIS);
    if (parser.seen('F')) retract_feedrate_mm_s = MMM_TO_MMS(parser.value_axis_units(E_AXIS));
    if (parser.seen('Z')) retract_zlift = parser.value_linear_units();
    #if EXTRUDERS > 1
      if (parser.seen('W')) retract_length_swap = parser.value_axis_units(E_AXIS);
    #endif
  }
  inline void gcode_M208() {
    if (parser.seen('S')) retract_recover_length = parser.value_axis_units(E_AXIS);
    if (parser.seen('F')) retract_recover_feedrate_mm_s = MMM_TO_MMS(parser.value_axis_units(E_AXIS));
    #if EXTRUDERS > 1
      if (parser.seen('W')) retract_recover_length_swap = parser.value_axis_units(E_AXIS);
    #endif
  }
  inline void gcode_M209() {
    if (parser.seen('S')) {
      autoretract_enabled = parser.value_bool();
      for (int i = 0; i < EXTRUDERS; i++) retracted[i] = false;
    }
  }
#endif 
inline void gcode_M211() {
  SERIAL_ECHO_START();
  #if HAS_SOFTWARE_ENDSTOPS
    if (parser.seen('S')) soft_endstops_enabled = parser.value_bool();
    SERIAL_ECHOPGM(MSG_SOFT_ENDSTOPS);
    serialprintPGM(soft_endstops_enabled ? PSTR(MSG_ON) : PSTR(MSG_OFF));
  #else
    SERIAL_ECHOPGM(MSG_SOFT_ENDSTOPS);
    SERIAL_ECHOPGM(MSG_OFF);
  #endif
  SERIAL_ECHOPGM(MSG_SOFT_MIN);
  SERIAL_ECHOPAIR(    MSG_X, soft_endstop_min[X_AXIS]);
  SERIAL_ECHOPAIR(" " MSG_Y, soft_endstop_min[Y_AXIS]);
  SERIAL_ECHOPAIR(" " MSG_Z, soft_endstop_min[Z_AXIS]);
  SERIAL_ECHOPGM(MSG_SOFT_MAX);
  SERIAL_ECHOPAIR(    MSG_X, soft_endstop_max[X_AXIS]);
  SERIAL_ECHOPAIR(" " MSG_Y, soft_endstop_max[Y_AXIS]);
  SERIAL_ECHOLNPAIR(" " MSG_Z, soft_endstop_max[Z_AXIS]);
}
#if HOTENDS > 1
  inline void gcode_M218() {
    if (get_target_extruder_from_command(218) || target_extruder == 0) return;
    if (parser.seenval('X')) hotend_offset[X_AXIS][target_extruder] = parser.value_linear_units();
    if (parser.seenval('Y')) hotend_offset[Y_AXIS][target_extruder] = parser.value_linear_units();
    #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(SWITCHING_NOZZLE)
      if (parser.seenval('Z')) hotend_offset[Z_AXIS][target_extruder] = parser.value_linear_units();
    #endif
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
    HOTEND_LOOP() {
      SERIAL_CHAR(' ');
      SERIAL_ECHO(hotend_offset[X_AXIS][e]);
      SERIAL_CHAR(',');
      SERIAL_ECHO(hotend_offset[Y_AXIS][e]);
      #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(SWITCHING_NOZZLE)
        SERIAL_CHAR(',');
        SERIAL_ECHO(hotend_offset[Z_AXIS][e]);
      #endif
    }
    SERIAL_EOL();
  }
#endif 
inline void gcode_M220() {
  if (parser.seenval('S')) feedrate_percentage = parser.value_int();
}
inline void gcode_M221() {
  if (get_target_extruder_from_command(221)) return;
  if (parser.seenval('S'))
    flow_percentage[target_extruder] = parser.value_int();
}
inline void gcode_M226() {
  if (parser.seen('P')) {
    const int pin_number = parser.value_int(),
              pin_state = parser.intval('S', -1); 
    if (WITHIN(pin_state, -1, 1) && pin_number > -1 && !pin_is_protected(pin_number)) {
      int target = LOW;
      stepper.synchronize();
      pinMode(pin_number, INPUT);
      switch (pin_state) {
        case 1:
          target = HIGH;
          break;
        case 0:
          target = LOW;
          break;
        case -1:
          target = !digitalRead(pin_number);
          break;
      }
      while (digitalRead(pin_number) != target) idle();
    } 
  } 
}
#if ENABLED(EXPERIMENTAL_I2CBUS)
  inline void gcode_M260() {
    if (parser.seen('A')) i2c.address(parser.value_byte());
    if (parser.seen('B')) i2c.addbyte(parser.value_byte());
    if (parser.seen('S')) i2c.send();
    else if (parser.seen('R')) i2c.reset();
  }
  inline void gcode_M261() {
    if (parser.seen('A')) i2c.address(parser.value_byte());
    uint8_t bytes = parser.byteval('B', 1);
    if (i2c.addr && bytes && bytes <= TWIBUS_BUFFER_SIZE) {
      i2c.relay(bytes);
    }
    else {
      SERIAL_ERROR_START();
      SERIAL_ERRORLN("Bad i2c request");
    }
  }
#endif 
#if HAS_SERVOS
  inline void gcode_M280() {
    if (!parser.seen('P')) return;
    const int servo_index = parser.value_int();
    if (WITHIN(servo_index, 0, NUM_SERVOS - 1)) {
      if (parser.seen('S'))
        MOVE_SERVO(servo_index, parser.value_int());
      else {
        SERIAL_ECHO_START();
        SERIAL_ECHOPAIR(" Servo ", servo_index);
        SERIAL_ECHOLNPAIR(": ", servo[servo_index].read());
      }
    }
    else {
      SERIAL_ERROR_START();
      SERIAL_ECHOPAIR("Servo ", servo_index);
      SERIAL_ECHOLNPGM(" out of range");
    }
  }
#endif 
#if HAS_BUZZER
  inline void gcode_M300() {
    uint16_t const frequency = parser.ushortval('S', 260);
    uint16_t duration = parser.ushortval('P', 1000);
    NOMORE(duration, 5000);
    BUZZ(duration, frequency);
  }
#endif 
#if ENABLED(PIDTEMP)
  inline void gcode_M301() {
    const uint8_t e = parser.byteval('E'); 
    if (e < HOTENDS) { 
      if (parser.seen('P')) PID_PARAM(Kp, e) = parser.value_float();
      if (parser.seen('I')) PID_PARAM(Ki, e) = scalePID_i(parser.value_float());
      if (parser.seen('D')) PID_PARAM(Kd, e) = scalePID_d(parser.value_float());
      #if ENABLED(PID_EXTRUSION_SCALING)
        if (parser.seen('C')) PID_PARAM(Kc, e) = parser.value_float();
        if (parser.seen('L')) lpq_len = parser.value_float();
        NOMORE(lpq_len, LPQ_MAX_LEN);
      #endif
      thermalManager.updatePID();
      SERIAL_ECHO_START();
      #if ENABLED(PID_PARAMS_PER_HOTEND)
        SERIAL_ECHOPAIR(" e:", e); 
      #endif 
      SERIAL_ECHOPAIR(" p:", PID_PARAM(Kp, e));
      SERIAL_ECHOPAIR(" i:", unscalePID_i(PID_PARAM(Ki, e)));
      SERIAL_ECHOPAIR(" d:", unscalePID_d(PID_PARAM(Kd, e)));
      #if ENABLED(PID_EXTRUSION_SCALING)
        SERIAL_ECHOPAIR(" c:", PID_PARAM(Kc, e));
      #endif
      SERIAL_EOL();
    }
    else {
      SERIAL_ERROR_START();
      SERIAL_ERRORLN(MSG_INVALID_EXTRUDER);
    }
  }
#endif 
#if ENABLED(PIDTEMPBED)
  inline void gcode_M304() {
    if (parser.seen('P')) thermalManager.bedKp = parser.value_float();
    if (parser.seen('I')) thermalManager.bedKi = scalePID_i(parser.value_float());
    if (parser.seen('D')) thermalManager.bedKd = scalePID_d(parser.value_float());
    thermalManager.updatePID();
    SERIAL_ECHO_START();
    SERIAL_ECHOPAIR(" p:", thermalManager.bedKp);
    SERIAL_ECHOPAIR(" i:", unscalePID_i(thermalManager.bedKi));
    SERIAL_ECHOLNPAIR(" d:", unscalePID_d(thermalManager.bedKd));
  }
#endif 
#if defined(CHDK) || HAS_PHOTOGRAPH
  inline void gcode_M240() {
    #ifdef CHDK
      OUT_WRITE(CHDK, HIGH);
      chdkHigh = millis();
      chdkActive = true;
    #elif HAS_PHOTOGRAPH
      const uint8_t NUM_PULSES = 16;
      const float PULSE_LENGTH = 0.01524;
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }
      delay(7.33);
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }
    #endif 
  }
#endif 
#if HAS_LCD_CONTRAST
  inline void gcode_M250() {
    if (parser.seen('C')) set_lcd_contrast(parser.value_int());
    SERIAL_PROTOCOLPGM("lcd contrast value: ");
    SERIAL_PROTOCOL(lcd_contrast);
    SERIAL_EOL();
  }
#endif 
#if ENABLED(PREVENT_COLD_EXTRUSION)
  inline void gcode_M302() {
    const bool seen_S = parser.seen('S');
    if (seen_S) {
      thermalManager.extrude_min_temp = parser.value_celsius();
      thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0);
    }
    if (parser.seen('P'))
      thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0) || parser.value_bool();
    else if (!seen_S) {
      SERIAL_ECHO_START();
      SERIAL_ECHOPAIR("Cold extrudes are ", (thermalManager.allow_cold_extrude ? "en" : "dis"));
      SERIAL_ECHOPAIR("abled (min temp ", thermalManager.extrude_min_temp);
      SERIAL_ECHOLNPGM("C)");
    }
  }
#endif 
inline void gcode_M303() {
  #if HAS_PID_HEATING
    const int e = parser.intval('E'), c = parser.intval('C', 5);
    const bool u = parser.boolval('U');
    int16_t temp = parser.celsiusval('S', e < 0 ? 70 : 150);
    if (WITHIN(e, 0, HOTENDS - 1))
      target_extruder = e;
    KEEPALIVE_STATE(NOT_BUSY); 
    thermalManager.PID_autotune(temp, e, c, u);
    KEEPALIVE_STATE(IN_HANDLER);
  #else
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_ERR_M303_DISABLED);
  #endif
}
#if ENABLED(MORGAN_SCARA)
  bool SCARA_move_to_cal(uint8_t delta_a, uint8_t delta_b) {
    if (IsRunning()) {
      forward_kinematics_SCARA(delta_a, delta_b);
      destination[X_AXIS] = LOGICAL_X_POSITION(cartes[X_AXIS]);
      destination[Y_AXIS] = LOGICAL_Y_POSITION(cartes[Y_AXIS]);
      destination[Z_AXIS] = current_position[Z_AXIS];
      prepare_move_to_destination();
      return true;
    }
    return false;
  }
  inline bool gcode_M360() {
    SERIAL_ECHOLNPGM(" Cal: Theta 0");
    return SCARA_move_to_cal(0, 120);
  }
  inline bool gcode_M361() {
    SERIAL_ECHOLNPGM(" Cal: Theta 90");
    return SCARA_move_to_cal(90, 130);
  }
  inline bool gcode_M362() {
    SERIAL_ECHOLNPGM(" Cal: Psi 0");
    return SCARA_move_to_cal(60, 180);
  }
  inline bool gcode_M363() {
    SERIAL_ECHOLNPGM(" Cal: Psi 90");
    return SCARA_move_to_cal(50, 90);
  }
  inline bool gcode_M364() {
    SERIAL_ECHOLNPGM(" Cal: Theta-Psi 90");
    return SCARA_move_to_cal(45, 135);
  }
#endif 
#if ENABLED(EXT_SOLENOID)
  void enable_solenoid(const uint8_t num) {
    switch (num) {
      case 0:
        OUT_WRITE(SOL0_PIN, HIGH);
        break;
        #if HAS_SOLENOID_1 && EXTRUDERS > 1
          case 1:
            OUT_WRITE(SOL1_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_2 && EXTRUDERS > 2
          case 2:
            OUT_WRITE(SOL2_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_3 && EXTRUDERS > 3
          case 3:
            OUT_WRITE(SOL3_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_4 && EXTRUDERS > 4
          case 4:
            OUT_WRITE(SOL4_PIN, HIGH);
            break;
        #endif
      default:
        SERIAL_ECHO_START();
        SERIAL_ECHOLNPGM(MSG_INVALID_SOLENOID);
        break;
    }
  }
  void enable_solenoid_on_active_extruder() { enable_solenoid(active_extruder); }
  void disable_all_solenoids() {
    OUT_WRITE(SOL0_PIN, LOW);
    #if HAS_SOLENOID_1 && EXTRUDERS > 1
      OUT_WRITE(SOL1_PIN, LOW);
    #endif
    #if HAS_SOLENOID_2 && EXTRUDERS > 2
      OUT_WRITE(SOL2_PIN, LOW);
    #endif
    #if HAS_SOLENOID_3 && EXTRUDERS > 3
      OUT_WRITE(SOL3_PIN, LOW);
    #endif
    #if HAS_SOLENOID_4 && EXTRUDERS > 4
      OUT_WRITE(SOL4_PIN, LOW);
    #endif
  }
  inline void gcode_M380() { enable_solenoid_on_active_extruder(); }
  inline void gcode_M381() { disable_all_solenoids(); }
#endif 
inline void gcode_M400() { stepper.synchronize(); }
#if HAS_BED_PROBE
  inline void gcode_M401() { DEPLOY_PROBE(); }
  inline void gcode_M402() { STOW_PROBE(); }
#endif 
#if ENABLED(FILAMENT_WIDTH_SENSOR)
  inline void gcode_M404() {
    if (parser.seen('W')) {
      filament_width_nominal = parser.value_linear_units();
    }
    else {
      SERIAL_PROTOCOLPGM("Filament dia (nominal mm):");
      SERIAL_PROTOCOLLN(filament_width_nominal);
    }
  }
  inline void gcode_M405() {
    if (parser.seen('D')) {
      meas_delay_cm = parser.value_byte();
      NOMORE(meas_delay_cm, MAX_MEASUREMENT_DELAY);
    }
    if (filwidth_delay_index[1] == -1) { 
      const uint8_t temp_ratio = thermalManager.widthFil_to_size_ratio() - 100; 
      for (uint8_t i = 0; i < COUNT(measurement_delay); ++i)
        measurement_delay[i] = temp_ratio;
      filwidth_delay_index[0] = filwidth_delay_index[1] = 0;
    }
    filament_sensor = true;
  }
  inline void gcode_M406() { filament_sensor = false; }
  inline void gcode_M407() {
    SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
    SERIAL_PROTOCOLLN(filament_width_meas);
  }
#endif 
void quickstop_stepper() {
  stepper.quick_stop();
  stepper.synchronize();
  set_current_from_steppers_for_axis(ALL_AXES);
  SYNC_PLAN_POSITION_KINEMATIC();
}
#if HAS_LEVELING
  inline void gcode_M420() {
    #if ENABLED(AUTO_BED_LEVELING_UBL)
      if (parser.seen('L')) {
        #if ENABLED(EEPROM_SETTINGS)
          const int8_t storage_slot = parser.has_value() ? parser.value_int() : ubl.state.storage_slot;
          const int16_t a = settings.calc_num_meshes();
          if (!a) {
            SERIAL_PROTOCOLLNPGM("?EEPROM storage not available.");
            return;
          }
          if (!WITHIN(storage_slot, 0, a - 1)) {
            SERIAL_PROTOCOLLNPGM("?Invalid storage slot.");
            SERIAL_PROTOCOLLNPAIR("?Use 0 to ", a - 1);
            return;
          }
          settings.load_mesh(storage_slot);
          ubl.state.storage_slot = storage_slot;
        #else
          SERIAL_PROTOCOLLNPGM("?EEPROM storage not available.");
          return;
        #endif
      }
      if (parser.seen('L') || parser.seen('V')) {
        ubl.display_map(0);  
        SERIAL_ECHOLNPAIR("UBL_MESH_VALID = ", UBL_MESH_VALID);
        SERIAL_ECHOLNPAIR("ubl.state.storage_slot = ", ubl.state.storage_slot);
      }
    #endif 
    if (parser.seen('V')) {
      #if ABL_PLANAR
        planner.bed_level_matrix.debug(PSTR("Bed Level Correction Matrix:"));
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (leveling_is_valid()) {
          print_bilinear_leveling_grid();
          #if ENABLED(ABL_BILINEAR_SUBDIVISION)
            bed_level_virt_print();
          #endif
        }
      #elif ENABLED(MESH_BED_LEVELING)
        if (leveling_is_valid()) {
          SERIAL_ECHOLNPGM("Mesh Bed Level data:");
          mbl_mesh_report();
        }
      #endif
    }
    const bool to_enable = parser.boolval('S');
    if (parser.seen('S'))
      set_bed_leveling_enabled(to_enable);
    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      if (parser.seen('Z')) set_z_fade_height(parser.value_linear_units());
    #endif
    const bool new_status = leveling_is_active();
    if (to_enable && !new_status) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM(MSG_ERR_M420_FAILED);
    }
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPAIR("Bed Leveling ", new_status ? MSG_ON : MSG_OFF);
    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      SERIAL_ECHO_START();
      SERIAL_ECHOPGM("Fade Height ");
      if (planner.z_fade_height > 0.0)
        SERIAL_ECHOLN(planner.z_fade_height);
      else
        SERIAL_ECHOLNPGM(MSG_OFF);
    #endif
  }
#endif
#if ENABLED(MESH_BED_LEVELING)
  inline void gcode_M421() {
    const bool hasX = parser.seen('X'), hasI = parser.seen('I');
    const int8_t ix = hasI ? parser.value_int() : hasX ? mbl.probe_index_x(RAW_X_POSITION(parser.value_linear_units())) : -1;
    const bool hasY = parser.seen('Y'), hasJ = parser.seen('J');
    const int8_t iy = hasJ ? parser.value_int() : hasY ? mbl.probe_index_y(RAW_Y_POSITION(parser.value_linear_units())) : -1;
    const bool hasZ = parser.seen('Z'), hasQ = !hasZ && parser.seen('Q');
    if (int(hasI && hasJ) + int(hasX && hasY) != 1 || !(hasZ || hasQ)) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM(MSG_ERR_M421_PARAMETERS);
    }
    else if (ix < 0 || iy < 0) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM(MSG_ERR_MESH_XY);
    }
    else
      mbl.set_z(ix, iy, parser.value_linear_units() + (hasQ ? mbl.z_values[ix][iy] : 0));
  }
#elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
  inline void gcode_M421() {
    int8_t ix = parser.intval('I', -1), iy = parser.intval('J', -1);
    const bool hasI = ix >= 0,
               hasJ = iy >= 0,
               hasZ = parser.seen('Z'),
               hasQ = !hasZ && parser.seen('Q');
    if (!hasI || !hasJ || !(hasZ || hasQ)) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM(MSG_ERR_M421_PARAMETERS);
    }
    else if (!WITHIN(ix, 0, GRID_MAX_POINTS_X - 1) || !WITHIN(iy, 0, GRID_MAX_POINTS_Y - 1)) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM(MSG_ERR_MESH_XY);
    }
    else {
      z_values[ix][iy] = parser.value_linear_units() + (hasQ ? z_values[ix][iy] : 0);
      #if ENABLED(ABL_BILINEAR_SUBDIVISION)
        bed_level_virt_interpolate();
      #endif
    }
  }
#elif ENABLED(AUTO_BED_LEVELING_UBL)
  inline void gcode_M421() {
    int8_t ix = parser.intval('I', -1), iy = parser.intval('J', -1);
    const bool hasI = ix >= 0,
               hasJ = iy >= 0,
               hasC = parser.seen('C'),
               hasZ = parser.seen('Z'),
               hasQ = !hasZ && parser.seen('Q');
    if (hasC) {
      const mesh_index_pair location = ubl.find_closest_mesh_point_of_type(REAL, current_position[X_AXIS], current_position[Y_AXIS], USE_NOZZLE_AS_REFERENCE, NULL, false);
      ix = location.x_index;
      iy = location.y_index;
    }
    if (int(hasC) + int(hasI && hasJ) != 1 || !(hasZ || hasQ)) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM(MSG_ERR_M421_PARAMETERS);
    }
    else if (!WITHIN(ix, 0, GRID_MAX_POINTS_X - 1) || !WITHIN(iy, 0, GRID_MAX_POINTS_Y - 1)) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM(MSG_ERR_MESH_XY);
    }
    else
      ubl.z_values[ix][iy] = parser.value_linear_units() + (hasQ ? ubl.z_values[ix][iy] : 0);
  }
#endif 
#if HAS_M206_COMMAND
  inline void gcode_M428() {
    bool err = false;
    LOOP_XYZ(i) {
      if (axis_homed[i]) {
        const float base = (current_position[i] > (soft_endstop_min[i] + soft_endstop_max[i]) * 0.5) ? base_home_pos((AxisEnum)i) : 0,
                    diff = base - RAW_POSITION(current_position[i], i);
        if (WITHIN(diff, -20, 20)) {
          set_home_offset((AxisEnum)i, diff);
        }
        else {
          SERIAL_ERROR_START();
          SERIAL_ERRORLNPGM(MSG_ERR_M428_TOO_FAR);
          LCD_ALERTMESSAGEPGM("Err: Too far!");
          BUZZ(200, 40);
          err = true;
          break;
        }
      }
    }
    if (!err) {
      SYNC_PLAN_POSITION_KINEMATIC();
      report_current_position();
      LCD_MESSAGEPGM(MSG_HOME_OFFSETS_APPLIED);
      BUZZ(100, 659);
      BUZZ(100, 698);
    }
  }
#endif 
inline void gcode_M500() {
  (void)settings.save();
}
inline void gcode_M501() {
  (void)settings.load();
}
inline void gcode_M502() {
  (void)settings.reset();
}
#if DISABLED(DISABLE_M503)
  inline void gcode_M503() {
    (void)settings.report(!parser.boolval('S', true));
  }
#endif
#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  inline void gcode_M540() {
    if (parser.seen('S')) stepper.abort_on_endstop_hit = parser.value_bool();
  }
#endif 
inline void gcode_M850()
{
    LOOP_XYZ(i)
    switch(i)
    {
    case X_AXIS:
        serialprintPGM(X_AXIS_STAT);
        break;
    case Y_AXIS:
        serialprintPGM(Y_AXIS_STAT);
        break;
    case Z_AXIS:
        serialprintPGM(Z_AXIS_STAT);
        SERIAL_PROTOCOLPGM(" " __DATE__ " " __TIME__);
        break;
    }
}
#if HAS_BED_PROBE
  void refresh_zprobe_zoffset(const bool no_babystep) {
    static float last_zoffset = NAN;
    if (!isnan(last_zoffset)) {
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(BABYSTEP_ZPROBE_OFFSET) || ENABLED(DELTA)
        const float diff = zprobe_zoffset - last_zoffset;
      #endif
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (diff) {
          for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
            for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
              z_values[x][y] -= diff;
        }
        #if ENABLED(ABL_BILINEAR_SUBDIVISION)
          bed_level_virt_interpolate();
        #endif
      #endif
      #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
        if (!no_babystep && leveling_is_active())
          thermalManager.babystep_axis(Z_AXIS, -LROUND(diff * planner.axis_steps_per_mm[Z_AXIS]));
      #else
        UNUSED(no_babystep);
      #endif
      #if ENABLED(DELTA) 
        home_offset[Z_AXIS] -= diff;
      #endif
    }
    last_zoffset = zprobe_zoffset;
  }
  inline void gcode_M851() {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(MSG_ZPROBE_ZOFFSET " ");
    if (parser.seen('Z')) {
      const float value = parser.value_linear_units();
      if (WITHIN(value, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) {
        zprobe_zoffset = value;
        refresh_zprobe_zoffset();
        SERIAL_ECHO(zprobe_zoffset);
      }
      else
        SERIAL_ECHOPGM(MSG_Z_MIN " " STRINGIFY(Z_PROBE_OFFSET_RANGE_MIN) " " MSG_Z_MAX " " STRINGIFY(Z_PROBE_OFFSET_RANGE_MAX));
    }
    else
    {
      SERIAL_ECHOPAIR(": ", zprobe_zoffset);
      if (planner.abl_enabled)
      {
        SERIAL_ECHOPGM("abl_enabled");
      }
      else
      {
        SERIAL_ECHOPGM("abl_disable");
      }
        }
    SERIAL_EOL();
  }
#endif 
#if ENABLED(ADVANCED_PAUSE_FEATURE)
  inline void gcode_M600() {
    #if ENABLED(HOME_BEFORE_FILAMENT_CHANGE)
      if (axis_unhomed_error()) home_all_axes();
    #endif
    const float retract = parser.seen('E') ? parser.value_axis_units(E_AXIS) : 0
      #ifdef PAUSE_PARK_RETRACT_LENGTH
        - (PAUSE_PARK_RETRACT_LENGTH)
      #endif
    ;
    const float z_lift = parser.linearval('Z', 0
      #ifdef PAUSE_PARK_Z_ADD
        + PAUSE_PARK_Z_ADD
      #endif
    );
    const float x_pos = parser.linearval('X', 0
      #ifdef PAUSE_PARK_X_POS
        + PAUSE_PARK_X_POS
      #endif
    );
    const float y_pos = parser.linearval('Y', 0
      #ifdef PAUSE_PARK_Y_POS
        + PAUSE_PARK_Y_POS
      #endif
    );
    const float unload_length = parser.seen('U') ? parser.value_axis_units(E_AXIS) : 0
      #if defined(FILAMENT_CHANGE_UNLOAD_LENGTH) && FILAMENT_CHANGE_UNLOAD_LENGTH > 0
        - (FILAMENT_CHANGE_UNLOAD_LENGTH)
      #endif
    ;
    const float load_length = parser.seen('L') ? parser.value_axis_units(E_AXIS) : 0
      #ifdef FILAMENT_CHANGE_LOAD_LENGTH
        + FILAMENT_CHANGE_LOAD_LENGTH
      #endif
    ;
    const int beep_count = parser.intval('B',
      #ifdef FILAMENT_CHANGE_NUMBER_OF_ALERT_BEEPS
        FILAMENT_CHANGE_NUMBER_OF_ALERT_BEEPS
      #else
        -1
      #endif
    );
    const bool job_running = print_job_timer.isRunning();
    if (pause_print(retract, z_lift, x_pos, y_pos, unload_length, beep_count, true)) {
      wait_for_filament_reload(beep_count);
      resume_print(load_length, ADVANCED_PAUSE_EXTRUDE_LENGTH, beep_count);
    }
    if (job_running) print_job_timer.start();
  }
#endif 
#if ENABLED(MK2_MULTIPLEXER)
  inline void select_multiplexed_stepper(const uint8_t e) {
    stepper.synchronize();
    disable_e_steppers();
    WRITE(E_MUX0_PIN, TEST(e, 0) ? HIGH : LOW);
    WRITE(E_MUX1_PIN, TEST(e, 1) ? HIGH : LOW);
    WRITE(E_MUX2_PIN, TEST(e, 2) ? HIGH : LOW);
    safe_delay(100);
  }
  inline void gcode_M702() {
    for (uint8_t s = 0; s < E_STEPPERS; s++) {
      select_multiplexed_stepper(e);
    }
    select_multiplexed_stepper(active_extruder);
    disable_e_steppers();
  }
#endif 
#if ENABLED(DUAL_X_CARRIAGE)
  inline void gcode_M605() {
    stepper.synchronize();
    if (parser.seen('S')) dual_x_carriage_mode = (DualXMode)parser.value_byte();
    switch (dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE:
      case DXC_AUTO_PARK_MODE:
        break;
      case DXC_DUPLICATION_MODE:
        if (parser.seen('X')) duplicate_extruder_x_offset = max(parser.value_linear_units(), X2_MIN_POS - x_home_pos(0));
        if (parser.seen('R')) duplicate_extruder_temp_offset = parser.value_celsius_diff();
        SERIAL_ECHO_START();
        SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
        SERIAL_CHAR(' ');
        SERIAL_ECHO(hotend_offset[X_AXIS][0]);
        SERIAL_CHAR(',');
        SERIAL_ECHO(hotend_offset[Y_AXIS][0]);
        SERIAL_CHAR(' ');
        SERIAL_ECHO(duplicate_extruder_x_offset);
        SERIAL_CHAR(',');
        SERIAL_ECHOLN(hotend_offset[Y_AXIS][1]);
        break;
      default:
        dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        break;
    }
    active_extruder_parked = false;
    extruder_duplication_enabled = false;
    delayed_move_time = 0;
  }
#elif ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
  inline void gcode_M605() {
    stepper.synchronize();
    extruder_duplication_enabled = parser.intval('S') == (int)DXC_DUPLICATION_MODE;
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPAIR(MSG_DUPLICATION_MODE, extruder_duplication_enabled ? MSG_ON : MSG_OFF);
  }
#endif 
#if ENABLED(LIN_ADVANCE)
  inline void gcode_M900() {
    stepper.synchronize();
    const float newK = parser.floatval('K', -1);
    if (newK >= 0) planner.extruder_advance_k = newK;
    float newR = parser.floatval('R', -1);
    if (newR < 0) {
      const float newD = parser.floatval('D', -1),
                  newW = parser.floatval('W', -1),
                  newH = parser.floatval('H', -1);
      if (newD >= 0 && newW >= 0 && newH >= 0)
        newR = newD ? (newW * newH) / (sq(newD * 0.5) * M_PI) : 0;
    }
    if (newR >= 0) planner.advance_ed_ratio = newR;
    SERIAL_ECHO_START();
    SERIAL_ECHOPAIR("Advance K=", planner.extruder_advance_k);
    SERIAL_ECHOPGM(" E/D=");
    const float ratio = planner.advance_ed_ratio;
    if (ratio) SERIAL_ECHO(ratio); else SERIAL_ECHOPGM("Auto");
    SERIAL_EOL();
  }
#endif 
#if ENABLED(HAVE_TMC2130)
  static void tmc2130_get_current(TMC2130Stepper &st, const char name) {
    SERIAL_CHAR(name);
    SERIAL_ECHOPGM(" axis driver current: ");
    SERIAL_ECHOLN(st.getCurrent());
  }
  static void tmc2130_set_current(TMC2130Stepper &st, const char name, const int mA) {
    st.setCurrent(mA, R_SENSE, HOLD_MULTIPLIER);
    tmc2130_get_current(st, name);
  }
  static void tmc2130_report_otpw(TMC2130Stepper &st, const char name) {
    SERIAL_CHAR(name);
    SERIAL_ECHOPGM(" axis temperature prewarn triggered: ");
    serialprintPGM(st.getOTPW() ? PSTR("true") : PSTR("false"));
    SERIAL_EOL();
  }
  static void tmc2130_clear_otpw(TMC2130Stepper &st, const char name) {
    st.clear_otpw();
    SERIAL_CHAR(name);
    SERIAL_ECHOLNPGM(" prewarn flag cleared");
  }
  static void tmc2130_get_pwmthrs(TMC2130Stepper &st, const char name, const uint16_t spmm) {
    SERIAL_CHAR(name);
    SERIAL_ECHOPGM(" stealthChop max speed set to ");
    SERIAL_ECHOLN(12650000UL * st.microsteps() / (256 * st.stealth_max_speed() * spmm));
  }
  static void tmc2130_set_pwmthrs(TMC2130Stepper &st, const char name, const int32_t thrs, const uint32_t spmm) {
    st.stealth_max_speed(12650000UL * st.microsteps() / (256 * thrs * spmm));
    tmc2130_get_pwmthrs(st, name, spmm);
  }
  static void tmc2130_get_sgt(TMC2130Stepper &st, const char name) {
    SERIAL_CHAR(name);
    SERIAL_ECHOPGM(" driver homing sensitivity set to ");
    SERIAL_ECHOLN(st.sgt());
  }
  static void tmc2130_set_sgt(TMC2130Stepper &st, const char name, const int8_t sgt_val) {
    st.sgt(sgt_val);
    tmc2130_get_sgt(st, name);
  }
  inline void gcode_M906() {
    uint16_t values[XYZE];
    LOOP_XYZE(i)
      values[i] = parser.intval(axis_codes[i]);
    #if ENABLED(X_IS_TMC2130)
      if (values[X_AXIS]) tmc2130_set_current(stepperX, 'X', values[X_AXIS]);
      else tmc2130_get_current(stepperX, 'X');
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (values[Y_AXIS]) tmc2130_set_current(stepperY, 'Y', values[Y_AXIS]);
      else tmc2130_get_current(stepperY, 'Y');
    #endif
    #if ENABLED(Z_IS_TMC2130)
      if (values[Z_AXIS]) tmc2130_set_current(stepperZ, 'Z', values[Z_AXIS]);
      else tmc2130_get_current(stepperZ, 'Z');
    #endif
    #if ENABLED(E0_IS_TMC2130)
      if (values[E_AXIS]) tmc2130_set_current(stepperE0, 'E', values[E_AXIS]);
      else tmc2130_get_current(stepperE0, 'E');
    #endif
    #if ENABLED(AUTOMATIC_CURRENT_CONTROL)
      if (parser.seen('S')) auto_current_control = parser.value_bool();
    #endif
  }
  inline void gcode_M911() {
    const bool reportX = parser.seen('X'), reportY = parser.seen('Y'), reportZ = parser.seen('Z'), reportE = parser.seen('E'),
             reportAll = (!reportX && !reportY && !reportZ && !reportE) || (reportX && reportY && reportZ && reportE);
    #if ENABLED(X_IS_TMC2130)
      if (reportX || reportAll) tmc2130_report_otpw(stepperX, 'X');
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (reportY || reportAll) tmc2130_report_otpw(stepperY, 'Y');
    #endif
    #if ENABLED(Z_IS_TMC2130)
      if (reportZ || reportAll) tmc2130_report_otpw(stepperZ, 'Z');
    #endif
    #if ENABLED(E0_IS_TMC2130)
      if (reportE || reportAll) tmc2130_report_otpw(stepperE0, 'E');
    #endif
  }
  inline void gcode_M912() {
    const bool clearX = parser.seen('X'), clearY = parser.seen('Y'), clearZ = parser.seen('Z'), clearE = parser.seen('E'),
             clearAll = (!clearX && !clearY && !clearZ && !clearE) || (clearX && clearY && clearZ && clearE);
    #if ENABLED(X_IS_TMC2130)
      if (clearX || clearAll) tmc2130_clear_otpw(stepperX, 'X');
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (clearY || clearAll) tmc2130_clear_otpw(stepperY, 'Y');
    #endif
    #if ENABLED(Z_IS_TMC2130)
      if (clearZ || clearAll) tmc2130_clear_otpw(stepperZ, 'Z');
    #endif
    #if ENABLED(E0_IS_TMC2130)
      if (clearE || clearAll) tmc2130_clear_otpw(stepperE0, 'E');
    #endif
  }
  #if ENABLED(HYBRID_THRESHOLD)
    inline void gcode_M913() {
      uint16_t values[XYZE];
      LOOP_XYZE(i)
        values[i] = parser.intval(axis_codes[i]);
      #if ENABLED(X_IS_TMC2130)
        if (values[X_AXIS]) tmc2130_set_pwmthrs(stepperX, 'X', values[X_AXIS], planner.axis_steps_per_mm[X_AXIS]);
        else tmc2130_get_pwmthrs(stepperX, 'X', planner.axis_steps_per_mm[X_AXIS]);
      #endif
      #if ENABLED(Y_IS_TMC2130)
        if (values[Y_AXIS]) tmc2130_set_pwmthrs(stepperY, 'Y', values[Y_AXIS], planner.axis_steps_per_mm[Y_AXIS]);
        else tmc2130_get_pwmthrs(stepperY, 'Y', planner.axis_steps_per_mm[Y_AXIS]);
      #endif
      #if ENABLED(Z_IS_TMC2130)
        if (values[Z_AXIS]) tmc2130_set_pwmthrs(stepperZ, 'Z', values[Z_AXIS], planner.axis_steps_per_mm[Z_AXIS]);
        else tmc2130_get_pwmthrs(stepperZ, 'Z', planner.axis_steps_per_mm[Z_AXIS]);
      #endif
      #if ENABLED(E0_IS_TMC2130)
        if (values[E_AXIS]) tmc2130_set_pwmthrs(stepperE0, 'E', values[E_AXIS], planner.axis_steps_per_mm[E_AXIS]);
        else tmc2130_get_pwmthrs(stepperE0, 'E', planner.axis_steps_per_mm[E_AXIS]);
      #endif
    }
  #endif 
  #if ENABLED(SENSORLESS_HOMING)
    inline void gcode_M914() {
      #if ENABLED(X_IS_TMC2130)
        if (parser.seen(axis_codes[X_AXIS])) tmc2130_set_sgt(stepperX, 'X', parser.value_int());
        else tmc2130_get_sgt(stepperX, 'X');
      #endif
      #if ENABLED(Y_IS_TMC2130)
        if (parser.seen(axis_codes[Y_AXIS])) tmc2130_set_sgt(stepperY, 'Y', parser.value_int());
        else tmc2130_get_sgt(stepperY, 'Y');
      #endif
    }
  #endif 
#endif 
inline void gcode_M907() {
  #if HAS_DIGIPOTSS
    LOOP_XYZE(i) if (parser.seen(axis_codes[i])) stepper.digipot_current(i, parser.value_int());
    if (parser.seen('B')) stepper.digipot_current(4, parser.value_int());
    if (parser.seen('S')) for (uint8_t i = 0; i <= 4; i++) stepper.digipot_current(i, parser.value_int());
  #elif HAS_MOTOR_CURRENT_PWM
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
      if (parser.seen('X')) stepper.digipot_current(0, parser.value_int());
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
      if (parser.seen('Z')) stepper.digipot_current(1, parser.value_int());
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
      if (parser.seen('E')) stepper.digipot_current(2, parser.value_int());
    #endif
  #endif
  #if ENABLED(DIGIPOT_I2C)
    LOOP_XYZE(i) if (parser.seen(axis_codes[i])) digipot_i2c_set_current(i, parser.value_float());
    for (uint8_t i = NUM_AXIS; i < DIGIPOT_I2C_NUM_CHANNELS; i++) if (parser.seen('B' + i - (NUM_AXIS))) digipot_i2c_set_current(i, parser.value_float());
  #endif
  #if ENABLED(DAC_STEPPER_CURRENT)
    if (parser.seen('S')) {
      const float dac_percent = parser.value_float();
      for (uint8_t i = 0; i <= 4; i++) dac_current_percent(i, dac_percent);
    }
    LOOP_XYZE(i) if (parser.seen(axis_codes[i])) dac_current_percent(i, parser.value_float());
  #endif
}
#if HAS_DIGIPOTSS || ENABLED(DAC_STEPPER_CURRENT)
  inline void gcode_M908() {
    #if HAS_DIGIPOTSS
      stepper.digitalPotWrite(
        parser.intval('P'),
        parser.intval('S')
      );
    #endif
    #ifdef DAC_STEPPER_CURRENT
      dac_current_raw(
        parser.byteval('P', -1),
        parser.ushortval('S', 0)
      );
    #endif
  }
  #if ENABLED(DAC_STEPPER_CURRENT) 
    inline void gcode_M909() { dac_print_values(); }
    inline void gcode_M910() { dac_commit_eeprom(); }
  #endif
#endif 
#if HAS_MICROSTEPS
  inline void gcode_M350() {
    if (parser.seen('S')) for (int i = 0; i <= 4; i++) stepper.microstep_mode(i, parser.value_byte());
    LOOP_XYZE(i) if (parser.seen(axis_codes[i])) stepper.microstep_mode(i, parser.value_byte());
    if (parser.seen('B')) stepper.microstep_mode(4, parser.value_byte());
    stepper.microstep_readings();
  }
  inline void gcode_M351() {
    if (parser.seenval('S')) switch (parser.value_byte()) {
      case 1:
        LOOP_XYZE(i) if (parser.seenval(axis_codes[i])) stepper.microstep_ms(i, parser.value_byte(), -1);
        if (parser.seenval('B')) stepper.microstep_ms(4, parser.value_byte(), -1);
        break;
      case 2:
        LOOP_XYZE(i) if (parser.seenval(axis_codes[i])) stepper.microstep_ms(i, -1, parser.value_byte());
        if (parser.seenval('B')) stepper.microstep_ms(4, -1, parser.value_byte());
        break;
    }
    stepper.microstep_readings();
  }
#endif 
#if HAS_CASE_LIGHT
  #ifndef INVERT_CASE_LIGHT
    #define INVERT_CASE_LIGHT false
  #endif
  int case_light_brightness;  
  bool case_light_on;
  void update_case_light() {
    pinMode(CASE_LIGHT_PIN, OUTPUT); 
    uint8_t case_light_bright = (uint8_t)case_light_brightness;
    if (case_light_on) {
      if (USEABLE_HARDWARE_PWM(CASE_LIGHT_PIN)) {
        analogWrite(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? 255 - case_light_brightness : case_light_brightness );
      }
      else digitalWrite(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? LOW : HIGH );
    }
    else digitalWrite(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? HIGH : LOW);
  }
#endif 
inline void gcode_M355() {
  #if HAS_CASE_LIGHT
    uint8_t args = 0;
    if (parser.seenval('P')) ++args, case_light_brightness = parser.value_byte();
    if (parser.seenval('S')) ++args, case_light_on = parser.value_bool();
    if (args) update_case_light();
    SERIAL_ECHO_START();
    if (!case_light_on) {
      SERIAL_ECHOLN("Case light: off");
    }
    else {
      if (!USEABLE_HARDWARE_PWM(CASE_LIGHT_PIN)) SERIAL_ECHOLN("Case light: on");
      else SERIAL_ECHOLNPAIR("Case light: ", case_light_brightness);
    }
  #else
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_ERR_M355_NONE);
  #endif 
}
#if ENABLED(MIXING_EXTRUDER)
  inline void gcode_M163() {
    const int mix_index = parser.intval('S');
    if (mix_index < MIXING_STEPPERS) {
      float mix_value = parser.floatval('P');
      NOLESS(mix_value, 0.0);
      mixing_factor[mix_index] = RECIPROCAL(mix_value);
    }
  }
  #if MIXING_VIRTUAL_TOOLS > 1
    inline void gcode_M164() {
      const int tool_index = parser.intval('S');
      if (tool_index < MIXING_VIRTUAL_TOOLS) {
        normalize_mix();
        for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
          mixing_virtual_tool_mix[tool_index][i] = mixing_factor[i];
      }
    }
  #endif
  #if ENABLED(DIRECT_MIXING_IN_G1)
    inline void gcode_M165() { gcode_get_mix(); }
  #endif
#endif 
inline void gcode_M999() {
  Running = true;
  lcd_reset_alert_level();
  if (parser.boolval('S')) return;
  FlushSerialRequestResend();
}
#if ENABLED(SWITCHING_EXTRUDER)
  #if EXTRUDERS > 3
    #define REQ_ANGLES 4
    #define _SERVO_NR (e < 2 ? SWITCHING_EXTRUDER_SERVO_NR : SWITCHING_EXTRUDER_E23_SERVO_NR)
  #else
    #define REQ_ANGLES 2
    #define _SERVO_NR SWITCHING_EXTRUDER_SERVO_NR
  #endif
  inline void move_extruder_servo(const uint8_t e) {
    constexpr int16_t angles[] = SWITCHING_EXTRUDER_SERVO_ANGLES;
    static_assert(COUNT(angles) == REQ_ANGLES, "SWITCHING_EXTRUDER_SERVO_ANGLES needs " STRINGIFY(REQ_ANGLES) " angles.");
    stepper.synchronize();
    #if EXTRUDERS & 1
      if (e < EXTRUDERS - 1)
    #endif
    {
      MOVE_SERVO(_SERVO_NR, angles[e]);
      safe_delay(500);
    }
  }
#endif 
#if ENABLED(SWITCHING_NOZZLE)
  inline void move_nozzle_servo(const uint8_t e) {
    const int16_t angles[2] = SWITCHING_NOZZLE_SERVO_ANGLES;
    stepper.synchronize();
    MOVE_SERVO(SWITCHING_NOZZLE_SERVO_NR, angles[e]);
    safe_delay(500);
  }
#endif
inline void invalid_extruder_error(const uint8_t e) {
  SERIAL_ECHO_START();
  SERIAL_CHAR('T');
  SERIAL_ECHO_F(e, DEC);
  SERIAL_CHAR(' ');
  SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
}
void tool_change(const uint8_t tmp_extruder, const float fr_mm_s, bool no_move) {
  #if ENABLED(MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1
    if (tmp_extruder >= MIXING_VIRTUAL_TOOLS)
      return invalid_extruder_error(tmp_extruder);
    for (uint8_t j = 0; j < MIXING_STEPPERS; j++)
      mixing_factor[j] = mixing_virtual_tool_mix[tmp_extruder][j];
  #else 
    if (tmp_extruder >= EXTRUDERS)
      return invalid_extruder_error(tmp_extruder);
    #if HOTENDS > 1
      const float old_feedrate_mm_s = fr_mm_s > 0.0 ? fr_mm_s : feedrate_mm_s;
      feedrate_mm_s = fr_mm_s > 0.0 ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;
      if (tmp_extruder != active_extruder) {
        if (!no_move && axis_unhomed_error()) {
          SERIAL_ECHOLNPGM("No move on toolchange");
          no_move = true;
        }
        set_destination_to_current();
        #if ENABLED(DUAL_X_CARRIAGE)
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOPGM("Dual X Carriage Mode ");
              switch (dual_x_carriage_mode) {
                case DXC_FULL_CONTROL_MODE: SERIAL_ECHOLNPGM("DXC_FULL_CONTROL_MODE"); break;
                case DXC_AUTO_PARK_MODE: SERIAL_ECHOLNPGM("DXC_AUTO_PARK_MODE"); break;
                case DXC_DUPLICATION_MODE: SERIAL_ECHOLNPGM("DXC_DUPLICATION_MODE"); break;
              }
            }
          #endif
          const float xhome = x_home_pos(active_extruder);
          if ((dual_x_carriage_mode == DXC_AUTO_PARK_MODE
              #if ENABLED(FYS_DXC_FYS_MODE)
              || dual_x_carriage_mode == DXC_FYS_MODE)
              #endif
              && IsRunning()
              && (delayed_move_time || current_position[X_AXIS] != xhome)
          ) {
            float raised_z = current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT;
            #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
              NOMORE(raised_z, soft_endstop_max[Z_AXIS]);
            #endif
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) {
                SERIAL_ECHOLNPAIR("Raise to ", raised_z);
                SERIAL_ECHOLNPAIR("MoveX to ", xhome);
                SERIAL_ECHOLNPAIR("Lower to ", current_position[Z_AXIS]);
              }
            #endif
            for (uint8_t i = 0; i < 3; i++)
              planner.buffer_line(
                i == 0 ? current_position[X_AXIS] : xhome,
                current_position[Y_AXIS],
                i == 2 ? current_position[Z_AXIS] : raised_z,
                current_position[E_AXIS],
                planner.max_feedrate_mm_s[i == 1 ? X_AXIS : Z_AXIS],
                active_extruder
              );
            stepper.synchronize();
          }
          current_position[Y_AXIS] -= hotend_offset[Y_AXIS][active_extruder] - hotend_offset[Y_AXIS][tmp_extruder];
          current_position[Z_AXIS] -= hotend_offset[Z_AXIS][active_extruder] - hotend_offset[Z_AXIS][tmp_extruder];
          active_extruder = tmp_extruder;
          set_axis_is_at_home(X_AXIS);
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("New Extruder", current_position);
          #endif
          if (dual_x_carriage_mode != DXC_AUTO_PARK_MODE) no_move = true;
          switch (dual_x_carriage_mode) {
            case DXC_FULL_CONTROL_MODE:
              current_position[X_AXIS] = LOGICAL_X_POSITION(inactive_extruder_x_pos);
              inactive_extruder_x_pos = RAW_X_POSITION(destination[X_AXIS]);
              break;
            case DXC_AUTO_PARK_MODE:
              COPY(raised_parked_position, current_position);
              raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
              #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
                NOMORE(raised_parked_position[Z_AXIS], soft_endstop_max[Z_AXIS]);
              #endif
              active_extruder_parked = true;
              delayed_move_time = 0;
              break;
            case DXC_DUPLICATION_MODE:
              active_extruder_parked = (active_extruder == 0);
              if (active_extruder_parked)
                current_position[X_AXIS] = LOGICAL_X_POSITION(inactive_extruder_x_pos);
              else
                current_position[X_AXIS] = destination[X_AXIS] + duplicate_extruder_x_offset;
              inactive_extruder_x_pos = RAW_X_POSITION(destination[X_AXIS]);
              extruder_duplication_enabled = false;
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING)) {
                  SERIAL_ECHOLNPAIR("Set inactive_extruder_x_pos=", inactive_extruder_x_pos);
                  SERIAL_ECHOLNPGM("Clear extruder_duplication_enabled");
                }
              #endif
              break;
            #if ENABLED(FYS_DXC_FYS_MODE)
              case DXC_FYS_MODE:
                  memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
                  active_extruder_parked = true;
                  break;
            #endif
          }
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOLNPAIR("Active extruder parked: ", active_extruder_parked ? "yes" : "no");
              DEBUG_POS("New extruder (parked)", current_position);
            }
          #endif
        #else 
          #if ENABLED(SWITCHING_NOZZLE)
            #define DONT_SWITCH (SWITCHING_EXTRUDER_SERVO_NR == SWITCHING_NOZZLE_SERVO_NR)
            const float z_diff = hotend_offset[Z_AXIS][active_extruder] - hotend_offset[Z_AXIS][tmp_extruder],
                        z_raise = 0.3 + (z_diff > 0.0 ? z_diff : 0.0);
            current_position[Z_AXIS] += z_raise;
            planner.buffer_line_kinematic(current_position, planner.max_feedrate_mm_s[Z_AXIS], active_extruder);
            move_nozzle_servo(tmp_extruder);
          #endif
          #if ABL_PLANAR
            vector_3 tmp_offset_vec = vector_3(hotend_offset[X_AXIS][tmp_extruder],
                                               hotend_offset[Y_AXIS][tmp_extruder],
                                               0),
                     act_offset_vec = vector_3(hotend_offset[X_AXIS][active_extruder],
                                               hotend_offset[Y_AXIS][active_extruder],
                                               0),
                     offset_vec = tmp_offset_vec - act_offset_vec;
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) {
                tmp_offset_vec.debug(PSTR("tmp_offset_vec"));
                act_offset_vec.debug(PSTR("act_offset_vec"));
                offset_vec.debug(PSTR("offset_vec (BEFORE)"));
              }
            #endif
            offset_vec.apply_rotation(planner.bed_level_matrix.transpose(planner.bed_level_matrix));
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) offset_vec.debug(PSTR("offset_vec (AFTER)"));
            #endif
            const float xydiff[2] = { offset_vec.x, offset_vec.y };
            current_position[Z_AXIS] += offset_vec.z;
          #else 
            const float xydiff[2] = {
              hotend_offset[X_AXIS][tmp_extruder] - hotend_offset[X_AXIS][active_extruder],
              hotend_offset[Y_AXIS][tmp_extruder] - hotend_offset[Y_AXIS][active_extruder]
            };
            #if ENABLED(MESH_BED_LEVELING)
              if (leveling_is_active()) {
                #if ENABLED(DEBUG_LEVELING_FEATURE)
                  if (DEBUGGING(LEVELING)) SERIAL_ECHOPAIR("Z before MBL: ", current_position[Z_AXIS]);
                #endif
                float x2 = current_position[X_AXIS] + xydiff[X_AXIS],
                      y2 = current_position[Y_AXIS] + xydiff[Y_AXIS],
                      z1 = current_position[Z_AXIS], z2 = z1;
                planner.apply_leveling(current_position[X_AXIS], current_position[Y_AXIS], z1);
                planner.apply_leveling(x2, y2, z2);
                current_position[Z_AXIS] += z2 - z1;
                #if ENABLED(DEBUG_LEVELING_FEATURE)
                  if (DEBUGGING(LEVELING))
                    SERIAL_ECHOLNPAIR(" after: ", current_position[Z_AXIS]);
                #endif
              }
            #endif 
          #endif 
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOPAIR("Offset Tool XY by { ", xydiff[X_AXIS]);
              SERIAL_ECHOPAIR(", ", xydiff[Y_AXIS]);
              SERIAL_ECHOLNPGM(" }");
            }
          #endif
          current_position[X_AXIS] += xydiff[X_AXIS];
          current_position[Y_AXIS] += xydiff[Y_AXIS];
          #if HAS_WORKSPACE_OFFSET || ENABLED(DUAL_X_CARRIAGE)
            for (uint8_t i = X_AXIS; i <= Y_AXIS; i++) {
              #if HAS_POSITION_SHIFT
                position_shift[i] += xydiff[i];
              #endif
              update_software_endstops((AxisEnum)i);
            }
          #endif
          active_extruder = tmp_extruder;
        #endif 
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("Sync After Toolchange", current_position);
        #endif
        SYNC_PLAN_POSITION_KINEMATIC();
        if (!no_move && IsRunning()) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("Move back", destination);
          #endif
          prepare_move_to_destination();
        }
        #if ENABLED(SWITCHING_NOZZLE)
          if (z_raise != z_diff) {
            destination[Z_AXIS] += z_diff;
            feedrate_mm_s = planner.max_feedrate_mm_s[Z_AXIS];
            prepare_move_to_destination();
          }
        #endif
      } 
      stepper.synchronize();
      #if ENABLED(EXT_SOLENOID)
        disable_all_solenoids();
        enable_solenoid_on_active_extruder();
      #endif 
      feedrate_mm_s = old_feedrate_mm_s;
    #else 
      UNUSED(fr_mm_s);
      UNUSED(no_move);
      #if ENABLED(SWITCHING_EXTRUDER) && !DONT_SWITCH
        stepper.synchronize();
        move_extruder_servo(tmp_extruder);
      #elif ENABLED(MK2_MULTIPLEXER)
        if (tmp_extruder >= E_STEPPERS)
          return invalid_extruder_error(tmp_extruder);
        select_multiplexed_stepper(tmp_extruder);
      #endif
    #endif 
    active_extruder = tmp_extruder;
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPAIR(MSG_ACTIVE_EXTRUDER, (int)active_extruder);
  #endif 
}
inline void gcode_T(uint8_t tmp_extruder) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR(">>> gcode_T(", tmp_extruder);
      SERIAL_CHAR(')');
      SERIAL_EOL();
      DEBUG_POS("BEFORE", current_position);
    }
  #endif
  #if HOTENDS == 1 || (ENABLED(MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1)
    tool_change(tmp_extruder);
  #elif HOTENDS > 1
    tool_change(
      tmp_extruder,
      MMM_TO_MMS(parser.linearval('F')),
      (tmp_extruder == active_extruder) || parser.boolval('S')
    );
  #endif
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      DEBUG_POS("AFTER", current_position);
      SERIAL_ECHOLNPGM("<<< gcode_T");
    }
  #endif
  #if ENABLED(FYS_LCD_EVENT)
    GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V004);
  #endif
}
#if ENABLED(FYS_LCD_EVENT)
uint16_t GLOBAL_var_V001 = 0x0000;
#endif
#if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
char GLOBAL_var_V004[FILENAME_LENGTH + 1] = { 0 };
#endif
#if ENABLED(FYS_POWER_STATUS)
  #if PIN_EXISTS(POW_BREAK_CHECK)||PIN_EXISTS(SHUTDOWN_CHECK)||PIN_EXISTS(PS_ON) 
    MACRO_var_V00E GLOBAL_var_V003 = MACRO_var_V009;
  #endif
#endif
#if ENABLED(FYS_SHUTDOWN_ONCE_PRINT_DONE)
  #if PIN_EXISTS(PS_ON)
    bool GLOBAL_var_V005 = false;
  #endif
#endif
static void FunV008(bool ifTurnOff) 
{
#ifdef FYS_MOVE_XY_TO_HOME 
    if (ifTurnOff)
    {
        FunV006("Move away X,Y...");
        SERIAL_ECHOLNPGM("Move away X Y.");
    }
    #if ENABLED(FYS_HOME_FUNCTION)
    FunV007();
    #endif
#endif
    if (ifTurnOff)
    {
        SERIAL_ECHOLNPGM("Move away OK.");
        disable_all_steppers();
        print_job_timer.stop();
        thermalManager.disable_all_heaters();
        wait_for_heatup = false;
        lcd_shutDown();
        _delay_ms(500);
    #if HAS_POWER_SWITCH
        WRITE(PS_ON_PIN, PS_ON_ASLEEP);
        #if ENABLED(USE_WATCHDOG)
        watchdog_reset();
        #endif
        delay(1000);
        WRITE(PS_ON_PIN, PS_ON_AWAKE);
    #endif
    }
    else
    {
        FunV052();
    }
  #ifdef SHUTDOWN_CHECK_PIN
    WRITE(SHUTDOWN_CHECK_PIN, HIGH);
  #endif
}
#if 0
static uint32_t FunV04F(uint8_t nGcodes)
{
    if (!card.isFileOpen())return 0;
    bool ifCmd = false;
    int32_t pos = card.getFilePos() - 1;
    while (pos > 0 && nGcodes)
    {
        card.setIndex(pos);
        const int16_t n = card.get();
        if (n != -1)
        {
            char sd_char = (char)n;
            if (sd_char == '\n' || sd_char == '\r') 
            {
                if (ifCmd)
                {
                    nGcodes--;
                }
                ifCmd = false;
            }
            else if (sd_char == '#' || sd_char == ';') 
            {
                ifCmd = false;
            }
            else
            {
                ifCmd = true;
            }
        }
        pos--;
    }
    if (pos <= 0)
    {
        card.setIndex(0);
        return 0;
    }
    pos++;
    card.setIndex(pos);
    return pos;
}
static void FunV050(bool ifIn) 
{
    char cmd[MAX_CMD_SIZE] = { 0 }, n = 10;
    uint32_t pos;
    if (ifIn)
    {
        while ((pos = FunV04F(1)) && n > 0)
        {
            FunV051(cmd);
            parser.parse(cmd);
            if (parser.seen('E'))
            {
                current_position[E_AXIS] = parser.value_axis_units(E_AXIS);
                break;
            }
            card.setIndex(pos);
            n--;
        }
        float axisPos[XYZE] = { 0 };
        axisPos[X_AXIS] = stepper.count_position[X_AXIS] * planner.steps_to_mm[X_AXIS];
        axisPos[Y_AXIS] = stepper.count_position[Y_AXIS] * planner.steps_to_mm[Y_AXIS];
        axisPos[Z_AXIS] = stepper.count_position[Z_AXIS] * planner.steps_to_mm[Z_AXIS];
        axisPos[E_AXIS] = stepper.count_position[E_AXIS] * planner.steps_to_mm[E_AXIS];
        #if PLANNER_LEVELING
        planner.unapply_leveling(axisPos);
        #endif
        if (n==0)current_position[E_AXIS] = axisPos[E_AXIS];
        current_position[Z_AXIS] = axisPos[Z_AXIS];
    }
    else
    {
        current_position[E_AXIS] = 0;
        current_position[Z_AXIS] = 0;
    }
}
void FunV003()
{
    long cX = stepper.counter_X, cY = stepper.counter_Y, cZ = stepper.counter_Z, cE = stepper.counter_E;
    long cp[NUM_AXIS] = { stepper.count_position[X_AXIS], stepper.count_position[Y_AXIS], stepper.count_position[Z_AXIS], stepper.count_position[E_AXIS] };
    int32_t stp[NUM_AXIS] = { stepper.current_block->steps[X_AXIS], stepper.current_block->steps[Y_AXIS], stepper.current_block->steps[Z_AXIS], stepper.current_block->steps[E_AXIS] };
    uint32_t ct = stepper.current_block->step_event_count;
    uint32_t stpNc = stepper.step_events_completed;
    signed char scd[NUM_AXIS] = { stepper.count_direction[X_AXIS], stepper.count_direction[Y_AXIS], stepper.count_direction[Z_AXIS], stepper.count_direction[E_AXIS] };
    while (stpNc++ < ct)
    {
        cX += stp[X_AXIS];
        if (cX > 0)
        {
            cX -= ct;
            cp[X_AXIS] += scd[X_AXIS];
        }
        cY += stp[Y_AXIS];
        if (cY > 0)
        {
            cY -= ct;
            cp[Y_AXIS] += scd[Y_AXIS];
        }
        cZ += stp[Z_AXIS];
        if (cZ > 0)
        {
            cZ -= ct;
            cp[Z_AXIS] += scd[Z_AXIS];
        }
        cE += stp[E_AXIS];
        if (cE > 0)
        {
            cE -= ct;
            cp[E_AXIS] += scd[E_AXIS];
        }
    }
    float myCartes[XYZE];
    for (uint8_t axis = X_AXIS; axis <= E_AXIS; axis++)
    {
        myCartes[axis] = cp[axis] * planner.steps_to_mm[axis];
    }
#if PLANNER_LEVELING
    planner.unapply_leveling(myCartes);
#endif
    COPY(current_position, myCartes);
}
#endif
#ifdef FYS_SAFE_PRINT_BREAK
#define     MACRO_var_V008   "AAAAA"
#define     CHECK_BYTE1     0x5C
#define     CHECK_BYTE2     0xC5
#define     CHECK_BYTE3     0xA4
#define     CHECK_BYTE4     0x39
void gcode_M1103()
{
    settings.FunV004();
    serialprintPGM(PSTR("Print restore:reset environment sign ok.\n"));
    #ifdef MACRO_VAR_V057
    card.removeFile(MACRO_var_V008);
    #endif
}
#ifdef MACRO_VAR_V057
static bool recordEnvironment()
{
#define     MACRO_var_V015(x)   card.file.write(&x,sizeof(x))
    if (!card.sdprinting)return true;
    uint8_t checkByte;
    uint32_t sdPos = card.getFilePos(),printT = print_job_timer.duration();
    card.stopSDPrint();
    print_job_timer.stop();
    card.openFile(MACRO_var_V008, false);
    if (!card.isFileOpen())
    {
        FunV006("Create record file fail.");
        SERIAL_ECHOLNPGM("Create record file fail.");
        return true;
    }
    MACRO_var_V015(sdPos);
    #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
    card.file.write(GLOBAL_var_V004, FILENAME_LENGTH + 1);
    #endif
    checkByte=CHECK_BYTE1;
    MACRO_var_V015(checkByte);
#if HAS_TEMP_0
    MACRO_var_V015(Temperature::target_temperature[0]);
#endif
#if HAS_TEMP_1
    MACRO_var_V015(Temperature::target_temperature[1]);
#endif
#if HAS_TEMP_2
    MACRO_var_V015(Temperature::target_temperature[2]);
#endif
#if HAS_TEMP_3
    MACRO_var_V015(Temperature::target_temperature[3]);
#endif
#if HAS_TEMP_4
    MACRO_var_V015(Temperature::target_temperature[4]);
#endif
#if HAS_TEMP_BED
#ifdef FYS_ENERGY_CONSERVE_HEIGHT
    if(Temperature::target_temperature_bed==0)MACRO_var_V015(recordBedTemperature);
    else
#endif
        MACRO_var_V015(Temperature::target_temperature_bed);
#endif
#if FAN_COUNT>0
    for (char i = 0; i<FAN_COUNT; i++)
    {
        MACRO_var_V015(fanSpeeds[i]);
    }
#endif
    DISABLE_STEPPER_DRIVER_INTERRUPT();
    MACRO_var_V015(active_extruder);
    MACRO_var_V015(feedrate_percentage);
    MACRO_var_V015(feedrate_mm_s);
    checkByte = CHECK_BYTE2;
    MACRO_var_V015(checkByte);
    if (planner.blocks_queued())
        FunV003();
    else
        set_current_from_steppers_for_axis(ALL_AXES);
    MACRO_var_V015(current_position[X_AXIS]);
    MACRO_var_V015(current_position[Y_AXIS]);
    MACRO_var_V015(current_position[Z_AXIS]);
    MACRO_var_V015(current_position[E_AXIS]);
    checkByte = CHECK_BYTE3;
    MACRO_var_V015(checkByte);
    uint8_t n = planner.movesplanned();
    if (n)n--;
    MACRO_var_V015(n); 
    if (planner.blocks_queued())
    {
        for (uint8_t tail = BLOCK_MOD(Planner::block_buffer_tail + 1); tail != Planner::block_buffer_head; tail = BLOCK_MOD(tail + 1))
        {
            MACRO_var_V015(planner.block_buffer[tail]);
        }
        Planner::block_buffer_head = BLOCK_MOD(Planner::block_buffer_tail + 1);
    }
    ENABLE_STEPPER_DRIVER_INTERRUPT();
    MACRO_var_V015(commands_in_queue);
    for (; commands_in_queue; --commands_in_queue) 
    {
        card.file.write(command_queue[
            #if defined(FYS_SAFE_PRINT_BREAK)||defined(mySERIAL_Nums)
            CMDPOS_SPECIAL_LEN+  
            #endif
            cmd_queue_index_r
        ], MAX_CMD_SIZE);
        MYSERIAL.println(command_queue[cmd_queue_index_r]
            #if defined(FYS_SAFE_PRINT_BREAK)||defined(mySERIAL_Nums)
            + CMDPOS_SPECIAL_LEN
            #endif
            );
        if (++cmd_queue_index_r >= BUFSIZE) cmd_queue_index_r = 0;
    }
    checkByte = CHECK_BYTE4;
    MACRO_var_V015(checkByte);
    MACRO_var_V015(printT);
    #if ENABLED(FYS_MIX_FIL_RANOUT_SAFE_PRINT_BREAK)
    #if ENABLED(FILAMENT_RUNOUT_SENSOR) 
    MACRO_var_V015(filament_ran_out); 
    #endif
    #endif
    card.closefile();
    settings.FunV00D();
    return false;
}
#else
static uint32_t FunV04F(uint8_t nGcodes) 
{
    if (!card.isFileOpen())return 0;
    bool ifCmd = false; 
    int32_t pos = card.getFilePos() - 1;
    while (pos > 0 && nGcodes)
    {
        card.setIndex(pos);
        const int16_t n = card.get();
        if (n != -1)
        {
            char sd_char = (char)n;
            if (sd_char == '\n' || sd_char == '\r') 
            {
                if (ifCmd) 
                {
                    nGcodes--;
                }
                ifCmd = false;
            }
            else if (sd_char == '#' || sd_char == ';') 
            {
                ifCmd = false;
            }
            else
            {
                ifCmd = true; 
            }
        }
        pos--;
    }
    if (pos <= 0)
    {
        card.setIndex(0);
        return 0;
    }
    pos++; 
    card.setIndex(pos);
    return pos;
}
static bool FunV051(char*cmd)
{
    bool  sd_comment_mode = false;
    uint16_t sd_count = 0;
    while (!card.eof())
    {
        const int16_t n = card.get();
        char sd_char = (char)n;
        if (n == -1
            || sd_char == '\n' || sd_char == '\r'
            || ((sd_char == '#' || sd_char == ':') && !sd_comment_mode)
            ) {
            if (n == -1) {
                SERIAL_ERROR_START();
                SERIAL_ECHOLNPGM(MSG_SD_ERR_READ);
            }
            sd_comment_mode = false; 
            if (sd_char == '#' || sd_char == ':')return false;
            if (!sd_count) continue; 
            cmd[sd_count] = '\0';
            return true;
        }
        else if (sd_count >= MAX_CMD_SIZE - 1) {
        }
        else {
            if (sd_char == ';') sd_comment_mode = true;
            if (!sd_comment_mode) cmd[sd_count++] = sd_char;
        }
    }
}
static void getPreviousEPos() 
{
    char cmd[MAX_CMD_SIZE] = { 0 }, n = 10;
    uint32_t pos;
    card.setIndex(currentCmdSdPos.n32); 
    while ((pos = FunV04F(1)) && n > 0)
    {
        FunV051(cmd);
        parser.parse(cmd);
        if (parser.seen('E'))
        {
            current_position[E_AXIS] = parser.value_axis_units(E_AXIS);
            break;
        }
        card.setIndex(pos);
        n--;
    }
    if (n == 0)current_position[E_AXIS] = stepper.get_axis_position_mm(E_AXIS);
}
bool recordEnvironment()
{
    #if ENABLED(FILAMENT_RUNOUT_SENSOR) && ENABLED(FYS_MIX_FIL_RANOUT_SAFE_PRINT_BREAK)
    if (!(card.sdprinting || (!card.sdprinting&&sd_print_paused&&filament_ran_out))) return true;
    #else
    if (!card.sdprinting)return true; 
    #endif
    uint32_t printT = print_job_timer.duration();
    currentCmdSdPos.n32 = 0;
    if (Planner::blocks_queued())
    {
        for (; Planner::blocks_queued();)
        {
            if (Planner::block_buffer[Planner::block_buffer_tail].sdPos > 0)
            {
                currentCmdSdPos.n32 = Planner::block_buffer[Planner::block_buffer_tail].sdPos;
                break;
            }
            Planner::block_buffer_tail = BLOCK_MOD(Planner::block_buffer_tail + 1);
        }
        feedrate_mm_s = Planner::block_buffer[Planner::block_buffer_tail].nominal_speed;
    }
    if (currentCmdSdPos.n32 == 0)
    {
        uint8_t r = cmd_queue_index_r;
        for (uint8_t i = 0; i < commands_in_queue; i++)
        {
            if (command_queue[r][CMDPOS_SD_CMD_SIGN] == 0x01)
            {
                for (uint8_t j = 0; j < 4; j++)
                    currentCmdSdPos.n8[j] = command_queue[r][CMDPOS_SD_POS + j];
                if (currentCmdSdPos.n32>0)break;
            }
            if (++r >= BUFSIZE) r = 0;
        }
    }
    getPreviousEPos();
    set_current_from_steppers_for_axis(ALL_AXES);
    settings.FunV04D(printT);
    settings.FunV00D();
    #if ENABLED(FYS_POWERBREAK_STEPPER_STATUS)
      stepper.powerBreakStatus = 1;
    #endif
    card.stopSDPrint();
    print_job_timer.stop();
    return false;
}
bool recordEnvironmentPure()
{
    #if ENABLED(FILAMENT_RUNOUT_SENSOR) && ENABLED(FYS_MIX_FIL_RANOUT_SAFE_PRINT_BREAK)
    if (!(card.sdprinting || (!card.sdprinting&&sd_print_paused&&filament_ran_out))) return true;
    #else
    if (!card.sdprinting)return true; 
    #endif
    uint32_t printT = print_job_timer.duration();
    currentCmdSdPos.n32 = 0;
    if (Planner::blocks_queued())
    {
        for (; Planner::blocks_queued();)
        {
            if (Planner::block_buffer[Planner::block_buffer_tail].sdPos > 0)
            {
                currentCmdSdPos.n32 = Planner::block_buffer[Planner::block_buffer_tail].sdPos;
                break;
            }
            Planner::block_buffer_tail = BLOCK_MOD(Planner::block_buffer_tail + 1);
        }
        feedrate_mm_s = Planner::block_buffer[Planner::block_buffer_tail].nominal_speed;
    }
    if (currentCmdSdPos.n32 == 0)
    {
        uint8_t r = cmd_queue_index_r;
        for (uint8_t i = 0; i < commands_in_queue; i++)
        {
            if (command_queue[r][CMDPOS_SD_CMD_SIGN] == 0x01)
            {
                for (uint8_t j = 0; j < 4; j++)
                    currentCmdSdPos.n8[j] = command_queue[r][CMDPOS_SD_POS + j];
                if (currentCmdSdPos.n32>0)break;
            }
            if (++r >= BUFSIZE) r = 0;
        }
    }
    getPreviousEPos();
    set_current_from_steppers_for_axis(ALL_AXES);
    settings.FunV04D(printT);
    return false;
}
#endif 
#ifdef MACRO_VAR_V057
bool FunV00B(uint32_t& sdPos,uint32_t& printedTime)
{
    #define     MACRO_var_V012(x) card.file.read(&x,sizeof(x))
    if (!card.cardOK)
    {
        FunV006("Sd init fail!.\nSo can't resume print.");
        SERIAL_ECHOPGM("Sd init fail! So can't resume print.");
        return true;
    }
    card.openFile(MACRO_var_V008, true);
    if (!card.isFileOpen())
    {
        FunV006("Record file lost.");
        SERIAL_ECHOLNPGM("Record file lost.");
        return true;
    }
    MACRO_var_V012(sdPos);
    #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
    card.file.read(GLOBAL_var_V004, FILENAME_LENGTH + 1);
    #endif
    uint8_t checkByte;
    int16_t t;
    MACRO_var_V012(checkByte);
    if (checkByte != CHECK_BYTE1)
    {
        card.closefile();
        gcode_M1103();
        FunV006("Record file check fail 1.");
        SERIAL_ECHOLNPGM("Record file check fail 1.");
        return true;
    }
    #if HAS_TEMP_0
    MACRO_var_V012(t);
    Temperature::setTargetHotend(t,0);
    #endif
    #if HAS_TEMP_1
    MACRO_var_V012(t);
    Temperature::setTargetHotend(t,1);
    #endif
    #if HAS_TEMP_2
    MACRO_var_V012(t);
    Temperature::setTargetHotend(t, 2);
    #endif
    #if HAS_TEMP_3
    MACRO_var_V012(t);
    Temperature::setTargetHotend(t, 3);
    #endif
    #if HAS_TEMP_4
    MACRO_var_V012(t);
    Temperature::setTargetHotend(t, 4);
    #endif
    #if HAS_TEMP_BED
    MACRO_var_V012(t);
    Temperature::setTargetBed(t);
    #endif
    #if FAN_COUNT>0
    for (char i = 0; i<FAN_COUNT; i++)
    {
        MACRO_var_V012(fanSpeeds[i]);
    }
    #endif
    uint8_t e;
    MACRO_var_V012(e);
    MACRO_var_V012(feedrate_percentage);
    MACRO_var_V012(feedrate_mm_s);
    MACRO_var_V012(checkByte);
    if (checkByte != CHECK_BYTE2)
    {
        card.closefile();
        gcode_M1103();
        FunV006("Record file check fail 2.");
        SERIAL_ECHOLNPGM("Record file check fail 2.");
        return true;
    }
    void FunV00A();
    #if HAS_TEMP_0
    gcode_T(0);
    FunV00A();
    #endif
    #if HAS_TEMP_1
    gcode_T(1);
    FunV00A();
    #endif
    #if HAS_TEMP_2
    gcode_T(2);
    FunV00A();
    #endif
    #if HAS_TEMP_3
    gcode_T(3);
    FunV00A();
    #endif
    #if HAS_TEMP_4
    gcode_T(4);
    FunV00A();
    #endif
    #if HAS_TEMP_BED
    void FunV00C();
    FunV00C();
    #endif
    #if ENABLED(USE_WATCHDOG)
    watchdog_reset();
    #endif
    gcode_T(e);
    MACRO_var_V012(current_position[X_AXIS]);
    MACRO_var_V012(current_position[Y_AXIS]);
    MACRO_var_V012(current_position[Z_AXIS]);
    MACRO_var_V012(current_position[E_AXIS]);
    MACRO_var_V012(checkByte);
    if (checkByte != CHECK_BYTE3)
    {
        card.closefile();
        gcode_M1103();
        FunV006("Record file check fail 3.");
        SERIAL_ECHOLNPGM("Record file check fail 3.");
        stepper.quick_stop();
        return true;
    }
    uint8_t n = 0;
    planner.set_e_position_mm(0);
    planner.buffer_line(0, 0, current_position[Z_AXIS], 0, homing_feedrate(Z_AXIS), active_extruder);
    stepper.synchronize();
    planner.buffer_line(0, 0, current_position[Z_AXIS], 4.0, 4.0, active_extruder);
    stepper.synchronize();
    sync_plan_position_e();
    #if ENABLED(FYS_LCD_EVENT)
      GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V006);
    #endif
    line_to_current_position();
    stepper.synchronize();
    MACRO_var_V012(n);
    DISABLE_STEPPER_DRIVER_INTERRUPT();
    for (; n; n--)
    {
        MACRO_var_V012(planner.block_buffer[Planner::block_buffer_head]);
        Planner::block_buffer_head = BLOCK_MOD(Planner::block_buffer_head + 1);
    }
    ENABLE_STEPPER_DRIVER_INTERRUPT();
    stepper.synchronize();
    set_current_from_steppers_for_axis(ALL_AXES);
    SYNC_PLAN_POSITION_KINEMATIC();
    MACRO_var_V012(n);
    clear_command_queue();
    for (; n; n--)
    {
        card.file.read(command_queue[cmd_queue_index_w]
            #if defined(FYS_SAFE_PRINT_BREAK)||defined(mySERIAL_Nums)
            + CMDPOS_SPECIAL_LEN
            #endif
            , MAX_CMD_SIZE);
        ++commands_in_queue;
        if (++cmd_queue_index_w >= BUFSIZE) cmd_queue_index_w = 0;
    }
    MACRO_var_V012(checkByte);
    if (checkByte != CHECK_BYTE4)
    {
        card.closefile();
        gcode_M1103();
        FunV006("Record file check fail 4.");
        SERIAL_ECHOLNPGM("Record file check fail 4.");
        stepper.quick_stop();
        return true;
    }
    MACRO_var_V012(printedTime);
    card.closefile();
    gcode_M1103();
    return false;
}
#else
static bool FunV00B(uint32_t& printedTime)
{      
    if (!card.cardOK)
    {
        FunV006("Sd init fail!.\nSo can't resume print.");
        SERIAL_ECHOPGM("Sd init fail! So can't resume print.");
        return true;
    }
    settings.FunV04E(printedTime);
    uint8_t e = active_extruder;
    float cposition[4],feedrate = feedrate_mm_s;
    int16_t feedpercentage = feedrate_percentage;
    feedrate_percentage = 100;
    feedrate_mm_s = 30.0;
    COPY(cposition, current_position);
    void FunV00A();
    memset(current_position,0,sizeof(current_position));
    #if 1
    #if HAS_TEMP_0
    gcode_T(0);
    FunV00A();
    #endif
    #if HAS_TEMP_1
    gcode_T(1);
    FunV00A();
    #endif
    #if HAS_TEMP_2
    gcode_T(2);
    FunV00A();
    #endif
    #if HAS_TEMP_3
    gcode_T(3);
    FunV00A();
    #endif
    #if HAS_TEMP_4
    gcode_T(4);
    FunV00A();
    #endif
    #if HAS_TEMP_BED
    void FunV00C();
    FunV00C();
    #endif
    #if ENABLED(USE_WATCHDOG)
    watchdog_reset();
    #endif
    gcode_T(e);
    #endif
    SERIAL_ECHOLNPGM("move z axis and e axis.");
    COPY(current_position, cposition);
#if PLANNER_LEVELING && IS_CARTESIAN
    float fPosition[4];
    COPY(fPosition, current_position);
    planner.apply_leveling(fPosition[X_AXIS], fPosition[Y_AXIS], fPosition[Z_AXIS]);
    planner.set_position_mm(X_AXIS, 0);
    planner.set_position_mm(Y_AXIS, 0); 
  #if defined(FYS_POWER_BREAK_Z_UNMOVE)
    planner.set_position_mm(Z_AXIS, fPosition[Z_AXIS]); 
  #endif
    planner.set_position_mm(E_AXIS, 0);
    planner._buffer_line(0, 0, fPosition[Z_AXIS], 4.0, homing_feedrate(Z_AXIS), active_extruder);
    stepper.synchronize();
#else 
    planner.set_e_position_mm(0);
    #if defined(FYS_POWER_BREAK_Z_UNMOVE)
    planner.set_position_mm(Z_AXIS, current_position[Z_AXIS]); 
    #endif
    planner.buffer_line_and_report(0, 0, current_position[Z_AXIS], 4.0, homing_feedrate(Z_AXIS), active_extruder);
    stepper.synchronize();
#endif
    planner.buffer_line_and_report(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], 4.0, 30.0, active_extruder);
    stepper.synchronize();
    sync_plan_position_e();
    SYNC_PLAN_POSITION_KINEMATIC();
    #if ENABLED(FYS_LCD_EVENT)
      GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V006);
    #endif
    clear_command_queue();
    return false;
}
#endif
void gcode_M1101(){ 
 if(settings.FunV009())
 {
   #ifdef FYS_PRINT_BREAK_HEATUP_FIRST
    ensure_safe_temperature_print_break();
   #endif
   #if ENABLED(FYS_MIX_FIL_RANOUT_SAFE_PRINT_BREAK)
    #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    bool ran_out=false;
    if(READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_INVERTING) ran_out=true;
    delay(200);
    if((READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_INVERTING) && ran_out==true) filament_ran_out=true;
    else filament_ran_out=false;
    if (filament_ran_out)
    {
        dwin_popup(PSTR("       Filament runout!\nMachine is homing,please wait."));
        SERIAL_ECHOLNPGM("Filament runout!\nMachine is homing,please wait.\r\n");
        #if defined(FYS_POWER_BREAK_Z_UNMOVE)
          #if ENABLED(FYS_HOME_FUNCTION)
            FunV007();
          #endif
        set_axis_is_at_home(Z_AXIS);
        #else
        gcode_G28(true);
        #endif
        do_blocking_move_to_z(current_position[Z_AXIS] + PAUSE_PARK_Z_ADD, PAUSE_PARK_Z_FEEDRATE); 
        ensure_safe_temperature();
        #if ENABLED(USE_WATCHDOG)
        watchdog_reset();
        #endif
        gcode_T(active_extruder);
        FunV05D();
        wait_for_filament_reload();
        FunV05E(FILAMENT_CHANGE_LOAD_LENGTH,ADVANCED_PAUSE_EXTRUDE_LENGTH);
        filament_ran_out=false;
    }
    else
    {
        SERIAL_ECHOLNPGM("Filament runout not detect!\r\n");
    }
    #endif
   #endif
     SERIAL_ECHOLNPGM("Print restore from last break.");
    if (card.cardOK) {
      #if ENABLED(INCH_MODE_SUPPORT)
        gcode_G21();
      #endif
      relative_mode = false;
      axis_relative_modes[E_AXIS] = false;
      #if defined(FYS_POWER_BREAK_Z_UNMOVE)
        #ifdef FYS_RESUME_PRINT_NO_HOME
          if(!g_resumePrintNoHome)
          {
            #if ENABLED(FYS_HOME_FUNCTION)
              FunV007();
            #endif
          }
          g_resumePrintNoHome = false;
        #else
          #if ENABLED(FYS_HOME_FUNCTION)
            FunV007();
          #endif
        #endif
        set_axis_is_at_home(Z_AXIS);
      #else
        #ifdef FYS_RESUME_PRINT_NO_HOME
          if(!g_resumePrintNoHome)
          {
            #if ENABLED(FYS_HOME_FUNCTION)
            homeZ();
            #endif
          }
          g_resumePrintNoHome = false;
        #else
          gcode_G28(true);
        #endif
      #endif
      uint32_t printedTime;
      if (FunV00B(printedTime))return;
      #if ENABLED(FYS_MIX_FIL_RANOUT_SAFE_PRINT_BREAK) && ENABLED(FYS_LCD_EVENT)
        GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V004);
      #endif
      #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
        card.openFile(GLOBAL_var_V004, true);
        if (!card.isFileOpen())
        {
            FunV006("Target file open fail.");
            SERIAL_ECHOLNPGM("Print restore: target file open fail.");
            return;
        }
        gcode_M1103();
        card.setIndex(currentCmdSdPos.n32);
        card.startFileprint();
        print_job_timer.resume(printedTime);
        SERIAL_ECHOPGM("Print restore : ok to reprint ");
        #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
        SERIAL_ECHOLN(GLOBAL_var_V004);
        #endif
      #endif
    }
    else
    {
        SERIAL_ECHOLNPGM("Print restore: fail as card init fail.");
        FunV006("Card init fail");
    }
 }
 else
 {
     SERIAL_ECHOLNPGM("Print restore: NO need.");
     FunV006("No print needed.");
 }
}
void gcode_M1102()
{
    if (settings.FunV009())
    {
      #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
        SERIAL_ECHOPGM("Print restore: question to restore File:");
        SERIAL_ECHOLN(GLOBAL_var_V004);
      #endif
      #if ENABLED(FYS_LCD_EVENT)
        GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V005);
      #endif
    }
    else
    {
      SERIAL_ECHOLNPGM("Print restore: NO need");
      #if ENABLED(FYS_LCD_EVENT)
        GLOBAL_var_V001 &= ~((uint16_t)0x0001 << MACRO_var_V005);
      #endif
    }
}
#endif
#if ENABLED(FYS_POWERBREAK_CHECK)
void gcode_M1100(){
  #ifdef POW_BREAK_CHECK_PIN&& ENABLED(FYS_POWER_STATUS)
    if (!READ(POW_BREAK_CHECK_PIN) && GLOBAL_var_V003 == MACRO_var_V009)
    {
      FunV006("Power break!");
      SERIAL_ECHOLNPGM("Power break!");
      disable_X();
      disable_Y();
      disable_E0();
      #ifdef FYS_SAFE_PRINT_BREAK
        #if ENABLED(FILAMENT_RUNOUT_SENSOR) && ENABLED(FYS_MIX_FIL_RANOUT_SAFE_PRINT_BREAK)
        if(filament_ran_out && !filament_ran_out_still_printing) settings.FunV00D();
        else
        #endif         
        if (recordEnvironment()){
          return;
        }
      #endif
      enable_X();
      enable_Y();
      enable_E0();
      FunV006("Save environment OK...");
      SERIAL_ECHOPGM("Save environment OK...");
    }
  #endif
}
#endif
#if ENABLED(FYS_DULX_Z_HEIGHT_DETECT)
void plan_get_position()
{
    vector_3 position = vector_3(stepper.get_axis_position_mm(X_AXIS), stepper.get_axis_position_mm(Y_AXIS), stepper.get_axis_position_mm(Z_AXIS));
    matrix_3x3 inverse = matrix_3x3::transpose(planner.bed_level_matrix);
    position.apply_rotation(inverse);
    current_position[X_AXIS] = position.x;
    current_position[Y_AXIS] = position.y;
    current_position[Z_AXIS] = position.z;
}
static void gcode_M1104()
{
    gcode_T(0);
    #if Z_MIN_PIN == -1
    #error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature!!! Z_MIN_PIN must point to a valid hardware pin."
    #endif
    if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]))
    {
        MYSERIAL.println("Abort M11104, since we don't know where we are.");
        return;
    }
    planner.bed_level_matrix.set_to_identity();
    plan_get_position();
    SYNC_PLAN_POSITION_KINEMATIC();
    current_position[X_AXIS] += X_GAP_AVOID_COLLISION_LEFT;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 200, LEFT_EXTRUDER);
    active_extruder = RIGHT_EXTRUDER;
    set_axis_is_at_home(X_AXIS); 
    SYNC_PLAN_POSITION_KINEMATIC();
    current_position[X_AXIS] -= X_GAP_AVOID_COLLISION_RIGHT;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 200, RIGHT_EXTRUDER);
    stepper.synchronize();
    active_extruder = LEFT_EXTRUDER;
    set_axis_is_at_home(X_AXIS);
    current_position[X_AXIS] += X_GAP_AVOID_COLLISION_LEFT;
    SYNC_PLAN_POSITION_KINEMATIC(); 
    SERIAL_PROTOCOLPGM("Zvalue after home:");
    MYSERIAL.println(current_position[Z_AXIS]);
    setup_for_endstop_or_probe_move();
    feedrate_mm_s = CALIB_FEEDRATE_ZAXIS;
    float z_at_pt_1 = probe_pt(DULX_LEFT_PROBE_POS1_X, DULX_LEFT_PROBE_POS1_Y, false, Z_RAISE_BEFORE_PROBING);
    float z_at_pt_2 = probe_pt(DULX_LEFT_PROBE_POS2_X, DULX_LEFT_PROBE_POS2_Y, false, current_position[Z_AXIS] + Z_RAISE_BEFORE_PROBING);
    float z_at_pt_3 = probe_pt(DULX_LEFT_PROBE_POS3_X, DULX_LEFT_PROBE_POS3_Y, false, current_position[Z_AXIS] + Z_RAISE_BEFORE_PROBING);
    current_position[Z_AXIS] += Z_CLEARANCE_BETWEEN_PROBES;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 15, LEFT_EXTRUDER);
    current_position[X_AXIS] = x_home_pos(active_extruder) + X_GAP_AVOID_COLLISION_LEFT;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 200, LEFT_EXTRUDER);
    stepper.synchronize();
    active_extruder = RIGHT_EXTRUDER;
    set_axis_is_at_home(X_AXIS); 
    current_position[X_AXIS] -= X_GAP_AVOID_COLLISION_RIGHT;
    SYNC_PLAN_POSITION_KINEMATIC();
    float z2_at_pt_3 = probe_pt(DULX_RIGHT_PROBE_POS3_X, DULX_RIGHT_PROBE_POS3_Y, false, Z_RAISE_BEFORE_PROBING);
    float z2_at_pt_2 = probe_pt(DULX_RIGHT_PROBE_POS2_X, DULX_RIGHT_PROBE_POS2_Y, false, current_position[Z_AXIS] + Z_RAISE_BEFORE_PROBING);
    float z2_at_pt_1 = probe_pt(DULX_RIGHT_PROBE_POS1_X, DULX_RIGHT_PROBE_POS1_Y, false, current_position[Z_AXIS] + Z_RAISE_BEFORE_PROBING);
    current_position[Z_AXIS] += Z_CLEARANCE_BETWEEN_PROBES;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 15, RIGHT_EXTRUDER);
    current_position[X_AXIS] = x_home_pos(active_extruder) - 10;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 200, RIGHT_EXTRUDER);
    stepper.synchronize();
    clean_up_after_endstop_or_probe_move();
    planner.bed_level_matrix.set_to_identity();
    vector_3 from_2_to_1_0, from_2_to_3_0, from_3_to_1_1, from_3_to_2_1;
    {
        vector_3 pt1_0 = vector_3(DULX_LEFT_PROBE_POS1_X, DULX_LEFT_PROBE_POS1_Y, z_at_pt_1);
        vector_3 pt2_0 = vector_3(DULX_LEFT_PROBE_POS2_X, DULX_LEFT_PROBE_POS2_Y, z_at_pt_2);
        vector_3 pt3_0 = vector_3(DULX_LEFT_PROBE_POS3_X, DULX_LEFT_PROBE_POS3_Y, z_at_pt_3);
        vector_3 pt1_1 = vector_3(DULX_RIGHT_PROBE_POS1_X, DULX_RIGHT_PROBE_POS1_Y, z2_at_pt_1);
        vector_3 pt2_1 = vector_3(DULX_RIGHT_PROBE_POS2_X, DULX_RIGHT_PROBE_POS2_Y, z2_at_pt_2);
        vector_3 pt3_1 = vector_3(DULX_RIGHT_PROBE_POS3_X, DULX_RIGHT_PROBE_POS3_Y, z2_at_pt_3);
        from_2_to_1_0 = (pt1_0 - pt2_0);
        from_2_to_3_0 = (pt3_0 - pt2_0);
        from_3_to_1_1 = (pt1_1 - pt3_1);
        from_3_to_2_1 = (pt2_1 - pt3_1);
    }
    vector_3 planeNormal_0 = vector_3::cross(from_2_to_1_0, from_2_to_3_0);
    planeNormal_0 = vector_3(planeNormal_0.x, planeNormal_0.y, planeNormal_0.z);
    vector_3 planeNormal_1 = vector_3::cross(from_3_to_1_1, from_3_to_2_1);
    planeNormal_1 = vector_3(planeNormal_1.x, planeNormal_1.y, planeNormal_1.z);
    float z1_0 = (-planeNormal_0.x*(SCREW1_X - DULX_LEFT_PROBE_POS1_X) - planeNormal_0.y*(SCREW1_Y - DULX_LEFT_PROBE_POS3_Y)) / planeNormal_0.z;
    float z1_1 = (-planeNormal_1.x*(SCREW1_X - DULX_LEFT_PROBE_POS1_X) - planeNormal_1.y*(SCREW1_Y - DULX_LEFT_PROBE_POS3_Y)) / planeNormal_1.z;
    float z2_0 = (-planeNormal_0.x*(SCREW2_X - DULX_LEFT_PROBE_POS1_X) - planeNormal_0.y*(SCREW2_Y - DULX_LEFT_PROBE_POS3_Y)) / planeNormal_0.z;
    float z2_1 = (-planeNormal_1.x*(SCREW2_X - DULX_LEFT_PROBE_POS1_X) - planeNormal_1.y*(SCREW2_Y - DULX_LEFT_PROBE_POS3_Y)) / planeNormal_1.z;
    float z3_0 = (-planeNormal_0.x*(SCREW3_X - DULX_LEFT_PROBE_POS1_X) - planeNormal_0.y*(SCREW3_Y - DULX_LEFT_PROBE_POS3_Y)) / planeNormal_0.z;
    float z3_1 = (-planeNormal_1.x*(SCREW3_X - DULX_LEFT_PROBE_POS1_X) - planeNormal_1.y*(SCREW3_Y - DULX_LEFT_PROBE_POS3_Y)) / planeNormal_1.z;
    SERIAL_PROTOCOLPGM("left1 z:");
    MYSERIAL.println(z_at_pt_1);
    SERIAL_PROTOCOLPGM("left2 z:");
    MYSERIAL.println(z_at_pt_2);
    SERIAL_PROTOCOLPGM("left3 z:");
    MYSERIAL.println(z_at_pt_3);
    SERIAL_PROTOCOLPGM("right1 z:");
    MYSERIAL.println(z2_at_pt_1);
    SERIAL_PROTOCOLPGM("right2 z:");
    MYSERIAL.println(z2_at_pt_2);
    SERIAL_PROTOCOLPGM("right3 z:");
    MYSERIAL.println(z2_at_pt_3);
    SERIAL_PROTOCOLPGM("z1_0:");
    MYSERIAL.println(z1_0);
    SERIAL_PROTOCOLPGM("z2_0:");
    MYSERIAL.println(z2_0);
    SERIAL_PROTOCOLPGM("z3_0:");
    MYSERIAL.println(z3_0);
    SERIAL_PROTOCOLPGM("z1_1:");
    MYSERIAL.println(z1_1);
    SERIAL_PROTOCOLPGM("z2_1:");
    MYSERIAL.println(z2_1);
    SERIAL_PROTOCOLPGM("z3_1:");
    MYSERIAL.println(z3_1);
    screwHeightDiffer[0] = (z2_0 + z2_1) / 2.0 - (z1_0 + z1_1) / 2.0;
    screwHeightDiffer[1] = (z3_0 + z3_1) / 2.0 - (z1_0 + z1_1) / 2.0;
    SERIAL_PROTOCOLPGM("screw2:");
    MYSERIAL.println(screwHeightDiffer[0]);
    SERIAL_PROTOCOLPGM("screw3:");
    MYSERIAL.println(screwHeightDiffer[1]);
    #if ENABLED(FYS_HOME_FUNCTION)
    FunV007();
    stepper.synchronize();
    #endif
    #if ENABLED(FYS_LCD_EVENT)
      GLOBAL_var_V001 |= ((uint16_t)0x0001<<LCDEVT_M1104_NEED_ADJUST);
    #endif
}
#endif
#if ENABLED(FYS_LOOP_EVENT)
static void myStopPrint()
{
    DISABLE_STEPPER_DRIVER_INTERRUPT();
    SERIAL_ECHOLNPGM("STOP printing");
  #if ENABLED(SDSUPPORT)
    card.stopSDPrint();
  #endif
    print_job_timer.stop();
  #ifdef AUTOTEMP
    if (planner.autotemp_enabled)planner.autotemp_enabled = false;
  #endif
    thermalManager.setTargetHotend(0, active_extruder);
    thermalManager.setTargetBed(0);
  #if FAN_COUNT > 0
    fanSpeeds[0] = 0;
  #endif
    if (planner.blocks_queued())
    {
        planner.block_buffer_head = BLOCK_MOD(planner.block_buffer_tail + 1);
    }
    ENABLE_STEPPER_DRIVER_INTERRUPT();
  #if ENABLED(FYS_HOME_FUNCTION)
    FunV007();
  #endif
  #if HAS_BED_PROBE
    DEPLOY_PROBE();
  #endif
    disable_all_steppers();
    clear_command_queue();
  #ifdef FYS_SAFE_PRINT_BREAK
    if (settings.FunV009())
        settings.FunV004();
  #endif
}
static inline void FunV01C()
{
#if defined(FYS_SAFE_PRINT_BREAK)
    cli();
    recordEnvironment();
    sei();
    #if ENABLED(FYS_POWERBREAK_STEPPER_STATUS)
      while(stepper.powerBreakStatus==1); 
    #endif
    #ifdef FYS_RESUME_PRINT_NO_HOME
    g_resumePrintNoHome = true;
    #endif
#else
    SERIAL_ECHOLNPGM("Pause.");
    card.pauseSDPrint();
    print_job_timer.pause();
    #if ENABLED(PARK_HEAD_ON_PAUSE)
    enqueue_and_echo_commands_P(PSTR("M125 L0"));
    #endif
    lcd_setstatusPGM(PSTR(MSG_PRINT_PAUSED));
#endif
}
static inline void FunV019()
{
#if defined(FYS_SAFE_PRINT_BREAK)
    gcode_M1101();
#else
    SERIAL_ECHOLNPGM("Resume.");
    #if ENABLED(PARK_HEAD_ON_PAUSE)
    resume_print();
    #endif
    card.startFileprint();
    print_job_timer.start();
    lcd_setstatusPGM(PSTR(""));
#endif
}
#endif
static void gcode_M1200()
{
  FysTLcd::FunV00F(parser.string_arg);
}
#if ENABLED(FYS_POWER_CONTROL_CHECK)
void FunV010()
{
  #if defined(SHUTDOWN_CHECK_PIN) && ENABLED(FYS_POWER_STATUS)
    if (!READ(SHUTDOWN_CHECK_PIN))
    {
        if (GLOBAL_var_V003 == MACRO_var_V009)
        {
            FunV006("Manual shutdown!");
            SERIAL_ECHOLNPGM("Manual shutdown.");
            GLOBAL_var_V003 = POWER_ON_SHUTDOWN_BTN_PRESSING;
            #ifdef FYS_SAFE_PRINT_BREAK
            if(recordEnvironment())return;
            #endif
            FunV006("Save environment OK!");
            SERIAL_ECHOLNPGM("Save environment OK.");
        }
        else if (GLOBAL_var_V003 == MACRO_var_V00B)
        {
            FunV029();
            SERIAL_ECHOLNPGM("Manual open.");
            GLOBAL_var_V003 = POWER_DOWN_SHUTDOWN_BTN_PRESSING;
            WRITE(SHUTDOWN_CHECK_PIN, HIGH);
            memset(current_position, 0, sizeof(current_position));
            SYNC_PLAN_POSITION_KINEMATIC();
        }
    }
    else
    {
        if (GLOBAL_var_V003 == POWER_ON_SHUTDOWN_BTN_PRESSING)GLOBAL_var_V003 = MACRO_var_V00A;
        if (GLOBAL_var_V003 == POWER_DOWN_SHUTDOWN_BTN_PRESSING)GLOBAL_var_V003 = MACRO_var_V009;
    }
  #endif
}
#endif
void process_next_command() {
#if defined(FYS_SAFE_PRINT_BREAK)||defined(mySERIAL_Nums)
    char *current_command = command_queue[cmd_queue_index_r];
    if (*current_command == 0x01)
    for (char i = 0; i < 4; i++)
        currentCmdSdPos.n8[i] = current_command[i + CMDPOS_SD_POS];
    else
        currentCmdSdPos.n32 = 0;
    current_command[CMDPOS_SD_CMD_SIGN] = 0x00;
    #if defined(mySERIAL_Nums)
    MYSERIAL.setTxActiveSerial(current_command[CMDPOS_SERIAL_NUM]);
    #endif
    current_command += CMDPOS_SPECIAL_LEN;
#else
    char * const current_command = command_queue[cmd_queue_index_r];
#endif
  if (DEBUGGING(ECHO)) {
    SERIAL_ECHO_START();
    #if defined(mySERIAL_Nums)
    SERIAL_ECHOLN(MYSERIAL.activeTxSerial);
    #endif
    SERIAL_ECHO_START();
    SERIAL_ECHOLN(current_command);
    #if ENABLED(M100_FREE_MEMORY_WATCHER)
      SERIAL_ECHOPAIR("slot:", cmd_queue_index_r);
      M100_dump_routine("   Command Queue:", (const char*)command_queue, (const char*)(command_queue + sizeof(command_queue)));
    #endif
  }
  KEEPALIVE_STATE(IN_HANDLER);
  parser.parse(current_command);
  switch (parser.command_letter) {
    case 'G': switch (parser.codenum) {
      case 0:
      case 1:
        #if IS_SCARA
          gcode_G0_G1(parser.codenum == 0);
        #else
          gcode_G0_G1();
        #endif
        break;
      #if ENABLED(ARC_SUPPORT) && DISABLED(SCARA)
        case 2: 
        case 3: 
          gcode_G2_G3(parser.codenum == 2);
          break;
      #endif
      case 4:
        gcode_G4();
        break;
      #if ENABLED(BEZIER_CURVE_SUPPORT)
        case 5: 
          gcode_G5();
          break;
      #endif 
      #if ENABLED(FWRETRACT)
        case 10: 
        case 11: 
          gcode_G10_G11(parser.codenum == 10);
          break;
      #endif 
      #if ENABLED(NOZZLE_CLEAN_FEATURE)
        case 12:
          gcode_G12(); 
          break;
      #endif 
      #if ENABLED(CNC_WORKSPACE_PLANES)
        case 17: 
          gcode_G17();
          break;
        case 18: 
          gcode_G18();
          break;
        case 19: 
          gcode_G19();
          break;
      #endif 
      #if ENABLED(INCH_MODE_SUPPORT)
        case 20: 
          gcode_G20();
          break;
        case 21: 
          gcode_G21();
          break;
      #endif 
      #if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(UBL_G26_MESH_VALIDATION)
        case 26: 
          gcode_G26();
          break;
      #endif 
      #if ENABLED(NOZZLE_PARK_FEATURE)
        case 27: 
          gcode_G27();
          break;
      #endif 
      case 28: 
        gcode_G28(false);
        break;
      #if HAS_LEVELING
        case 29: 
          gcode_G29();
          break;
        case 34:
        {
            SERIAL_ECHOPAIR("x:",axis_known_position[X_AXIS]);
            SERIAL_ECHOPAIR(" y:",axis_known_position[Y_AXIS]);
            SERIAL_ECHOLNPAIR(" z:",axis_known_position[Z_AXIS]);
        }
        break;
      #endif 
      #if HAS_BED_PROBE
        case 30: 
          gcode_G30();
          break;
        #if ENABLED(Z_PROBE_SLED)
            case 31: 
              gcode_G31();
              break;
            case 32: 
              gcode_G32();
              break;
        #endif 
      #endif 
      #if PROBE_SELECTED
        #if ENABLED(DELTA_AUTO_CALIBRATION)
          case 33: 
            gcode_G33();
            break;
        #endif 
      #endif 
      #if ENABLED(G38_PROBE_TARGET)
        case 38: 
          if (subcode == 2 || subcode == 3)
            gcode_G38(subcode == 2);
          break;
      #endif
      case 90: 
        relative_mode = false;
        break;
      case 91: 
        relative_mode = true;
        break;
      case 92: 
        gcode_G92();
        break;
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(MESH_BED_LEVELING)
        case 42:
          gcode_G42();
          break;
      #endif
      #if ENABLED(DEBUG_GCODE_PARSER)
        case 800:
          parser.debug(); 
          break;
      #endif
    }
    break;
    case 'M': switch (parser.codenum) {
      #if HAS_RESUME_CONTINUE
        case 0: 
        case 1: 
          gcode_M0_M1();
          break;
      #endif 
      #if ENABLED(SPINDLE_LASER_ENABLE)
        case 3:
          gcode_M3_M4(true);   
          break;               
        case 4:
          gcode_M3_M4(false);  
          break;               
        case 5:
          gcode_M5();     
          break;          
      #endif
      case 17: 
        gcode_M17();
        break;
      #if ENABLED(SDSUPPORT)
        case 20: 
          gcode_M20(); break;
        case 21: 
          gcode_M21(); break;
        case 22: 
          gcode_M22(); break;
        case 23: 
          gcode_M23(); break;
        case 24: 
          gcode_M24(); break;
        case 25: 
          gcode_M25(); break;
        case 26: 
          gcode_M26(); break;
        case 27: 
          gcode_M27(); break;
        case 28: 
          gcode_M28(); break;
        case 29: 
          gcode_M29(); break;
        case 30: 
          gcode_M30(); break;
        case 32: 
          gcode_M32(); break;
        #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
          case 33: 
            gcode_M33(); break;
        #endif
        #if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_GCODE)
          case 34: 
            gcode_M34(); break;
        #endif 
        case 928: 
          gcode_M928(); break;
      #endif 
      case 31: 
        gcode_M31(); break;
      case 42: 
        gcode_M42(); break;
      #if ENABLED(PINS_DEBUGGING)
        case 43: 
          gcode_M43(); break;
      #endif
      #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
        case 48: 
          gcode_M48();
          break;
      #endif 
      #if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(UBL_G26_MESH_VALIDATION)
        case 49: 
          gcode_M49();
          break;
      #endif 
      case 75: 
        gcode_M75(); break;
      case 76: 
        gcode_M76(); break;
      case 77: 
        gcode_M77(); break;
      #if ENABLED(PRINTCOUNTER)
        case 78: 
          gcode_M78(); break;
      #endif
      #if ENABLED(M100_FREE_MEMORY_WATCHER)
        case 100: 
          gcode_M100();
          break;
      #endif
      case 104: 
        gcode_M104();
        break;
      case 110: 
        gcode_M110();
        break;
      case 111: 
        gcode_M111();
        break;
      #if DISABLED(EMERGENCY_PARSER)
        case 108: 
          gcode_M108();
          break;
        case 112: 
          gcode_M112();
          break;
        case 410: 
          gcode_M410();
          break;
      #endif
      #if ENABLED(HOST_KEEPALIVE_FEATURE)
        case 113: 
          gcode_M113();
          break;
      #endif
      case 140: 
        gcode_M140();
        break;
      case 105: 
        gcode_M105();
        KEEPALIVE_STATE(NOT_BUSY);
        return; 
      #if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
        case 155: 
          gcode_M155();
          break;
      #endif
      case 109: 
        gcode_M109();
        break;
      #if HAS_TEMP_BED
        case 190: 
          gcode_M190();
          break;
      #endif 
      #if FAN_COUNT > 0
        case 106: 
          gcode_M106();
          break;
        case 107: 
          gcode_M107();
          break;
      #endif 
      #if ENABLED(PARK_HEAD_ON_PAUSE)
        case 125: 
          gcode_M125(); break;
      #endif
      #if ENABLED(BARICUDA)
        #if HAS_HEATER_1
          case 126: 
            gcode_M126();
            break;
          case 127: 
            gcode_M127();
            break;
        #endif 
        #if HAS_HEATER_2
          case 128: 
            gcode_M128();
            break;
          case 129: 
            gcode_M129();
            break;
        #endif 
      #endif 
      #if HAS_POWER_SWITCH
        case 80: 
          gcode_M80();
          break;
      #endif 
      case 81: 
        gcode_M81();
        break;
      case 82: 
        gcode_M82();
        break;
      case 83: 
        gcode_M83();
        break;
      case 18: 
      case 84: 
        gcode_M18_M84();
        break;
      case 85: 
        gcode_M85();
        break;
      case 92: 
        gcode_M92();
        break;
      case 114: 
        gcode_M114();
        break;
      case 115: 
        gcode_M115();
        break;
      case 117: 
        gcode_M117();
        break;
      case 118: 
        gcode_M118();
        break;
      case 119: 
        gcode_M119();
        break;
      case 120: 
        gcode_M120();
        break;
      case 121: 
        gcode_M121();
        break;
        case 145: 
          gcode_M145();
          break;
      #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
        case 149: 
          gcode_M149();
          break;
      #endif
      #if HAS_COLOR_LEDS
        case 150: 
          gcode_M150();
          break;
      #endif 
      #if ENABLED(MIXING_EXTRUDER)
        case 163: 
          gcode_M163();
          break;
        #if MIXING_VIRTUAL_TOOLS > 1
          case 164: 
            gcode_M164();
            break;
        #endif
        #if ENABLED(DIRECT_MIXING_IN_G1)
          case 165: 
            gcode_M165();
            break;
        #endif
      #endif
      case 200: 
        gcode_M200();
        break;
      case 201: 
        gcode_M201();
        break;
      #if 0 
        case 202: 
          gcode_M202();
          break;
      #endif
      case 203: 
        gcode_M203();
        break;
      case 204: 
        gcode_M204();
        break;
      case 205: 
        gcode_M205();
        break;
      #if HAS_M206_COMMAND
        case 206: 
          gcode_M206();
          break;
      #endif
      #if ENABLED(DELTA)
        case 665: 
          gcode_M665();
          break;
      #endif
      #if ENABLED(DELTA) || ENABLED(Z_DUAL_ENDSTOPS)
        case 666: 
          gcode_M666();
          break;
      #endif
      #if ENABLED(FWRETRACT)
        case 207: 
          gcode_M207();
          break;
        case 208: 
          gcode_M208();
          break;
        case 209: 
          gcode_M209();
          break;
      #endif 
      case 211: 
        gcode_M211();
        break;
      #if HOTENDS > 1
        case 218: 
          gcode_M218();
          break;
      #endif
      case 220: 
        gcode_M220();
        break;
      case 221: 
        gcode_M221();
        break;
      case 226: 
        gcode_M226();
        break;
      #if HAS_SERVOS
        case 280: 
          gcode_M280();
          break;
      #endif 
      #if HAS_BUZZER
        case 300: 
          gcode_M300();
          break;
      #endif 
      #if ENABLED(PIDTEMP)
        case 301: 
          gcode_M301();
          break;
      #endif 
      #if ENABLED(PIDTEMPBED)
        case 304: 
          gcode_M304();
          break;
      #endif 
      #if defined(CHDK) || HAS_PHOTOGRAPH
        case 240: 
          gcode_M240();
          break;
      #endif 
    #if HAS_LCD_CONTRAST
        case 250: 
          gcode_M250();
          break;
      #endif 
      #if ENABLED(EXPERIMENTAL_I2CBUS)
        case 260: 
          gcode_M260();
          break;
        case 261: 
          gcode_M261();
          break;
      #endif 
      #if ENABLED(PREVENT_COLD_EXTRUSION)
        case 302: 
          gcode_M302();
          break;
      #endif 
      case 303: 
        gcode_M303();
        break;
      #if ENABLED(MORGAN_SCARA)
        case 360:  
          if (gcode_M360()) return;
          break;
        case 361:  
          if (gcode_M361()) return;
          break;
        case 362:  
          if (gcode_M362()) return;
          break;
        case 363:  
          if (gcode_M363()) return;
          break;
        case 364:  
          if (gcode_M364()) return;
          break;
      #endif 
      case 400: 
        gcode_M400();
        break;
      #if HAS_BED_PROBE
        case 401: 
          gcode_M401();
          break;
        case 402: 
          gcode_M402();
          break;
      #endif 
      #if ENABLED(FILAMENT_WIDTH_SENSOR)
        case 404:  
          gcode_M404();
          break;
        case 405:  
          gcode_M405();
          break;
        case 406:  
          gcode_M406();
          break;
        case 407:   
          gcode_M407();
          break;
      #endif 
      #if HAS_LEVELING
        case 420: 
          gcode_M420();
          break;
      #endif
      #if ENABLED(MESH_BED_LEVELING) || ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(AUTO_BED_LEVELING_BILINEAR)
        case 421: 
          gcode_M421();
          break;
      #endif
      #if HAS_M206_COMMAND
        case 428: 
          gcode_M428();
          break;
      #endif
      case 500: 
        gcode_M500();
        break;
      case 501: 
        gcode_M501();
        break;
      case 502: 
        gcode_M502();
        break;
      #if DISABLED(DISABLE_M503)
        case 503: 
          gcode_M503();
          break;
      #endif
      #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
        case 540: 
          gcode_M540();
          break;
      #endif
        case 850:
            gcode_M850();
            break;
      #if HAS_BED_PROBE
        case 851: 
          gcode_M851();
          break;
      #endif 
      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        case 600: 
          gcode_M600();
          break;
      #endif 
      #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
        case 605: 
          gcode_M605();
          break;
      #endif 
      #if ENABLED(MK2_MULTIPLEXER)
        case 702: 
          gcode_M702();
          break;
      #endif
      #if ENABLED(LIN_ADVANCE)
        case 900: 
          gcode_M900();
          break;
      #endif
      #if ENABLED(HAVE_TMC2130)
        case 906: 
          gcode_M906();
          break;
      #endif
      case 907: 
        gcode_M907();
        break;
      #if HAS_DIGIPOTSS || ENABLED(DAC_STEPPER_CURRENT)
        case 908: 
          gcode_M908();
          break;
        #if ENABLED(DAC_STEPPER_CURRENT) 
          case 909: 
            gcode_M909();
            break;
          case 910: 
            gcode_M910();
            break;
        #endif
      #endif 
      #if ENABLED(HAVE_TMC2130)
        case 911: 
          gcode_M911();
          break;
        case 912: 
          gcode_M912();
          break;
        #if ENABLED(HYBRID_THRESHOLD)
          case 913: 
            gcode_M913();
            break;
        #endif
        #if ENABLED(SENSORLESS_HOMING)
          case 914: 
            gcode_M914();
            break;
        #endif
      #endif
      #if HAS_MICROSTEPS
        case 350: 
          gcode_M350();
          break;
        case 351: 
          gcode_M351();
          break;
      #endif 
      case 355: 
        gcode_M355();
        break;
      #if ENABLED(DEBUG_GCODE_PARSER)
        case 800:
          parser.debug(); 
          break;
      #endif
      #if ENABLED(I2C_POSITION_ENCODERS)
        case 860: 
          gcode_M860();
          break;
        case 861: 
          gcode_M861();
          break;
        case 862: 
          gcode_M862();
          break;
        case 863: 
          gcode_M863();
          break;
        case 864: 
          gcode_M864();
          break;
        case 865: 
          gcode_M865();
          break;
        case 866: 
          gcode_M866();
          break;
        case 867: 
          gcode_M867();
          break;
        case 868: 
          gcode_M868();
          break;
        case 869: 
          gcode_M869();
          break;
      #endif 
      case 999: 
        gcode_M999();
        break;
        #if defined(FYS_SAFE_PRINT_BREAK)
        case 1101:
            gcode_M1101();
            break;
        case 1102:
            gcode_M1102();
            break;
        case 1103:
            gcode_M1103();
            break;
        #endif
        #if ENABLED(FYS_DULX_Z_HEIGHT_DETECT)
        case 1104:
        {
          gcode_M1104();
        }
        break;
        #endif
        case 1200:
            gcode_M1200();
            break;
    }
    break;
    case 'T':
      gcode_T(parser.codenum);
      break;
    default: parser.unknown_command_error();
  }
  KEEPALIVE_STATE(NOT_BUSY);
  ok_to_send();
}
void FlushSerialRequestResend() {
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ok_to_send();
}
void ok_to_send() {
  refresh_cmd_timeout();
  if (!send_ok[cmd_queue_index_r]) return;
  SERIAL_PROTOCOLPGM(MSG_OK);
  #if ENABLED(ADVANCED_OK)
    #if defined(FYS_SAFE_PRINT_BREAK)||defined(mySERIAL_Nums)
    char* p = command_queue[cmd_queue_index_r]+CMDPOS_SPECIAL_LEN;
    #else
    char* p = command_queue[cmd_queue_index_r];
    #endif
    if (*p == 'N') {
      SERIAL_PROTOCOL(' ');
      SERIAL_ECHO(*p++);
      while (NUMERIC_SIGNED(*p))
        SERIAL_ECHO(*p++);
    }
    SERIAL_PROTOCOLPGM(" P"); SERIAL_PROTOCOL(int(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
    SERIAL_PROTOCOLPGM(" B"); SERIAL_PROTOCOL(BUFSIZE - commands_in_queue);
  #endif
  SERIAL_EOL();
}
#if HAS_SOFTWARE_ENDSTOPS
  void clamp_to_software_endstops(float target[XYZ]) {
    if (!soft_endstops_enabled) return;
    #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
      #ifdef FYS_POSITION_ERR
        if (target[X_AXIS]< soft_endstop_min[X_AXIS])
        {
            target[X_AXIS] = soft_endstop_min[X_AXIS];
            position_error = true;
        }
        if (target[Y_AXIS]< soft_endstop_min[Y_AXIS])
        {
            target[Y_AXIS] = soft_endstop_min[Y_AXIS];
            position_error = true;
        }
        if (target[Z_AXIS]< soft_endstop_min[Z_AXIS])
        {
            target[Z_AXIS] = soft_endstop_min[Z_AXIS];
            position_error = true;
        }
      #else
        NOLESS(target[X_AXIS], soft_endstop_min[X_AXIS]);
        NOLESS(target[Y_AXIS], soft_endstop_min[Y_AXIS]);
        NOLESS(target[Z_AXIS], soft_endstop_min[Z_AXIS]);
      #endif
    #endif
    #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
      #if FYS_POSITION_ERR
        if (target[X_AXIS]> soft_endstop_max[X_AXIS])
        {
            target[X_AXIS] = soft_endstop_max[X_AXIS];
            position_error = true;
        }
        if (target[Y_AXIS]> soft_endstop_max[Y_AXIS])
        {
            target[Y_AXIS] = soft_endstop_max[Y_AXIS];
            position_error = true;
        }
        if (target[Z_AXIS]> soft_endstop_max[Z_AXIS])
        {
            target[Z_AXIS] = soft_endstop_max[Z_AXIS];
        }
      #else
        NOMORE(target[X_AXIS], soft_endstop_max[X_AXIS]);
        NOMORE(target[Y_AXIS], soft_endstop_max[Y_AXIS]);
        NOMORE(target[Z_AXIS], soft_endstop_max[Z_AXIS]);
      #endif
    #endif
  }
#endif
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #if ENABLED(ABL_BILINEAR_SUBDIVISION)
    #define ABL_BG_SPACING(A) bilinear_grid_spacing_virt[A]
    #define ABL_BG_FACTOR(A)  bilinear_grid_factor_virt[A]
    #define ABL_BG_POINTS_X   ABL_GRID_POINTS_VIRT_X
    #define ABL_BG_POINTS_Y   ABL_GRID_POINTS_VIRT_Y
    #define ABL_BG_GRID(X,Y)  z_values_virt[X][Y]
  #else
    #define ABL_BG_SPACING(A) bilinear_grid_spacing[A]
    #define ABL_BG_FACTOR(A)  bilinear_grid_factor[A]
    #define ABL_BG_POINTS_X   GRID_MAX_POINTS_X
    #define ABL_BG_POINTS_Y   GRID_MAX_POINTS_Y
    #define ABL_BG_GRID(X,Y)  z_values[X][Y]
  #endif
  float bilinear_z_offset(const float logical[XYZ]) {
    static float z1, d2, z3, d4, L, D, ratio_x, ratio_y,
                 last_x = -999.999, last_y = -999.999;
    static int8_t gridx, gridy, nextx, nexty,
                  last_gridx = -99, last_gridy = -99;
    const float x = RAW_X_POSITION(logical[X_AXIS]) - bilinear_start[X_AXIS],
                y = RAW_Y_POSITION(logical[Y_AXIS]) - bilinear_start[Y_AXIS];
    #if ENABLED(EXTRAPOLATE_BEYOND_GRID)
      #define FAR_EDGE_OR_BOX 2
    #else
      #define FAR_EDGE_OR_BOX 1
    #endif
    if (last_x != x) {
      last_x = x;
      ratio_x = x * ABL_BG_FACTOR(X_AXIS);
      const float gx = constrain(FLOOR(ratio_x), 0, ABL_BG_POINTS_X - FAR_EDGE_OR_BOX);
      ratio_x -= gx;      
      #if DISABLED(EXTRAPOLATE_BEYOND_GRID)
        NOLESS(ratio_x, 0); 
      #endif
      gridx = gx;
      nextx = min(gridx + 1, ABL_BG_POINTS_X - 1);
    }
    if (last_y != y || last_gridx != gridx) {
      if (last_y != y) {
        last_y = y;
        ratio_y = y * ABL_BG_FACTOR(Y_AXIS);
        const float gy = constrain(FLOOR(ratio_y), 0, ABL_BG_POINTS_Y - FAR_EDGE_OR_BOX);
        ratio_y -= gy;
        #if DISABLED(EXTRAPOLATE_BEYOND_GRID)
          NOLESS(ratio_y, 0); 
        #endif
        gridy = gy;
        nexty = min(gridy + 1, ABL_BG_POINTS_Y - 1);
      }
      if (last_gridx != gridx || last_gridy != gridy) {
        last_gridx = gridx;
        last_gridy = gridy;
        z1 = ABL_BG_GRID(gridx, gridy);       
        d2 = ABL_BG_GRID(gridx, nexty) - z1;  
        z3 = ABL_BG_GRID(nextx, gridy);       
        d4 = ABL_BG_GRID(nextx, nexty) - z3;  
      }
                  L = z1 + d2 * ratio_y;   
      const float R = z3 + d4 * ratio_y;   
      D = R - L;
    }
    const float offset = L + ratio_x * D;   
    return offset;
  }
#endif 
#if ENABLED(DELTA)
  void recalc_delta_settings(float radius, float diagonal_rod) {
    const float trt[ABC] = DELTA_RADIUS_TRIM_TOWER,
                drt[ABC] = DELTA_DIAGONAL_ROD_TRIM_TOWER;
    delta_tower[A_AXIS][X_AXIS] = cos(RADIANS(210 + delta_tower_angle_trim[A_AXIS])) * (radius + trt[A_AXIS]); 
    delta_tower[A_AXIS][Y_AXIS] = sin(RADIANS(210 + delta_tower_angle_trim[A_AXIS])) * (radius + trt[A_AXIS]);
    delta_tower[B_AXIS][X_AXIS] = cos(RADIANS(330 + delta_tower_angle_trim[B_AXIS])) * (radius + trt[B_AXIS]); 
    delta_tower[B_AXIS][Y_AXIS] = sin(RADIANS(330 + delta_tower_angle_trim[B_AXIS])) * (radius + trt[B_AXIS]);
    delta_tower[C_AXIS][X_AXIS] = 0.0; 
    delta_tower[C_AXIS][Y_AXIS] = (radius + trt[C_AXIS]);
    delta_diagonal_rod_2_tower[A_AXIS] = sq(diagonal_rod + drt[A_AXIS]);
    delta_diagonal_rod_2_tower[B_AXIS] = sq(diagonal_rod + drt[B_AXIS]);
    delta_diagonal_rod_2_tower[C_AXIS] = sq(diagonal_rod + drt[C_AXIS]);
  }
  #if ENABLED(DELTA_FAST_SQRT)
    float Q_rsqrt(float number) {
      long i;
      float x2, y;
      const float threehalfs = 1.5f;
      x2 = number * 0.5f;
      y  = number;
      i  = * ( long * ) &y;                       
      i  = 0x5F3759DF - ( i >> 1 );               
      y  = * ( float * ) &i;
      y  = y * ( threehalfs - ( x2 * y * y ) );   
      return y;
    }
    #define _SQRT(n) (1.0f / Q_rsqrt(n))
  #else
    #define _SQRT(n) SQRT(n)
  #endif
  #define DELTA_Z(T) raw[Z_AXIS] + _SQRT(     \
    delta_diagonal_rod_2_tower[T] - HYPOT2(   \
        delta_tower[T][X_AXIS] - raw[X_AXIS], \
        delta_tower[T][Y_AXIS] - raw[Y_AXIS]  \
      )                                       \
    )
  #define DELTA_RAW_IK() do {        \
    delta[A_AXIS] = DELTA_Z(A_AXIS); \
    delta[B_AXIS] = DELTA_Z(B_AXIS); \
    delta[C_AXIS] = DELTA_Z(C_AXIS); \
  }while(0)
  #define DELTA_LOGICAL_IK() do {      \
    const float raw[XYZ] = {           \
      RAW_X_POSITION(logical[X_AXIS]), \
      RAW_Y_POSITION(logical[Y_AXIS]), \
      RAW_Z_POSITION(logical[Z_AXIS])  \
    };                                 \
    DELTA_RAW_IK();                    \
  }while(0)
  #define DELTA_DEBUG() do { \
      SERIAL_ECHOPAIR("cartesian X:", raw[X_AXIS]); \
      SERIAL_ECHOPAIR(" Y:", raw[Y_AXIS]);          \
      SERIAL_ECHOLNPAIR(" Z:", raw[Z_AXIS]);        \
      SERIAL_ECHOPAIR("delta A:", delta[A_AXIS]);   \
      SERIAL_ECHOPAIR(" B:", delta[B_AXIS]);        \
      SERIAL_ECHOLNPAIR(" C:", delta[C_AXIS]);      \
    }while(0)
  void inverse_kinematics(const float logical[XYZ]) {
    DELTA_LOGICAL_IK();
  }
  float delta_safe_distance_from_top() {
    float cartesian[XYZ] = {
      LOGICAL_X_POSITION(0),
      LOGICAL_Y_POSITION(0),
      LOGICAL_Z_POSITION(0)
    };
    inverse_kinematics(cartesian);
    float distance = delta[A_AXIS];
    cartesian[Y_AXIS] = LOGICAL_Y_POSITION(DELTA_PRINTABLE_RADIUS);
    inverse_kinematics(cartesian);
    return FABS(distance - delta[A_AXIS]);
  }
  void forward_kinematics_DELTA(float z1, float z2, float z3) {
    float p12[3] = { delta_tower[B_AXIS][X_AXIS] - delta_tower[A_AXIS][X_AXIS], delta_tower[B_AXIS][Y_AXIS] - delta_tower[A_AXIS][Y_AXIS], z2 - z1 };
    float d = SQRT( sq(p12[0]) + sq(p12[1]) + sq(p12[2]) );
    float ex[3] = { p12[0] / d, p12[1] / d, p12[2] / d };
    float p13[3] = { delta_tower[C_AXIS][X_AXIS] - delta_tower[A_AXIS][X_AXIS], delta_tower[C_AXIS][Y_AXIS] - delta_tower[A_AXIS][Y_AXIS], z3 - z1 };
    float i = ex[0] * p13[0] + ex[1] * p13[1] + ex[2] * p13[2];
    float iex[3] = { ex[0] * i, ex[1] * i, ex[2] * i };
    float ey[3] = { p13[0] - iex[0], p13[1] - iex[1], p13[2] - iex[2] };
    float j = SQRT( sq(ey[0]) + sq(ey[1]) + sq(ey[2]) );
    ey[0] /= j; ey[1] /= j;  ey[2] /= j;
    float ez[3] = {
      ex[1] * ey[2] - ex[2] * ey[1],
      ex[2] * ey[0] - ex[0] * ey[2],
      ex[0] * ey[1] - ex[1] * ey[0]
    };
    float Xnew = (delta_diagonal_rod_2_tower[A_AXIS] - delta_diagonal_rod_2_tower[B_AXIS] + sq(d)) / (d * 2),
          Ynew = ((delta_diagonal_rod_2_tower[A_AXIS] - delta_diagonal_rod_2_tower[C_AXIS] + HYPOT2(i, j)) / 2 - i * Xnew) / j,
          Znew = SQRT(delta_diagonal_rod_2_tower[A_AXIS] - HYPOT2(Xnew, Ynew));
    cartes[X_AXIS] = delta_tower[A_AXIS][X_AXIS] + ex[0] * Xnew + ey[0] * Ynew - ez[0] * Znew;
    cartes[Y_AXIS] = delta_tower[A_AXIS][Y_AXIS] + ex[1] * Xnew + ey[1] * Ynew - ez[1] * Znew;
    cartes[Z_AXIS] =             z1 + ex[2] * Xnew + ey[2] * Ynew - ez[2] * Znew;
  }
  void forward_kinematics_DELTA(float point[ABC]) {
    forward_kinematics_DELTA(point[A_AXIS], point[B_AXIS], point[C_AXIS]);
  }
#endif 
void get_cartesian_from_steppers() {
  #if ENABLED(DELTA)
    forward_kinematics_DELTA(
      stepper.get_axis_position_mm(A_AXIS),
      stepper.get_axis_position_mm(B_AXIS),
      stepper.get_axis_position_mm(C_AXIS)
    );
    cartes[X_AXIS] += LOGICAL_X_POSITION(0);
    cartes[Y_AXIS] += LOGICAL_Y_POSITION(0);
    cartes[Z_AXIS] += LOGICAL_Z_POSITION(0);
  #elif IS_SCARA
    forward_kinematics_SCARA(
      stepper.get_axis_position_degrees(A_AXIS),
      stepper.get_axis_position_degrees(B_AXIS)
    );
    cartes[X_AXIS] += LOGICAL_X_POSITION(0);
    cartes[Y_AXIS] += LOGICAL_Y_POSITION(0);
    cartes[Z_AXIS] = stepper.get_axis_position_mm(Z_AXIS);
  #else
    cartes[X_AXIS] = stepper.get_axis_position_mm(X_AXIS);
    cartes[Y_AXIS] = stepper.get_axis_position_mm(Y_AXIS);
    cartes[Z_AXIS] = stepper.get_axis_position_mm(Z_AXIS);
  #endif
}
void set_current_from_steppers_for_axis(const AxisEnum axis) {
  get_cartesian_from_steppers();
  #if PLANNER_LEVELING
    planner.unapply_leveling(cartes);
  #endif
  if (axis == ALL_AXES)
    COPY(current_position, cartes);
  else
    current_position[axis] = cartes[axis];
}
#if ENABLED(MESH_BED_LEVELING)
  void mesh_line_to_destination(float fr_mm_s, uint8_t x_splits = 0xFF, uint8_t y_splits = 0xFF) {
    int cx1 = mbl.cell_index_x(RAW_CURRENT_POSITION(X)),
        cy1 = mbl.cell_index_y(RAW_CURRENT_POSITION(Y)),
        cx2 = mbl.cell_index_x(RAW_X_POSITION(destination[X_AXIS])),
        cy2 = mbl.cell_index_y(RAW_Y_POSITION(destination[Y_AXIS]));
    NOMORE(cx1, GRID_MAX_POINTS_X - 2);
    NOMORE(cy1, GRID_MAX_POINTS_Y - 2);
    NOMORE(cx2, GRID_MAX_POINTS_X - 2);
    NOMORE(cy2, GRID_MAX_POINTS_Y - 2);
    if (cx1 == cx2 && cy1 == cy2) {
      line_to_destination(fr_mm_s);
      set_current_to_destination();
      return;
    }
    #define MBL_SEGMENT_END(A) (current_position[A ##_AXIS] + (destination[A ##_AXIS] - current_position[A ##_AXIS]) * normalized_dist)
    float normalized_dist, end[XYZE];
    const int8_t gcx = max(cx1, cx2), gcy = max(cy1, cy2);
    if (cx2 != cx1 && TEST(x_splits, gcx)) {
      COPY(end, destination);
      destination[X_AXIS] = LOGICAL_X_POSITION(mbl.index_to_xpos[gcx]);
      normalized_dist = (destination[X_AXIS] - current_position[X_AXIS]) / (end[X_AXIS] - current_position[X_AXIS]);
      destination[Y_AXIS] = MBL_SEGMENT_END(Y);
      CBI(x_splits, gcx);
    }
    else if (cy2 != cy1 && TEST(y_splits, gcy)) {
      COPY(end, destination);
      destination[Y_AXIS] = LOGICAL_Y_POSITION(mbl.index_to_ypos[gcy]);
      normalized_dist = (destination[Y_AXIS] - current_position[Y_AXIS]) / (end[Y_AXIS] - current_position[Y_AXIS]);
      destination[X_AXIS] = MBL_SEGMENT_END(X);
      CBI(y_splits, gcy);
    }
    else {
      line_to_destination(fr_mm_s);
      set_current_to_destination();
      return;
    }
    destination[Z_AXIS] = MBL_SEGMENT_END(Z);
    destination[E_AXIS] = MBL_SEGMENT_END(E);
    mesh_line_to_destination(fr_mm_s, x_splits, y_splits);
    COPY(destination, end);
    mesh_line_to_destination(fr_mm_s, x_splits, y_splits);
  }
#elif ENABLED(AUTO_BED_LEVELING_BILINEAR) && !IS_KINEMATIC
  #define CELL_INDEX(A,V) ((RAW_##A##_POSITION(V) - bilinear_start[A##_AXIS]) * ABL_BG_FACTOR(A##_AXIS))
  void bilinear_line_to_destination(float fr_mm_s, uint16_t x_splits = 0xFFFF, uint16_t y_splits = 0xFFFF) {
    int cx1 = CELL_INDEX(X, current_position[X_AXIS]),
        cy1 = CELL_INDEX(Y, current_position[Y_AXIS]),
        cx2 = CELL_INDEX(X, destination[X_AXIS]),
        cy2 = CELL_INDEX(Y, destination[Y_AXIS]);
    cx1 = constrain(cx1, 0, ABL_BG_POINTS_X - 2);
    cy1 = constrain(cy1, 0, ABL_BG_POINTS_Y - 2);
    cx2 = constrain(cx2, 0, ABL_BG_POINTS_X - 2);
    cy2 = constrain(cy2, 0, ABL_BG_POINTS_Y - 2);
    if (cx1 == cx2 && cy1 == cy2) {
      line_to_destination(fr_mm_s);
      set_current_to_destination();
      return;
    }
    #define LINE_SEGMENT_END(A) (current_position[A ##_AXIS] + (destination[A ##_AXIS] - current_position[A ##_AXIS]) * normalized_dist)
    float normalized_dist, end[XYZE];
    const int8_t gcx = max(cx1, cx2), gcy = max(cy1, cy2);
    if (cx2 != cx1 && TEST(x_splits, gcx)) {
      COPY(end, destination);
      destination[X_AXIS] = LOGICAL_X_POSITION(bilinear_start[X_AXIS] + ABL_BG_SPACING(X_AXIS) * gcx);
      normalized_dist = (destination[X_AXIS] - current_position[X_AXIS]) / (end[X_AXIS] - current_position[X_AXIS]);
      destination[Y_AXIS] = LINE_SEGMENT_END(Y);
      CBI(x_splits, gcx);
    }
    else if (cy2 != cy1 && TEST(y_splits, gcy)) {
      COPY(end, destination);
      destination[Y_AXIS] = LOGICAL_Y_POSITION(bilinear_start[Y_AXIS] + ABL_BG_SPACING(Y_AXIS) * gcy);
      normalized_dist = (destination[Y_AXIS] - current_position[Y_AXIS]) / (end[Y_AXIS] - current_position[Y_AXIS]);
      destination[X_AXIS] = LINE_SEGMENT_END(X);
      CBI(y_splits, gcy);
    }
    else {
      line_to_destination(fr_mm_s);
      set_current_to_destination();
      return;
    }
    destination[Z_AXIS] = LINE_SEGMENT_END(Z);
    destination[E_AXIS] = LINE_SEGMENT_END(E);
    bilinear_line_to_destination(fr_mm_s, x_splits, y_splits);
    COPY(destination, end);
    bilinear_line_to_destination(fr_mm_s, x_splits, y_splits);
  }
#endif 
#if IS_KINEMATIC && !UBL_DELTA
  inline bool prepare_kinematic_move_to(float ltarget[XYZE]) {
    const float _feedrate_mm_s = MMS_SCALED(feedrate_mm_s);
    if (ltarget[X_AXIS] == current_position[X_AXIS] && ltarget[Y_AXIS] == current_position[Y_AXIS]) {
      planner.buffer_line_kinematic(ltarget, _feedrate_mm_s, active_extruder);
      return false;
    }
    if (!position_is_reachable_xy(ltarget[X_AXIS], ltarget[Y_AXIS])) return true;
    const float difference[XYZE] = {
      ltarget[X_AXIS] - current_position[X_AXIS],
      ltarget[Y_AXIS] - current_position[Y_AXIS],
      ltarget[Z_AXIS] - current_position[Z_AXIS],
      ltarget[E_AXIS] - current_position[E_AXIS]
    };
    float cartesian_mm = SQRT(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));
    if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = FABS(difference[E_AXIS]);
    if (UNEAR_ZERO(cartesian_mm)) return true;
    const float seconds = cartesian_mm / _feedrate_mm_s;
    uint16_t segments = delta_segments_per_second * seconds;
    #if IS_SCARA
      NOMORE(segments, cartesian_mm * 4);
    #endif
    NOLESS(segments, 1);
    const float inv_segments = 1.0 / float(segments),
                segment_distance[XYZE] = {
                  difference[X_AXIS] * inv_segments,
                  difference[Y_AXIS] * inv_segments,
                  difference[Z_AXIS] * inv_segments,
                  difference[E_AXIS] * inv_segments
                };
    #if IS_SCARA && ENABLED(SCARA_FEEDRATE_SCALING)
      const float inv_segment_length = min(10.0, float(segments) / cartesian_mm), 
                  feed_factor = inv_segment_length * _feedrate_mm_s;
      float oldA = stepper.get_axis_position_degrees(A_AXIS),
            oldB = stepper.get_axis_position_degrees(B_AXIS);
    #endif
    float logical[XYZE];
    COPY(logical, current_position);
    --segments;
    for (uint16_t s = segments + 1; --s;) {
      LOOP_XYZE(i) logical[i] += segment_distance[i];
      #if ENABLED(DELTA)
        DELTA_LOGICAL_IK(); 
      #else
        inverse_kinematics(logical);
      #endif
      ADJUST_DELTA(logical); 
      #if IS_SCARA && ENABLED(SCARA_FEEDRATE_SCALING)
        const float adiff = abs(delta[A_AXIS] - oldA),
                    bdiff = abs(delta[B_AXIS] - oldB);
        planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], logical[E_AXIS], max(adiff, bdiff) * feed_factor, active_extruder);
        oldA = delta[A_AXIS];
        oldB = delta[B_AXIS];
      #else
        planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], logical[E_AXIS], _feedrate_mm_s, active_extruder);
      #endif
    }
    #if IS_SCARA && ENABLED(SCARA_FEEDRATE_SCALING)
      inverse_kinematics(ltarget);
      ADJUST_DELTA(ltarget);
      const float adiff = abs(delta[A_AXIS] - oldA),
                  bdiff = abs(delta[B_AXIS] - oldB);
      planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], logical[E_AXIS], max(adiff, bdiff) * feed_factor, active_extruder);
    #else
      planner.buffer_line_kinematic(ltarget, _feedrate_mm_s, active_extruder);
    #endif
    return false;
  }
#else 
  inline bool prepare_move_to_destination_cartesian() {
    #if ENABLED(AUTO_BED_LEVELING_UBL)
      const float fr_scaled = MMS_SCALED(feedrate_mm_s);
      if (ubl.state.active) { 
        ubl.line_to_destination_cartesian(fr_scaled, active_extruder);
        return true;
      }
      else
        line_to_destination(fr_scaled);
    #else
      if (current_position[X_AXIS] == destination[X_AXIS] && current_position[Y_AXIS] == destination[Y_AXIS])
        line_to_destination();
      else {
        const float fr_scaled = MMS_SCALED(feedrate_mm_s);
        #if ENABLED(MESH_BED_LEVELING)
          if (mbl.active()) { 
            mesh_line_to_destination(fr_scaled);
            return true;
          }
          else
        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
          if (planner.abl_enabled) { 
            bilinear_line_to_destination(fr_scaled);
            return true;
          }
          else
        #endif
            line_to_destination(fr_scaled);
      }
    #endif
    return false;
  }
#endif 
#if ENABLED(DUAL_X_CARRIAGE)
  inline bool prepare_move_to_destination_dualx() {
    if (active_extruder_parked) {
      switch (dual_x_carriage_mode) {
        case DXC_FULL_CONTROL_MODE:
          break;
        case DXC_AUTO_PARK_MODE:
        #if ENABLED(FYS_DXC_FYS_MODE)
          case DXC_FYS_MODE: 
        #endif
          if (current_position[E_AXIS] == destination[E_AXIS]) {
            if (delayed_move_time != 0xFFFFFFFFUL) {
              set_current_to_destination();
              NOLESS(raised_parked_position[Z_AXIS], destination[Z_AXIS]);
              delayed_move_time = millis();
              return true;
            }
          }
          for (uint8_t i = 0; i < 3; i++)
            planner.buffer_line(
              i == 0 ? raised_parked_position[X_AXIS] : current_position[X_AXIS],
              i == 0 ? raised_parked_position[Y_AXIS] : current_position[Y_AXIS],
              i == 2 ? current_position[Z_AXIS] : raised_parked_position[Z_AXIS],
              current_position[E_AXIS],
              i == 1 ? PLANNER_XY_FEEDRATE() : planner.max_feedrate_mm_s[Z_AXIS],
              active_extruder
            );
          delayed_move_time = 0;
          active_extruder_parked = false;
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Clear active_extruder_parked");
          #endif
          break;
        case DXC_DUPLICATION_MODE:
          if (active_extruder == 0) {
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) {
                SERIAL_ECHOPAIR("Set planner X", LOGICAL_X_POSITION(inactive_extruder_x_pos));
                SERIAL_ECHOLNPAIR(" ... Line to X", current_position[X_AXIS] + duplicate_extruder_x_offset);
              }
            #endif
            planner.set_position_mm(
              LOGICAL_X_POSITION(inactive_extruder_x_pos),
              current_position[Y_AXIS],
              current_position[Z_AXIS],
              current_position[E_AXIS]
            );
            planner.buffer_line(
              current_position[X_AXIS] + duplicate_extruder_x_offset,
              current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],
              planner.max_feedrate_mm_s[X_AXIS], 1
            );
            SYNC_PLAN_POSITION_KINEMATIC();
            stepper.synchronize();
            extruder_duplication_enabled = true;
            active_extruder_parked = false;
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Set extruder_duplication_enabled\nClear active_extruder_parked");
            #endif
          }
          else {
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Active extruder not 0");
            #endif
          }
          break;
      }
    }
    return false;
  }
#endif 
void prepare_move_to_destination() {
  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();
  #if ENABLED(PREVENT_COLD_EXTRUSION)
    if (!DEBUGGING(DRYRUN)) {
      if (destination[E_AXIS] != current_position[E_AXIS]) {
        if (thermalManager.tooColdToExtrude(active_extruder)) {
          current_position[E_AXIS] = destination[E_AXIS]; 
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
        }
        #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
          if (destination[E_AXIS] - current_position[E_AXIS] > EXTRUDE_MAXLENGTH) {
            current_position[E_AXIS] = destination[E_AXIS]; 
            SERIAL_ECHO_START();
            SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
          }
        #endif
      }
    }
  #endif
  if (
    #if UBL_DELTA 
      ubl.prepare_segmented_line_to(destination, feedrate_mm_s)
    #elif IS_KINEMATIC
      prepare_kinematic_move_to(destination)
    #elif ENABLED(DUAL_X_CARRIAGE)
      prepare_move_to_destination_dualx()
    #else
      prepare_move_to_destination_cartesian()
    #endif
  ) return;
  set_current_to_destination();
}
#if ENABLED(ARC_SUPPORT)
  #if N_ARC_CORRECTION < 1
    #undef N_ARC_CORRECTION
    #define N_ARC_CORRECTION 1
  #endif
  void plan_arc(
    float logical[XYZE], 
    float *offset,       
    uint8_t clockwise    
  ) {
    #if ENABLED(CNC_WORKSPACE_PLANES)
      AxisEnum p_axis, q_axis, l_axis;
      switch (workspace_plane) {
        case PLANE_XY: p_axis = X_AXIS; q_axis = Y_AXIS; l_axis = Z_AXIS; break;
        case PLANE_ZX: p_axis = Z_AXIS; q_axis = X_AXIS; l_axis = Y_AXIS; break;
        case PLANE_YZ: p_axis = Y_AXIS; q_axis = Z_AXIS; l_axis = X_AXIS; break;
      }
    #else
      constexpr AxisEnum p_axis = X_AXIS, q_axis = Y_AXIS, l_axis = Z_AXIS;
    #endif
    float r_P = -offset[0], r_Q = -offset[1];
    const float radius = HYPOT(r_P, r_Q),
                center_P = current_position[p_axis] - r_P,
                center_Q = current_position[q_axis] - r_Q,
                rt_X = logical[p_axis] - center_P,
                rt_Y = logical[q_axis] - center_Q,
                linear_travel = logical[l_axis] - current_position[l_axis],
                extruder_travel = logical[E_AXIS] - current_position[E_AXIS];
    float angular_travel = ATAN2(r_P * rt_Y - r_Q * rt_X, r_P * rt_X + r_Q * rt_Y);
    if (angular_travel < 0) angular_travel += RADIANS(360);
    if (clockwise) angular_travel -= RADIANS(360);
    if (angular_travel == 0 && current_position[p_axis] == logical[p_axis] && current_position[q_axis] == logical[q_axis])
      angular_travel = RADIANS(360);
    const float mm_of_travel = HYPOT(angular_travel * radius, FABS(linear_travel));
    if (mm_of_travel < 0.001) return;
    uint16_t segments = FLOOR(mm_of_travel / (MM_PER_ARC_SEGMENT));
    if (segments == 0) segments = 1;
    float arc_target[XYZE];
    const float theta_per_segment = angular_travel / segments,
                linear_per_segment = linear_travel / segments,
                extruder_per_segment = extruder_travel / segments,
                sin_T = theta_per_segment,
                cos_T = 1 - 0.5 * sq(theta_per_segment); 
    arc_target[l_axis] = current_position[l_axis];
    arc_target[E_AXIS] = current_position[E_AXIS];
    const float fr_mm_s = MMS_SCALED(feedrate_mm_s);
    millis_t next_idle_ms = millis() + 200UL;
    #if N_ARC_CORRECTION > 1
      int8_t count = N_ARC_CORRECTION;
    #endif
    for (uint16_t i = 1; i < segments; i++) { 
      thermalManager.manage_heater();
      if (ELAPSED(millis(), next_idle_ms)) {
        next_idle_ms = millis() + 200UL;
        idle();
      }
      #if N_ARC_CORRECTION > 1
        if (--count) {
          const float r_new_Y = r_P * sin_T + r_Q * cos_T;
          r_P = r_P * cos_T - r_Q * sin_T;
          r_Q = r_new_Y;
        }
        else
      #endif
      {
        #if N_ARC_CORRECTION > 1
          count = N_ARC_CORRECTION;
        #endif
        const float cos_Ti = cos(i * theta_per_segment), sin_Ti = sin(i * theta_per_segment);
        r_P = -offset[0] * cos_Ti + offset[1] * sin_Ti;
        r_Q = -offset[0] * sin_Ti - offset[1] * cos_Ti;
      }
      arc_target[p_axis] = center_P + r_P;
      arc_target[q_axis] = center_Q + r_Q;
      arc_target[l_axis] += linear_per_segment;
      arc_target[E_AXIS] += extruder_per_segment;
      clamp_to_software_endstops(arc_target);
      planner.buffer_line_kinematic(arc_target, fr_mm_s, active_extruder);
    }
    planner.buffer_line_kinematic(logical, fr_mm_s, active_extruder);
    set_current_to_destination();
  }
#endif
#if ENABLED(BEZIER_CURVE_SUPPORT)
  void plan_cubic_move(const float offset[4]) {
    cubic_b_spline(current_position, destination, offset, MMS_SCALED(feedrate_mm_s), active_extruder);
    set_current_to_destination();
  }
#endif 
#if ENABLED(USE_CONTROLLER_FAN)
  void controllerFan() {
    static millis_t lastMotorOn = 0, 
                    nextMotorCheck = 0; 
    const millis_t ms = millis();
    if (ELAPSED(ms, nextMotorCheck)) {
      nextMotorCheck = ms + 2500UL; 
      if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON || thermalManager.soft_pwm_amount_bed > 0
          || E0_ENABLE_READ == E_ENABLE_ON 
          #if E_STEPPERS > 1
            || E1_ENABLE_READ == E_ENABLE_ON
            #if HAS_X2_ENABLE
              || X2_ENABLE_READ == X_ENABLE_ON
            #endif
            #if E_STEPPERS > 2
              || E2_ENABLE_READ == E_ENABLE_ON
              #if E_STEPPERS > 3
                || E3_ENABLE_READ == E_ENABLE_ON
                #if E_STEPPERS > 4
                  || E4_ENABLE_READ == E_ENABLE_ON
                #endif 
              #endif 
            #endif 
          #endif 
      ) {
        lastMotorOn = ms; 
      }
      uint8_t speed = (!lastMotorOn || ELAPSED(ms, lastMotorOn + (CONTROLLERFAN_SECS) * 1000UL)) ? 0 : CONTROLLERFAN_SPEED;
      WRITE(CONTROLLER_FAN_PIN, speed);
      analogWrite(CONTROLLER_FAN_PIN, speed);
    }
  }
#endif 
#if ENABLED(MORGAN_SCARA)
  void forward_kinematics_SCARA(const float &a, const float &b) {
    float a_sin = sin(RADIANS(a)) * L1,
          a_cos = cos(RADIANS(a)) * L1,
          b_sin = sin(RADIANS(b)) * L2,
          b_cos = cos(RADIANS(b)) * L2;
    cartes[X_AXIS] = a_cos + b_cos + SCARA_OFFSET_X;  
    cartes[Y_AXIS] = a_sin + b_sin + SCARA_OFFSET_Y;  
  }
  void inverse_kinematics(const float logical[XYZ]) {
    static float C2, S2, SK1, SK2, THETA, PSI;
    float sx = RAW_X_POSITION(logical[X_AXIS]) - SCARA_OFFSET_X,  
          sy = RAW_Y_POSITION(logical[Y_AXIS]) - SCARA_OFFSET_Y;  
    if (L1 == L2)
      C2 = HYPOT2(sx, sy) / L1_2_2 - 1;
    else
      C2 = (HYPOT2(sx, sy) - (L1_2 + L2_2)) / (2.0 * L1 * L2);
    S2 = SQRT(1 - sq(C2));
    SK1 = L1 + L2 * C2;
    SK2 = L2 * S2;
    THETA = ATAN2(SK1, SK2) - ATAN2(sx, sy);
    PSI = ATAN2(S2, C2);
    delta[A_AXIS] = DEGREES(THETA);        
    delta[B_AXIS] = DEGREES(THETA + PSI);  
    delta[C_AXIS] = logical[Z_AXIS];
  }
#endif 
#if ENABLED(TEMP_STAT_LEDS)
  static bool red_led = false;
  static millis_t next_status_led_update_ms = 0;
  void handle_status_leds(void) {
    if (ELAPSED(millis(), next_status_led_update_ms)) {
      next_status_led_update_ms += 500; 
      float max_temp = 0.0;
      #if HAS_TEMP_BED
        max_temp = MAX3(max_temp, thermalManager.degTargetBed(), thermalManager.degBed());
      #endif
      HOTEND_LOOP()
        max_temp = MAX3(max_temp, thermalManager.degHotend(e), thermalManager.degTargetHotend(e));
      const bool new_led = (max_temp > 55.0) ? true : (max_temp < 54.0) ? false : red_led;
      if (new_led != red_led) {
        red_led = new_led;
        #if PIN_EXISTS(STAT_LED_RED)
          WRITE(STAT_LED_RED_PIN, new_led ? HIGH : LOW);
          #if PIN_EXISTS(STAT_LED_BLUE)
            WRITE(STAT_LED_BLUE_PIN, new_led ? LOW : HIGH);
          #endif
        #else
          WRITE(STAT_LED_BLUE_PIN, new_led ? HIGH : LOW);
        #endif
      }
    }
  }
#endif
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  void handle_filament_runout() {
    if (!filament_ran_out) {
      filament_ran_out = true;
      #if ENABLED(FYS_MIX_FIL_RANOUT_SAFE_PRINT_BREAK)
      filament_ran_out_still_printing = true; 
      #endif
      enqueue_and_echo_commands_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
      stepper.synchronize();
    }
  }
#endif 
#if ENABLED(FAST_PWM_FAN)
  void setPwmFrequency(uint8_t pin, int val) {
    val &= 0x07;
    switch (digitalPinToTimer(pin)) {
      #ifdef TCCR0A
        #if !AVR_AT90USB1286_FAMILY
          case TIMER0A:
        #endif
        case TIMER0B:
          break;
      #endif
      #ifdef TCCR1A
        case TIMER1A:
        case TIMER1B:
          break;
      #endif
      #ifdef TCCR2
        case TIMER2:
        case TIMER2:
          _SET_CS(2, val);
          break;
      #endif
      #ifdef TCCR2A
        case TIMER2A:
        case TIMER2B:
          _SET_CS(2, val);
          break;
      #endif
      #ifdef TCCR3A
        case TIMER3A:
        case TIMER3B:
        case TIMER3C:
          _SET_CS(3, val);
          break;
      #endif
      #ifdef TCCR4A
        case TIMER4A:
        case TIMER4B:
        case TIMER4C:
          _SET_CS(4, val);
          break;
      #endif
      #ifdef TCCR5A
        case TIMER5A:
        case TIMER5B:
        case TIMER5C:
          _SET_CS(5, val);
          break;
      #endif
    }
  }
#endif 
float calculate_volumetric_multiplier(float diameter) {
  if (!volumetric_enabled || diameter == 0) return 1.0;
  return 1.0 / (M_PI * sq(diameter * 0.5));
}
void calculate_volumetric_multipliers() {
  for (uint8_t i = 0; i < COUNT(filament_size); i++)
    volumetric_multiplier[i] = calculate_volumetric_multiplier(filament_size[i]);
}
void enable_all_steppers() {
  enable_X();
  enable_Y();
  enable_Z();
  enable_E0();
  enable_E1();
  enable_E2();
  enable_E3();
  enable_E4();
}
void disable_e_steppers() {
  disable_E0();
  disable_E1();
  disable_E2();
  disable_E3();
  disable_E4();
}
void disable_all_steppers() {
  disable_X();
  disable_Y();
  disable_Z();
  disable_e_steppers();
}
#if ENABLED(HAVE_TMC2130)
  void automatic_current_control(TMC2130Stepper &st, String axisID) {
    const bool is_otpw = st.checkOT();
    static bool previous_otpw = false;
    if (is_otpw && !previous_otpw) {
      char timestamp[10];
      duration_t elapsed = print_job_timer.duration();
      const bool has_days = (elapsed.value > 60*60*24L);
      (void)elapsed.toDigital(timestamp, has_days);
      SERIAL_ECHO(timestamp);
      SERIAL_ECHOPGM(": ");
      SERIAL_ECHO(axisID);
      SERIAL_ECHOLNPGM(" driver overtemperature warning!");
    }
    previous_otpw = is_otpw;
    #if CURRENT_STEP > 0 && ENABLED(AUTOMATIC_CURRENT_CONTROL)
      if (!auto_current_control) return;
      uint16_t current = st.getCurrent();
      if (is_otpw) {
        st.setCurrent(current - CURRENT_STEP, R_SENSE, HOLD_MULTIPLIER);
        #if ENABLED(REPORT_CURRENT_CHANGE)
          SERIAL_ECHO(axisID);
          SERIAL_ECHOPAIR(" current decreased to ", st.getCurrent());
        #endif
      }
      else if (!st.isEnabled())
        return;
      else if (!is_otpw && !st.getOTPW()) {
        current += CURRENT_STEP;
        if (current <= AUTO_ADJUST_MAX) {
          st.setCurrent(current, R_SENSE, HOLD_MULTIPLIER);
          #if ENABLED(REPORT_CURRENT_CHANGE)
            SERIAL_ECHO(axisID);
            SERIAL_ECHOPAIR(" current increased to ", st.getCurrent());
          #endif
        }
      }
      SERIAL_EOL();
    #endif
  }
  void checkOverTemp() {
    static millis_t next_cOT = 0;
    if (ELAPSED(millis(), next_cOT)) {
      next_cOT = millis() + 5000;
      #if ENABLED(X_IS_TMC2130)
        automatic_current_control(stepperX, "X");
      #endif
      #if ENABLED(Y_IS_TMC2130)
        automatic_current_control(stepperY, "Y");
      #endif
      #if ENABLED(Z_IS_TMC2130)
        automatic_current_control(stepperZ, "Z");
      #endif
      #if ENABLED(X2_IS_TMC2130)
        automatic_current_control(stepperX2, "X2");
      #endif
      #if ENABLED(Y2_IS_TMC2130)
        automatic_current_control(stepperY2, "Y2");
      #endif
      #if ENABLED(Z2_IS_TMC2130)
        automatic_current_control(stepperZ2, "Z2");
      #endif
      #if ENABLED(E0_IS_TMC2130)
        automatic_current_control(stepperE0, "E0");
      #endif
      #if ENABLED(E1_IS_TMC2130)
        automatic_current_control(stepperE1, "E1");
      #endif
      #if ENABLED(E2_IS_TMC2130)
        automatic_current_control(stepperE2, "E2");
      #endif
      #if ENABLED(E3_IS_TMC2130)
        automatic_current_control(stepperE3, "E3");
      #endif
      #if ENABLED(E4_IS_TMC2130)
        automatic_current_control(stepperE4, "E4");
      #endif
      #if ENABLED(E4_IS_TMC2130)
        automatic_current_control(stepperE4);
      #endif
    }
  }
#endif 
void manage_inactivity(bool ignore_stepper_queue) {
  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    #if ENABLED(FYS_FILAMENT_RUNOUT_ADV)
      bool filStateNow = READ(FIL_RUNOUT_PIN);
      static bool filStateNowDelay = filStateNow;
      static bool bStartTiming=false;
      static millis_t msF=0;
      if(!bStartTiming && filStateNow!=filStateNowDelay) 
      {
          msF=millis();
          bStartTiming = true;
      }
      if(bStartTiming && filStateNow==filStateNowDelay)
      {
          msF=millis();
      }
      if(bStartTiming && millis() > msF+150) 
      {
          filStateNowDelay = filStateNow;
          bStartTiming = false;
      }
      if((IS_SD_PRINTING || print_job_timer.isRunning()) && (filStateOld!=FIL_RUNOUT_INVERTING && filStateNowDelay==FIL_RUNOUT_INVERTING))
      {
          handle_filament_runout();
      }
      filStateOld = filStateNowDelay;
    #elif ENABLED(FYS_FILAMENT_RUNOUT_ADV2)
      bool filStateNow = READ(FIL_RUNOUT_PIN);
      if((IS_SD_PRINTING || print_job_timer.isRunning()) && (filStateOld!=FIL_RUNOUT_INVERTING && filStateNow==FIL_RUNOUT_INVERTING))
      {
          handle_filament_runout();
      }
      filStateOld = filStateNow;
    #else 
      if ((IS_SD_PRINTING || print_job_timer.isRunning()) && (READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_INVERTING))
      {
          handle_filament_runout();
      }
    #endif
  #endif
  if (commands_in_queue < BUFSIZE) get_available_commands();
  const millis_t ms = millis();
#if defined(FYS_ACTIVE_TIME_OVER)
  if (max_inactive_time && ELAPSED(ms, previous_cmd_ms + max_inactive_time)){
      if(commands_in_queue||print_job_timer.isPaused())previous_cmd_ms = ms;
    #if PIN_EXISTS(PS_ON)&& ENABLED(FYS_POWER_STATUS)
      else GLOBAL_var_V003 = MACRO_var_V00A;
    #else
      else
      {
          SERIAL_ERROR_START();
          SERIAL_ECHOLNPAIR(MSG_KILL_INACTIVE_TIME, parser.command_ptr);
          FunV006("Kill caused by too much inactive time.");
          kill(PSTR(MSG_KILLED));
      }
    #endif
  }
#else
  if (max_inactive_time && ELAPSED(ms, previous_cmd_ms + max_inactive_time)) {
    SERIAL_ERROR_START();
    SERIAL_ECHOLNPAIR(MSG_KILL_INACTIVE_TIME, parser.command_ptr);
    #if ENABLED(FYS_LCD_EXTRA_INFO)
    FunV006("Kill caused by too much inactive time.");
    #endif
    kill(PSTR(MSG_KILLED));
  }
#endif
  #if ENABLED(ADVANCED_PAUSE_FEATURE) && ENABLED(PAUSE_PARK_NO_STEPPER_TIMEOUT)
    #define MOVE_AWAY_TEST !move_away_flag
  #else
    #define MOVE_AWAY_TEST true
  #endif
  if (MOVE_AWAY_TEST && stepper_inactive_time && ELAPSED(ms, previous_cmd_ms + stepper_inactive_time)
      && !ignore_stepper_queue && !planner.blocks_queued()) {
    #if ENABLED(DISABLE_INACTIVE_X)
      disable_X();
    #endif
    #if ENABLED(DISABLE_INACTIVE_Y)
      disable_Y();
    #endif
    #if ENABLED(DISABLE_INACTIVE_Z)
      disable_Z();
    #endif
    #if ENABLED(DISABLE_INACTIVE_E)
      disable_e_steppers();
    #endif
    #if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(ULTRA_LCD)  
      ubl_lcd_map_control = defer_return_to_status = false;
    #endif
  }
  #ifdef CHDK 
    if (chdkActive && ELAPSED(ms, chdkHigh + CHDK_DELAY)) {
      chdkActive = false;
      WRITE(CHDK, LOW);
    }
  #endif
  #if HAS_KILL
    static int killCount = 0;   
    const int KILL_DELAY = 750;
    if (!READ(KILL_PIN))
      killCount++;
    else if (killCount > 0)
      killCount--;
    if (killCount >= KILL_DELAY) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM(MSG_KILL_BUTTON);
      #if ENABLED(FYS_LCD_EXTRA_INFO)    
        FunV006(MSG_KILL_BUTTON);
      #endif
      kill(PSTR(MSG_KILLED));
    }
  #endif
  #if HAS_HOME
    static int homeDebounceCount = 0;   
    const int HOME_DEBOUNCE_DELAY = 2500;
    if (!IS_SD_PRINTING && !READ(HOME_PIN)) {
      if (!homeDebounceCount) {
        enqueue_and_echo_commands_P(PSTR("G28"));
        LCD_MESSAGEPGM(MSG_AUTO_HOME);
      }
      if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
        homeDebounceCount++;
      else
        homeDebounceCount = 0;
    }
  #endif
  #if ENABLED(USE_CONTROLLER_FAN)
    controllerFan(); 
  #endif
  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    if (ELAPSED(ms, previous_cmd_ms + (EXTRUDER_RUNOUT_SECONDS) * 1000UL)
      && thermalManager.degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP) {
      bool oldstatus;
      #if ENABLED(SWITCHING_EXTRUDER)
        oldstatus = E0_ENABLE_READ;
        enable_E0();
      #else 
        switch (active_extruder) {
          case 0: oldstatus = E0_ENABLE_READ; enable_E0(); break;
          #if E_STEPPERS > 1
            case 1: oldstatus = E1_ENABLE_READ; enable_E1(); break;
            #if E_STEPPERS > 2
              case 2: oldstatus = E2_ENABLE_READ; enable_E2(); break;
              #if E_STEPPERS > 3
                case 3: oldstatus = E3_ENABLE_READ; enable_E3(); break;
                #if E_STEPPERS > 4
                  case 4: oldstatus = E4_ENABLE_READ; enable_E4(); break;
                #endif 
              #endif 
            #endif 
          #endif 
        }
      #endif 
      previous_cmd_ms = ms; 
      const float olde = current_position[E_AXIS];
      current_position[E_AXIS] += EXTRUDER_RUNOUT_EXTRUDE;
      planner.buffer_line_kinematic(current_position, MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED), active_extruder);
      current_position[E_AXIS] = olde;
      planner.set_e_position_mm(olde);
      stepper.synchronize();
      #if ENABLED(SWITCHING_EXTRUDER)
        E0_ENABLE_WRITE(oldstatus);
      #else
        switch (active_extruder) {
          case 0: E0_ENABLE_WRITE(oldstatus); break;
          #if E_STEPPERS > 1
            case 1: E1_ENABLE_WRITE(oldstatus); break;
            #if E_STEPPERS > 2
              case 2: E2_ENABLE_WRITE(oldstatus); break;
              #if E_STEPPERS > 3
                case 3: E3_ENABLE_WRITE(oldstatus); break;
                #if E_STEPPERS > 4
                  case 4: E4_ENABLE_WRITE(oldstatus); break;
                #endif 
              #endif 
            #endif 
          #endif 
        }
      #endif 
    }
  #endif 
  #if ENABLED(DUAL_X_CARRIAGE)
    if (delayed_move_time && ELAPSED(ms, delayed_move_time + 1000UL) && IsRunning()) {
      delayed_move_time = 0xFFFFFFFFUL; 
      set_destination_to_current();
      prepare_move_to_destination();
    }
  #endif
  #if ENABLED(TEMP_STAT_LEDS)
    handle_status_leds();
  #endif
  #if ENABLED(HAVE_TMC2130)
    checkOverTemp();
  #endif
  planner.check_axes_activity();
}
void idle(
  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    bool no_stepper_sleep
  #endif
) {
  lcd_update();
#ifdef FYS_CONTROL_PROTOCOL
  fysControlProtocol();
#endif
  host_keepalive();
  #if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
    auto_report_temperatures();
  #endif
  manage_inactivity(
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      no_stepper_sleep
    #endif
  );
  thermalManager.manage_heater();
  #if ENABLED(PRINTCOUNTER)
    print_job_timer.tick();
  #endif
  #if HAS_BUZZER && DISABLED(LCD_USE_I2C_BUZZER)
    buzzer.tick();
  #endif
  #if ENABLED(I2C_POSITION_ENCODERS)
    if (planner.blocks_queued() &&
        ( (blockBufferIndexRef != planner.block_buffer_head) ||
          ((lastUpdateMillis + I2CPE_MIN_UPD_TIME_MS) < millis())) ) {
      blockBufferIndexRef = planner.block_buffer_head;
      I2CPEM.update();
      lastUpdateMillis = millis();
    }
  #endif
}
void kill(const char* lcd_msg) {
  SERIAL_ERROR_START();
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  thermalManager.disable_all_heaters();
  disable_all_steppers();
  #ifndef FYS_NO_KILL_ONLY_TEMP_LCD_INFO
      UNUSED(lcd_msg);
  #endif
  _delay_ms(600); 
  cli(); 
  _delay_ms(250); 
  thermalManager.disable_all_heaters(); 
  #if defined(ACTION_ON_KILL)
    SERIAL_ECHOLNPGM("//action:" ACTION_ON_KILL);
  #endif
  #if HAS_POWER_SWITCH
    SET_INPUT(PS_ON_PIN);
  #endif
  suicide();
  while (1) {
    #if ENABLED(USE_WATCHDOG)
      watchdog_reset();
    #endif
  } // Wait for reset
}
#if ENABLED(FYS_NO_KILL_ONLY_TEMP_LCD_INFO)
/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void kill_temp(const char* lcd_msg) {
  SERIAL_ERROR_START();
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  thermalManager.disable_all_heaters();
  disable_all_steppers();
    UNUSED(lcd_msg);
  _delay_ms(600); 
  cli(); 
  _delay_ms(250); 
  thermalManager.disable_all_heaters(); 
  #if defined(ACTION_ON_KILL)
    SERIAL_ECHOLNPGM("//action:" ACTION_ON_KILL);
  #endif
  #if HAS_POWER_SWITCH
    SET_INPUT(PS_ON_PIN);
  #endif
  suicide();
  while (1) {
    #if ENABLED(USE_WATCHDOG)
      watchdog_reset();
    #endif
  } // Wait for reset
}
#endif
/**
 * Turn off heaters and stop the print in progress
 * After a stop the machine may be resumed with M999
 */
void stop() {
  thermalManager.disable_all_heaters(); 
  #if ENABLED(PROBING_FANS_OFF)
    if (fans_paused) fans_pause(false); 
  #endif
  if (IsRunning()) {
    Stopped_gcode_LastN = gcode_LastN; 
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
    safe_delay(350);       
    Running = false;
  }
}
#if ENABLED(FYS_POWERBREAK_CHECK)||ENABLED(FYS_POWER_CONTROL_CHECK)
#ifdef PCINT0_vect
ISR(PCINT0_vect) 
{ 
    #if ENABLED(FYS_POWERBREAK_CHECK)&& defined(POW_BREAK_CHECK_PIN)&&digitalPinToPCICRbit(POW_BREAK_CHECK_PIN)==0
    gcode_M1100(); 
    #endif
    #if ENABLED(FYS_POWER_CONTROL_CHECK)&&defined(SHUTDOWN_CHECK_PIN)&&digitalPinToPCICRbit(SHUTDOWN_CHECK_PIN)==0
    FunV010();
    #endif
}
#endif
#ifdef PCINT1_vect
ISR(PCINT1_vect) 
{
    #if ENABLED(FYS_POWERBREAK_CHECK)&&defined(POW_BREAK_CHECK_PIN)&&digitalPinToPCICRbit(POW_BREAK_CHECK_PIN)==1
    gcode_M1100();
    #endif
    #if ENABLED(FYS_POWER_CONTROL_CHECK)&&defined(SHUTDOWN_CHECK_PIN)&&digitalPinToPCICRbit(SHUTDOWN_CHECK_PIN)==1
    FunV010();
    #endif
}
#endif
#ifdef PCINT2_vect
ISR(PCINT2_vect) 
{
    #if ENABLED(FYS_POWERBREAK_CHECK)&&defined(POW_BREAK_CHECK_PIN)&&digitalPinToPCICRbit(POW_BREAK_CHECK_PIN)==2
    gcode_M1100();
    #endif
    #if ENABLED(FYS_POWER_CONTROL_CHECK)&&defined(SHUTDOWN_CHECK_PIN)&&digitalPinToPCICRbit(SHUTDOWN_CHECK_PIN)==2
    FunV010();
    #endif
}
#endif
#ifdef PCINT3_vect
ISR(PCINT3_vect) 
{ 
    #if ENABLED(FYS_POWERBREAK_CHECK)&&defined(POW_BREAK_CHECK_PIN)&&digitalPinToPCICRbit(POW_BREAK_CHECK_PIN)==3
    gcode_M1100(); 
    #endif
    #if ENABLED(FYS_POWER_CONTROL_CHECK)&&defined(SHUTDOWN_CHECK_PIN)&&digitalPinToPCICRbit(SHUTDOWN_CHECK_PIN)==3
    FunV010();
    #endif
}
#endif
#endif
#if ENABLED(FYS_SETUP_INIT)
static void mySetupApi()
{
  #if ENABLED(FYS_POWERBREAK_CHECK)&&defined(POW_BREAK_CHECK_PIN)
    #ifdef FYS_POWER_CHECK_USE_POLLING
      SET_INPUT_PULLUP(POW_BREAK_CHECK_PIN);
    #else
      #if (digitalPinToInterrupt(POW_BREAK_CHECK_PIN) != NOT_AN_INTERRUPT)
        attachInterrupt(digitalPinToInterrupt(POW_BREAK_CHECK_PIN), gcode_M1100, CHANGE);
      #else
        static_assert(digitalPinToPCICR(POW_BREAK_CHECK_PIN) != NULL, "POW_BREAK_CHECK_PIN is not interrupt-capable");
        SBI(*digitalPinToPCMSK(POW_BREAK_CHECK_PIN), digitalPinToPCMSKbit(POW_BREAK_CHECK_PIN));  
        SBI(PCIFR, digitalPinToPCICRbit(POW_BREAK_CHECK_PIN)); 
        SBI(PCICR, digitalPinToPCICRbit(POW_BREAK_CHECK_PIN)); 
      #endif
    #endif
  #endif
  #ifdef FYS_SAFE_PRINT_BREAK
    gcode_M1102();
  #endif
  #if ENABLED(FYS_POWER_CONTROL_CHECK)&&defined(SHUTDOWN_CHECK_PIN)
    #ifdef FYS_POWER_CHECK_USE_POLLING
    SET_INPUT_PULLUP(SHUTDOWN_CHECK_PIN);
    #else
    #if (digitalPinToInterrupt(SHUTDOWN_CHECK_PIN) != NOT_AN_INTERRUPT)
    attachInterrupt(digitalPinToInterrupt(SHUTDOWN_CHECK_PIN), FunV010, CHANGE);
    #else
    static_assert(digitalPinToPCICR(SHUTDOWN_CHECK_PIN) != NULL, "SHUTDOWN_CHECK_PIN is not interrupt-capable");
    SBI(*digitalPinToPCMSK(SHUTDOWN_CHECK_PIN), digitalPinToPCMSKbit(SHUTDOWN_CHECK_PIN));  
    SBI(PCIFR, digitalPinToPCICRbit(SHUTDOWN_CHECK_PIN)); 
    SBI(PCICR, digitalPinToPCICRbit(SHUTDOWN_CHECK_PIN)); 
    #endif
    #endif
  #endif
  #ifdef FYS_CONTROL_PROTOCOL
    firstSerlCheck();
  #endif
}
#endif
#if ENABLED(FYS_LOOP_EVENT)
static void FunV023()
{
    switch(GLOBAL_var_V00E)
    {
    case MACRO_var_V020:
        FunV01C();
        break;
    case MACRO_var_V021:
        FunV019();
        break;
    case MACRO_var_V022:
        myStopPrint();
        break;
    case MACRO_var_V025:
      {
        #if ENABLED(FYS_SHUTDOWN_ONCE_PRINT_DONE) && ENABLED(FYS_POWER_STATUS)
          #if PIN_EXISTS(PS_ON)
            if(GLOBAL_var_V005)
            {
                GLOBAL_var_V005=false;
                GLOBAL_var_V003 = MACRO_var_V00A;
            }
          #endif
        #endif
      }
      break;
    case MACRO_VAR_V056:
        stepper.synchronize();
        disable_all_steppers();
        break;
    }
  #if ENABLED(FYS_POWERBREAK_CHECK)
    #if defined(FYS_POWER_CHECK_USE_POLLING)
      gcode_M1100();
    #endif
  #endif
  #if ENABLED(FYS_POWER_CONTROL_CHECK)
    #if defined(FYS_POWER_CHECK_USE_POLLING)
      FunV010();
    #endif
  #endif
  #if ENABLED(FYS_POWERBREAK_STEPPER_STATUS)
    if (stepper.powerBreakStatus == 2)
    {
        stepper.powerBreakStatus = 0;
        clear_command_queue();
        #if ENABLED(FYS_POWER_STATUS)
          #if PIN_EXISTS(POW_BREAK_CHECK)||PIN_EXISTS(SHUTDOWN_CHECK)||PIN_EXISTS(PS_ON)
            if (GLOBAL_var_V00E != MACRO_var_V020) GLOBAL_var_V003 = MACRO_var_V00A;
          #endif
        #endif
    }
  #endif
  #if ENABLED(FYS_POWER_STATUS)
    #if PIN_EXISTS(POW_BREAK_CHECK)||PIN_EXISTS(SHUTDOWN_CHECK)||PIN_EXISTS(PS_ON)
      if (GLOBAL_var_V003 == MACRO_var_V00A)
      {
          #if defined(FYS_SAFE_PRINT_BREAK)
          if (card.sdprinting)
          recordEnvironment();
          #endif
          FunV008(true);
          GLOBAL_var_V003 = MACRO_var_V00B;
          kill(PSTR("System stop for shutdown."));          
      }
      else
    #endif
  #endif
    if (GLOBAL_var_V00E == MACRO_var_V020)
    {
        FunV008(false);
    }
    GLOBAL_var_V00E = MACRO_var_V01C;
  #if defined(FYS_ENERGY_CONSERVE_HEIGHT)
    if (ifEnergyConserve&&current_position[Z_AXIS] > zEnergyHeight&&IS_SD_PRINTING)
    {
        recordBedTemperature=thermalManager.degTargetBed();
        thermalManager.setTargetBed(0);
    }
  #endif
}
#endif
void setup() {
  #ifdef DISABLE_JTAG
    MCUCR = 0x80;
    MCUCR = 0x80;
  #endif
  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    setup_filrunoutpin();
  #endif
  setup_killpin();
  setup_powerhold();
  #if HAS_STEPPER_RESET
    disableStepperDrivers();
  #endif
#ifdef mySERIAL_Nums
    MYSERIAL.begin();
 for(char i=0;i<mySERIAL_Nums;i++)
 if(MYSERIAL.sCmd[i])serial_line_buffer[i]=new char[MAX_CMD_SIZE];
 else serial_line_buffer[i]=nullptr;
#else
  MYSERIAL.begin(BAUDRATE);
#endif
#if ENABLED(FYS_SETUP_CHECK_SD_AUTO_START)
  #if ENABLED(SDSUPPORT)
    card.checkautostart(false);
  #endif
#endif
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START();
  byte mcu = MCUSR;
  if (mcu &  1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if (mcu &  2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if (mcu &  4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if (mcu &  8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if (mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR = 0;
  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_CHAR(' ');
  SERIAL_ECHOLNPGM(SHORT_BUILD_VERSION);
  SERIAL_EOL();
  #if defined(STRING_DISTRIBUTION_DATE) && defined(STRING_CONFIG_H_AUTHOR)
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
    SERIAL_ECHOPGM(STRING_DISTRIBUTION_DATE);
    SERIAL_ECHOLNPGM(MSG_AUTHOR STRING_CONFIG_H_AUTHOR);
    SERIAL_ECHOLNPGM("Compiled: " __DATE__);
  #endif
  SERIAL_ECHO_START();
  SERIAL_ECHOPAIR(MSG_FREE_MEMORY, freeMemory());
  SERIAL_ECHOLNPAIR(MSG_PLANNER_BUFFER_BYTES, (int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
  for (int8_t i = 0; i < BUFSIZE; i++) send_ok[i] = true;
  (void)settings.load();
  #if HAS_M206_COMMAND
    COPY(current_position, home_offset);
  #else
    ZERO(current_position);
  #endif
  SYNC_PLAN_POSITION_KINEMATIC();
  thermalManager.init();    
  #if ENABLED(USE_WATCHDOG)
    watchdog_init();
  #endif
  stepper.init();    
  servo_init();
  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
  #if HAS_CASE_LIGHT
    case_light_on = CASE_LIGHT_DEFAULT_ON;
    case_light_brightness = CASE_LIGHT_DEFAULT_BRIGHTNESS;
    update_case_light();
  #endif
  #if ENABLED(SPINDLE_LASER_ENABLE)
    OUT_WRITE(SPINDLE_LASER_ENABLE_PIN, !SPINDLE_LASER_ENABLE_INVERT);  
    #if SPINDLE_DIR_CHANGE
      OUT_WRITE(SPINDLE_DIR_PIN, SPINDLE_INVERT_DIR ? 255 : 0);  
    #endif
    #if ENABLED(SPINDLE_LASER_PWM)
      SET_OUTPUT(SPINDLE_LASER_PWM_PIN);
      analogWrite(SPINDLE_LASER_PWM_PIN, SPINDLE_LASER_PWM_INVERT ? 255 : 0);  
    #endif
  #endif
  #if HAS_BED_PROBE
    endstops.enable_z_probe(false);
  #endif
  #if ENABLED(USE_CONTROLLER_FAN)
    SET_OUTPUT(CONTROLLER_FAN_PIN); 
  #endif
  #if HAS_STEPPER_RESET
    enableStepperDrivers();
  #endif
  #if ENABLED(DIGIPOT_I2C)
    digipot_i2c_init();
  #endif
  #if ENABLED(DAC_STEPPER_CURRENT)
    dac_init();
  #endif
  #if (ENABLED(Z_PROBE_SLED) || ENABLED(SOLENOID_PROBE)) && HAS_SOLENOID_1
    OUT_WRITE(SOL1_PIN, LOW); 
  #endif
  #if HAS_HOME
    SET_INPUT_PULLUP(HOME_PIN);
  #endif
  #if PIN_EXISTS(STAT_LED_RED)
    OUT_WRITE(STAT_LED_RED_PIN, LOW); 
  #endif
  #if PIN_EXISTS(STAT_LED_BLUE)
    OUT_WRITE(STAT_LED_BLUE_PIN, LOW); 
  #endif
  #if ENABLED(RGB_LED) || ENABLED(RGBW_LED)
    SET_OUTPUT(RGB_LED_R_PIN);
    SET_OUTPUT(RGB_LED_G_PIN);
    SET_OUTPUT(RGB_LED_B_PIN);
    #if ENABLED(RGBW_LED)
      SET_OUTPUT(RGB_LED_W_PIN);
    #endif
  #endif
  #if ENABLED(MK2_MULTIPLEXER)
    SET_OUTPUT(E_MUX0_PIN);
    SET_OUTPUT(E_MUX1_PIN);
    SET_OUTPUT(E_MUX2_PIN);
  #endif
  lcd_init();
  #ifndef CUSTOM_BOOTSCREEN_TIMEOUT
    #define CUSTOM_BOOTSCREEN_TIMEOUT 2500
  #endif
  #if ENABLED(SHOW_BOOTSCREEN)
    #if ENABLED(DOGLCD)                           
      #if ENABLED(SHOW_CUSTOM_BOOTSCREEN)
        safe_delay(CUSTOM_BOOTSCREEN_TIMEOUT);    
        lcd_bootscreen();                         
      #endif
      safe_delay(BOOTSCREEN_TIMEOUT);             
    #elif ENABLED(ULTRA_LCD)
      lcd_bootscreen();
      #if DISABLED(SDSUPPORT)
        lcd_init();
      #endif
    #endif
  #endif
  #if ENABLED(MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
      mixing_factor[i] = (i == 0) ? 1.0 : 0.0;
    for (uint8_t t = 0; t < MIXING_VIRTUAL_TOOLS; t++)
      for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
        mixing_virtual_tool_mix[t][i] = mixing_factor[i];
  #endif
  #if ENABLED(BLTOUCH)
    bltouch_command(BLTOUCH_RESET);
    set_bltouch_deployed(true); 
    set_bltouch_deployed(false);
  #endif
  #if ENABLED(I2C_POSITION_ENCODERS)
    I2CPEM.init();
  #endif
  #if ENABLED(EXPERIMENTAL_I2CBUS) && I2C_SLAVE_ADDRESS > 0
    i2c.onReceive(i2c_on_receive);
    i2c.onRequest(i2c_on_request);
  #endif
  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    setup_endstop_interrupts();
  #endif
  #if ENABLED(SWITCHING_EXTRUDER)
    move_extruder_servo(0);  
  #endif
  #if ENABLED(SWITCHING_NOZZLE)
    move_nozzle_servo(0);  
  #endif
  #if ENABLED(FYS_SETUP_INIT)
    mySetupApi();
  #endif
}
void loop() {
  if (commands_in_queue < BUFSIZE) get_available_commands();
  #if ENABLED(SDSUPPORT)
    card.checkautostart(false);
  #endif
    if (commands_in_queue){
    #if ENABLED(SDSUPPORT)
      if (card.saving) {
        #if defined(FYS_SAFE_PRINT_BREAK)||defined(mySERIAL_Nums)
          char* command = command_queue[cmd_queue_index_r] + CMDPOS_SPECIAL_LEN;
        #else
        char* command = command_queue[cmd_queue_index_r];
        #endif
        if (strstr_P(command, PSTR("M29"))) {
          card.closefile();
          SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
          ok_to_send();
        }
        else {
          card.write_command(command);
          if (card.logging)
            process_next_command(); 
          else
            ok_to_send();
        }
      }
      else
        process_next_command();
    #else
      process_next_command();
    #endif 
      cli();
      if (commands_in_queue) {
      --commands_in_queue;
      if (++cmd_queue_index_r >= BUFSIZE) cmd_queue_index_r = 0;
    }
      sei();
  }
  endstops.report_state();
  idle();
  #if ENABLED(FYS_LOOP_EVENT)
    FunV023();
  #endif
}
