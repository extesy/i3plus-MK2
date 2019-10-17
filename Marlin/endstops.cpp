#include "Marlin.h"
#include "cardreader.h"
#include "endstops.h"
#include "temperature.h"
#include "stepper.h"
#include "ultralcd.h"
#define TEST_ENDSTOP(ENDSTOP) (TEST(current_endstop_bits & old_endstop_bits, ENDSTOP))
Endstops endstops;
bool  Endstops::enabled = true,
      Endstops::enabled_globally =
        #if ENABLED(ENDSTOPS_ALWAYS_ON_DEFAULT)
          (true)
        #else
          (false)
        #endif
      ;
volatile char Endstops::endstop_hit_bits; 
#if ENABLED(Z_DUAL_ENDSTOPS)
  uint16_t
#else
  byte
#endif
    Endstops::current_endstop_bits = 0,
    Endstops::old_endstop_bits = 0;
#if HAS_BED_PROBE
  volatile bool Endstops::z_probe_enabled = false;
#endif
void Endstops::init() {
  #if HAS_X_MIN
    #if ENABLED(ENDSTOPPULLUP_XMIN)
      SET_INPUT_PULLUP(X_MIN_PIN);
    #else
      SET_INPUT(X_MIN_PIN);
    #endif
  #endif
  #if HAS_Y_MIN
    #if ENABLED(ENDSTOPPULLUP_YMIN)
      SET_INPUT_PULLUP(Y_MIN_PIN);
    #else
      SET_INPUT(Y_MIN_PIN);
    #endif
  #endif
  #if HAS_Z_MIN
    #if ENABLED(ENDSTOPPULLUP_ZMIN)
      SET_INPUT_PULLUP(Z_MIN_PIN);
    #else
      SET_INPUT(Z_MIN_PIN);
    #endif
  #endif
  #if HAS_Z2_MIN
    #if ENABLED(ENDSTOPPULLUP_ZMIN)
      SET_INPUT_PULLUP(Z2_MIN_PIN);
    #else
      SET_INPUT(Z2_MIN_PIN);
    #endif
  #endif
  #if HAS_X_MAX
    #if ENABLED(ENDSTOPPULLUP_XMAX)
      SET_INPUT_PULLUP(X_MAX_PIN);
    #else
      SET_INPUT(X_MAX_PIN);
    #endif
  #endif
  #if HAS_Y_MAX
    #if ENABLED(ENDSTOPPULLUP_YMAX)
      SET_INPUT_PULLUP(Y_MAX_PIN);
    #else
      SET_INPUT(Y_MAX_PIN);
    #endif
  #endif
  #if HAS_Z_MAX
    #if ENABLED(ENDSTOPPULLUP_ZMAX)
      SET_INPUT_PULLUP(Z_MAX_PIN);
    #else
      SET_INPUT(Z_MAX_PIN);
    #endif
  #endif
  #if HAS_Z2_MAX
    #if ENABLED(ENDSTOPPULLUP_ZMAX)
      SET_INPUT_PULLUP(Z2_MAX_PIN);
    #else
      SET_INPUT(Z2_MAX_PIN);
    #endif
  #endif
  #if ENABLED(Z_MIN_PROBE_ENDSTOP)
    #if ENABLED(ENDSTOPPULLUP_ZMIN_PROBE)
      SET_INPUT_PULLUP(Z_MIN_PROBE_PIN);
    #else
      SET_INPUT(Z_MIN_PROBE_PIN);
    #endif
  #endif
} 
void Endstops::report_state() {
  if (endstop_hit_bits) {
    #if ENABLED(ULTRA_LCD)
      char chrX = ' ', chrY = ' ', chrZ = ' ', chrP = ' ';
      #define _SET_STOP_CHAR(A,C) (chr## A = C)
    #else
      #define _SET_STOP_CHAR(A,C) ;
    #endif
    #define _ENDSTOP_HIT_ECHO(A,C) do{ \
      SERIAL_ECHOPAIR(" " STRINGIFY(A) ":", stepper.triggered_position_mm(A ##_AXIS)); \
      _SET_STOP_CHAR(A,C); }while(0)
    #define _ENDSTOP_HIT_TEST(A,C) \
      if (TEST(endstop_hit_bits, A ##_MIN) || TEST(endstop_hit_bits, A ##_MAX)) \
        _ENDSTOP_HIT_ECHO(A,C)
    #define ENDSTOP_HIT_TEST_X() _ENDSTOP_HIT_TEST(X,'X')
    #define ENDSTOP_HIT_TEST_Y() _ENDSTOP_HIT_TEST(Y,'Y')
    #define ENDSTOP_HIT_TEST_Z() _ENDSTOP_HIT_TEST(Z,'Z')
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(MSG_ENDSTOPS_HIT);
    ENDSTOP_HIT_TEST_X();
    ENDSTOP_HIT_TEST_Y();
    ENDSTOP_HIT_TEST_Z();
    #if ENABLED(Z_MIN_PROBE_ENDSTOP)
      #define P_AXIS Z_AXIS
      if (TEST(endstop_hit_bits, Z_MIN_PROBE)) _ENDSTOP_HIT_ECHO(P, 'P');
    #endif
    SERIAL_EOL();
    #if ENABLED(ULTRA_LCD)
      lcd_status_printf_P(0, PSTR(MSG_LCD_ENDSTOPS " %c %c %c %c"), chrX, chrY, chrZ, chrP);
    #endif
    hit_on_purpose();
    #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED) && ENABLED(SDSUPPORT)
      if (stepper.abort_on_endstop_hit) {
        card.sdprinting = false;
        card.closefile();
        quickstop_stepper();
        thermalManager.disable_all_heaters(); 
      }
    #endif
  }
} 
void Endstops::M119() {
    SERIAL_PROTOCOLLNPGM(MSG_M119_REPORT);
  #if HAS_X_MIN
    SERIAL_PROTOCOLPGM(MSG_X_MIN);
    SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_X_MAX
    SERIAL_PROTOCOLPGM(MSG_X_MAX);
    SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MIN
    SERIAL_PROTOCOLPGM(MSG_Y_MIN);
    SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MAX
    SERIAL_PROTOCOLPGM(MSG_Y_MAX);
    SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MIN
    SERIAL_PROTOCOLPGM(MSG_Z_MIN);
    SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z2_MIN
    SERIAL_PROTOCOLPGM(MSG_Z2_MIN);
    SERIAL_PROTOCOLLN(((READ(Z2_MIN_PIN)^Z2_MIN_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MAX
    SERIAL_PROTOCOLPGM(MSG_Z_MAX);
    SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z2_MAX
    SERIAL_PROTOCOLPGM(MSG_Z2_MAX);
    SERIAL_PROTOCOLLN(((READ(Z2_MAX_PIN)^Z2_MAX_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if ENABLED(Z_MIN_PROBE_ENDSTOP)
    SERIAL_PROTOCOLPGM(MSG_Z_PROBE);
    SERIAL_PROTOCOLLN(((READ(Z_MIN_PROBE_PIN)^Z_MIN_PROBE_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    SERIAL_PROTOCOLPGM(MSG_FILAMENT_RUNOUT_SENSOR);
    SERIAL_PROTOCOLLN(((READ(FIL_RUNOUT_PIN)^FIL_RUNOUT_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
} 
#if ENABLED(Z_DUAL_ENDSTOPS)
  void Endstops::test_dual_z_endstops(const EndstopEnum es1, const EndstopEnum es2) {
    byte z_test = TEST_ENDSTOP(es1) | (TEST_ENDSTOP(es2) << 1); 
    if (z_test && stepper.current_block->steps[Z_AXIS] > 0) {
      SBI(endstop_hit_bits, Z_MIN);
      if (!stepper.performing_homing || (z_test == 0x3))  
        stepper.kill_current_block();
    }
  }
#endif
void Endstops::update() {
  #define _ENDSTOP(AXIS, MINMAX) AXIS ##_## MINMAX
  #define _ENDSTOP_PIN(AXIS, MINMAX) AXIS ##_## MINMAX ##_PIN
  #define _ENDSTOP_INVERTING(AXIS, MINMAX) AXIS ##_## MINMAX ##_ENDSTOP_INVERTING
  #define _ENDSTOP_HIT(AXIS) SBI(endstop_hit_bits, _ENDSTOP(AXIS, MIN))
  #define UPDATE_ENDSTOP_BIT(AXIS, MINMAX) SET_BIT(current_endstop_bits, _ENDSTOP(AXIS, MINMAX), (READ(_ENDSTOP_PIN(AXIS, MINMAX)) != _ENDSTOP_INVERTING(AXIS, MINMAX)))
  #define COPY_BIT(DST, SRC_BIT, DST_BIT) SET_BIT(DST, DST_BIT, TEST(DST, SRC_BIT))
  #define UPDATE_ENDSTOP(AXIS,MINMAX) do { \
      UPDATE_ENDSTOP_BIT(AXIS, MINMAX); \
      if (TEST_ENDSTOP(_ENDSTOP(AXIS, MINMAX)) && stepper.current_block->steps[_AXIS(AXIS)] > 0) { \
        _ENDSTOP_HIT(AXIS); \
        stepper.endstop_triggered(_AXIS(AXIS)); \
      } \
    } while(0)
  #if ENABLED(G38_PROBE_TARGET) && PIN_EXISTS(Z_MIN_PROBE) && !(CORE_IS_XY || CORE_IS_XZ)
    if (G38_move) {
      UPDATE_ENDSTOP_BIT(Z, MIN_PROBE);
      if (TEST_ENDSTOP(_ENDSTOP(Z, MIN_PROBE))) {
        if      (stepper.current_block->steps[_AXIS(X)] > 0) { _ENDSTOP_HIT(X); stepper.endstop_triggered(_AXIS(X)); }
        else if (stepper.current_block->steps[_AXIS(Y)] > 0) { _ENDSTOP_HIT(Y); stepper.endstop_triggered(_AXIS(Y)); }
        else if (stepper.current_block->steps[_AXIS(Z)] > 0) { _ENDSTOP_HIT(Z); stepper.endstop_triggered(_AXIS(Z)); }
        G38_endstop_hit = true;
      }
    }
  #endif
  #if IS_CORE
    #define S_(N) stepper.current_block->steps[CORE_AXIS_##N]
    #define D_(N) stepper.motor_direction(CORE_AXIS_##N)
  #endif
  #if CORE_IS_XY || CORE_IS_XZ
    #if ENABLED(COREXY) || ENABLED(COREXZ)
      #define X_CMP ==
    #else
      #define X_CMP !=
    #endif
    #define X_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && D_(1) X_CMP D_(2)) )
    #define X_AXIS_HEAD X_HEAD
  #else
    #define X_MOVE_TEST stepper.current_block->steps[X_AXIS] > 0
    #define X_AXIS_HEAD X_AXIS
  #endif
  #if CORE_IS_XY || CORE_IS_YZ
    #if ENABLED(COREYX) || ENABLED(COREYZ)
      #define Y_CMP ==
    #else
      #define Y_CMP !=
    #endif
    #define Y_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && D_(1) Y_CMP D_(2)) )
    #define Y_AXIS_HEAD Y_HEAD
  #else
    #define Y_MOVE_TEST stepper.current_block->steps[Y_AXIS] > 0
    #define Y_AXIS_HEAD Y_AXIS
  #endif
  #if CORE_IS_XZ || CORE_IS_YZ
    #if ENABLED(COREZX) || ENABLED(COREZY)
      #define Z_CMP ==
    #else
      #define Z_CMP !=
    #endif
    #define Z_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && D_(1) Z_CMP D_(2)) )
    #define Z_AXIS_HEAD Z_HEAD
  #else
    #define Z_MOVE_TEST stepper.current_block->steps[Z_AXIS] > 0
    #define Z_AXIS_HEAD Z_AXIS
  #endif
  #if ENABLED(DUAL_X_CARRIAGE)
    #define E0_ACTIVE stepper.current_block->active_extruder == 0
    #define X_MIN_TEST ((X_HOME_DIR < 0 && E0_ACTIVE) || (X2_HOME_DIR < 0 && !E0_ACTIVE))
    #define X_MAX_TEST ((X_HOME_DIR > 0 && E0_ACTIVE) || (X2_HOME_DIR > 0 && !E0_ACTIVE))
  #else
    #define X_MIN_TEST true
    #define X_MAX_TEST true
  #endif
  if (X_MOVE_TEST) {
    if (stepper.motor_direction(X_AXIS_HEAD)) {
      if (X_MIN_TEST) { 
        #if HAS_X_MIN
          UPDATE_ENDSTOP(X, MIN);
        #endif
      }
    }
    else if (X_MAX_TEST) { 
      #if HAS_X_MAX
        UPDATE_ENDSTOP(X, MAX);
      #endif
    }
  }
  if (Y_MOVE_TEST) {
    if (stepper.motor_direction(Y_AXIS_HEAD)) { 
      #if HAS_Y_MIN
        UPDATE_ENDSTOP(Y, MIN);
      #endif
    }
    else { 
      #if HAS_Y_MAX
        UPDATE_ENDSTOP(Y, MAX);
      #endif
    }
  }
  if (Z_MOVE_TEST) {
    if (stepper.motor_direction(Z_AXIS_HEAD)) { 
      #if HAS_Z_MIN
        #if ENABLED(Z_DUAL_ENDSTOPS)
          UPDATE_ENDSTOP_BIT(Z, MIN);
          #if HAS_Z2_MIN
            UPDATE_ENDSTOP_BIT(Z2, MIN);
          #else
            COPY_BIT(current_endstop_bits, Z_MIN, Z2_MIN);
          #endif
          test_dual_z_endstops(Z_MIN, Z2_MIN);
        #else 
          #if ENABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN)
            if (z_probe_enabled) UPDATE_ENDSTOP(Z, MIN);
          #else
            UPDATE_ENDSTOP(Z, MIN);
          #endif
        #endif 
      #endif 
      #if ENABLED(Z_MIN_PROBE_ENDSTOP)
        if (z_probe_enabled) {
          UPDATE_ENDSTOP(Z, MIN_PROBE);
          if (TEST_ENDSTOP(Z_MIN_PROBE)) SBI(endstop_hit_bits, Z_MIN_PROBE);
        }
      #endif
    }
    else { 
      #if HAS_Z_MAX
        #if ENABLED(Z_DUAL_ENDSTOPS)
          UPDATE_ENDSTOP_BIT(Z, MAX);
          #if HAS_Z2_MAX
            UPDATE_ENDSTOP_BIT(Z2, MAX);
          #else
            COPY_BIT(current_endstop_bits, Z_MAX, Z2_MAX);
          #endif
          test_dual_z_endstops(Z_MAX, Z2_MAX);
        #elif DISABLED(Z_MIN_PROBE_ENDSTOP) || Z_MAX_PIN != Z_MIN_PROBE_PIN
          UPDATE_ENDSTOP(Z, MAX);
        #endif 
      #endif 
    }
  }
  old_endstop_bits = current_endstop_bits;
} 
