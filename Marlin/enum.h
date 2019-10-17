#ifndef __ENUM_H__
#define __ENUM_H__
#include "MarlinConfig.h"
enum AxisEnum {
  NO_AXIS   = -1,
  X_AXIS    = 0,
  A_AXIS    = 0,
  Y_AXIS    = 1,
  B_AXIS    = 1,
  Z_AXIS    = 2,
  C_AXIS    = 2,
  E_AXIS    = 3,
  X_HEAD    = 4,
  Y_HEAD    = 5,
  Z_HEAD    = 6,
  ALL_AXES  = 100
};
#define LOOP_S_LE_N(VAR, S, N) for (uint8_t VAR=S; VAR<=N; VAR++)
#define LOOP_S_L_N(VAR, S, N) for (uint8_t VAR=S; VAR<N; VAR++)
#define LOOP_LE_N(VAR, N) LOOP_S_LE_N(VAR, 0, N)
#define LOOP_L_N(VAR, N) LOOP_S_L_N(VAR, 0, N)
#define LOOP_NA(VAR) LOOP_L_N(VAR, NUM_AXIS)
#define LOOP_XYZ(VAR) LOOP_S_LE_N(VAR, X_AXIS, Z_AXIS)
#define LOOP_XYZE(VAR) LOOP_S_LE_N(VAR, X_AXIS, E_AXIS)
#define LOOP_XYZE_N(VAR) LOOP_S_L_N(VAR, X_AXIS, XYZE_N)
typedef enum {
  LINEARUNIT_MM,
  LINEARUNIT_INCH
} LinearUnit;
typedef enum {
  TEMPUNIT_C,
  TEMPUNIT_K,
  TEMPUNIT_F
} TempUnit;
enum DebugFlags {
  DEBUG_NONE          = 0,
  DEBUG_ECHO          = _BV(0), 
  DEBUG_INFO          = _BV(1), 
  DEBUG_ERRORS        = _BV(2), 
  DEBUG_DRYRUN        = _BV(3), 
  DEBUG_COMMUNICATION = _BV(4), 
  DEBUG_LEVELING      = _BV(5), 
  DEBUG_MESH_ADJUST   = _BV(6), 
  DEBUG_ALL           = 0xFF
};
enum EndstopEnum {
  X_MIN,
  Y_MIN,
  Z_MIN,
  Z_MIN_PROBE,
  X_MAX,
  Y_MAX,
  Z_MAX,
  Z2_MIN,
  Z2_MAX
};
#if ENABLED(EMERGENCY_PARSER)
  enum e_parser_state {
    state_RESET,
    state_N,
    state_M,
    state_M1,
    state_M10,
    state_M108,
    state_M11,
    state_M112,
    state_M4,
    state_M41,
    state_M410,
    state_IGNORE 
  };
#endif
#if ENABLED(ADVANCED_PAUSE_FEATURE)
  enum AdvancedPauseMenuResponse {
    ADVANCED_PAUSE_RESPONSE_WAIT_FOR,
    ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE,
    ADVANCED_PAUSE_RESPONSE_RESUME_PRINT
  };
  #if ENABLED(ULTIPANEL)
    enum AdvancedPauseMessage {
      ADVANCED_PAUSE_MESSAGE_INIT,
      ADVANCED_PAUSE_MESSAGE_UNLOAD,
      ADVANCED_PAUSE_MESSAGE_INSERT,
      ADVANCED_PAUSE_MESSAGE_LOAD,
      ADVANCED_PAUSE_MESSAGE_EXTRUDE,
      ADVANCED_PAUSE_MESSAGE_OPTION,
      ADVANCED_PAUSE_MESSAGE_RESUME,
      ADVANCED_PAUSE_MESSAGE_STATUS,
      ADVANCED_PAUSE_MESSAGE_CLICK_TO_HEAT_NOZZLE,
      #ifdef FYS_PRINT_BREAK_HEATUP_FIRST
        ADVANCED_PRINT_BREAK_MESSAGE_HEAT_NOZZLE,
      #endif
      ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT
    };
  #endif
#endif
#if ENABLED(HOST_KEEPALIVE_FEATURE)
  enum MarlinBusyState {
    NOT_BUSY,           
    IN_HANDLER,         
    IN_PROCESS,         
    PAUSED_FOR_USER,    
    PAUSED_FOR_INPUT    
  };
#endif
enum LsAction { LS_SerialPrint, LS_Count, LS_GetFilename };
enum LCDViewAction {
  LCDVIEW_NONE,
  LCDVIEW_REDRAW_NOW,
  LCDVIEW_CALL_REDRAW_NEXT,
  LCDVIEW_CLEAR_CALL_REDRAW,
  LCDVIEW_CALL_NO_REDRAW
};
#if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
  enum DualXMode {
    DXC_FULL_CONTROL_MODE,  
    DXC_AUTO_PARK_MODE,     
    DXC_DUPLICATION_MODE,
    #if ENABLED(FYS_DXC_FYS_MODE)
    DXC_FYS_MODE 
    #endif
  };
#endif
#if ENABLED(CNC_WORKSPACE_PLANES)
  enum WorkspacePlane { PLANE_XY, PLANE_ZX, PLANE_YZ };
#endif
#endif 
