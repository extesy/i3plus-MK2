#ifndef Ultralcd_DulX_globalVar_H
#define Ultralcd_DulX_globalVar_H
#include "Marlin.h"
#if ENABLED(FYS_DULX_Z_HEIGHT_DETECT)
#define Z_RAISE_BEFORE_PROBING      5
#define CALIBFULL_HOTEND_STANDBY_TEMP   0
#define CALIBFULL_HOTEND_STANDBY_BED    0
#define X_GAP_AVOID_COLLISION_LEFT  5
#define X_GAP_AVOID_COLLISION_RIGHT 8
#define CALIB_FEEDRATE_ZAXIS  6*60
#define LEFT_EXTRUDER  0
#define DULX_LEFT_PROBE_POS1_X      80
#define DULX_LEFT_PROBE_POS1_Y      150
#define DULX_LEFT_PROBE_POS2_X      80
#define DULX_LEFT_PROBE_POS2_Y      10
#define DULX_LEFT_PROBE_POS3_X      250
#define DULX_LEFT_PROBE_POS3_Y      10
#define RIGHT_EXTRUDER 1
#define DULX_RIGHT_PROBE_POS1_X     250
#define DULX_RIGHT_PROBE_POS1_Y     150
#define DULX_RIGHT_PROBE_POS2_X     250
#define DULX_RIGHT_PROBE_POS2_Y     10
#define DULX_RIGHT_PROBE_POS3_X     80
#define DULX_RIGHT_PROBE_POS3_Y     10
#define SCREW1_X                    80
#define SCREW1_Y                    100
#define SCREW2_X                    250
#define SCREW2_Y                    50
#define SCREW3_X                    160
#define SCREW3_Y                    10
#define CLOSE_TO_LEVELING_DIFFER    0.8
  extern bool dulx_need_save_param;
  extern bool flag_utilities_calibration_calibbeddone;
  extern bool flag_utilities_calibration_calibfull;
  extern bool flag_utilities_calibration_calibfull_skipZcalib;
  extern bool flag_utilities_calibration_bedcomensationmode;
  extern uint8_t which_extruder;
  extern uint8_t Bed_Compensation_state;
  extern int Bed_compensation_redo_offset;
  extern float screwHeightDiffer[3];
#endif
#endif
