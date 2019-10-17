#include "ultralcd_DulX_globalVar.h"
#if ENABLED(FYS_DULX_Z_HEIGHT_DETECT)
bool dulx_need_save_param = true;
bool flag_utilities_calibration_calibbeddone = false;
bool flag_utilities_calibration_calibfull = false;
bool flag_utilities_calibration_calibfull_skipZcalib = false;
bool flag_utilities_calibration_bedcomensationmode = false;
uint8_t which_extruder = 0;
uint8_t Bed_Compensation_state = 0;
int Bed_compensation_redo_offset = 0;
float screwHeightDiffer[3] = { 0.0 };
#endif
