#define EEPROM_VERSION "V10"
#define EEPROM_OFFSET 100
#define FYS_EEPROM_ENVIRONMENT_POS  1200
#define FYS_EEPROM_POW_BREAK_SIGN              0xA0A0
#include "configuration_store.h"
MarlinSettings settings;
#include "Marlin.h"
#include "language.h"
#include "endstops.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "stepper.h"
#if ENABLED(INCH_MODE_SUPPORT) || (ENABLED(ULTIPANEL) && ENABLED(TEMPERATURE_UNITS_SUPPORT))
  #include "gcode.h"
#endif
#if ENABLED(MESH_BED_LEVELING)
  #include "mesh_bed_leveling.h"
#endif
#if ENABLED(HAVE_TMC2130)
  #include "stepper_indirection.h"
#endif
#if ENABLED(AUTO_BED_LEVELING_UBL)
  #include "ubl.h"
#endif
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  extern void refresh_bed_level();
#endif
void MarlinSettings::postprocess() {
  planner.reset_acceleration_rates();
  #if ENABLED(DELTA)
    recalc_delta_settings(delta_radius, delta_diagonal_rod);
  #endif
  planner.refresh_positioning();
  #if ENABLED(PIDTEMP)
    thermalManager.updatePID();
  #endif
  calculate_volumetric_multipliers();
  #if HAS_HOME_OFFSET || ENABLED(DUAL_X_CARRIAGE)
    LOOP_XYZ(i) update_software_endstops((AxisEnum)i);
  #endif
  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    set_z_fade_height(planner.z_fade_height);
  #endif
  #if HAS_BED_PROBE
    refresh_zprobe_zoffset();
  #endif
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    refresh_bed_level();
  #endif
  #if HAS_MOTOR_CURRENT_PWM
    stepper.refresh_motor_power();
  #endif
}
#if ENABLED(EEPROM_SETTINGS)
  #define DUMMY_PID_VALUE 3000.0f
  #define EEPROM_START() int eeprom_index = EEPROM_OFFSET
  #define EEPROM_SKIP(VAR) eeprom_index += sizeof(VAR)
  #define EEPROM_WRITE(VAR) write_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_READ(VAR) read_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_ASSERT(TST,ERR) if (!(TST)) do{ SERIAL_ERROR_START(); SERIAL_ERRORLNPGM(ERR); eeprom_read_error = true; }while(0)
  const char version[4] = EEPROM_VERSION;
  bool MarlinSettings::eeprom_error;
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    int MarlinSettings::meshes_begin;
  #endif
  void MarlinSettings::write_data(int &pos, const uint8_t *value, uint16_t size, uint16_t *crc) {
    if (eeprom_error) return;
    while (size--) {
      uint8_t * const p = (uint8_t * const)pos;
      uint8_t v = *value;
      if (v != eeprom_read_byte(p)) {
        eeprom_write_byte(p, v);
        if (eeprom_read_byte(p) != v) {
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_ERR_EEPROM_WRITE);
          eeprom_error = true;
          return;
        }
      }
      crc16(crc, &v, 1);
      pos++;
      value++;
    };
  }
  void MarlinSettings::read_data(int &pos, uint8_t* value, uint16_t size, uint16_t *crc) {
    if (eeprom_error) return;
    do {
      uint8_t c = eeprom_read_byte((unsigned char*)pos);
      *value = c;
      crc16(crc, &c, 1);
      pos++;
      value++;
    } while (--size);
  }
  bool MarlinSettings::save() {
      char amm=0;
    float dummy = 0.0f;
    char ver[4] = "000";
    uint16_t working_crc = 0;
    EEPROM_START();
    eeprom_error = false;
    EEPROM_WRITE(ver);     
    EEPROM_SKIP(working_crc); 
    working_crc = 0; 
    const uint8_t esteppers = COUNT(planner.axis_steps_per_mm) - XYZ;
    EEPROM_WRITE(esteppers);
    EEPROM_WRITE(planner.axis_steps_per_mm);
    EEPROM_WRITE(planner.max_feedrate_mm_s);
    EEPROM_WRITE(planner.max_acceleration_mm_per_s2);
    EEPROM_WRITE(planner.acceleration);
    EEPROM_WRITE(planner.retract_acceleration);
    EEPROM_WRITE(planner.travel_acceleration);
    EEPROM_WRITE(planner.min_feedrate_mm_s);
    EEPROM_WRITE(planner.min_travel_feedrate_mm_s);
    EEPROM_WRITE(planner.min_segment_time);
    EEPROM_WRITE(planner.max_jerk);
    #if !HAS_HOME_OFFSET
      const float home_offset[XYZ] = { 0 };
    #endif
    #if ENABLED(DELTA)
      dummy = 0.0;
      EEPROM_WRITE(dummy);
      EEPROM_WRITE(dummy);
      dummy = DELTA_HEIGHT + home_offset[Z_AXIS];
      EEPROM_WRITE(dummy);
    #else
      EEPROM_WRITE(home_offset);
    #endif
    #if HOTENDS > 1
      for (uint8_t e = 1; e < HOTENDS; e++)
        LOOP_XYZ(i) EEPROM_WRITE(hotend_offset[i][e]);
    #endif
    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      const float zfh = planner.z_fade_height;
    #else
      const float zfh = 10.0;
    #endif
    EEPROM_WRITE(zfh);
    #if ENABLED(MESH_BED_LEVELING)
      static_assert(
        sizeof(mbl.z_values) == GRID_MAX_POINTS * sizeof(mbl.z_values[0][0]),
        "MBL Z array is the wrong size."
      );
      const bool leveling_is_on = TEST(mbl.status, MBL_STATUS_HAS_MESH_BIT);
      const uint8_t mesh_num_x = GRID_MAX_POINTS_X, mesh_num_y = GRID_MAX_POINTS_Y;
      EEPROM_WRITE(leveling_is_on);
      EEPROM_WRITE(mbl.z_offset);
      EEPROM_WRITE(mesh_num_x);
      EEPROM_WRITE(mesh_num_y);
      EEPROM_WRITE(mbl.z_values);
    #else 
      const bool leveling_is_on = false;
      dummy = 0.0f;
      const uint8_t mesh_num_x = 3, mesh_num_y = 3;
      EEPROM_WRITE(leveling_is_on);
      EEPROM_WRITE(dummy); 
      EEPROM_WRITE(mesh_num_x);
      EEPROM_WRITE(mesh_num_y);
      for (uint8_t q = mesh_num_x * mesh_num_y; q--;) EEPROM_WRITE(dummy);
    #endif 
    #if !HAS_BED_PROBE
      const float zprobe_zoffset = 0;
    #endif
    EEPROM_WRITE(zprobe_zoffset);
    SERIAL_ECHOLNPAIR("zprobe_zoffset", zprobe_zoffset);
    #if HAS_ABL
    EEPROM_WRITE(planner.abl_enabled);
    #else
    EEPROM_WRITE(amm);
    #endif
    #if ABL_PLANAR
      EEPROM_WRITE(planner.bed_level_matrix);
    #else
      dummy = 0.0;
      for (uint8_t q = 9; q--;) EEPROM_WRITE(dummy);
    #endif
    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      static_assert(
        sizeof(z_values) == GRID_MAX_POINTS * sizeof(z_values[0][0]),
        "Bilinear Z array is the wrong size."
      );
      const uint8_t grid_max_x = GRID_MAX_POINTS_X, grid_max_y = GRID_MAX_POINTS_Y;
      EEPROM_WRITE(grid_max_x);            
      EEPROM_WRITE(grid_max_y);            
      EEPROM_WRITE(bilinear_grid_spacing); 
      EEPROM_WRITE(bilinear_start);        
      EEPROM_WRITE(z_values);              
    #else
      const uint8_t grid_max_x = 3, grid_max_y = 3;
      const int bilinear_start[2] = { 0 }, bilinear_grid_spacing[2] = { 0 };
      dummy = 0.0f;
      EEPROM_WRITE(grid_max_x);
      EEPROM_WRITE(grid_max_y);
      EEPROM_WRITE(bilinear_grid_spacing);
      EEPROM_WRITE(bilinear_start);
      for (uint16_t q = grid_max_x * grid_max_y; q--;) EEPROM_WRITE(dummy);
    #endif 
    #if ENABLED(AUTO_BED_LEVELING_UBL)
      EEPROM_WRITE(ubl.state.active);
      EEPROM_WRITE(ubl.state.z_offset);
      EEPROM_WRITE(ubl.state.storage_slot);
    #else
      const bool ubl_active = false;
      dummy = 0.0f;
      const int8_t storage_slot = -1;
      EEPROM_WRITE(ubl_active);
      EEPROM_WRITE(dummy);
      EEPROM_WRITE(storage_slot);
    #endif 
    #if ENABLED(DELTA)
      EEPROM_WRITE(endstop_adj);               
      EEPROM_WRITE(delta_radius);              
      EEPROM_WRITE(delta_diagonal_rod);        
      EEPROM_WRITE(delta_segments_per_second); 
      EEPROM_WRITE(delta_calibration_radius);  
      EEPROM_WRITE(delta_tower_angle_trim);    
      dummy = 0.0f;
      for (uint8_t q = 3; q--;) EEPROM_WRITE(dummy);
    #elif ENABLED(Z_DUAL_ENDSTOPS)
      EEPROM_WRITE(z_endstop_adj);             
      dummy = 0.0f;
      for (uint8_t q = 11; q--;) EEPROM_WRITE(dummy);
    #else
      dummy = 0.0f;
      for (uint8_t q = 12; q--;) EEPROM_WRITE(dummy);
    #endif
    #if DISABLED(ULTIPANEL)&&DISABLED(FYSTLCD_V1)
      constexpr int lcd_preheat_hotend_temp[2] = { PREHEAT_1_TEMP_HOTEND, PREHEAT_2_TEMP_HOTEND },
                    lcd_preheat_bed_temp[2] = { PREHEAT_1_TEMP_BED, PREHEAT_2_TEMP_BED },
                    lcd_preheat_fan_speed[2] = { PREHEAT_1_FAN_SPEED, PREHEAT_2_FAN_SPEED };
    #endif
    EEPROM_WRITE(lcd_preheat_hotend_temp);
    EEPROM_WRITE(lcd_preheat_bed_temp);
    EEPROM_WRITE(lcd_preheat_fan_speed);
    for (uint8_t e = 0; e < MAX_EXTRUDERS; e++) {
      #if ENABLED(PIDTEMP)
        if (e < HOTENDS) {
          EEPROM_WRITE(PID_PARAM(Kp, e));
          EEPROM_WRITE(PID_PARAM(Ki, e));
          EEPROM_WRITE(PID_PARAM(Kd, e));
          #if ENABLED(PID_EXTRUSION_SCALING)
            EEPROM_WRITE(PID_PARAM(Kc, e));
          #else
            dummy = 1.0f; 
            EEPROM_WRITE(dummy);
          #endif
        }
        else
      #endif 
        {
          dummy = DUMMY_PID_VALUE; 
          EEPROM_WRITE(dummy); 
          dummy = 0.0f;
          for (uint8_t q = 3; q--;) EEPROM_WRITE(dummy); 
        }
    } 
    #if DISABLED(PID_EXTRUSION_SCALING)
      int lpq_len = 20;
    #endif
    EEPROM_WRITE(lpq_len);
    #if DISABLED(PIDTEMPBED)
      dummy = DUMMY_PID_VALUE;
      for (uint8_t q = 3; q--;) EEPROM_WRITE(dummy);
    #else
      EEPROM_WRITE(thermalManager.bedKp);
      EEPROM_WRITE(thermalManager.bedKi);
      EEPROM_WRITE(thermalManager.bedKd);
    #endif
    #if !HAS_LCD_CONTRAST
      const uint16_t lcd_contrast = 32;
    #endif
    EEPROM_WRITE(lcd_contrast);
    #if ENABLED(FWRETRACT)
      EEPROM_WRITE(autoretract_enabled);
      EEPROM_WRITE(retract_length);
      #if EXTRUDERS > 1
        EEPROM_WRITE(retract_length_swap);
      #else
        dummy = 0.0f;
        EEPROM_WRITE(dummy);
      #endif
      EEPROM_WRITE(retract_feedrate_mm_s);
      EEPROM_WRITE(retract_zlift);
      EEPROM_WRITE(retract_recover_length);
      #if EXTRUDERS > 1
        EEPROM_WRITE(retract_recover_length_swap);
      #else
        dummy = 0.0f;
        EEPROM_WRITE(dummy);
      #endif
      EEPROM_WRITE(retract_recover_feedrate_mm_s);
    #endif 
    EEPROM_WRITE(volumetric_enabled);
    for (uint8_t q = 0; q < MAX_EXTRUDERS; q++) {
      if (q < COUNT(filament_size)) dummy = filament_size[q];
      EEPROM_WRITE(dummy);
    }
    uint16_t val;
    #if ENABLED(HAVE_TMC2130)
      #if ENABLED(X_IS_TMC2130)
        val = stepperX.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(Y_IS_TMC2130)
        val = stepperY.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(Z_IS_TMC2130)
        val = stepperZ.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(X2_IS_TMC2130)
        val = stepperX2.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(Y2_IS_TMC2130)
        val = stepperY2.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(Z2_IS_TMC2130)
        val = stepperZ2.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(E0_IS_TMC2130)
        val = stepperE0.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(E1_IS_TMC2130)
        val = stepperE1.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(E2_IS_TMC2130)
        val = stepperE2.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(E3_IS_TMC2130)
        val = stepperE3.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(E4_IS_TMC2130)
        val = stepperE4.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
    #else
      val = 0;
      for (uint8_t q = 0; q < 11; ++q) EEPROM_WRITE(val);
    #endif
    #if ENABLED(LIN_ADVANCE)
      EEPROM_WRITE(planner.extruder_advance_k);
      EEPROM_WRITE(planner.advance_ed_ratio);
    #else
      dummy = 0.0f;
      EEPROM_WRITE(dummy);
      EEPROM_WRITE(dummy);
    #endif
    #if HAS_MOTOR_CURRENT_PWM
      for (uint8_t q = 3; q--;) EEPROM_WRITE(stepper.motor_current_setting[q]);
    #else
      const uint32_t dummyui32 = 0;
      for (uint8_t q = 3; q--;) EEPROM_WRITE(dummyui32);
    #endif
    #ifdef FYS_ENERGY_CONSERVE_HEIGHT
        EEPROM_WRITE(zEnergyHeight);
    #endif
    #ifdef FYS_ACTIVE_TIME_OVER
      int16_t tmillis = max_inactive_time/1000;
      EEPROM_WRITE(tmillis);
    #endif
    if (!eeprom_error) {
      const int eeprom_size = eeprom_index;
      const uint16_t final_crc = working_crc;
      eeprom_index = EEPROM_OFFSET;
      EEPROM_WRITE(version);
      EEPROM_WRITE(final_crc);
      #if ENABLED(EEPROM_CHITCHAT)
        SERIAL_ECHO_START();
        SERIAL_ECHOPAIR("Settings Stored (", eeprom_size - (EEPROM_OFFSET));
        SERIAL_ECHOPAIR(" bytes; crc ", final_crc);
        SERIAL_ECHOLNPGM(")");
      #endif
    }
    #if ENABLED(UBL_SAVE_ACTIVE_ON_M500)
      if (ubl.state.storage_slot >= 0)
        store_mesh(ubl.state.storage_slot);
    #endif
    return !eeprom_error;
  }
#ifdef FYS_SAFE_PRINT_BREAK  
  void MarlinSettings::FunV04D(uint32_t& printedTime)
  {
    int16_t t;
    uint16_t working_crc = 0;
    int eeprom_index = FYS_EEPROM_ENVIRONMENT_POS+sizeof(uint32_t);
    #ifdef FYS_POWER_CHECK_USE_POLLING
    cli(); 
    #endif
    #if HAS_TEMP_0
    t=Temperature::target_temperature[0];
    EEPROM_WRITE(t);
    #endif
    #if HAS_TEMP_1
    t=Temperature::target_temperature[1];
    EEPROM_WRITE(t);
    #endif
    #if HAS_TEMP_2
    t=Temperature::target_temperature[2];
    EEPROM_WRITE(t);
    #endif
    #if HAS_TEMP_3
    t=Temperature::target_temperature[3];
    EEPROM_WRITE(t);
    #endif
    #if HAS_TEMP_4
    t=Temperature::target_temperature[4];
    EEPROM_WRITE(t);
    #endif
    #if HAS_TEMP_BED
    #ifdef FYS_ENERGY_CONSERVE_HEIGHT
    if (Temperature::target_temperature_bed == 0)t=recordBedTemperature;
    else
    #endif
        t=Temperature::target_temperature_bed;
        EEPROM_WRITE(t);
    #endif
    #if FAN_COUNT>0
    for (char i = 0; i<FAN_COUNT; i++)
    {
        EEPROM_WRITE(fanSpeeds[i]);
    }
    #endif
    EEPROM_WRITE(active_extruder);
    EEPROM_WRITE(feedrate_percentage);
    EEPROM_WRITE(feedrate_mm_s);
    EEPROM_WRITE(currentCmdSdPos.n32);
    #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
      EEPROM_WRITE(GLOBAL_var_V004);  
    #endif
    EEPROM_WRITE(current_position[X_AXIS]);
    EEPROM_WRITE(current_position[Y_AXIS]);
    EEPROM_WRITE(current_position[Z_AXIS]);
    EEPROM_WRITE(current_position[E_AXIS]);
    EEPROM_WRITE(printedTime);
    #if ENABLED(MIX_FIL_RANOUT_SAFE_PRINT_BREAK)
    saveEnvironmentFilRanOutParam(); 
    #endif
    #ifdef FYS_POWER_CHECK_USE_POLLING
    sei(); 
    #endif
  }
  void MarlinSettings::FunV04E(uint32_t& printedTime)
  {
      int16_t t;
    uint16_t working_crc = 0;
    int eeprom_index = FYS_EEPROM_ENVIRONMENT_POS+sizeof(uint32_t);
    #if HAS_TEMP_0
    EEPROM_READ(t);
    Temperature::setTargetHotend(t,0);
    #endif
    #if HAS_TEMP_1
    EEPROM_READ(t);
    Temperature::setTargetHotend(t,1);
    #endif
    #if HAS_TEMP_2
    EEPROM_READ(t);
    Temperature::setTargetHotend(t,2);
    #endif
    #if HAS_TEMP_3
    EEPROM_READ(t);
    Temperature::setTargetHotend(t,3);
    #endif
    #if HAS_TEMP_4
    EEPROM_READ(t);
    Temperature::setTargetHotend(t,4);
    #endif
    #if HAS_TEMP_BED
    EEPROM_READ(t);
    Temperature::setTargetBed(t);
    #endif
    #if FAN_COUNT>0
    for (char i = 0; i<FAN_COUNT; i++)
    {
        EEPROM_READ(fanSpeeds[i]);
    }
    #endif
    EEPROM_READ(active_extruder);
    EEPROM_READ(feedrate_percentage);
    EEPROM_READ(feedrate_mm_s);
    EEPROM_READ(currentCmdSdPos.n32);
    #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
    EEPROM_READ(GLOBAL_var_V004);
    #endif
    EEPROM_READ(current_position[X_AXIS]);
    EEPROM_READ(current_position[Y_AXIS]);
    EEPROM_READ(current_position[Z_AXIS]);
    EEPROM_READ(current_position[E_AXIS]);
    EEPROM_READ(printedTime);
    #if ENABLED(MIX_FIL_RANOUT_SAFE_PRINT_BREAK)
    loadEnvironmentFilRanOutParam();
    #endif
  }
  bool MarlinSettings::FunV009()
  {
    uint16_t working_crc = 0;
    int eeprom_index = FYS_EEPROM_ENVIRONMENT_POS;
    uint32_t tn;
    EEPROM_READ(tn);
    return tn == FYS_EEPROM_POW_BREAK_SIGN;
  }
  void MarlinSettings::FunV00D()
  {
    uint16_t working_crc = 0;
    int eeprom_index = FYS_EEPROM_ENVIRONMENT_POS;
    uint32_t tn=FYS_EEPROM_POW_BREAK_SIGN;
    #ifdef FYS_POWER_CHECK_USE_POLLING
    cli(); 
    #endif
    EEPROM_WRITE(tn);
    #ifdef FYS_POWER_CHECK_USE_POLLING
    sei(); 
    #endif
  }
  void MarlinSettings::FunV004()
  {
    uint16_t working_crc = 0;
    int eeprom_index = FYS_EEPROM_ENVIRONMENT_POS;
    uint32_t tn=0xFFFF;
    cli(); 
    EEPROM_WRITE(tn);
    sei(); 
  }
#endif
  bool MarlinSettings::load() {
    uint16_t working_crc = 0;
    EEPROM_START();
    char stored_ver[4];
    EEPROM_READ(stored_ver);
    uint16_t stored_crc;
    EEPROM_READ(stored_crc);
    if (strncmp(version, stored_ver, 3) != 0) {
      if (stored_ver[0] != 'V') {
        stored_ver[0] = '?';
        stored_ver[1] = '\0';
      }
      #if ENABLED(EEPROM_CHITCHAT)
        SERIAL_ECHO_START();
        SERIAL_ECHOPGM("EEPROM version mismatch ");
        SERIAL_ECHOPAIR("(EEPROM=", stored_ver);
        SERIAL_ECHOLNPGM(" Marlin=" EEPROM_VERSION ")");
      #endif
      reset();
    }
    else {
      float dummy = 0;
      working_crc = 0; 
      uint8_t esteppers;
      EEPROM_READ(esteppers);
      const float def1[] = DEFAULT_AXIS_STEPS_PER_UNIT, def2[] = DEFAULT_MAX_FEEDRATE;
      const uint32_t def3[] = DEFAULT_MAX_ACCELERATION;
      float tmp1[XYZ + esteppers], tmp2[XYZ + esteppers];
      uint32_t tmp3[XYZ + esteppers];
      EEPROM_READ(tmp1);
      EEPROM_READ(tmp2);
      EEPROM_READ(tmp3);
      LOOP_XYZE_N(i) {
        planner.axis_steps_per_mm[i]          = i < XYZ + esteppers ? tmp1[i] : def1[i < COUNT(def1) ? i : COUNT(def1) - 1];
        planner.max_feedrate_mm_s[i]          = i < XYZ + esteppers ? tmp2[i] : def2[i < COUNT(def2) ? i : COUNT(def2) - 1];
        planner.max_acceleration_mm_per_s2[i] = i < XYZ + esteppers ? tmp3[i] : def3[i < COUNT(def3) ? i : COUNT(def3) - 1];
      }
      EEPROM_READ(planner.acceleration);
      EEPROM_READ(planner.retract_acceleration);
      EEPROM_READ(planner.travel_acceleration);
      EEPROM_READ(planner.min_feedrate_mm_s);
      EEPROM_READ(planner.min_travel_feedrate_mm_s);
      EEPROM_READ(planner.min_segment_time);
      EEPROM_READ(planner.max_jerk);
      #if !HAS_HOME_OFFSET
        float home_offset[XYZ];
      #endif
      EEPROM_READ(home_offset);
      #if ENABLED(DELTA)
        home_offset[X_AXIS] = 0.0;
        home_offset[Y_AXIS] = 0.0;
        home_offset[Z_AXIS] -= DELTA_HEIGHT;
      #endif
      #if HOTENDS > 1
        for (uint8_t e = 1; e < HOTENDS; e++)
          LOOP_XYZ(i) EEPROM_READ(hotend_offset[i][e]);
      #endif
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        EEPROM_READ(planner.z_fade_height);
      #else
        EEPROM_READ(dummy);
      #endif
      bool leveling_is_on;
      uint8_t mesh_num_x, mesh_num_y;
      EEPROM_READ(leveling_is_on);
      EEPROM_READ(dummy);
      EEPROM_READ(mesh_num_x);
      EEPROM_READ(mesh_num_y);
      #if ENABLED(MESH_BED_LEVELING)
        mbl.status = leveling_is_on ? _BV(MBL_STATUS_HAS_MESH_BIT) : 0;
        mbl.z_offset = dummy;
        if (mesh_num_x == GRID_MAX_POINTS_X && mesh_num_y == GRID_MAX_POINTS_Y) {
          EEPROM_READ(mbl.z_values);
        }
        else {
          mbl.reset();
          for (uint16_t q = mesh_num_x * mesh_num_y; q--;) EEPROM_READ(dummy);
        }
      #else
        for (uint16_t q = mesh_num_x * mesh_num_y; q--;) EEPROM_READ(dummy);
      #endif 
      #if !HAS_BED_PROBE
        float zprobe_zoffset;
      #endif
      EEPROM_READ(zprobe_zoffset);
      #if HAS_ABL
      EEPROM_READ(planner.abl_enabled);
      #else
      char amm = 0;
      EEPROM_READ(amm);
      #endif
      #if ABL_PLANAR
        EEPROM_READ(planner.bed_level_matrix);
      #else
        for (uint8_t q = 9; q--;) EEPROM_READ(dummy);
      #endif
      uint8_t grid_max_x, grid_max_y;
      EEPROM_READ(grid_max_x);                       
      EEPROM_READ(grid_max_y);                       
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (grid_max_x == GRID_MAX_POINTS_X && grid_max_y == GRID_MAX_POINTS_Y) {
          #ifdef FYS_SAFE_PRINT_BREAK
          #else
          set_bed_leveling_enabled(false);
          #endif
          EEPROM_READ(bilinear_grid_spacing);        
          EEPROM_READ(bilinear_start);               
          EEPROM_READ(z_values);                     
        }
        else 
      #endif 
        {
          int bgs[2], bs[2];
          EEPROM_READ(bgs);
          EEPROM_READ(bs);
          for (uint16_t q = grid_max_x * grid_max_y; q--;) EEPROM_READ(dummy);
        }
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        EEPROM_READ(ubl.state.active);
        EEPROM_READ(ubl.state.z_offset);
        EEPROM_READ(ubl.state.storage_slot);
      #else
        bool dummyb;
        uint8_t dummyui8;
        EEPROM_READ(dummyb);
        EEPROM_READ(dummy);
        EEPROM_READ(dummyui8);
      #endif 
      #if ENABLED(DELTA)
        EEPROM_READ(endstop_adj);               
        EEPROM_READ(delta_radius);              
        EEPROM_READ(delta_diagonal_rod);        
        EEPROM_READ(delta_segments_per_second); 
        EEPROM_READ(delta_calibration_radius);  
        EEPROM_READ(delta_tower_angle_trim);    
        dummy = 0.0f;
        for (uint8_t q=3; q--;) EEPROM_READ(dummy);
      #elif ENABLED(Z_DUAL_ENDSTOPS)
        EEPROM_READ(z_endstop_adj);
        dummy = 0.0f;
        for (uint8_t q=11; q--;) EEPROM_READ(dummy);
      #else
        dummy = 0.0f;
        for (uint8_t q=12; q--;) EEPROM_READ(dummy);
      #endif
      #if DISABLED(ULTIPANEL)&&DISABLED(FYSTLCD_V1)
        int lcd_preheat_hotend_temp[2], lcd_preheat_bed_temp[2], lcd_preheat_fan_speed[2];
      #endif
      EEPROM_READ(lcd_preheat_hotend_temp);
      EEPROM_READ(lcd_preheat_bed_temp);
      EEPROM_READ(lcd_preheat_fan_speed);
      #if ENABLED(PIDTEMP)
        for (uint8_t e = 0; e < MAX_EXTRUDERS; e++) {
          EEPROM_READ(dummy); 
          if (e < HOTENDS && dummy != DUMMY_PID_VALUE) {
            PID_PARAM(Kp, e) = dummy;
            EEPROM_READ(PID_PARAM(Ki, e));
            EEPROM_READ(PID_PARAM(Kd, e));
            #if ENABLED(PID_EXTRUSION_SCALING)
              EEPROM_READ(PID_PARAM(Kc, e));
            #else
              EEPROM_READ(dummy);
            #endif
          }
          else {
            for (uint8_t q=3; q--;) EEPROM_READ(dummy); 
          }
        }
      #else 
        for (uint8_t q = MAX_EXTRUDERS * 4; q--;) EEPROM_READ(dummy);  
      #endif 
      #if DISABLED(PID_EXTRUSION_SCALING)
        int lpq_len;
      #endif
      EEPROM_READ(lpq_len);
      #if ENABLED(PIDTEMPBED)
        EEPROM_READ(dummy); 
        if (dummy != DUMMY_PID_VALUE) {
          thermalManager.bedKp = dummy;
          EEPROM_READ(thermalManager.bedKi);
          EEPROM_READ(thermalManager.bedKd);
        }
      #else
        for (uint8_t q=3; q--;) EEPROM_READ(dummy); 
      #endif
      #if !HAS_LCD_CONTRAST
        uint16_t lcd_contrast;
      #endif
      EEPROM_READ(lcd_contrast);
      #if ENABLED(FWRETRACT)
        EEPROM_READ(autoretract_enabled);
        EEPROM_READ(retract_length);
        #if EXTRUDERS > 1
          EEPROM_READ(retract_length_swap);
        #else
          EEPROM_READ(dummy);
        #endif
        EEPROM_READ(retract_feedrate_mm_s);
        EEPROM_READ(retract_zlift);
        EEPROM_READ(retract_recover_length);
        #if EXTRUDERS > 1
          EEPROM_READ(retract_recover_length_swap);
        #else
          EEPROM_READ(dummy);
        #endif
        EEPROM_READ(retract_recover_feedrate_mm_s);
      #endif 
      EEPROM_READ(volumetric_enabled);
      for (uint8_t q = 0; q < MAX_EXTRUDERS; q++) {
        EEPROM_READ(dummy);
        if (q < COUNT(filament_size)) filament_size[q] = dummy;
      }
      uint16_t val;
      #if ENABLED(HAVE_TMC2130)
        EEPROM_READ(val);
        #if ENABLED(X_IS_TMC2130)
          stepperX.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(Y_IS_TMC2130)
          stepperY.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(Z_IS_TMC2130)
          stepperZ.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(X2_IS_TMC2130)
          stepperX2.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(Y2_IS_TMC2130)
          stepperY2.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(Z2_IS_TMC2130)
          stepperZ2.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(E0_IS_TMC2130)
          stepperE0.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(E1_IS_TMC2130)
          stepperE1.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(E2_IS_TMC2130)
          stepperE2.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(E3_IS_TMC2130)
          stepperE3.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(E4_IS_TMC2130)
          stepperE4.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
      #else
        for (uint8_t q = 0; q < 11; q++) EEPROM_READ(val);
      #endif
      #if ENABLED(LIN_ADVANCE)
        EEPROM_READ(planner.extruder_advance_k);
        EEPROM_READ(planner.advance_ed_ratio);
      #else
        EEPROM_READ(dummy);
        EEPROM_READ(dummy);
      #endif
      #if HAS_MOTOR_CURRENT_PWM
        for (uint8_t q = 3; q--;) EEPROM_READ(stepper.motor_current_setting[q]);
      #else
        uint32_t dummyui32;
        for (uint8_t q = 3; q--;) EEPROM_READ(dummyui32);
      #endif
    #ifdef FYS_ENERGY_CONSERVE_HEIGHT
        EEPROM_READ(zEnergyHeight);
    #endif
    #ifdef FYS_ACTIVE_TIME_OVER
        int16_t tmillis;
        EEPROM_READ(tmillis);
        max_inactive_time = (millis_t)tmillis* 1000UL;
    #endif
      if (working_crc == stored_crc) {
        postprocess();
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_ECHO_START();
          SERIAL_ECHO(version);
          SERIAL_ECHOPAIR(" stored settings retrieved (", eeprom_index - (EEPROM_OFFSET));
          SERIAL_ECHOPAIR(" bytes; crc ", working_crc);
          SERIAL_ECHOLNPGM(")");
        #endif
      }
      else {
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_ERROR_START();
          SERIAL_ERRORPGM("EEPROM CRC mismatch - (stored) ");
          SERIAL_ERROR(stored_crc);
          SERIAL_ERRORPGM(" != ");
          SERIAL_ERROR(working_crc);
          SERIAL_ERRORLNPGM(" (calculated)!");
        #endif
        reset();
      }
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        meshes_begin = (eeprom_index + 32) & 0xFFF8;  
        ubl.report_state();
        if (!ubl.sanity_check()) {
          SERIAL_EOL();
          #if ENABLED(EEPROM_CHITCHAT)
            ubl.echo_name();
            SERIAL_ECHOLNPGM(" initialized.\n");
          #endif
        }
        else {
          #if ENABLED(EEPROM_CHITCHAT)
            SERIAL_PROTOCOLPGM("?Can't enable ");
            ubl.echo_name();
            SERIAL_PROTOCOLLNPGM(".");
          #endif
          ubl.reset();
        }
        if (ubl.state.storage_slot >= 0) {
          load_mesh(ubl.state.storage_slot);
          #if ENABLED(EEPROM_CHITCHAT)
            SERIAL_ECHOPAIR("Mesh ", ubl.state.storage_slot);
            SERIAL_ECHOLNPGM(" loaded from storage.");
          #endif
        }
        else {
          ubl.reset();
          #if ENABLED(EEPROM_CHITCHAT)
            SERIAL_ECHOLNPGM("UBL System reset()");
          #endif
        }
      #endif
    }
    #if ENABLED(EEPROM_CHITCHAT) && DISABLED(DISABLE_M503)
      report();
    #endif
    return !eeprom_error;
  }
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    #if ENABLED(EEPROM_CHITCHAT)
      void ubl_invalid_slot(const int s) {
        SERIAL_PROTOCOLLNPGM("?Invalid slot.");
        SERIAL_PROTOCOL(s);
        SERIAL_PROTOCOLLNPGM(" mesh slots available.");
      }
    #endif
    int MarlinSettings::calc_num_meshes() {
      if (meshes_begin <= 0) return 0;
      return (meshes_end - meshes_begin) / sizeof(ubl.z_values);
    }
    void MarlinSettings::store_mesh(int8_t slot) {
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        const int a = calc_num_meshes();
        if (!WITHIN(slot, 0, a - 1)) {
          #if ENABLED(EEPROM_CHITCHAT)
            ubl_invalid_slot(a);
            SERIAL_PROTOCOLPAIR("E2END=", E2END);
            SERIAL_PROTOCOLPAIR(" meshes_end=", meshes_end);
            SERIAL_PROTOCOLLNPAIR(" slot=", slot);
            SERIAL_EOL();
          #endif
          return;
        }
        uint16_t crc = 0;
        int pos = meshes_end - (slot + 1) * sizeof(ubl.z_values);
        write_data(pos, (uint8_t *)&ubl.z_values, sizeof(ubl.z_values), &crc);
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_PROTOCOLLNPAIR("Mesh saved in slot ", slot);
        #endif
      #else
      #endif
    }
    void MarlinSettings::load_mesh(int8_t slot, void *into ) {
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        const int16_t a = settings.calc_num_meshes();
        if (!WITHIN(slot, 0, a - 1)) {
          #if ENABLED(EEPROM_CHITCHAT)
            ubl_invalid_slot(a);
          #endif
          return;
        }
        uint16_t crc = 0;
        int pos = meshes_end - (slot + 1) * sizeof(ubl.z_values);
        uint8_t * const dest = into ? (uint8_t*)into : (uint8_t*)&ubl.z_values;
        read_data(pos, dest, sizeof(ubl.z_values), &crc);
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_PROTOCOLLNPAIR("Mesh loaded from slot ", slot);
        #endif
      #else
      #endif
    }
  #endif 
#else 
  bool MarlinSettings::save() {
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM("EEPROM disabled");
    return false;
  }
#endif 
void MarlinSettings::reset() {
  static const float tmp1[] PROGMEM = DEFAULT_AXIS_STEPS_PER_UNIT, tmp2[] PROGMEM = DEFAULT_MAX_FEEDRATE;
  static const uint32_t tmp3[] PROGMEM = DEFAULT_MAX_ACCELERATION;
  LOOP_XYZE_N(i) {
    planner.axis_steps_per_mm[i]          = pgm_read_float(&tmp1[i < COUNT(tmp1) ? i : COUNT(tmp1) - 1]);
    planner.max_feedrate_mm_s[i]          = pgm_read_float(&tmp2[i < COUNT(tmp2) ? i : COUNT(tmp2) - 1]);
    planner.max_acceleration_mm_per_s2[i] = pgm_read_dword_near(&tmp3[i < COUNT(tmp3) ? i : COUNT(tmp3) - 1]);
  }
  planner.acceleration = DEFAULT_ACCELERATION;
  planner.retract_acceleration = DEFAULT_RETRACT_ACCELERATION;
  planner.travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
  planner.min_feedrate_mm_s = DEFAULT_MINIMUMFEEDRATE;
  planner.min_segment_time = DEFAULT_MINSEGMENTTIME;
  planner.min_travel_feedrate_mm_s = DEFAULT_MINTRAVELFEEDRATE;
  planner.max_jerk[X_AXIS] = DEFAULT_XJERK;
  planner.max_jerk[Y_AXIS] = DEFAULT_YJERK;
  planner.max_jerk[Z_AXIS] = DEFAULT_ZJERK;
  planner.max_jerk[E_AXIS] = DEFAULT_EJERK;
  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    planner.z_fade_height = 0.0;
  #endif
  #if HAS_HOME_OFFSET
    #if defined(MANUAL_X_HOME_POS)
        home_offset[X_AXIS] = MANUAL_X_HOME_POS;
    #else
        home_offset[X_AXIS] = 0;
    #endif
    #if defined(MANUAL_X_HOME_POS)
        home_offset[Y_AXIS] = MANUAL_Y_HOME_POS;
    #else
        home_offset[Y_AXIS] = 0;
    #endif
    #if defined(MANUAL_Z_HOME_POS)
        home_offset[Z_AXIS] = MANUAL_Z_HOME_POS;
    #else
        home_offset[Z_AXIS] = 0;
    #endif
  #endif
  #if HOTENDS > 1
    constexpr float tmp4[XYZ][HOTENDS] = {
      HOTEND_OFFSET_X,
      HOTEND_OFFSET_Y
      #ifdef HOTEND_OFFSET_Z
        , HOTEND_OFFSET_Z
      #else
        , { 0 }
      #endif
    };
    static_assert(
      tmp4[X_AXIS][0] == 0 && tmp4[Y_AXIS][0] == 0 && tmp4[Z_AXIS][0] == 0,
      "Offsets for the first hotend must be 0.0."
    );
    LOOP_XYZ(i) HOTEND_LOOP() hotend_offset[i][e] = tmp4[i][e];
  #endif
  #if HAS_LEVELING
    reset_bed_level();
  #endif
  #if HAS_BED_PROBE
    zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
  #endif
  #if ENABLED(DELTA)
    const float adj[ABC] = DELTA_ENDSTOP_ADJ,
                dta[ABC] = DELTA_TOWER_ANGLE_TRIM;
    COPY(endstop_adj, adj);
    delta_radius = DELTA_RADIUS;
    delta_diagonal_rod = DELTA_DIAGONAL_ROD;
    delta_segments_per_second = DELTA_SEGMENTS_PER_SECOND;
    delta_calibration_radius = DELTA_CALIBRATION_RADIUS;
    delta_tower_angle_trim[A_AXIS] = dta[A_AXIS] - dta[C_AXIS];
    delta_tower_angle_trim[B_AXIS] = dta[B_AXIS] - dta[C_AXIS];
    home_offset[Z_AXIS] = 0;
  #elif ENABLED(Z_DUAL_ENDSTOPS)
    z_endstop_adj =
      #ifdef Z_DUAL_ENDSTOPS_ADJUSTMENT
        Z_DUAL_ENDSTOPS_ADJUSTMENT
      #else
        0
      #endif
    ;
  #endif
    lcd_preheat_hotend_temp[0] = PREHEAT_1_TEMP_HOTEND;
    lcd_preheat_hotend_temp[1] = PREHEAT_2_TEMP_HOTEND;
    lcd_preheat_bed_temp[0] = PREHEAT_1_TEMP_BED;
    lcd_preheat_bed_temp[1] = PREHEAT_2_TEMP_BED;
    lcd_preheat_fan_speed[0] = PREHEAT_1_FAN_SPEED;
    lcd_preheat_fan_speed[1] = PREHEAT_2_FAN_SPEED;
  #if HAS_LCD_CONTRAST
    lcd_contrast = DEFAULT_LCD_CONTRAST;
  #endif
  #if ENABLED(PIDTEMP)
    #if ENABLED(PID_PARAMS_PER_HOTEND) && HOTENDS > 1
      HOTEND_LOOP()
    #endif
    {
      PID_PARAM(Kp, e) = DEFAULT_Kp;
      PID_PARAM(Ki, e) = scalePID_i(DEFAULT_Ki);
      PID_PARAM(Kd, e) = scalePID_d(DEFAULT_Kd);
      #if ENABLED(PID_EXTRUSION_SCALING)
        PID_PARAM(Kc, e) = DEFAULT_Kc;
      #endif
    }
    #if ENABLED(PID_EXTRUSION_SCALING)
      lpq_len = 20; 
    #endif
  #endif 
  #if ENABLED(PIDTEMPBED)
    thermalManager.bedKp = DEFAULT_bedKp;
    thermalManager.bedKi = scalePID_i(DEFAULT_bedKi);
    thermalManager.bedKd = scalePID_d(DEFAULT_bedKd);
  #endif
  #if ENABLED(FWRETRACT)
    autoretract_enabled = false;
    retract_length = RETRACT_LENGTH;
    #if EXTRUDERS > 1
      retract_length_swap = RETRACT_LENGTH_SWAP;
    #endif
    retract_feedrate_mm_s = RETRACT_FEEDRATE;
    retract_zlift = RETRACT_ZLIFT;
    retract_recover_length = RETRACT_RECOVER_LENGTH;
    #if EXTRUDERS > 1
      retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
    #endif
    retract_recover_feedrate_mm_s = RETRACT_RECOVER_FEEDRATE;
  #endif
  volumetric_enabled =
    #if ENABLED(VOLUMETRIC_DEFAULT_ON)
      true
    #else
      false
    #endif
  ;
  for (uint8_t q = 0; q < COUNT(filament_size); q++)
    filament_size[q] = DEFAULT_NOMINAL_FILAMENT_DIA;
  endstops.enable_globally(
    #if ENABLED(ENDSTOPS_ALWAYS_ON_DEFAULT)
      true
    #else
      false
    #endif
  );
  #if ENABLED(HAVE_TMC2130)
    #if ENABLED(X_IS_TMC2130)
      stepperX.setCurrent(X_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(Y_IS_TMC2130)
      stepperY.setCurrent(Y_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(Z_IS_TMC2130)
      stepperZ.setCurrent(Z_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(X2_IS_TMC2130)
      stepperX2.setCurrent(X2_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(Y2_IS_TMC2130)
      stepperY2.setCurrent(Y2_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(Z2_IS_TMC2130)
      stepperZ2.setCurrent(Z2_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(E0_IS_TMC2130)
      stepperE0.setCurrent(E0_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(E1_IS_TMC2130)
      stepperE1.setCurrent(E1_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(E2_IS_TMC2130)
      stepperE2.setCurrent(E2_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(E3_IS_TMC2130)
      stepperE3.setCurrent(E3_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
  #endif
  #if ENABLED(LIN_ADVANCE)
    planner.extruder_advance_k = LIN_ADVANCE_K;
    planner.advance_ed_ratio = LIN_ADVANCE_E_D_RATIO;
  #endif
  #if HAS_MOTOR_CURRENT_PWM
    uint32_t tmp_motor_current_setting[3] = PWM_MOTOR_CURRENT;
    for (uint8_t q = 3; q--;)
      stepper.digipot_current(q, (stepper.motor_current_setting[q] = tmp_motor_current_setting[q]));
  #endif
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    ubl.reset();
  #endif
  postprocess();
    #ifdef FYS_ENERGY_CONSERVE_HEIGHT
      zEnergyHeight = FYS_ENERGY_CONSERVE_HEIGHT;
    #endif
    #ifdef FYS_ACTIVE_TIME_OVER
      max_inactive_time = (FYS_ACTIVE_TIME_OVER) * 1000UL;
    #endif
  #if ENABLED(EEPROM_CHITCHAT)
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");
  #endif
}
#if DISABLED(DISABLE_M503)
  #define CONFIG_ECHO_START do{ if (!forReplay) SERIAL_ECHO_START(); }while(0)
  void MarlinSettings::report(bool forReplay) {
    CONFIG_ECHO_START;
    #if ENABLED(INCH_MODE_SUPPORT)
      #define LINEAR_UNIT(N) ((N) / parser.linear_unit_factor)
      #define VOLUMETRIC_UNIT(N) ((N) / (volumetric_enabled ? parser.volumetric_unit_factor : parser.linear_unit_factor))
      SERIAL_ECHOPGM("  G2");
      SERIAL_CHAR(parser.linear_unit_factor == 1.0 ? '1' : '0');
      SERIAL_ECHOPGM(" ; Units in ");
      serialprintPGM(parser.linear_unit_factor == 1.0 ? PSTR("mm\n") : PSTR("inches\n"));
    #else
      #define LINEAR_UNIT(N) N
      #define VOLUMETRIC_UNIT(N) N
      SERIAL_ECHOLNPGM("  G21    ; Units in mm");
    #endif
      CONFIG_ECHO_START;
      #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
        #define TEMP_UNIT(N) parser.to_temp_units(N)
        SERIAL_ECHOPGM("  M149 ");
        SERIAL_CHAR(parser.temp_units_code());
        SERIAL_ECHOPGM(" ; Units in ");
        serialprintPGM(parser.temp_units_name());
      #else
        #define TEMP_UNIT(N) N
        SERIAL_ECHOLNPGM("  M149 C ; Units in Celsius");
      #endif
    SERIAL_EOL();
    if (!forReplay) {
      CONFIG_ECHO_START;
      SERIAL_ECHOPGM("Filament settings:");
      if (volumetric_enabled)
        SERIAL_EOL();
      else
        SERIAL_ECHOLNPGM(" Disabled");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M200 D", filament_size[0]);
    SERIAL_EOL();
    #if EXTRUDERS > 1
      CONFIG_ECHO_START;
      SERIAL_ECHOPAIR("  M200 T1 D", filament_size[1]);
      SERIAL_EOL();
      #if EXTRUDERS > 2
        CONFIG_ECHO_START;
        SERIAL_ECHOPAIR("  M200 T2 D", filament_size[2]);
        SERIAL_EOL();
        #if EXTRUDERS > 3
          CONFIG_ECHO_START;
          SERIAL_ECHOPAIR("  M200 T3 D", filament_size[3]);
          SERIAL_EOL();
          #if EXTRUDERS > 4
            CONFIG_ECHO_START;
            SERIAL_ECHOPAIR("  M200 T4 D", filament_size[4]);
            SERIAL_EOL();
          #endif 
        #endif 
      #endif 
    #endif 
    if (!volumetric_enabled) {
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPGM("  M200 D0");
    }
    if (!forReplay) {
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPGM("Steps per unit:");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M92 X", LINEAR_UNIT(planner.axis_steps_per_mm[X_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(planner.axis_steps_per_mm[Y_AXIS]));
    SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(planner.axis_steps_per_mm[Z_AXIS]));
    #if DISABLED(DISTINCT_E_FACTORS)
      SERIAL_ECHOPAIR(" E", VOLUMETRIC_UNIT(planner.axis_steps_per_mm[E_AXIS]));
    #endif
    SERIAL_EOL();
    #if ENABLED(DISTINCT_E_FACTORS)
      CONFIG_ECHO_START;
      for (uint8_t i = 0; i < E_STEPPERS; i++) {
        SERIAL_ECHOPAIR("  M92 T", (int)i);
        SERIAL_ECHOLNPAIR(" E", VOLUMETRIC_UNIT(planner.axis_steps_per_mm[E_AXIS + i]));
      }
    #endif
    if (!forReplay) {
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPGM("Maximum feedrates (units/s):");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M203 X", LINEAR_UNIT(planner.max_feedrate_mm_s[X_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(planner.max_feedrate_mm_s[Y_AXIS]));
    SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(planner.max_feedrate_mm_s[Z_AXIS]));
    #if DISABLED(DISTINCT_E_FACTORS)
      SERIAL_ECHOPAIR(" E", VOLUMETRIC_UNIT(planner.max_feedrate_mm_s[E_AXIS]));
    #endif
    SERIAL_EOL();
    #if ENABLED(DISTINCT_E_FACTORS)
      CONFIG_ECHO_START;
      for (uint8_t i = 0; i < E_STEPPERS; i++) {
        SERIAL_ECHOPAIR("  M203 T", (int)i);
        SERIAL_ECHOLNPAIR(" E", VOLUMETRIC_UNIT(planner.max_feedrate_mm_s[E_AXIS + i]));
      }
    #endif
    if (!forReplay) {
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPGM("Maximum Acceleration (units/s2):");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M201 X", LINEAR_UNIT(planner.max_acceleration_mm_per_s2[X_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(planner.max_acceleration_mm_per_s2[Y_AXIS]));
    SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(planner.max_acceleration_mm_per_s2[Z_AXIS]));
    #if DISABLED(DISTINCT_E_FACTORS)
      SERIAL_ECHOPAIR(" E", VOLUMETRIC_UNIT(planner.max_acceleration_mm_per_s2[E_AXIS]));
    #endif
    SERIAL_EOL();
    #if ENABLED(DISTINCT_E_FACTORS)
      CONFIG_ECHO_START;
      for (uint8_t i = 0; i < E_STEPPERS; i++) {
        SERIAL_ECHOPAIR("  M201 T", (int)i);
        SERIAL_ECHOLNPAIR(" E", VOLUMETRIC_UNIT(planner.max_acceleration_mm_per_s2[E_AXIS + i]));
      }
    #endif
    if (!forReplay) {
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPGM("Acceleration (units/s2): P<print_accel> R<retract_accel> T<travel_accel>");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M204 P", LINEAR_UNIT(planner.acceleration));
    SERIAL_ECHOPAIR(" R", LINEAR_UNIT(planner.retract_acceleration));
    SERIAL_ECHOLNPAIR(" T", LINEAR_UNIT(planner.travel_acceleration));
    if (!forReplay) {
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPGM("Advanced: S<min_feedrate> T<min_travel_feedrate> B<min_segment_time_ms> X<max_xy_jerk> Z<max_z_jerk> E<max_e_jerk>");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M205 S", LINEAR_UNIT(planner.min_feedrate_mm_s));
    SERIAL_ECHOPAIR(" T", LINEAR_UNIT(planner.min_travel_feedrate_mm_s));
    SERIAL_ECHOPAIR(" B", planner.min_segment_time);
    SERIAL_ECHOPAIR(" X", LINEAR_UNIT(planner.max_jerk[X_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(planner.max_jerk[Y_AXIS]));
    SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(planner.max_jerk[Z_AXIS]));
    SERIAL_ECHOLNPAIR(" E", LINEAR_UNIT(planner.max_jerk[E_AXIS]));
    #if HAS_M206_COMMAND
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Home offset:");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOPAIR("  M206 X", LINEAR_UNIT(home_offset[X_AXIS]));
      SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(home_offset[Y_AXIS]));
      SERIAL_ECHOLNPAIR(" Z", LINEAR_UNIT(home_offset[Z_AXIS]));
    #endif
    #if HOTENDS > 1
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Hotend offsets:");
      }
      CONFIG_ECHO_START;
      for (uint8_t e = 1; e < HOTENDS; e++) {
        SERIAL_ECHOPAIR("  M218 T", (int)e);
        SERIAL_ECHOPAIR(" X", LINEAR_UNIT(hotend_offset[X_AXIS][e]));
        SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(hotend_offset[Y_AXIS][e]));
        #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(SWITCHING_NOZZLE)
          SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(hotend_offset[Z_AXIS][e]));
        #endif
        SERIAL_EOL();
      }
    #endif
    #if ENABLED(MESH_BED_LEVELING)
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Mesh Bed Leveling:");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOPAIR("  M420 S", leveling_is_valid() ? 1 : 0);
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(planner.z_fade_height));
      #endif
      SERIAL_EOL();
      for (uint8_t py = 0; py < GRID_MAX_POINTS_Y; py++) {
        for (uint8_t px = 0; px < GRID_MAX_POINTS_X; px++) {
          CONFIG_ECHO_START;
          SERIAL_ECHOPAIR("  G29 S3 X", (int)px + 1);
          SERIAL_ECHOPAIR(" Y", (int)py + 1);
          SERIAL_ECHOPGM(" Z");
          SERIAL_PROTOCOL_F(LINEAR_UNIT(mbl.z_values[px][py]), 5);
          SERIAL_EOL();
        }
      }
    #elif ENABLED(AUTO_BED_LEVELING_UBL)
      if (!forReplay) {
        CONFIG_ECHO_START;
        ubl.echo_name();
        SERIAL_ECHOLNPGM(":");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOPAIR("  M420 S", leveling_is_active() ? 1 : 0);
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        SERIAL_ECHOPAIR(" Z", planner.z_fade_height);
      #endif
      SERIAL_EOL();
      if (!forReplay) {
        SERIAL_EOL();
        ubl.report_state();
        SERIAL_ECHOLNPAIR("\nActive Mesh Slot: ", ubl.state.storage_slot);
        SERIAL_ECHOPGM("z_offset: ");
        SERIAL_ECHO_F(ubl.state.z_offset, 6);
        SERIAL_EOL();
        SERIAL_ECHOPAIR("EEPROM can hold ", calc_num_meshes());
        SERIAL_ECHOLNPGM(" meshes.\n");
      }
    #elif HAS_ABL
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Auto Bed Leveling:");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOPAIR("  M420 S", leveling_is_active() ? 1 : 0);
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        SERIAL_ECHOPAIR(" Z", LINEAR_UNIT(planner.z_fade_height));
      #endif
      SERIAL_EOL();
    #endif
    #if ENABLED(DELTA)
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Endstop adjustment:");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOPAIR("  M666 X", LINEAR_UNIT(endstop_adj[X_AXIS]));
      SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(endstop_adj[Y_AXIS]));
      SERIAL_ECHOLNPAIR(" Z", LINEAR_UNIT(endstop_adj[Z_AXIS]));
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Delta settings: L<diagonal_rod> R<radius> H<height> S<segments_per_s> B<calibration radius> XYZ<tower angle corrections>");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOPAIR("  M665 L", LINEAR_UNIT(delta_diagonal_rod));
      SERIAL_ECHOPAIR(" R", LINEAR_UNIT(delta_radius));
      SERIAL_ECHOPAIR(" H", LINEAR_UNIT(DELTA_HEIGHT + home_offset[Z_AXIS]));
      SERIAL_ECHOPAIR(" S", delta_segments_per_second);
      SERIAL_ECHOPAIR(" B", LINEAR_UNIT(delta_calibration_radius));
      SERIAL_ECHOPAIR(" X", LINEAR_UNIT(delta_tower_angle_trim[A_AXIS]));
      SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(delta_tower_angle_trim[B_AXIS]));
      SERIAL_ECHOPAIR(" Z", 0.00);
      SERIAL_EOL();
    #elif ENABLED(Z_DUAL_ENDSTOPS)
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Z2 Endstop adjustment:");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPAIR("  M666 Z", LINEAR_UNIT(z_endstop_adj));
    #endif 
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Material heatup parameters:");
      }
      CONFIG_ECHO_START;
      for (uint8_t i = 0; i < COUNT(lcd_preheat_hotend_temp); i++) {
        SERIAL_ECHOPAIR("  M145 S", (int)i);
        SERIAL_ECHOPAIR(" H", TEMP_UNIT(lcd_preheat_hotend_temp[i]));
        SERIAL_ECHOPAIR(" B", TEMP_UNIT(lcd_preheat_bed_temp[i]));
        SERIAL_ECHOLNPAIR(" F", lcd_preheat_fan_speed[i]);
      }
    #if HAS_PID_HEATING
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("PID settings:");
      }
      #if ENABLED(PIDTEMP)
        #if HOTENDS > 1
          if (forReplay) {
            HOTEND_LOOP() {
              CONFIG_ECHO_START;
              SERIAL_ECHOPAIR("  M301 E", e);
              SERIAL_ECHOPAIR(" P", PID_PARAM(Kp, e));
              SERIAL_ECHOPAIR(" I", unscalePID_i(PID_PARAM(Ki, e)));
              SERIAL_ECHOPAIR(" D", unscalePID_d(PID_PARAM(Kd, e)));
              #if ENABLED(PID_EXTRUSION_SCALING)
                SERIAL_ECHOPAIR(" C", PID_PARAM(Kc, e));
                if (e == 0) SERIAL_ECHOPAIR(" L", lpq_len);
              #endif
              SERIAL_EOL();
            }
          }
          else
        #endif 
        {
          CONFIG_ECHO_START;
          SERIAL_ECHOPAIR("  M301 P", PID_PARAM(Kp, 0)); 
          SERIAL_ECHOPAIR(" I", unscalePID_i(PID_PARAM(Ki, 0)));
          SERIAL_ECHOPAIR(" D", unscalePID_d(PID_PARAM(Kd, 0)));
          #if ENABLED(PID_EXTRUSION_SCALING)
            SERIAL_ECHOPAIR(" C", PID_PARAM(Kc, 0));
            SERIAL_ECHOPAIR(" L", lpq_len);
          #endif
          SERIAL_EOL();
        }
      #endif 
      #if ENABLED(PIDTEMPBED)
        CONFIG_ECHO_START;
        SERIAL_ECHOPAIR("  M304 P", thermalManager.bedKp);
        SERIAL_ECHOPAIR(" I", unscalePID_i(thermalManager.bedKi));
        SERIAL_ECHOPAIR(" D", unscalePID_d(thermalManager.bedKd));
        SERIAL_EOL();
      #endif
    #endif 
    #if HAS_LCD_CONTRAST
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("LCD Contrast:");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPAIR("  M250 C", lcd_contrast);
    #endif
    #if ENABLED(FWRETRACT)
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Retract: S<length> F<units/m> Z<lift>");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOPAIR("  M207 S", LINEAR_UNIT(retract_length));
      #if EXTRUDERS > 1
        SERIAL_ECHOPAIR(" W", LINEAR_UNIT(retract_length_swap));
      #endif
      SERIAL_ECHOPAIR(" F", MMS_TO_MMM(LINEAR_UNIT(retract_feedrate_mm_s)));
      SERIAL_ECHOLNPAIR(" Z", LINEAR_UNIT(retract_zlift));
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Recover: S<length> F<units/m>");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOPAIR("  M208 S", LINEAR_UNIT(retract_recover_length));
      #if EXTRUDERS > 1
        SERIAL_ECHOPAIR(" W", LINEAR_UNIT(retract_recover_length_swap));
      #endif
      SERIAL_ECHOLNPAIR(" F", MMS_TO_MMM(LINEAR_UNIT(retract_recover_feedrate_mm_s)));
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Auto-Retract: S=0 to disable, 1 to interpret extrude-only moves as retracts or recoveries");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPAIR("  M209 S", autoretract_enabled ? 1 : 0);
    #endif 
    #if HAS_BED_PROBE
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Z-Probe Offset (mm):");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPAIR("  M851 Z", LINEAR_UNIT(zprobe_zoffset));
    #endif
    #if ENABLED(HAVE_TMC2130)
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Stepper driver current:");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHO("  M906");
      #if ENABLED(X_IS_TMC2130)
        SERIAL_ECHOPAIR(" X", stepperX.getCurrent());
      #endif
      #if ENABLED(Y_IS_TMC2130)
        SERIAL_ECHOPAIR(" Y", stepperY.getCurrent());
      #endif
      #if ENABLED(Z_IS_TMC2130)
        SERIAL_ECHOPAIR(" Z", stepperZ.getCurrent());
      #endif
      #if ENABLED(X2_IS_TMC2130)
        SERIAL_ECHOPAIR(" X2", stepperX2.getCurrent());
      #endif
      #if ENABLED(Y2_IS_TMC2130)
        SERIAL_ECHOPAIR(" Y2", stepperY2.getCurrent());
      #endif
      #if ENABLED(Z2_IS_TMC2130)
        SERIAL_ECHOPAIR(" Z2", stepperZ2.getCurrent());
      #endif
      #if ENABLED(E0_IS_TMC2130)
        SERIAL_ECHOPAIR(" E0", stepperE0.getCurrent());
      #endif
      #if ENABLED(E1_IS_TMC2130)
        SERIAL_ECHOPAIR(" E1", stepperE1.getCurrent());
      #endif
      #if ENABLED(E2_IS_TMC2130)
        SERIAL_ECHOPAIR(" E2", stepperE2.getCurrent());
      #endif
      #if ENABLED(E3_IS_TMC2130)
        SERIAL_ECHOPAIR(" E3", stepperE3.getCurrent());
      #endif
      SERIAL_EOL();
    #endif
    #if ENABLED(LIN_ADVANCE)
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM("Linear Advance:");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOPAIR("  M900 K", planner.extruder_advance_k);
      SERIAL_ECHOLNPAIR(" R", planner.advance_ed_ratio);
    #endif
    #if HAS_MOTOR_CURRENT_PWM
      CONFIG_ECHO_START;
      if (!forReplay) {
        SERIAL_ECHOLNPGM("Stepper motor currents:");
        CONFIG_ECHO_START;
      }
      SERIAL_ECHOPAIR("  M907 X", stepper.motor_current_setting[0]);
      SERIAL_ECHOPAIR(" Z", stepper.motor_current_setting[1]);
      SERIAL_ECHOPAIR(" E", stepper.motor_current_setting[2]);
      SERIAL_EOL();
    #endif
  }
#endif 
