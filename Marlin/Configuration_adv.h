#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H_VERSION 010100
#if DISABLED(PIDTEMPBED)
  #define BED_CHECK_INTERVAL 5000 
  #if ENABLED(BED_LIMIT_SWITCHING)
    #define BED_HYSTERESIS 2 
  #endif
#endif
#if ENABLED(THERMAL_PROTECTION_HOTENDS)
    #define THERMAL_PROTECTION_PERIOD 40        
    #define THERMAL_PROTECTION_HYSTERESIS 4     
    #define WATCH_TEMP_PERIOD 120                
    #define WATCH_TEMP_INCREASE 2               
#endif
#if ENABLED(THERMAL_PROTECTION_BED)
    #define THERMAL_PROTECTION_BED_PERIOD 120    
    #define THERMAL_PROTECTION_BED_HYSTERESIS 2 
    #define WATCH_BED_TEMP_PERIOD 100                
    #define WATCH_BED_TEMP_INCREASE 2               
#endif
#if ENABLED(PIDTEMP)
  #if ENABLED(PID_EXTRUSION_SCALING)
    #define DEFAULT_Kc (100) 
    #define LPQ_MAX_LEN 50
  #endif
#endif
#define AUTOTEMP
#if ENABLED(AUTOTEMP)
  #define AUTOTEMP_OLDWEIGHT 0.98
#endif
#if ENABLED(EXTRUDER_RUNOUT_PREVENT)
  #define EXTRUDER_RUNOUT_MINTEMP 190
  #define EXTRUDER_RUNOUT_SECONDS 30
  #define EXTRUDER_RUNOUT_SPEED 1500  
  #define EXTRUDER_RUNOUT_EXTRUDE 5   
#endif
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0
#if ENABLED(USE_CONTROLLER_FAN)
  #define CONTROLLERFAN_SECS 60          
  #define CONTROLLERFAN_SPEED 255        
#endif
#define E0_AUTO_FAN_PIN -1
#define E1_AUTO_FAN_PIN -1
#define E2_AUTO_FAN_PIN -1
#define E3_AUTO_FAN_PIN -1
#define E4_AUTO_FAN_PIN -1
#define EXTRUDER_AUTO_FAN_TEMPERATURE 50
#define EXTRUDER_AUTO_FAN_SPEED   255  
#if ENABLED(CASE_LIGHT_ENABLE)
  #define INVERT_CASE_LIGHT false             
  #define CASE_LIGHT_DEFAULT_ON true          
  #define CASE_LIGHT_DEFAULT_BRIGHTNESS 105   
#endif
#if ENABLED(X_DUAL_STEPPER_DRIVERS)
  #define INVERT_X2_VS_X_DIR true
#endif
#if ENABLED(Y_DUAL_STEPPER_DRIVERS)
  #define INVERT_Y2_VS_Y_DIR true
#endif
#if ENABLED(Z_DUAL_STEPPER_DRIVERS)
  #if ENABLED(Z_DUAL_ENDSTOPS)
    #define Z2_USE_ENDSTOP _XMAX_
    #define Z_DUAL_ENDSTOPS_ADJUSTMENT  0  
  #endif
#endif 
#if ENABLED(DUAL_X_CARRIAGE)
  #define X2_MIN_POS 80     
  #define X2_MAX_POS 353    
  #define X2_HOME_DIR 1     
  #define X2_HOME_POS X2_MAX_POS 
  #if ENABLED(FYS_DXC_FYS_MODE)
    #define DEFAULT_DUAL_X_CARRIAGE_MODE DXC_FYS_MODE 
  #else
    #define DEFAULT_DUAL_X_CARRIAGE_MODE DXC_FULL_CONTROL_MODE
  #endif
  #define TOOLCHANGE_PARK_ZLIFT   0.2      
  #define TOOLCHANGE_UNPARK_ZLIFT 1        
  #define DEFAULT_DUPLICATION_X_OFFSET 100
#endif 
#define X_HOME_BUMP_MM 5
#define Y_HOME_BUMP_MM 5
#define Z_HOME_BUMP_MM 2 
#define HOMING_BUMP_DIVISOR {2, 2, 2}  
#define AXIS_RELATIVE_MODES {false, false, false, false}
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false
#define DEFAULT_STEPPER_DEACTIVE_TIME 120
#define DISABLE_INACTIVE_X true
#define DISABLE_INACTIVE_Y true
#define DISABLE_INACTIVE_Z true  
#define DISABLE_INACTIVE_E true
#define DEFAULT_MINIMUMFEEDRATE       0.0     
#define DEFAULT_MINTRAVELFEEDRATE     0.0
#if ENABLED(ULTIPANEL)
  #define MANUAL_FEEDRATE {50*60, 50*60, 4*60, 60} 
  #define ULTIPANEL_FEEDMULTIPLY  
#endif
#define DEFAULT_MINSEGMENTTIME        20000
#define SLOWDOWN
#define MINIMUM_PLANNER_SPEED 0.05
#define MICROSTEP_MODES {16,16,16,16,16} 
#define DIGIPOT_I2C_NUM_CHANNELS 8 
#define DIGIPOT_I2C_MOTOR_CURRENTS { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 }  
#define ENCODER_RATE_MULTIPLIER         
#define ENCODER_10X_STEPS_PER_SEC 75    
#define ENCODER_100X_STEPS_PER_SEC 160  
#define CHDK_DELAY 50 
#if ENABLED(SDSUPPORT)
  #define SD_DETECT_INVERTED
  #define SD_FINISHED_STEPPERRELEASE true  
  #define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E" 
  #define SDCARD_RATHERRECENTFIRST  
  #if ENABLED(SDCARD_SORT_ALPHA)
    #define SDSORT_LIMIT       40     
    #define FOLDER_SORTING     -1     
    #define SDSORT_GCODE       false  
    #define SDSORT_USES_RAM    false  
    #define SDSORT_USES_STACK  false  
    #define SDSORT_CACHE_NAMES false  
    #define SDSORT_DYNAMIC_RAM false  
  #endif
  #if ENABLED(LCD_PROGRESS_BAR)
    #define PROGRESS_BAR_BAR_TIME 2000
    #define PROGRESS_BAR_MSG_TIME 3000
    #define PROGRESS_MSG_EXPIRE   0
  #endif
#endif 
#if ENABLED(DOGLCD)
  #define XYZ_HOLLOW_FRAME
#endif 
#define USE_WATCHDOG
#if ENABLED(USE_WATCHDOG)
#endif
    #define BABYSTEPPING
    #if ENABLED(BABYSTEPPING)
      #define BABYSTEP_INVERT_Z false  
      #define BABYSTEP_MULTIPLICATOR 1 
      #define BABYSTEP_ZPROBE_OFFSET 
      #define DOUBLECLICK_MAX_INTERVAL 1250 
     #endif
#if ENABLED(ADVANCE)
  #define EXTRUDER_ADVANCE_K .0
  #define D_FILAMENT 2.85
#endif
#if ENABLED(LIN_ADVANCE)
  #define LIN_ADVANCE_K 75
  #define LIN_ADVANCE_E_D_RATIO 0 
#endif
#if ENABLED(MESH_BED_LEVELING)
  #define MESH_MIN_X (X_MIN_POS + MESH_INSET)
  #define MESH_MAX_X (X_MAX_POS - (MESH_INSET))
  #define MESH_MIN_Y (Y_MIN_POS + MESH_INSET)
  #define MESH_MAX_Y (Y_MAX_POS - (MESH_INSET))
#elif ENABLED(AUTO_BED_LEVELING_UBL)
  #define UBL_MESH_MIN_X (X_MIN_POS + UBL_MESH_INSET)
  #define UBL_MESH_MAX_X (X_MAX_POS - (UBL_MESH_INSET))
  #define UBL_MESH_MIN_Y (Y_MIN_POS + UBL_MESH_INSET)
  #define UBL_MESH_MAX_Y (Y_MAX_POS - (UBL_MESH_INSET))
  #define UBL_SAVE_ACTIVE_ON_M500
#endif
#define ARC_SUPPORT               
#if ENABLED(ARC_SUPPORT)
  #define MM_PER_ARC_SEGMENT  1   
  #define N_ARC_CORRECTION   25   
#endif
#if ENABLED(G38_PROBE_TARGET)
  #define G38_MINIMUM_MOVE 0.0275 
#endif
#define MIN_STEPS_PER_SEGMENT 6
#define MINIMUM_STEPPER_PULSE 0 
#if ENABLED(SDSUPPORT)
  #define BLOCK_BUFFER_SIZE 16 
#else
  #define BLOCK_BUFFER_SIZE 16 
#endif
#define MAX_CMD_SIZE 64
#define BUFSIZE 8 
#define TX_BUFFER_SIZE 0
#if ENABLED(FWRETRACT)
  #define MIN_RETRACT 0.1                
  #define RETRACT_LENGTH 3               
  #define RETRACT_LENGTH_SWAP 13         
  #define RETRACT_FEEDRATE 45            
  #define RETRACT_ZLIFT 0                
  #define RETRACT_RECOVER_LENGTH 0       
  #define RETRACT_RECOVER_LENGTH_SWAP 0  
  #define RETRACT_RECOVER_FEEDRATE 8     
#endif
#if CLIENT_VAR!=WANHAO_ONEPLUS_201707VAR
#define ADVANCED_PAUSE_FEATURE 
#endif
#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #define PAUSE_PARK_X_POS 3                  
  #define PAUSE_PARK_Y_POS 3                  
    #define PAUSE_PARK_Z_ADD 10                 
  #define PAUSE_PARK_XY_FEEDRATE 100          
  #define PAUSE_PARK_Z_FEEDRATE 5             
  #define PAUSE_PARK_RETRACT_FEEDRATE 60      
  #define PAUSE_PARK_RETRACT_LENGTH 2         
  #define FILAMENT_CHANGE_UNLOAD_FEEDRATE 10  
    #define FILAMENT_CHANGE_UNLOAD_LENGTH 100   
  #define FILAMENT_CHANGE_LOAD_FEEDRATE 6     
  #define FILAMENT_CHANGE_LOAD_LENGTH 0       
  #define ADVANCED_PAUSE_EXTRUDE_FEEDRATE 3   
  #define ADVANCED_PAUSE_EXTRUDE_LENGTH 50    
    #define PAUSE_PARK_NOZZLE_TIMEOUT 45        
  #define FILAMENT_CHANGE_NUMBER_OF_ALERT_BEEPS 5 
  #define PAUSE_PARK_NO_STEPPER_TIMEOUT       
#endif
#if ENABLED(HAVE_TMCDRIVER)
  #define X_MAX_CURRENT     1000 
  #define X_SENSE_RESISTOR    91 
  #define X_MICROSTEPS        16 
  #define X2_MAX_CURRENT    1000
  #define X2_SENSE_RESISTOR   91
  #define X2_MICROSTEPS       16
  #define Y_MAX_CURRENT     1000
  #define Y_SENSE_RESISTOR    91
  #define Y_MICROSTEPS        16
  #define Y2_MAX_CURRENT    1000
  #define Y2_SENSE_RESISTOR   91
  #define Y2_MICROSTEPS       16
  #define Z_MAX_CURRENT     1000
  #define Z_SENSE_RESISTOR    91
  #define Z_MICROSTEPS        16
  #define Z2_MAX_CURRENT    1000
  #define Z2_SENSE_RESISTOR   91
  #define Z2_MICROSTEPS       16
  #define E0_MAX_CURRENT    1000
  #define E0_SENSE_RESISTOR   91
  #define E0_MICROSTEPS       16
  #define E1_MAX_CURRENT    1000
  #define E1_SENSE_RESISTOR   91
  #define E1_MICROSTEPS       16
  #define E2_MAX_CURRENT    1000
  #define E2_SENSE_RESISTOR   91
  #define E2_MICROSTEPS       16
  #define E3_MAX_CURRENT    1000
  #define E3_SENSE_RESISTOR   91
  #define E3_MICROSTEPS       16
  #define E4_MAX_CURRENT    1000
  #define E4_SENSE_RESISTOR   91
  #define E4_MICROSTEPS       16
#endif
#if ENABLED(HAVE_TMC2130)
  #define R_SENSE           0.11  
  #define HOLD_MULTIPLIER    0.5  
  #define INTERPOLATE          1  
  #define X_CURRENT         1000  
  #define X_MICROSTEPS        16  
  #define Y_CURRENT         1000
  #define Y_MICROSTEPS        16
  #define Z_CURRENT         1000
  #define Z_MICROSTEPS        16
  #define STEALTHCHOP
  #if ENABLED(AUTOMATIC_CURRENT_CONTROL)
    #define CURRENT_STEP          50  
    #define AUTO_ADJUST_MAX     1300  
    #define REPORT_CURRENT_CHANGE
  #endif
  #define X_HYBRID_THRESHOLD     100  
  #define X2_HYBRID_THRESHOLD    100
  #define Y_HYBRID_THRESHOLD     100
  #define Y2_HYBRID_THRESHOLD    100
  #define Z_HYBRID_THRESHOLD       4
  #define Z2_HYBRID_THRESHOLD      4
  #define E0_HYBRID_THRESHOLD     30
  #define E1_HYBRID_THRESHOLD     30
  #define E2_HYBRID_THRESHOLD     30
  #define E3_HYBRID_THRESHOLD     30
  #define E4_HYBRID_THRESHOLD     30
  #if ENABLED(SENSORLESS_HOMING)
    #define X_HOMING_SENSITIVITY  19
    #define Y_HOMING_SENSITIVITY  19
  #endif
  #define  TMC2130_ADV() {  }
#endif 
#if ENABLED(HAVE_L6470DRIVER)
  #define X_MICROSTEPS      16 
  #define X_K_VAL           50 
  #define X_OVERCURRENT   2000 
  #define X_STALLCURRENT  1500 
  #define X2_MICROSTEPS     16
  #define X2_K_VAL          50
  #define X2_OVERCURRENT  2000
  #define X2_STALLCURRENT 1500
  #define Y_MICROSTEPS      16
  #define Y_K_VAL           50
  #define Y_OVERCURRENT   2000
  #define Y_STALLCURRENT  1500
  #define Y2_MICROSTEPS     16
  #define Y2_K_VAL          50
  #define Y2_OVERCURRENT  2000
  #define Y2_STALLCURRENT 1500
  #define Z_MICROSTEPS      16
  #define Z_K_VAL           50
  #define Z_OVERCURRENT   2000
  #define Z_STALLCURRENT  1500
  #define Z2_MICROSTEPS     16
  #define Z2_K_VAL          50
  #define Z2_OVERCURRENT  2000
  #define Z2_STALLCURRENT 1500
  #define E0_MICROSTEPS     16
  #define E0_K_VAL          50
  #define E0_OVERCURRENT  2000
  #define E0_STALLCURRENT 1500
  #define E1_MICROSTEPS     16
  #define E1_K_VAL          50
  #define E1_OVERCURRENT  2000
  #define E1_STALLCURRENT 1500
  #define E2_MICROSTEPS     16
  #define E2_K_VAL          50
  #define E2_OVERCURRENT  2000
  #define E2_STALLCURRENT 1500
  #define E3_MICROSTEPS     16
  #define E3_K_VAL          50
  #define E3_OVERCURRENT  2000
  #define E3_STALLCURRENT 1500
  #define E4_MICROSTEPS     16
  #define E4_K_VAL          50
  #define E4_OVERCURRENT  2000
  #define E4_STALLCURRENT 1500
#endif
#define I2C_SLAVE_ADDRESS  0 
#if ENABLED(SPINDLE_LASER_ENABLE)
  #define SPINDLE_LASER_ENABLE_INVERT   false  
  #define SPINDLE_LASER_PWM             true   
  #define SPINDLE_LASER_PWM_INVERT      true   
  #define SPINDLE_LASER_POWERUP_DELAY   5000   
  #define SPINDLE_LASER_POWERDOWN_DELAY 5000   
  #define SPINDLE_DIR_CHANGE            true   
  #define SPINDLE_INVERT_DIR            false
  #define SPINDLE_STOP_ON_DIR_CHANGE    true   
  #define SPEED_POWER_SLOPE    118.4
  #define SPEED_POWER_INTERCEPT  0
  #define SPEED_POWER_MIN     5000
  #define SPEED_POWER_MAX    30000    
#endif
#define AUTO_REPORT_TEMPERATURES
#define EXTENDED_CAPABILITIES_REPORT
#define PROPORTIONAL_FONT_RATIO 1.0
#define FASTER_GCODE_PARSER
#if ENABLED(CUSTOM_USER_MENUS)
  #define USER_SCRIPT_DONE "M117 User Script Done"
  #define USER_DESC_1 "Home & UBL Info"
  #define USER_GCODE_1 "G28\nG29 W"
  #define USER_DESC_2 "Preheat for PLA"
  #define USER_GCODE_2 "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND)
  #define USER_DESC_3 "Preheat for ABS"
  #define USER_GCODE_3 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND)
  #define USER_DESC_4 "Heat Bed/Home/Level"
  #define USER_GCODE_4 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nG28\nG29"
  #define USER_DESC_5 "Home & Info"
  #define USER_GCODE_5 "G28\nM503"
#endif
#if ENABLED(I2C_POSITION_ENCODERS)
  #define I2CPE_ENCODER_CNT         1                       
  #define I2CPE_ENC_1_ADDR          I2CPE_PRESET_ADDR_X     
  #define I2CPE_ENC_1_AXIS          X_AXIS                  
  #define I2CPE_ENC_1_TYPE          I2CPE_ENC_TYPE_LINEAR   
  #define I2CPE_ENC_1_TICKS_UNIT    2048                    
  #define I2CPE_ENC_1_EC_METHOD     I2CPE_ECM_NONE          
  #define I2CPE_ENC_1_EC_THRESH     0.10                    
  #define I2CPE_ENC_2_ADDR          I2CPE_PRESET_ADDR_Y     
  #define I2CPE_ENC_2_AXIS          Y_AXIS
  #define I2CPE_ENC_2_TYPE          I2CPE_ENC_TYPE_LINEAR
  #define I2CPE_ENC_2_TICKS_UNIT    2048
  #define I2CPE_ENC_2_EC_METHOD     I2CPE_ECM_NONE
  #define I2CPE_ENC_2_EC_THRESH     0.10
  #define I2CPE_ENC_3_ADDR          I2CPE_PRESET_ADDR_Z     
  #define I2CPE_ENC_3_AXIS          Z_AXIS                  
  #define I2CPE_ENC_4_ADDR          I2CPE_PRESET_ADDR_E     
  #define I2CPE_ENC_4_AXIS          E_AXIS
  #define I2CPE_ENC_5_ADDR          34                      
  #define I2CPE_ENC_5_AXIS          E_AXIS
  #define I2CPE_DEF_TYPE            I2CPE_ENC_TYPE_LINEAR
  #define I2CPE_DEF_ENC_TICKS_UNIT  2048
  #define I2CPE_DEF_TICKS_REV       (16 * 200)
  #define I2CPE_DEF_EC_METHOD       I2CPE_ECM_NONE
  #define I2CPE_DEF_EC_THRESH       0.1
  #define I2CPE_TIME_TRUSTED        10000                   
  #define I2CPE_MIN_UPD_TIME_MS     100                     
  #define I2CPE_ERR_ROLLING_AVERAGE
#endif 
#endif 
