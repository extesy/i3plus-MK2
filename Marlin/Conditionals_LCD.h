#ifndef CONDITIONALS_LCD_H 
#define CONDITIONALS_LCD_H
  #define LCD_HAS_DIRECTIONAL_BUTTONS (BUTTON_EXISTS(UP) || BUTTON_EXISTS(DWN) || BUTTON_EXISTS(LFT) || BUTTON_EXISTS(RT))
  #if ENABLED(CARTESIO_UI)
    #define DOGLCD
    #define ULTIPANEL
    #define NEWPANEL
    #define DEFAULT_LCD_CONTRAST 90
    #define LCD_CONTRAST_MIN 60
    #define LCD_CONTRAST_MAX 140
  #elif ENABLED(MAKRPANEL) || ENABLED(MINIPANEL)
    #define DOGLCD
    #define ULTIPANEL
    #define NEWPANEL
    #define DEFAULT_LCD_CONTRAST 255
  #elif ENABLED(ANET_KEYPAD_LCD)
    #define REPRAPWORLD_KEYPAD
    #define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0
    #define ADC_KEYPAD
    #define ADC_KEY_NUM 8
    #define ULTIPANEL
    #define ENCODER_STEPS_PER_MENU_ITEM 1
    #define REVERSE_MENU_DIRECTION
  #elif ENABLED(ANET_FULL_GRAPHICS_LCD)
    #define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
  #elif ENABLED(BQ_LCD_SMART_CONTROLLER)
    #define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
    #define LONG_FILENAME_HOST_SUPPORT
  #elif ENABLED(miniVIKI) || ENABLED(VIKI2) || ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
    #define ULTRA_LCD  
    #define DOGLCD  
    #define ULTIMAKERCONTROLLER 
    #if ENABLED(miniVIKI)
      #define LCD_CONTRAST_MIN  75
      #define LCD_CONTRAST_MAX 115
      #define DEFAULT_LCD_CONTRAST 95
    #elif ENABLED(VIKI2)
      #define DEFAULT_LCD_CONTRAST 40
    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
      #define LCD_CONTRAST_MIN  90
      #define LCD_CONTRAST_MAX 130
      #define DEFAULT_LCD_CONTRAST 110
      #define U8GLIB_LM6059_AF
      #define SD_DETECT_INVERTED
    #endif
  #elif ENABLED(OLED_PANEL_TINYBOY2)
    #define U8GLIB_SSD1306
    #define ULTIPANEL
    #define NEWPANEL
    #define REVERSE_ENCODER_DIRECTION
    #define REVERSE_MENU_DIRECTION
  #elif ENABLED(RA_CONTROL_PANEL)
    #define LCD_I2C_TYPE_PCA8574
    #define LCD_I2C_ADDRESS 0x27   
    #define ULTIPANEL
    #define NEWPANEL
  #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
    #define DOGLCD
    #define U8GLIB_ST7920
    #define ULTIPANEL
    #define NEWPANEL
  #endif
  #if ENABLED(U8GLIB_SSD1306) || ENABLED(U8GLIB_SH1106)
    #define ULTRA_LCD  
    #define DOGLCD  
  #endif
  #if ENABLED(PANEL_ONE) || ENABLED(U8GLIB_SH1106)
    #define ULTIMAKERCONTROLLER
  #endif
  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER) || ENABLED(LCD_FOR_MELZI)
    #define DOGLCD
    #define U8GLIB_ST7920
    #define REPRAP_DISCOUNT_SMART_CONTROLLER
  #endif
  #if ENABLED(ULTIMAKERCONTROLLER)                 || ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)    || ENABLED(G3D_PANEL)                           || ENABLED(RIGIDBOT_PANEL)
    #define ULTIPANEL
    #define NEWPANEL
  #endif
  #if ENABLED(REPRAPWORLD_KEYPAD)
    #define NEWPANEL
    #if ENABLED(ULTIPANEL) && !defined(REPRAPWORLD_KEYPAD_MOVE_STEP)
      #define REPRAPWORLD_KEYPAD_MOVE_STEP 1.0
    #endif
  #endif
  #if ENABLED(LCD_I2C_SAINSMART_YWROBOT)
    #define LCD_I2C_TYPE_PCF8575
    #define LCD_I2C_ADDRESS 0x27   
    #define ULTIPANEL
    #define NEWPANEL
  #elif ENABLED(LCD_I2C_PANELOLU2)
    #define LCD_I2C_TYPE_MCP23017
    #define LCD_I2C_ADDRESS 0x20 
    #define LCD_USE_I2C_BUZZER 
    #define ULTIPANEL
    #define NEWPANEL
  #elif ENABLED(LCD_I2C_VIKI)
    #define LCD_I2C_TYPE_MCP23017
    #define LCD_I2C_ADDRESS 0x20 
    #define LCD_USE_I2C_BUZZER 
    #define ULTIPANEL
    #define NEWPANEL
    #define ENCODER_FEEDRATE_DEADZONE 4
    #ifndef ENCODER_PULSES_PER_STEP
      #define ENCODER_PULSES_PER_STEP 1
    #endif
    #ifndef ENCODER_STEPS_PER_MENU_ITEM
      #define ENCODER_STEPS_PER_MENU_ITEM 2
    #endif
  #endif
  #if ENABLED(miniVIKI) || ENABLED(VIKI2) || ENABLED(ELB_FULL_GRAPHIC_CONTROLLER) || ENABLED(OLED_PANEL_TINYBOY2)    || ENABLED(BQ_LCD_SMART_CONTROLLER) || ENABLED(LCD_I2C_PANELOLU2) || ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
    #ifndef ENCODER_PULSES_PER_STEP
      #define ENCODER_PULSES_PER_STEP 4
    #endif
    #ifndef ENCODER_STEPS_PER_MENU_ITEM
      #define ENCODER_STEPS_PER_MENU_ITEM 1
    #endif
  #endif
  #if ENABLED(SAV_3DLCD)
    #define SR_LCD_2W_NL    
    #define ULTIPANEL
    #define NEWPANEL
  #endif
  #if ENABLED(DOGLCD) 
    #ifndef LCD_WIDTH
      #define LCD_WIDTH 22
    #endif
    #ifndef LCD_HEIGHT
      #define LCD_HEIGHT 5
    #endif
  #endif
  #if ENABLED(ULTIPANEL)
    #define NEWPANEL  
    #ifdef FYS_ULTRA_LCD_WANHAO_ONEPLUS
    #else
    #define ULTRA_LCD
    #endif
    #ifndef LCD_WIDTH
      #define LCD_WIDTH 20
    #endif
    #ifndef LCD_HEIGHT
      #define LCD_HEIGHT 4
    #endif
  #else 
    #if ENABLED(ULTRA_LCD)
      #ifndef LCD_WIDTH
        #define LCD_WIDTH 16
      #endif
      #ifndef LCD_HEIGHT
        #define LCD_HEIGHT 2
      #endif
    #endif
  #endif
  #if ENABLED(DOGLCD)
    #define LCD_STR_REFRESH     "\x01"
    #define LCD_STR_FOLDER      "\x02"
    #define LCD_STR_ARROW_RIGHT "\x03"
    #define LCD_STR_UPLEVEL     "\x04"
    #define LCD_STR_CLOCK       "\x05"
    #define LCD_STR_FEEDRATE    "\x06"
    #define LCD_STR_BEDTEMP     "\x07"
    #define LCD_STR_THERMOMETER "\x08"
    #define LCD_STR_DEGREE      "\x09"
    #define LCD_STR_SPECIAL_MAX '\x09'
    #define LCD_STR_FILAM_DIA   "\xf8"
    #define LCD_STR_FILAM_MUL   "\xa4"
  #else
    #define LCD_BEDTEMP_CHAR     0x00  
    #define LCD_DEGREE_CHAR      0x01
    #define LCD_STR_THERMOMETER "\x02" 
    #define LCD_UPLEVEL_CHAR     0x03
    #define LCD_STR_REFRESH     "\x04"
    #define LCD_STR_FOLDER      "\x05"
    #define LCD_FEEDRATE_CHAR    0x06
    #define LCD_CLOCK_CHAR       0x07
    #define LCD_STR_ARROW_RIGHT ">"  
    #if ENABLED(AUTO_BED_LEVELING_UBL)
      #define LCD_UBL_BOXTOP_CHAR 0x01
      #define LCD_UBL_BOXBOT_CHAR 0x02
    #endif
  #endif
  #if ENABLED(DOGLCD)
    #define HAS_LCD_CONTRAST ( \
        ENABLED(MAKRPANEL) \
     || ENABLED(CARTESIO_UI) \
     || ENABLED(VIKI2) \
     || ENABLED(miniVIKI) \
     || ENABLED(ELB_FULL_GRAPHIC_CONTROLLER) \
     || ENABLED(FYS_ULTRA_LCD_WANHAO_ONEPLUS)\
     || ENABLED(ULTRA_LCD)\
    )
    #if HAS_LCD_CONTRAST
      #ifndef LCD_CONTRAST_MIN
        #define LCD_CONTRAST_MIN 0
      #endif
      #ifndef LCD_CONTRAST_MAX
        #define LCD_CONTRAST_MAX 255
      #endif
      #ifndef DEFAULT_LCD_CONTRAST
        #define DEFAULT_LCD_CONTRAST 32
      #endif
    #endif
  #endif
  #ifndef BOOTSCREEN_TIMEOUT
    #define BOOTSCREEN_TIMEOUT 2500
  #endif
  #define HAS_DEBUG_MENU ENABLED(LCD_PROGRESS_BAR_TEST)
  #if ENABLED(MK2_MULTIPLEXER)
    #define SINGLENOZZLE
  #endif
  #if ENABLED(SINGLENOZZLE) || ENABLED(MIXING_EXTRUDER)         
    #define HOTENDS       1
    #undef TEMP_SENSOR_1_AS_REDUNDANT
    #undef HOTEND_OFFSET_X
    #undef HOTEND_OFFSET_Y
  #else                                                         
    #define HOTENDS       EXTRUDERS
    #if ENABLED(SWITCHING_NOZZLE) && !defined(HOTEND_OFFSET_Z)
      #define HOTEND_OFFSET_Z { 0 }
    #endif
  #endif
  #if ENABLED(SWITCHING_EXTRUDER) || ENABLED(MIXING_EXTRUDER)   
    #if ENABLED(MIXING_EXTRUDER)
      #define E_STEPPERS  MIXING_STEPPERS
    #else
      #define E_STEPPERS  1                                     
    #endif
    #define E_MANUAL      1
    #define TOOL_E_INDEX  0
  #else
    #define E_STEPPERS    EXTRUDERS
    #define E_MANUAL      EXTRUDERS
    #define TOOL_E_INDEX  current_block->active_extruder
  #endif
  #if ENABLED(DISTINCT_E_FACTORS) && E_STEPPERS > 1
    #define XYZE_N (XYZ + E_STEPPERS)
    #define E_AXIS_N (E_AXIS + extruder)
  #else
    #undef DISTINCT_E_FACTORS
    #define XYZE_N XYZE
    #define E_AXIS_N E_AXIS
  #endif
  #if ENABLED(BLTOUCH)
    #ifndef Z_ENDSTOP_SERVO_NR
      #define Z_ENDSTOP_SERVO_NR 0
    #endif
    #ifndef NUM_SERVOS
      #define NUM_SERVOS (Z_ENDSTOP_SERVO_NR + 1)
    #endif
    #undef DEACTIVATE_SERVOS_AFTER_MOVE
    #undef SERVO_DELAY
    #define SERVO_DELAY 50
    #ifndef BLTOUCH_DELAY
      #define BLTOUCH_DELAY 375
    #endif
    #undef Z_SERVO_ANGLES
    #define Z_SERVO_ANGLES { BLTOUCH_DEPLOY, BLTOUCH_STOW }
    #define BLTOUCH_DEPLOY    10
    #define BLTOUCH_STOW      90
    #define BLTOUCH_SELFTEST 120
    #define BLTOUCH_RESET    160
    #define _TEST_BLTOUCH(P) (READ(P##_PIN) != P##_ENDSTOP_INVERTING)
    #undef Z_MIN_PROBE_ENDSTOP_INVERTING
    #define Z_MIN_PROBE_ENDSTOP_INVERTING false
    #if ENABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN)
      #undef Z_MIN_ENDSTOP_INVERTING
      #define Z_MIN_ENDSTOP_INVERTING false
      #define TEST_BLTOUCH() _TEST_BLTOUCH(Z_MIN)
    #else
      #define TEST_BLTOUCH() _TEST_BLTOUCH(Z_MIN_PROBE)
    #endif
  #endif
  #define HAS_Z_SERVO_ENDSTOP (defined(Z_ENDSTOP_SERVO_NR) && Z_ENDSTOP_SERVO_NR >= 0)
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    #undef PROBE_MANUALLY
  #endif
  #define PROBE_SELECTED (ENABLED(PROBE_MANUALLY) || ENABLED(FIX_MOUNTED_PROBE) || ENABLED(Z_PROBE_ALLEN_KEY) || HAS_Z_SERVO_ENDSTOP || ENABLED(Z_PROBE_SLED) || ENABLED(SOLENOID_PROBE))
  #if !PROBE_SELECTED || ENABLED(PROBE_MANUALLY)
    #undef Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
    #undef Z_MIN_PROBE_ENDSTOP
  #endif
  #define HAS_SOFTWARE_ENDSTOPS (ENABLED(MIN_SOFTWARE_ENDSTOPS) || ENABLED(MAX_SOFTWARE_ENDSTOPS))
#define HAS_RESUME_CONTINUE (ENABLED(NEWPANEL) || ENABLED(EMERGENCY_PARSER)||ENABLED(ENABLE_ULTILCD2)||ENABLED(FYSTLCD_V1)) 
  #define HAS_COLOR_LEDS (ENABLED(BLINKM) || ENABLED(RGB_LED) || ENABLED(RGBW_LED) || ENABLED(PCA9632))
#endif 
