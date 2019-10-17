#ifndef ULTRALCD_H
#define ULTRALCD_H
#include "Marlin.h"
#ifdef FYS_SAFE_PRINT_BREAK
void gcode_M1101();
void gcode_M1103();
#endif
#if ENABLED(ULTRA_LCD)
  #define BUTTON_EXISTS(BN) (defined(BTN_## BN) && BTN_## BN >= 0)
  #define BUTTON_PRESSED(BN) !READ(BTN_## BN)
  extern int16_t lcd_preheat_hotend_temp[2], lcd_preheat_bed_temp[2], lcd_preheat_fan_speed[2];
  int16_t lcd_strlen(const char* s);
  int16_t lcd_strlen_P(const char* s);
  void lcd_update();
  void lcd_init();
  bool lcd_hasstatus();
  void lcd_setstatus(const char* message, const bool persist=false);
  void lcd_setstatusPGM(const char* message, const int8_t level=0);
  void lcd_setalertstatusPGM(const char* message);
  void lcd_status_printf_P(const uint8_t level, const char * const fmt, ...);
  void lcd_reset_alert_level();
  void lcd_kill_screen();
  void kill_screen(const char* lcd_msg);
  bool lcd_detected(void);
  extern const float homing_feedrate_mm_s[];
  extern uint8_t lcdDrawUpdate;
  inline void lcd_refresh() { lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; }
  #define FunV006(x)
  #define FunV052()
  #define lcd_shutDown()
  #define is_command_queued() (commands_in_queue > 0)
  #if HAS_BUZZER
    void lcd_buzz(long duration, uint16_t freq);
  #endif
  #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
    void dontExpireStatus();
  #endif
  #if ENABLED(ADC_KEYPAD)
    uint8_t get_ADC_keyValue();
  #endif
  #if ENABLED(DOGLCD)
    extern uint16_t lcd_contrast;
    void set_lcd_contrast(const uint16_t value);
  #endif
  #if ENABLED(SHOW_BOOTSCREEN)
    void lcd_bootscreen();
  #endif
  #define LCD_UPDATE_INTERVAL 100
  #if ENABLED(ULTIPANEL)
    #define BLEN_A 0
    #define BLEN_B 1
    #if BUTTON_EXISTS(ENC)
      #define BLEN_C 2
    #endif
    #define EN_A (_BV(BLEN_A))
    #define EN_B (_BV(BLEN_B))
    #define EN_C (_BV(BLEN_C))
    extern volatile uint8_t buttons;  
    void lcd_buttons_update();
    void lcd_quick_feedback();        
    void lcd_completion_feedback(const bool good=true);
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      void lcd_advanced_pause_show_message(const AdvancedPauseMessage message);
    #endif 
  #else
    inline void lcd_buttons_update() {}
  #endif
  #if ENABLED(FILAMENT_LCD_DISPLAY) && ENABLED(SDSUPPORT)
    extern millis_t previous_lcd_status_ms;
  #endif
  bool lcd_blink();
  #if ENABLED(REPRAPWORLD_KEYPAD) 
    #define REPRAPWORLD_BTN_OFFSET 0 
    #define BLEN_REPRAPWORLD_KEYPAD_F3     0
    #define BLEN_REPRAPWORLD_KEYPAD_F2     1
    #define BLEN_REPRAPWORLD_KEYPAD_F1     2
    #define BLEN_REPRAPWORLD_KEYPAD_DOWN   3
    #define BLEN_REPRAPWORLD_KEYPAD_RIGHT  4
    #define BLEN_REPRAPWORLD_KEYPAD_MIDDLE 5
    #define BLEN_REPRAPWORLD_KEYPAD_UP     6
    #define BLEN_REPRAPWORLD_KEYPAD_LEFT   7
    #define EN_REPRAPWORLD_KEYPAD_F3      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_F3))
    #define EN_REPRAPWORLD_KEYPAD_F2      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_F2))
    #define EN_REPRAPWORLD_KEYPAD_F1      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_F1))
    #define EN_REPRAPWORLD_KEYPAD_DOWN    (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_DOWN))
    #define EN_REPRAPWORLD_KEYPAD_RIGHT   (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_RIGHT))
    #define EN_REPRAPWORLD_KEYPAD_MIDDLE  (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_MIDDLE))
    #define EN_REPRAPWORLD_KEYPAD_UP      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_UP))
    #define EN_REPRAPWORLD_KEYPAD_LEFT    (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_LEFT))
    #define REPRAPWORLD_KEYPAD_MOVE_Z_DOWN  (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_F3)
    #define REPRAPWORLD_KEYPAD_MOVE_Z_UP    (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_F2)
    #define REPRAPWORLD_KEYPAD_MOVE_MENU    (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_F1)
    #define REPRAPWORLD_KEYPAD_MOVE_Y_DOWN  (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_DOWN)
    #define REPRAPWORLD_KEYPAD_MOVE_X_RIGHT (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_RIGHT)
    #define REPRAPWORLD_KEYPAD_MOVE_HOME    (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_MIDDLE)
    #define REPRAPWORLD_KEYPAD_MOVE_Y_UP    (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_UP)
    #define REPRAPWORLD_KEYPAD_MOVE_X_LEFT  (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_LEFT)
    #if ENABLED(ADC_KEYPAD)
      #define REPRAPWORLD_KEYPAD_MOVE_HOME  (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_F1)
      #define KEYPAD_EN_C                   EN_REPRAPWORLD_KEYPAD_MIDDLE
    #else
      #define REPRAPWORLD_KEYPAD_MOVE_HOME  (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_MIDDLE)
      #define KEYPAD_EN_C                   EN_REPRAPWORLD_KEYPAD_F1
    #endif
    #define REPRAPWORLD_KEYPAD_MOVE_MENU    (buttons_reprapworld_keypad & KEYPAD_EN_C)
    #if BUTTON_EXISTS(ENC)
      #define LCD_CLICKED ((buttons & EN_C) || REPRAPWORLD_KEYPAD_MOVE_MENU)
    #else
      #define LCD_CLICKED REPRAPWORLD_KEYPAD_MOVE_MENU
    #endif
    #define REPRAPWORLD_KEYPAD_PRESSED      (buttons_reprapworld_keypad & ( \
                                              EN_REPRAPWORLD_KEYPAD_F3 | \
                                              EN_REPRAPWORLD_KEYPAD_F2 | \
                                              EN_REPRAPWORLD_KEYPAD_F1 | \
                                              EN_REPRAPWORLD_KEYPAD_DOWN | \
                                              EN_REPRAPWORLD_KEYPAD_RIGHT | \
                                              EN_REPRAPWORLD_KEYPAD_MIDDLE | \
                                              EN_REPRAPWORLD_KEYPAD_UP | \
                                              EN_REPRAPWORLD_KEYPAD_LEFT) \
                                            )
  #elif ENABLED(NEWPANEL)
    #define LCD_CLICKED (buttons & EN_C)
  #else
    #define LCD_CLICKED false
  #endif
#elif defined(MINIPANEL)    
  #define BUTTON_EXISTS(BN) (defined(BTN_## BN) && BTN_## BN >= 0)
  #define BUTTON_PRESSED(BN) !READ(BTN_## BN)
  extern int lcd_preheat_hotend_temp[2], lcd_preheat_bed_temp[2], lcd_preheat_fan_speed[2];
  int lcd_strlen(const char* s);
  int lcd_strlen_P(const char* s);
  void lcd_update();
  void lcd_init();
  bool lcd_hasstatus();
  void lcd_setstatus(const char* message, const bool persist = false);
  void lcd_setstatusPGM(const char* message, const uint8_t level = 0);
  void lcd_setalertstatusPGM(const char* message);
  void lcd_reset_alert_level();
  void lcd_kill_screen();
  void kill_screen(const char* lcd_msg);
  extern uint8_t lcdDrawUpdate;
  inline void lcd_refresh() { lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; }
  #define   FunV006(x)    lcd_setstatusPGM(PSTR(x))
  #define lcd_shutDown()
  #define FunV052()
  #if (ENABLED(LCD_I2C_TYPE_MCP23017) || ENABLED(LCD_I2C_TYPE_MCP23008)) && ENABLED(DETECT_DEVICE)
  bool lcd_detected();
  #else
  inline bool lcd_detected() { return true; }
  #endif
  #if HAS_BUZZER
  void lcd_buzz(long duration, uint16_t freq);
  #endif
  #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
  void dontExpireStatus();
  #endif
  #if ENABLED(DOGLCD)
  extern uint16_t lcd_contrast;
  void set_lcd_contrast(int value);
  #elif ENABLED(SHOW_BOOTSCREEN)
  void bootscreen();
  #endif
  #define LCD_UPDATE_INTERVAL 100
  #define LCD_TIMEOUT_TO_STATUS 60000
  #if ENABLED(ULTIPANEL)
    #define BLEN_A 0
    #define BLEN_B 1
    #if BUTTON_EXISTS(ENC)
    #define BLEN_C 2
    #endif
    #define EN_A (_BV(BLEN_A))
    #define EN_B (_BV(BLEN_B))
    #define EN_C (_BV(BLEN_C))
    extern volatile uint8_t buttons;  
    void lcd_buttons_update();
    void lcd_quick_feedback(); 
    #if ENABLED(FILAMENT_CHANGE_FEATURE)
    void lcd_filament_change_show_message(const FilamentChangeMessage message);
    #endif 
  #else
   inline void lcd_buttons_update() {}
  #endif
  #if ENABLED(FILAMENT_LCD_DISPLAY)
  extern millis_t previous_lcd_status_ms;
  #endif
  bool lcd_blink();
  #if ENABLED(REPRAPWORLD_KEYPAD) 
  #define REPRAPWORLD_BTN_OFFSET 0 
  #define BLEN_REPRAPWORLD_KEYPAD_F3     0
  #define BLEN_REPRAPWORLD_KEYPAD_F2     1
  #define BLEN_REPRAPWORLD_KEYPAD_F1     2
  #define BLEN_REPRAPWORLD_KEYPAD_DOWN   3
  #define BLEN_REPRAPWORLD_KEYPAD_RIGHT  4
  #define BLEN_REPRAPWORLD_KEYPAD_MIDDLE 5
  #define BLEN_REPRAPWORLD_KEYPAD_UP     6
  #define BLEN_REPRAPWORLD_KEYPAD_LEFT   7
  #define EN_REPRAPWORLD_KEYPAD_F3      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_F3))
  #define EN_REPRAPWORLD_KEYPAD_F2      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_F2))
  #define EN_REPRAPWORLD_KEYPAD_F1      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_F1))
  #define EN_REPRAPWORLD_KEYPAD_DOWN    (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_DOWN))
  #define EN_REPRAPWORLD_KEYPAD_RIGHT   (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_RIGHT))
  #define EN_REPRAPWORLD_KEYPAD_MIDDLE  (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_MIDDLE))
  #define EN_REPRAPWORLD_KEYPAD_UP      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_UP))
  #define EN_REPRAPWORLD_KEYPAD_LEFT    (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_LEFT))
  #define REPRAPWORLD_KEYPAD_MOVE_Z_DOWN  (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_F3)
  #define REPRAPWORLD_KEYPAD_MOVE_Z_UP    (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_F2)
  #define REPRAPWORLD_KEYPAD_MOVE_MENU    (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_F1)
  #define REPRAPWORLD_KEYPAD_MOVE_Y_DOWN  (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_DOWN)
  #define REPRAPWORLD_KEYPAD_MOVE_X_RIGHT (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_RIGHT)
  #define REPRAPWORLD_KEYPAD_MOVE_HOME    (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_MIDDLE)
  #define REPRAPWORLD_KEYPAD_MOVE_Y_UP    (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_UP)
  #define REPRAPWORLD_KEYPAD_MOVE_X_LEFT  (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_LEFT)
  #define REPRAPWORLD_KEYPAD_PRESSED      (buttons_reprapworld_keypad & ( \
    EN_REPRAPWORLD_KEYPAD_F3 | \
    EN_REPRAPWORLD_KEYPAD_F2 | \
    EN_REPRAPWORLD_KEYPAD_F1 | \
    EN_REPRAPWORLD_KEYPAD_DOWN | \
    EN_REPRAPWORLD_KEYPAD_RIGHT | \
    EN_REPRAPWORLD_KEYPAD_MIDDLE | \
    EN_REPRAPWORLD_KEYPAD_UP | \
    EN_REPRAPWORLD_KEYPAD_LEFT) \
    )
  #define LCD_CLICKED ((buttons & EN_C) || (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_F1))
  #elif ENABLED(NEWPANEL)
  #define LCD_CLICKED (buttons & EN_C)
  #else
  #define LCD_CLICKED false
  #endif
  #define is_command_queued() (commands_in_queue > 0)
#elif defined(FYSTLCD_V1) 
  #include "FysTLcd.h"
  #if HAS_SERVOS
  #include "servo.h"
  extern Servo servo[NUM_SERVOS];
  #endif
  inline bool _enqueuecommand(const char* cmd, bool say_ok = false);
  inline void line_to_current_position();
  extern const float homing_feedrate_mm_s[];
  void lcd_update();
  void lcd_init();
  extern int16_t lcd_preheat_hotend_temp[2], lcd_preheat_bed_temp[2], lcd_preheat_fan_speed[2];
  #define lcd_hasstatus() false
  #define lcd_setstatusPGM(x)
  #define lcd_setalertstatusPGM(x)
  #define   lcd_status_printf_P(x,y,...)
  #define   lcd_buttons_update()
  #define   lcd_reset_alert_level()
  #define lcd_detected()    true
  #define   lcd_refresh()
  void dwin_popup(const char*msg, char pageChoose = 0, char funid = 0);
  #define   FunV006(x,...)    dwin_popup(PSTR(x))
  void FunV052();
  void FunV029();
  void lcd_shutDown();
  void lcd_setstatus(const char* message, const bool persist = false);
  #define   lcd_setPage(x)  if(GLOBAL_var_V00C!=x){FysTLcd::FunV024(x); GLOBAL_var_V00D =x;GLOBAL_var_V00C=x;}
  #define   lcd_setPageForce(x) do{FysTLcd::FunV024(x); GLOBAL_var_V00D =x;GLOBAL_var_V00C=x;} while(0)
  #define   lcd_popPage(x)  if(GLOBAL_var_V00C!=x){FysTLcd::FunV024(x); GLOBAL_var_V00C=x;}
  void lcd_zstep_zoffset(bool dir); 
#else 
  inline void lcd_update() {}
  inline void lcd_init() {}
  inline bool lcd_hasstatus() { return false; }
  inline void lcd_setstatus(const char* const message, const bool persist=false) { UNUSED(message); UNUSED(persist); }
  inline void lcd_setstatusPGM(const char* const message, const int8_t level=0) { UNUSED(message); UNUSED(level); }
  inline void lcd_setalertstatusPGM(const char* message) { UNUSED(message); }
  inline void lcd_status_printf_P(const uint8_t level, const char * const fmt, ...) { UNUSED(level); UNUSED(fmt); }
  inline void lcd_buttons_update() {}
  inline void lcd_reset_alert_level() {}
  inline bool lcd_detected() { return true; }
  inline void lcd_refresh() {}
  #define   FunV006(x)
  #define   FunV052()
#endif 
#define LCD_MESSAGEPGM(x)      lcd_setstatusPGM(PSTR(x))
#define LCD_ALERTMESSAGEPGM(x) lcd_setalertstatusPGM(PSTR(x))
void lcd_reset_status();
#if ENABLED(AUTO_BED_LEVELING_UBL)
  extern bool ubl_lcd_map_control;
  void lcd_mesh_edit_setup(float initial);
  float lcd_mesh_edit();
  void lcd_z_offset_edit_setup(float);
  float lcd_z_offset_edit();
#endif
#if ENABLED(DELTA_CALIBRATION_MENU)
  float lcd_probe_pt(const float &lx, const float &ly);
#endif
#if ENABLED(FYS_LCD_EVENT)
  #define MACRO_var_V005             0   
  #define MACRO_var_V006          1   
  #define MACRO_var_V004              2
  #define MACRO_var_V05F                 3
  #define MACRO_var_V024            4
  #define MACRO_VAR_V058            5
  #define LCDEVT_M1104_NEED_ADJUST            6
  extern uint16_t GLOBAL_var_V001;
#endif
#endif 
