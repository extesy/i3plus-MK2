#include "temperature.h"
#include "ultralcd.h"
#include "FysTLcdAddrLayout.h"
#include "Marlin.h"
#include "language.h"
#include "cardreader.h"
#include "temperature.h"
#include "stepper.h"
#include "configuration_store.h"
#include <avr/wdt.h>
typedef void(*generalVoidFun)();
static uint8_t GLOBAL_var_V00A = 0x00;
#define FTSTATE_LCD_OLD_CARDSTATUS  0x01    
#define FTSTATE_NEED_SAVE_PARAM     0x02    
#define FTSTATE_PROMPT_EXE          0x04    
#define FTSTATE_SERVO_STATUS        0x08    
#define FTSTATE_AUTOPID_ING         0x10    
#define FTSTATE_ACTIVE_WARNING      0x20    
char tempChoice = 0;
char optionId = 0;
uint16_t GLOBAL_var_V00C = 0xFFFF, GLOBAL_var_V00D = 0xFFFF;
uint16_t file_num = 0;
int16_t lcd_preheat_hotend_temp[2] = { PREHEAT_1_TEMP_HOTEND, PREHEAT_2_TEMP_HOTEND },
lcd_preheat_bed_temp[2] = { PREHEAT_1_TEMP_BED, PREHEAT_2_TEMP_BED },
lcd_preheat_fan_speed[2] = { PREHEAT_1_FAN_SPEED, PREHEAT_2_FAN_SPEED };
float movDis, movFeedrate;
generalVoidFun periodFun = nullptr;
static FysTLcd myFysTLcd;
#define MOVE_E_FEEDRATE 3.0
#define MOVE_XYZ_FEEDRATE 50.0
#if ENABLED(BABYSTEPPING)
  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
    static void lcd_babystep_zoffset(bool dir);
  #else
    static void lcd_babystep_z();
  #endif
#endif
#if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
bool g_sd_status;
#endif
static void FunV03C();
static void FunV036();
static void FunV035();
static void FunV040();
static void FunV020();
static void FunV021();
#if HOTENDS > 1
static void FunV034();
static void FunV033();
static void FunV039();
static void FunV048();
static void FunV045();
static void FunV04B();
#endif
static void FunV038();
static void FunV04C();
static void FunV046();
static void FunV03A();
static void FunV030();
static void FunV02F();
static void FunV02D();
static void FunV041();
static void FunV054();
static void FunV055();
static void FunV01F(int16_t s);
static void FunV01B();
static void FunV014(millis_t& tNow);
static void FunV042();
static void dwinCheck();
static void dwinExeCmd(millis_t& tNow);
static void firstScreenData();
#if 1 
static int16_t oldFanSpeed = 0;
static uint16_t oldFeedratePercentage = 0;
#endif
void lcd_init()
{
#if defined (SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    pinMode(SD_DETECT_PIN, INPUT);
    WRITE(SD_DETECT_PIN, HIGH);
#endif
    FysTLcd::FunV001();
    FunV029(); 
#if defined (SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    if (READ(SD_DETECT_PIN) == 0)
    {
        GLOBAL_var_V00A |= FTSTATE_LCD_OLD_CARDSTATUS;
        #if 1 
        myFysTLcd.FunV02C(VARADDR_STATUS_SD);
        myFysTLcd.FunV031(1);
        myFysTLcd.FunV02E();
        #endif
    }
    else
    {
        #if 1 
        myFysTLcd.FunV02C(VARADDR_STATUS_SD);
        myFysTLcd.FunV031(0);
        myFysTLcd.FunV02E();
        #endif
    }
    file_num = 0;
#endif
#if FYSTLCD_PAGE_EXIST(MAIN)
    GLOBAL_var_V00D = FTPAGE(MAIN);
    GLOBAL_var_V00C = FTPAGE(MAIN);
#endif
    #if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    g_sd_status = IS_SD_INSERTED;
    #endif 
    firstScreenData();
}
void lcd_update()
{
    millis_t t = millis();
    FunV042();
    dwinCheck();
    dwinExeCmd(t);
    FunV014(t);
}
#if ENABLED(FYS_LCD_EVENT)
static void FunV042()
{
    if (GLOBAL_var_V001 == 0x0000)return;
    uint8_t n;
    char sdFileName[FYSTLCD_FILENAME_LEN],*t;
    for (n = 0; n < 16; n++)
    {
        if (GLOBAL_var_V001&(1 << n))
        {
            GLOBAL_var_V001 &= ~(uint16_t)(1 << n);
            break;
        }
    }
    switch (n)
    {
    case MACRO_var_V005:
    {
        #if FYSTLCD_PAGE_EXIST(INFO_POPUP)
        lcd_setPage(FTPAGE(ASK_RESUM_PRINT));
        #endif
    }
    break;
    case MACRO_var_V006:
    {
      if (card.longFilename[0])strncpy(sdFileName, card.longFilename, FYSTLCD_FILENAME_LEN);
      else strncpy(sdFileName, card.filename, FYSTLCD_FILENAME_LEN);
      t = strchr(sdFileName, '.');
      while (*t)*t++ = '\0';
      FysTLcd::FunV015(MACRO_var_V040, sdFileName, FYSTLCD_FILENAME_LEN);
    }
    break;
    case MACRO_var_V05F:
    {
      GLOBAL_var_V00A &= ~FTSTATE_AUTOPID_ING;
    }
    case MACRO_var_V004:
    {
      GLOBAL_var_V00A |= FTSTATE_PROMPT_EXE;
      FunV020();
    }
    break;
    case MACRO_var_V024:
        {            
        #if FYSTLCD_PAGE_EXIST(AUTO_LEVELING_COMPLETE)&&!FYSTLCD_PAGE_EXIST(AUTO_LEVELING)
        FunV038();
            if( !IS_SD_PRINTING && !print_job_timer.isRunning() ) 
            { 
                lcd_setPage(FTPAGE(MAIN));
            }
        #endif
            disable_X(); 
            disable_Y(); 
        }
    break;
    case MACRO_VAR_V058:
        {
        #if FYSTLCD_PAGE_EXIST(ACCOMPLISH_PRINT)
        lcd_setPage(FTPAGE(ACCOMPLISH_PRINT));
        #elif FYSTLCD_PAGE_EXIST(MAIN)
        lcd_setPage(FTPAGE(MAIN));
        #endif
        }
    break;
    case LCDEVT_M1104_NEED_ADJUST:
        {
        }
    break;
    }
}
#endif
static void dwinCheck()
{   
#if defined (SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    const bool sdNow = IS_SD_INSERTED;
    if (sdNow != g_sd_status)
    {
        if (!sdNow)
        {
            SERIAL_ECHOPGM("SD card inserted.");
            card.initsd();
            #if 1 
            if(card.cardOK)
            {
                myFysTLcd.FunV02C(VARADDR_STATUS_SD);
                myFysTLcd.FunV031(1);
                myFysTLcd.FunV02E();
            }
            #else
            FunV006("SD card inserted.");
            #endif
        }
        else
        {
            bool changePage = false
                #if FYSTLCD_PAGE_EXIST(FILELIST)
                ||GLOBAL_var_V00C == PAGENUM_FILELIST
                #endif
                #if FYSTLCD_PAGE_EXIST(ACCOMPLISH_PRINT)
                ||GLOBAL_var_V00C==PAGENUM_ACCOMPLISH_PRINT
                #endif
                ;
            if (changePage)
            {
            #if FYSTLCD_PAGE_EXIST(MAIN)
                lcd_setPage(FTPAGE(MAIN));
            #endif
            }
            card.release();
            SERIAL_ECHOPGM("SD card removed.");
            #if 1 
            myFysTLcd.FunV02C(VARADDR_STATUS_SD);
            myFysTLcd.FunV026(2);
            myFysTLcd.FunV02E();
            #else
            FunV006("SD card removed.");
            #endif
        }
        g_sd_status = sdNow;
        file_num = 0;
    }
#endif
#if 1 
    #if FAN_COUNT > 0
    if (oldFanSpeed != fanSpeeds[active_extruder])
    {
        oldFanSpeed=fanSpeeds[active_extruder];
        myFysTLcd.FunV02C(VARADDR_TUNE_FAN_SPEED);
        myFysTLcd.FunV031(oldFanSpeed);
        myFysTLcd.FunV02E();
    }
    #endif
    if (oldFeedratePercentage != feedrate_percentage)
    {
        oldFeedratePercentage = feedrate_percentage;
        myFysTLcd.FunV02C(VARADDR_TUNE_PRINT_PERCENTAGE);
        myFysTLcd.FunV031(oldFeedratePercentage);
        myFysTLcd.FunV02E();
    }
#endif
}
static void FunV014(millis_t& tNow)
{
    static millis_t period = 1000;
    if ((GLOBAL_var_V00A&FTSTATE_NEED_SAVE_PARAM) && (commands_in_queue < BUFSIZE))
    {
        enqueue_and_echo_commands_P(PSTR("M500"));
        GLOBAL_var_V00A &= ~FTSTATE_NEED_SAVE_PARAM;
    }
    if (tNow > period || (GLOBAL_var_V00A&FTSTATE_PROMPT_EXE)) 
    {
        if (periodFun)periodFun();
        millis_t distance = 0;
#if defined(FYS_ACTIVE_TIME_OVER)
        if (tNow<previous_cmd_ms)
        {
            previous_cmd_ms = tNow;
            distance = max_inactive_time;
        }
        else
        {
            distance = tNow - previous_cmd_ms;
            if (distance > max_inactive_time)distance = 0;
            else distance = max_inactive_time - distance;
        }
        if (distance<21000)
        {
            if (!(GLOBAL_var_V00A&FTSTATE_ACTIVE_WARNING))
            {
                GLOBAL_var_V00A |= FTSTATE_ACTIVE_WARNING;
                FunV052();
            }
        }
        else if (GLOBAL_var_V00A&FTSTATE_ACTIVE_WARNING)
        {
            GLOBAL_var_V00A &= ~FTSTATE_ACTIVE_WARNING;
            FunV052();
        }
#endif
        FunV01F(distance / 1000);
        period = tNow + 1000;
        GLOBAL_var_V00A &= ~FTSTATE_PROMPT_EXE;
    }
#if defined(MACRO_var_V03E)&&MACRO_var_V03E>0
    static millis_t prompt = 200;
    if (tNow > prompt)
    {
        FunV01B();
        prompt = tNow + 200;
    }
#endif
}
static void dwinSave()
{
    if (commands_in_queue < BUFSIZE)
    {
        enqueue_and_echo_commands_P(PSTR("M500"));
        GLOBAL_var_V00A &= ~FTSTATE_NEED_SAVE_PARAM;
    }
    else 
        GLOBAL_var_V00A |= FTSTATE_NEED_SAVE_PARAM;
}
static inline void dwinPid_autoTune()
{
    char str[30];
    FunV021();
    sprintf(str, "M303 E%d C5 S%d U1", active_extruder, thermalManager.target_temperature[active_extruder]);
    enqueue_and_echo_command(str,true);
    GLOBAL_var_V00A |= FTSTATE_AUTOPID_ING;
}
static void moveAxis(AxisEnum axis, float val)
{
    if(planner.blocks_queued()) return;
    if(axis==E_AXIS)
    {           
        if(thermalManager.degHotend(active_extruder) > 180)
        {
            current_position[axis] += val;
                        movFeedrate = MOVE_E_FEEDRATE;
        }
        else
        {
            return;
        }
    }
    else
    {
        float min = current_position[axis] - 1000,
                max = current_position[axis] + 1000;
          #if HAS_SOFTWARE_ENDSTOPS
            if (soft_endstops_enabled) {
              #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
                min = soft_endstop_min[axis];
              #endif
              #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
                max = soft_endstop_max[axis];
              #endif
            }
          #endif
          current_position[axis] += val;
          NOLESS(current_position[axis], min);
          NOMORE(current_position[axis], max);
              movFeedrate = MOVE_XYZ_FEEDRATE;
      }
      planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], movFeedrate>0.0 ? movFeedrate : pgm_read_float(&homing_feedrate_mm_s[axis]), active_extruder);
}
static void manualLevelingMove(float x, float y)
{
  current_position[Z_AXIS] = Z_CLEARANCE_DEPLOY_PROBE;
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
  current_position[X_AXIS] = x;
  current_position[Y_AXIS] = y;
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
  current_position[Z_AXIS] = 0;
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
  #if ENABLED(FYS_LOOP_EVENT)
    GLOBAL_var_V00E = MACRO_VAR_V056;
  #endif
}
static int16_t FunV058(bool ifHeatBed = true)
{
    int16_t tempe, tempb;
    if (tempChoice < 2)
    {
        tempe = lcd_preheat_hotend_temp[tempChoice];
        tempb = lcd_preheat_bed_temp[tempChoice];
    }
    else
    {
        tempe = 260;
        tempb = 100;
    }
    if (ifHeatBed)thermalManager.setTargetBed(tempb);
    thermalManager.setTargetHotend(tempe, active_extruder);
    #if FAN_COUNT > 0
    if(active_extruder<FAN_COUNT)fanSpeeds[active_extruder] = lcd_preheat_fan_speed[active_extruder];
    #endif
    #if 1 
    myFysTLcd.FunV02C(VARADDR_TUNE_HOTEND_TEMP);
    myFysTLcd.FunV031(tempe);
    if (ifHeatBed)myFysTLcd.FunV031(tempb);
    else myFysTLcd.FunV026(2);
    myFysTLcd.FunV02E();
    #endif
    return tempe;
}
static void FunV057()
{
    int16_t tempe = FunV058(false) - 5;
    if (!planner.is_full() && thermalManager.degHotend(active_extruder) > tempe)
    {
        current_position[E_AXIS] += 2.0;
        planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 
            MOVE_E_FEEDRATE, active_extruder);
    }
}
static void FunV056()
{
    int16_t tempe = FunV058(false) - 5;
    if (!planner.is_full() && thermalManager.degHotend(active_extruder) > tempe)
    {
        current_position[E_AXIS] -= 2.0;
        planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 
            MOVE_E_FEEDRATE, active_extruder);
    }
}
static void updateFileList(bool valid)
{
    char sdFileName[FYSTLCD_FILENAME_LEN + 1] = { 0 };
    for (uint8_t i = 0; i<FILES_NUM; i++)
    {
        if (sdFileName[0] || sdFileName[1])memset(sdFileName, 0, FYSTLCD_FILENAME_LEN);
        if (valid)
        {
            if ((file_num - i)>0 && (file_num - i) < 60000)
            {
                card.getfilename(file_num - 1 - i);
                strncpy(sdFileName, card.longFilename, FYSTLCD_FILENAME_LEN);
                char*t = strchr(sdFileName, '.');
                while (*t)*t++ = '\0';
            }
        }
        FysTLcd::FunV015(VARADDR_FILES_NAME[i], sdFileName, FYSTLCD_FILENAME_LEN);
    }
}
static void updateTargetFile(uint16_t sd_num)
{
    char sdFileName[FYSTLCD_FILENAME_LEN + 1] = { 0 };
    card.getfilename(sd_num - 1);
    #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
      strcpy(GLOBAL_var_V004, card.filename);
    #endif
    strncpy(sdFileName, card.longFilename, FYSTLCD_FILENAME_LEN);
    if (card.isFileOpen())card.closefile();
    card.openFile(card.filename, true);
#if !defined(FILE_PRINT_NEED_CONRIRM)
    card.startFileprint();
    print_job_timer.start();
#endif
    char*t = strchr(sdFileName, '.');
    while (*t)*t++ = '\0';
    FysTLcd::FunV015(MACRO_var_V040, sdFileName, FYSTLCD_FILENAME_LEN);
}
static void reportCmdContent(uint16_t& tar)
{
    for (uint8_t i = 0; i<4; i++)
    {
        char c = (tar >> ((3 - i) << 2)) & 0x0F;
        if (c > 9)MYSERIAL.write(c - 10 + 'A');
        else MYSERIAL.write(c + '0');
    }
}
static void dwinExeCmd(millis_t& tNow)
{
    if (!FysTLcd::FunV059())return;
#if defined(FYS_ACTIVE_TIME_OVER)
    previous_cmd_ms = tNow;
#endif
    uint16_t tval = FysTLcd::FunV05A();
    uint8_t cmd[2];
    switch (FysTLcd::ftAddr)
    {
    case MACRO_var_V068:
        switch (tval)
        {
        case MACRO_var_V069:
            FunV020();
            GLOBAL_var_V00A |= FTSTATE_PROMPT_EXE;
            #if FYSTLCD_PAGE_EXIST(TUNE_PID)
            lcd_setPage(FTPAGE(TUNE_PID));
            #endif
            break;
        case MACRO_var_V06A:
            FunV021();
            break;
        case MACRO_var_V068B:
            FunV021();
            dwinSave();
            break;
        case MACRO_var_V06C:
            enqueue_and_echo_commands_P(PSTR("G28"));
            break;
        case MACRO_var_V06D:
            enqueue_and_echo_commands_P(PSTR("G28 X0"));
            break;
        case MACRO_var_V06E:
            enqueue_and_echo_commands_P(PSTR("G28 Y0"));
            break;
        case MACRO_var_V06F:
            enqueue_and_echo_commands_P(PSTR("G28 Z0"));
            break;
        case VARVAL_TOOL_HOME_XY:
            enqueue_and_echo_commands_P(PSTR("G28 X0 Y0"));
            break;
        case MACRO_var_V070:
          {
            #if ENABLED(FYS_POWER_STATUS)&&PIN_EXISTS(PS_ON)
              GLOBAL_var_V003 = MACRO_var_V00A;
            #endif
          }
          break;
        case MACRO_var_V071:
            for (uint8_t e = 0; e<EXTRUDERS; e++)
                thermalManager.setTargetHotend(0, e);
            thermalManager.setTargetBed(0);
            break;
        case MACRO_var_V072:
            tempChoice = 0;
            FunV058();
            FunV020();
            break;
        case MACRO_var_V073:
            tempChoice = 1;
            FunV058();
            FunV020();
            break;
        case MACRO_var_V074:
            Running = true;
            MYSERIAL.flush();
            SERIAL_ECHOLNPGM("M999 ok.");
            break;
        case MACRO_var_V075:
            thermalManager.setTargetHotend(0, active_extruder);
            break;
        case MACRO_var_V076:
            dwinPid_autoTune();
            break;
        case MACRO_var_V077:
            myFysTLcd.FunV02C(VARADDR_STATUS_AXIS_LOCK);
            if (X_ENABLE_READ == X_ENABLE_ON)
            {
                disable_all_steppers();
                myFysTLcd.FunV026(2);
            }
            else
            {
                enable_all_steppers();
                myFysTLcd.FunV031(1);
            }
            myFysTLcd.FunV02E();
            break;
        #if FAN_COUNT>0
        case MACRO_var_V078:
            myFysTLcd.FunV02C(VARADDR_STATUS_FAN);
            if (fanSpeeds[active_extruder]>0)
            {
                fanSpeeds[active_extruder]=0;
                myFysTLcd.FunV026(2);
            }
            else
            {
                fanSpeeds[active_extruder]=255;
                myFysTLcd.FunV031(1);
            }
            myFysTLcd.FunV02E();
            break;
        #endif
        #if HAS_SERVOS
        case MACRO_var_V079:
            myFysTLcd.FunV02C(VARADDR_STATUS_SERVO);
            if(GLOBAL_var_V00A&FTSTATE_SERVO_STATUS)
            {
                GLOBAL_var_V00A&=~FTSTATE_SERVO_STATUS;
                servo[0].move(90);
                myFysTLcd.FunV026(2);
            }
            else
            {
                GLOBAL_var_V00A|=FTSTATE_SERVO_STATUS;
                servo[0].move(10);
                myFysTLcd.FunV031(1);
            }
            myFysTLcd.FunV02E();
            break;
        case MACRO_var_V07A:
            servo[0].move(160);
            break;
        #endif
        case MACRO_var_V07B:
            #if EXTRUDERS>1
            if (active_extruder + 1 < EXTRUDERS)
            {
                char str[3] = { 'T', active_extruder + '1', 0 };
                enqueue_and_echo_command(str);
            }
            else
            {
                char str[3] = { 'T', '0', 0 };
                enqueue_and_echo_command(str);
            }
            #endif
            break;
        case MACRO_var_V07C:
            wait_for_user = false;
            FunV052();
            break;
        case MACRO_var_V07D:
            if (periodFun == FunV057 || periodFun == FunV056)
            {
                periodFun = nullptr;
                tempChoice = 0;
            }
            stepper.quick_stop();
            thermalManager.setTargetHotend(0, active_extruder);
            FunV020();
            break;
        case MACRO_var_V07E:
          {
            zprobe_zoffset -=0.3;
            dwin_popup(PSTR("\nLeveling is in progress."),1); 
            enqueue_and_echo_commands_P(PSTR("G28"));
            enqueue_and_echo_commands_P(PSTR("G29"));
          }
          break;
        case MACRO_var_V07F:
            break;
        case MACRO_var_V080:
        {
            #if FYSTLCD_PAGE_EXIST(MANUAL_LEVELING)
            lcd_setPage(FTPAGE(MANUAL_LEVELING));
            #endif
            enqueue_and_echo_commands_P(PSTR("G28")); 
            #if HAS_LEVELING            
            reset_bed_level();
            enqueue_and_echo_commands_P(PSTR("M500")); 
            #endif
        }
        break;
        #if ENABLED(BABYSTEPPING)
        case VARVAL_TOOL_BABYSTEP_UP_Z:
        {
            #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
              movDis = 0.1;
              lcd_babystep_zoffset(true);
            #endif
        }
        break;
        case VARVAL_TOOL_BABYSTEP_DOWN_Z:
        {
            #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
              movDis = 0.1;
              lcd_babystep_zoffset(false);
            #endif
        }
        break;
        #endif
        #if HAS_BED_PROBE 
        case VARVAL_TOOL_LEVELING_ZOFFSET_UP_Z:
        {
            if(!planner.blocks_queued()) {
              lcd_zstep_zoffset(true);
            }
        }
        break;
        case VARVAL_TOOL_LEVELING_ZOFFSET_DOWN_Z:
        {
          if(!planner.blocks_queued()) {
            lcd_zstep_zoffset(false);
          }
        }
        break;
        case VARVAL_TOOL_LEVELING_ADJUST_ZOFFSET:
        {                             
          const bool level_active = leveling_is_active();
          if (level_active) {
            set_bed_leveling_enabled(false);  
          }
          dwin_popup(PSTR("\nLeveling is in progress."),1);
          #if ENABLED(FYS_RECORD_ZOFFSET_LAST)
            zprobe_zoffset_last = zprobe_zoffset;
          #endif
          zprobe_zoffset+=0.3;
          movDis = 0.1;
          gcode_G28(true);
          stepper.synchronize();
          current_position[X_AXIS] = X_CENTER;
          current_position[Y_AXIS] = Y_CENTER;
          planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);           
          #if ENABLED(FYS_HOME_FUNCTION)
            homeZ();
          #endif
          current_position[Z_AXIS] = 0;
          planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS],  current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);            
          stepper.synchronize();
          disable_X();
          disable_Y();
          #if FYSTLCD_PAGE_EXIST(MANUAL_LEVELING)
            lcd_setPageForce(FTPAGE(MANUAL_LEVELING));
          #endif
        }
        break;
        case VARVAL_TOOL_LEVELING_ADJUST_ZOFFSET_CONFIRM:
        {
            zprobe_zoffset -=0.3;
            do_probe_raise(5);
            #if ENABLED(FYS_HOME_FUNCTION)
              FunV007();
            #endif
            dwinSave(); 
            #if FYSTLCD_PAGE_EXIST(MAIN)
            lcd_setPageForce(FTPAGE(MAIN));
            #endif
        }
        break;
        case VARVAL_TOOL_LEVELING_ADJUST_ZOFFSET_BACK:
        {
            #if ENABLED(FYS_RECORD_ZOFFSET_LAST)
              zprobe_zoffset = zprobe_zoffset_last;
            #endif
            current_position[Z_AXIS] = 10;
            planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS],  current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
            stepper.synchronize();
            #if ENABLED(FYS_HOME_FUNCTION)
              FunV007();
            #endif
            #if FYSTLCD_PAGE_EXIST(MAIN)
              lcd_setPageForce(FTPAGE(MAIN));
            #endif
        }
        break;
        #endif
        #ifdef AUTO_BED_LEVELING_LINEAR
        case MACRO_var_V081:
            manualLevelingMove(X_MIN_POS + MESH_INSET, Y_MIN_POS + MESH_INSET);
            break;
        case MACRO_var_V082:
            manualLevelingMove(X_MAX_POS - MESH_INSET, Y_MIN_POS + MESH_INSET);
            break;
        case MACRO_var_V083:
            manualLevelingMove(X_MAX_POS - MESH_INSET, Y_MAX_POS - MESH_INSET);
            break;
        case MACRO_var_V084:
            manualLevelingMove(X_MIN_POS + MESH_INSET, Y_MAX_POS - MESH_INSET);
            break;
        #endif
        case MACRO_var_V085:
            planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
            current_position[Z_AXIS] += 1;
            line_to_current_position();
            break;
        case MACRO_var_V086:
            planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
            current_position[Z_AXIS] -= 1;
            line_to_current_position();
            break;
        case MACRO_var_V087:
            {
                            moveAxis(X_AXIS, -5);
            }
            break;
        case MACRO_var_V088:
            {
                            moveAxis(X_AXIS, 5);
            }
            break;
        case MACRO_var_V089:
            {
                            moveAxis(Y_AXIS, -5);
            }
            break;
        case MACRO_var_V08A:
            {
                            moveAxis(Y_AXIS, 5);
            }
            break;
        case MACRO_var_V08B:
            {
                            moveAxis(Z_AXIS, -5);
            }
            break;
        case MACRO_var_V08C:
            {
                            moveAxis(Z_AXIS, 5);
            }
            break;
        case MACRO_var_V08D:
            {
                            moveAxis(E_AXIS, -5);
            }
            break;
        case MACRO_var_V08E:
            {
                            moveAxis(E_AXIS, 5);
            }
            break;
        case MACRO_var_V08F:
            break;
        case VARVAL_TOOL_ENTER_PAPER_HEIGHT:
            #if FYSTLCD_PAGE_EXIST(TOOL_PAPERHEIGHT)
            lcd_setPage(FTPAGE(TOOL_PAPERHEIGHT));
            #endif
            break;
        case VARVAL_TOOL_RESET:
            wdt_enable(WDTO_30MS);
            while (1) {};
            break;
        case VARVAL_TOOL_OPTION_LEFT:
            FunV052();
            switch (optionId)
            {
            #if ENABLED(ADVANCED_PAUSE_FEATURE)
            case 1:
                advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_RESUME_PRINT;
                break;
            #endif
            default:
                break;
            }
            break;
        case VARVAL_TOOL_OPTION_RIGHT:
            FunV052();
            switch (optionId)
            {
            #if ENABLED(ADVANCED_PAUSE_FEATURE)
            case 1:
                advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE;
                break;
            #endif
            default:
                break;
            }
            break;
        case VARVAL_TOOL_CONNECT_WIFI:
            break;
        case VARVAL_TOOL_COOLDOWN_HOTEND:
            thermalManager.setTargetHotend(0, active_extruder);
            #if 1 
            myFysTLcd.FunV02C(VARADDR_TUNE_HOTEND_TEMP);
            myFysTLcd.FunV026(2);
            myFysTLcd.FunV02E();
            #endif
            break;
        case VARVAL_TOOL_COOLDOWN_BED:
            thermalManager.setTargetBed(0);
            #if 1 
            myFysTLcd.FunV02C(VARADDR_TUNE_BED_TEMP);
            myFysTLcd.FunV026(2);
            myFysTLcd.FunV02E();
            #endif
            break;
          case VARVAL_TOOL_EMERGENCY_STOP_MOTOR:
          {
            #if ENABLED(FYS_POWERBREAK_STEPPER_STATUS)
              unsigned char _sreg = SREG; 
              sei();
              stepper.powerBreakStatus = 1;
              while (stepper.powerBreakStatus == 1);
              stepper.powerBreakStatus = 0;
              disable_all_steppers();
              SREG = _sreg;
            #endif
          }
          break;
          case VARVAL_TOOL_WIFI_UPDATE:
          {
            #ifdef FYS_WIFI_HLK
              if(!card.sdprinting)
              {
                MYSERIAL.setTxActiveSerial(1);
                safe_delay(100);
                SERIAL_ECHOPGM("+++");
                safe_delay(2000);
                SERIAL_ECHOLNPGM("at+wifi_conf=?\r");
                wifiSwitchModeStage=WIFI_SWITCH_STAGE_INIT;
                MYSERIAL.setTxActiveSerial(0);
              }
            #endif
          }
          break;
        }
        break;
    case MACRO_var_V026:
      #if ENABLED(SDSUPPORT)
        if (card.cardOK)
        {
            if (tval == MACRO_var_V02C || tval == MACRO_var_V02A)
            {
                uint16_t fileCnt = card.getnrfilenames();
                if (file_num<1)file_num = fileCnt;
                card.getWorkDirName();
                if (tval == MACRO_var_V02A)
                {
                    if (file_num > FILES_NUM)
                        file_num = file_num - FILES_NUM;
                }
                else
                {
                    if (file_num <= (fileCnt - FILES_NUM))
                        file_num = file_num + FILES_NUM;
                }
                updateFileList(true);
            }
            else
            switch (tval)
            {
                case MACRO_var_V028:
                {
                  if (print_job_timer.isRunning() || print_job_timer.isPaused())
                  {
                      #if FYSTLCD_PAGE_EXIST(PRINT)
                      lcd_setPage(FTPAGE(PRINT));
                      #endif
                  }
                  else
                  {
                      #if FYSTLCD_PAGE_EXIST(FILELIST)
                      lcd_setPage(FTPAGE(FILELIST));
                      #endif
                      file_num = card.getnrfilenames();
                      card.getWorkDirName();
                      updateFileList(true);
                  }
                }
                break;
                case MACRO_var_V030:
                {
                  #if ENABLED(FYS_LOOP_EVENT)
                    GLOBAL_var_V00E = MACRO_var_V021;
                  #endif
                }
                break;
                case MACRO_var_V032:
                  #if ENABLED(FYS_LOOP_EVENT)
                    GLOBAL_var_V00E = MACRO_var_V020;
                  #endif
                    break;
                case MACRO_var_V033:
                    #if ENABLED(FYS_LOOP_EVENT)
                      GLOBAL_var_V00E = MACRO_var_V022;
                    #endif
                    wait_for_heatup = false;
                    #if FYSTLCD_PAGE_EXIST(MAIN)
                    lcd_setPage(FTPAGE(MAIN));
                    #endif
                    break;
                case MACRO_var_V035:
                    FunV03C();
                    #if FYSTLCD_PAGE_EXIST(TUNE)
                    lcd_setPage(FTPAGE(TUNE));
                    #endif
                    break;
                case MACRO_var_V037:
                    GLOBAL_var_V00D = PAGENUM_PRINT;
                    GLOBAL_var_V00C = PAGENUM_PRINT;
                    FunV036();
                    #if ENABLED(BABYSTEPPING)
                      settings.save();
                    #endif
                    break;
                #ifdef FYS_SAFE_PRINT_BREAK
                case MACRO_var_V03B: 
                    #if FYSTLCD_PAGE_EXIST(PRINT)
                    lcd_setPage(FTPAGE(PRINT));
                    #endif
                    delay(20);
                    gcode_M1101();
                    break;
                case MACRO_var_V03D:  
                    #if FYSTLCD_PAGE_EXIST(MAIN)
                    lcd_setPage(FTPAGE(MAIN));
                    #endif
                    delay(20);
                    gcode_M1103();
                    break;
                #endif
                #if defined(FILE_PRINT_NEED_CONRIRM)
                case VARVAL_PRINT_CONFIRM:
                    card.startFileprint();
                    print_job_timer.start();
                    #if FYSTLCD_PAGE_EXIST(PRINT)
                    lcd_setPage(FTPAGE(PRINT));
                    #endif
                    break;
                #endif
                case VARVAL_PRINT_SD_REFRESH:
                    card.initsd();
                    if (card.cardOK)
                    {
                        file_num = card.getnrfilenames();
                        card.getWorkDirName();
                        updateFileList(true);
                        SERIAL_ECHOLNPGM("Refresh ok.");
                    }
                    else
                    {
                        file_num = 0;
                        updateFileList(false);
                    }
                    break;
                case VARVAL_PRINT_REPRINT:
                    #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
                      if (card.isFileOpen()) card.closefile();                    
                        card.openFile(GLOBAL_var_V004, true);
                        if (!card.isFileOpen())
                        {
                            FunV006("Target file open fail.");
                            return;
                        }
                        card.startFileprint();
                        print_job_timer.start();
                    #endif
                    #if FYSTLCD_PAGE_EXIST(PRINT)
                      lcd_setPage(FTPAGE(PRINT));
                    #endif
                    break;
                default:
                    for (uint8_t i = 0; i < FILES_NUM; i++)
                    if (tval == MACRO_var_V02E[i]&&file_num>i)
                    {
                        updateTargetFile(file_num - i);
                        #if defined(FILE_PRINT_NEED_CONRIRM)
                          #if FYSTLCD_PAGE_EXIST(PRINTFILE_CONFIRM)
                          lcd_setPage(FTPAGE(PRINTFILE_CONFIRM));
                          #endif
                        #elif FYSTLCD_PAGE_EXIST(PRINT)
                          lcd_setPage(FTPAGE(PRINT));
                        #endif
                        break;
                    }
                    break;
            }
        }
        else
        {
            card.initsd();
            switch (tval)
            {
            case VARVAL_PRINT_SD_REFRESH:
                if (card.cardOK)
                {
                    file_num = card.getnrfilenames();
                    card.getWorkDirName();
                    updateFileList(true);
                }
                else
                {
                    file_num = 0;
                    updateFileList(false);
                }
                break;
            #if defined(FYS_SAFE_PRINT_BREAK)&&FYSTLCD_PAGE_EXIST(MAIN)
              case MACRO_var_V03D:  
                #if FYSTLCD_PAGE_EXIST(MAIN)
                lcd_setPage(FTPAGE(MAIN));
                #endif
                delay(20);
                gcode_M1103();
                break;
              #endif
            }
        }
      #endif
        break;
    case MACRO_var_V044:
        switch (tval)
        {
        case MACRO_var_V045:
            FunV035();
            #if FYSTLCD_PAGE_EXIST(SETTING_MOTOR)
            lcd_setPage(FTPAGE(SETTING_MOTOR));
            #endif
            break;
        case MACRO_var_V046:
            FunV040();
            break;
        case MACRO_var_V047:
            FunV040();
            dwinSave();
            break;
        case MACRO_var_V048:
            FunV038();
            #if FYSTLCD_PAGE_EXIST(SETTING_LEVELING)
            lcd_setPage(FTPAGE(SETTING_LEVELING));
            #endif
            break;
        case MACRO_var_V049:
            FunV04C();
            break;
        case MACRO_var_V04A:
            FunV04C();
            dwinSave();
            break;
        case MACRO_var_V04B:
            FunV046();
            #if FYSTLCD_PAGE_EXIST(SETTING_TEMP)
            lcd_setPage(FTPAGE(SETTING_TEMP));
            #endif
            break;
        case MACRO_var_V04C:
            FunV03A();
            break;
        case MACRO_var_V04D:
            FunV03A();
            dwinSave();
            break;
        case MACRO_var_V04E:
            FunV030();
            #if FYSTLCD_PAGE_EXIST(SETTING_MATERIAL)
            lcd_setPage(FTPAGE(SETTING_MATERIAL));
            #endif
            break;
        case MACRO_var_V04F:
            FunV02F();
            break;
        case MACRO_var_V050:
            FunV02F();
            dwinSave();
            break;
        case MACRO_var_V051:
            FunV02D();
            #if FYSTLCD_PAGE_EXIST(SETTING_TMC2130)
            lcd_setPage(FTPAGE(SETTING_TMC2130));
            #endif
            break;
        case MACRO_var_V052:
            FunV041();
            break;
        case MACRO_var_V053:
            FunV041();
            dwinSave();
            break;
        case MACRO_var_V054:
            dwinSave();
            break;
        case MACRO_var_V055:
            settings.reset();
            break;
        case MACRO_var_V063:
            settings.reset();
            dwinSave();
            break;
        case MACRO_var_V064:
            FunV054();
            #if FYSTLCD_PAGE_EXIST(SETTING_SYSTEM)
            lcd_setPage(FTPAGE(SETTING_SYSTEM));
            #endif
            break;
        case MACRO_var_V065:
            FunV055();
            break;
        case MACRO_var_V066:
            FunV055();
            dwinSave();
            break;
        }
        break;
#if HOTENDS > 1
    case MACRO_var_V091:
        switch (tval)
        {
        case MACRO_var_V092:
            FunV034();
            #if FYSTLCD_PAGE_EXIST(SETTING_EXTRUDERS_OFFSET)
            lcd_setPage(FTPAGE(SETTING_EXTRUDERS_OFFSET));
            #endif
            break;
        case MACRO_var_V093:
            FunV033();
            break;
        case MACRO_var_V094:
            FunV033();
            dwinSave();
            break;
        case MACRO_var_V095:
            FunV045();
            #if FYSTLCD_PAGE_EXIST(SETTING_EXTRUDERS_MOTOR)
            lcd_setPage(FTPAGE(SETTING_EXTRUDERS_MOTOR));
            #endif
            break;
        case MACRO_var_V096:
            FunV04B();
            break;
        case MACRO_var_V097:
            FunV04B();
            dwinSave();
            break;
        case MACRO_var_V098:
            FunV039();
            #if FYSTLCD_PAGE_EXIST(SETTING_EXTRUDERS_TEMP)
            lcd_setPage(FTPAGE(SETTING_EXTRUDERS_TEMP));
            #endif
            break;
        case MACRO_var_V099:
            FunV048();
            break;
        case MACRO_var_V09A:
            FunV048();
            dwinSave();
            break;
        }
        break;
#endif
    case MACRO_var_V09E:
    case MACRO_var_V09F:
        tempChoice = tval - 1;
        if (FysTLcd::ftAddr == MACRO_var_V09E)periodFun = FunV057;
        else periodFun = FunV056;
        break;
    case MACRO_var_V09C:
        movDis = tval;
        movDis /= FYSTLCD_DOT_TEN_MUL;
        myFysTLcd.FunV02C(VARADDR_MOVE_DIS_SIGN);
        myFysTLcd.FunV031(tval);
        myFysTLcd.FunV02E();
        break;
    case MACRO_var_V09D:
        movFeedrate = tval;
        movFeedrate /= FYSTLCD_DOT_TEN_MUL;
        myFysTLcd.FunV02C(VARADDR_MOVE_SPEED_SIGN);
        myFysTLcd.FunV031(tval);
        myFysTLcd.FunV02E();
        break;
    case VARADDR_JUMP_PAGE:
        GLOBAL_var_V00D = tval; 
        GLOBAL_var_V00C = tval;
        break;
    #if 1 
    case VARADDR_TUNE_PRINT_PERCENTAGE:
        oldFeedratePercentage = tval;
        feedrate_percentage = tval;
        break;
    case VARADDR_TUNE_FAN_SPEED:
        #if FAN_COUNT > 0
        oldFanSpeed=tval;
        fanSpeeds[active_extruder] = tval;
        #endif
        break;
    case VARADDR_TUNE_HOTEND_TEMP:
        thermalManager.setTargetHotend(tval, active_extruder);
        break;
    case VARADDR_TUNE_BED_TEMP:
        thermalManager.setTargetBed(tval);
        break;
    #endif
    default:
        break;;
    }
}
#if defined(MACRO_var_V03E)&&MACRO_var_V03E>0
static void FunV01B()
{
    myFysTLcd.FunV02C(MACRO_var_V03E);
    #if HAS_X_MIN 
    if(READ(X_MIN_PIN) ^ X_MIN_ENDSTOP_INVERTING)myFysTLcd.FunV031(1);
    else myFysTLcd.FunV026(2);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if HAS_X_MAX 
    if (READ(X_MAX_PIN) ^ X_MAX_ENDSTOP_INVERTING)myFysTLcd.FunV031(1);
    else myFysTLcd.FunV026(2);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if HAS_Y_MIN 
    if(READ(Y_MIN_PIN) ^ Y_MIN_ENDSTOP_INVERTING)myFysTLcd.FunV031(1);
    else myFysTLcd.FunV026(2);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if HAS_Y_MAX 
    if(READ(Y_MAX_PIN) ^ Y_MAX_ENDSTOP_INVERTING)myFysTLcd.FunV031(1);
    else myFysTLcd.FunV026(2);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if HAS_Z_MIN 
    if(READ(Z_MIN_PIN) ^ Z_MIN_ENDSTOP_INVERTING)myFysTLcd.FunV031(1);
    else myFysTLcd.FunV026(2);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if HAS_Z_MAX 
    if(READ(Z_MAX_PIN) ^ Z_MAX_ENDSTOP_INVERTING)myFysTLcd.FunV031(1);
    else myFysTLcd.FunV026(2);
    #else
    myFysTLcd.FunV026(2);
    #endif
    myFysTLcd.FunV026(2);
    static uint8_t dynamicIcon = 0;
    dynamicIcon++;
    if (dynamicIcon > 9)dynamicIcon = 0;
    #if FAN_COUNT > 0 
    if (fanSpeeds[active_extruder] > 0)
        myFysTLcd.FunV031(dynamicIcon);
    else
        myFysTLcd.FunV026(2);
    #else
    myFysTLcd.FunV026(2);
    #endif
    if (card.sdprinting)
        myFysTLcd.FunV031(dynamicIcon);
    else
        myFysTLcd.FunV026(2);
    myFysTLcd.FunV026(2);
    if (GLOBAL_var_V00A&FTSTATE_AUTOPID_ING)
        myFysTLcd.FunV031(dynamicIcon);
    else
        myFysTLcd.FunV026(2);
    myFysTLcd.FunV02E();
}
#endif
static void FunV01F(int16_t s)
{
    uint8_t i;
    myFysTLcd.FunV02C(MACRO_var_V039);
    myFysTLcd.FunV032(thermalManager.degHotend(active_extruder));
    #if FAN_COUNT > 0
    myFysTLcd.FunV031(fanSpeeds[active_extruder]);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if HAS_TEMP_BED
    myFysTLcd.FunV032(thermalManager.degBed());
    #else
    myFysTLcd.FunV026(2);
    #endif
    myFysTLcd.FunV031(feedrate_percentage);
    for (i = 0; i < 4;i++)
    {
        myFysTLcd.FunV032(current_position[i]);
    }
    #if ENABLED(SDSUPPORT)
    static int progress = 0;
    if (IS_SD_PRINTING)progress = card.percentDone();
    if (progress> 100)progress = 0;
    myFysTLcd.FunV031(progress);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if defined(FYS_ACTIVE_TIME_OVER)
    myFysTLcd.FunV031(s);
    #else
    myFysTLcd.FunV026(2);
    #endif
    myFysTLcd.FunV02E();
    #if HAS_BED_PROBE
    myFysTLcd.FunV02C(VARADDR_ZOFFSET_DATA);
    myFysTLcd.ftPutF16_2(zprobe_zoffset);
    myFysTLcd.FunV02E();
    #endif
    if (IS_SD_PRINTING)
    {
        myFysTLcd.FunV02C(MACRO_var_V03C);
        millis_t pt = print_job_timer.duration();
        float total = float(pt) * float(card.getFileSize()) / float(card.getFilePos());
        static float smoothTotal = 0;
        myFysTLcd.FunV049(pt);
        myFysTLcd.FunV02E(8);
        smoothTotal = (smoothTotal * 999L + total) / 1000L;
        if (isinf(smoothTotal))smoothTotal = total;
        if (pt < 120)
        {
            myFysTLcd.FunV04A(PSTR("Unknown."));
            smoothTotal = total;
        }
        else
        {
            pt = smoothTotal;
            myFysTLcd.FunV049(pt);
        }
        myFysTLcd.FunV02E();
    }
}
static void FunV020()
{
    myFysTLcd.FunV02C(MACRO_var_V03A);
    int n = active_extruder;
    myFysTLcd.FunV031(n);
    myFysTLcd.FunV031(thermalManager.target_temperature[active_extruder]);
    myFysTLcd.FunV03D(PID_PARAM(Kp, active_extruder));
    myFysTLcd.FunV03D(unscalePID_i(PID_PARAM(Ki, active_extruder)));
    myFysTLcd.FunV03D(unscalePID_d(PID_PARAM(Kd, active_extruder)));
    #if ENABLED(PID_EXTRUSION_SCALING)
    myFysTLcd.FunV03D(PID_PARAM(Kc, active_extruder));
    #else
    myFysTLcd.FunV026(4);
    #endif
    #if HAS_TEMP_BED
    myFysTLcd.FunV031(thermalManager.target_temperature_bed);
    #else
    myFysTLcd.FunV026(2);
    #endif
    myFysTLcd.FunV02E();
}
static void FunV021()
{
    myFysTLcd.FunV02C(MACRO_var_V03A);
    if (myFysTLcd.FunV027(22))
    {
        myFysTLcd.FunV026(2);
        int16_t t;
        myFysTLcd.FunV03E(t);
        myFysTLcd.FunV047(PID_PARAM(Kp, active_extruder));
        myFysTLcd.FunV047(PID_PARAM(Ki, active_extruder));
        PID_PARAM(Ki, active_extruder) = scalePID_i(PID_PARAM(Ki, active_extruder));
        myFysTLcd.FunV047(PID_PARAM(Kd, active_extruder));
        PID_PARAM(Kd, active_extruder) = scalePID_d(PID_PARAM(Kd, active_extruder));
    #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.FunV047(PID_PARAM(Kc, active_extruder));
    #else
        myFysTLcd.FunV026(4);
    #endif
        thermalManager.setTargetHotend(t, active_extruder);
        myFysTLcd.FunV03E(t);
        thermalManager.setTargetBed(t);
    }
}
static void FunV03C()
{
    uint8_t e;
    myFysTLcd.FunV02C(MACRO_var_V027);
    #if HAS_TEMP_0
    myFysTLcd.FunV031(thermalManager.target_temperature[0]);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if HAS_TEMP_1
    myFysTLcd.FunV031(thermalManager.target_temperature[1]);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if HAS_TEMP_2
    myFysTLcd.FunV031(thermalManager.target_temperature[2]);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if HAS_TEMP_3
    myFysTLcd.FunV031(thermalManager.target_temperature[3]);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if HAS_TEMP_4
    myFysTLcd.FunV031(thermalManager.target_temperature[4]);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if HAS_TEMP_BED
    myFysTLcd.FunV031(thermalManager.target_temperature_bed);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if FAN_COUNT > 0
    for(e=0;e<FAN_COUNT;e++)
    {
        myFysTLcd.FunV031(fanSpeeds[e]);
    }
    for (; e<5; e++)myFysTLcd.FunV026(2);
    #else
    myFysTLcd.FunV026(10);
    #endif
    myFysTLcd.FunV031(feedrate_percentage);
    for (e = 0; e < EXTRUDERS; e++)
    {
        myFysTLcd.FunV031(flow_percentage[e]);
    }
    for (; e < 5; e++)myFysTLcd.FunV026(2);
    #if ENABLED(FYS_SHUTDOWN_ONCE_PRINT_DONE)&&PIN_EXISTS(PS_ON)
    if (GLOBAL_var_V005)myFysTLcd.FunV031(4);
    else myFysTLcd.FunV031(5);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #ifdef FYS_ENERGY_CONSERVE_HEIGHT
    if (ifEnergyConserve)myFysTLcd.FunV031(4);
    else myFysTLcd.FunV031(5);
    #else
    myFysTLcd.FunV026(2);
    #endif
    myFysTLcd.FunV02E();
}
static void FunV036()
{
    int16_t t;
    myFysTLcd.FunV02C(MACRO_var_V027);
    if (myFysTLcd.FunV027(38))
    {
        #if HAS_TEMP_0
          myFysTLcd.FunV03E(t);
          thermalManager.setTargetHotend(t, 0);
        #else
          myFysTLcd.FunV026(2);
        #endif
        #if HAS_TEMP_1
        myFysTLcd.FunV03E(t);
        thermalManager.setTargetHotend(t, 1);
        #else
        myFysTLcd.FunV026(2);
        #endif
        #if HAS_TEMP_2
        myFysTLcd.FunV03E(t);
        thermalManager.setTargetHotend(t,2);
        #else
        myFysTLcd.FunV026(2);
        #endif
        #if HAS_TEMP_3
        myFysTLcd.FunV03E(t);
        thermalManager.setTargetHotend(t,3);
        #else
        myFysTLcd.FunV026(2);
        #endif
        #if HAS_TEMP_4
        myFysTLcd.FunV03E(t);
        thermalManager.setTargetHotend(t, 4);
        #else
        myFysTLcd.FunV026(2);
        #endif
        #if HAS_TEMP_BED
        myFysTLcd.FunV03E(t);
        thermalManager.setTargetBed(t);
        #else
        myFysTLcd.FunV026(2);
        #endif
        uint8_t e;
        #if FAN_COUNT > 0
          for (e = 0; e<FAN_COUNT; e++) myFysTLcd.FunV03E(fanSpeeds[e]);
          for (; e<5; e++) myFysTLcd.FunV026(2);
        #else
          myFysTLcd.FunV026(10);
        #endif
        myFysTLcd.FunV03E(feedrate_percentage);
        for (e = 0; e < EXTRUDERS; e++)
        {
          myFysTLcd.FunV03E(flow_percentage[e]);
        }
        for (; e<5; e++)myFysTLcd.FunV026(2);
        int16_t n;
        #if ENABLED(FYS_SHUTDOWN_ONCE_PRINT_DONE)&&PIN_EXISTS(PS_ON)
          myFysTLcd.FunV03E(n);  
          if (n == 4)GLOBAL_var_V005 = true;
          else GLOBAL_var_V005 = false;
        #endif
        #ifdef FYS_ENERGY_CONSERVE_HEIGHT
          myFysTLcd.FunV03E(n);
          if (n == 4)ifEnergyConserve = true;
          else ifEnergyConserve = false;
        #endif
    }
}
static void FunV035()
{
    myFysTLcd.FunV02C(MACRO_var_V029);
    for (uint8_t i = 0; i < 4; i++) 
    {
        myFysTLcd.FunV03D(planner.axis_steps_per_mm[i]);
    }
    for (uint8_t i = 0; i < 4; i++) 
    {
        myFysTLcd.FunV03D(planner.max_feedrate_mm_s[i]);
    }
    myFysTLcd.FunV02E();
    for (uint8_t i = 0; i < 4; i++) 
    {
        myFysTLcd.FunV037((int32_t)planner.max_acceleration_mm_per_s2[i]);
    }
    myFysTLcd.FunV03D(planner.acceleration);
    myFysTLcd.FunV03D(planner.retract_acceleration);
    myFysTLcd.FunV03D(planner.travel_acceleration);
    myFysTLcd.FunV03D(planner.min_feedrate_mm_s);
    myFysTLcd.FunV02E();
    myFysTLcd.FunV037((int32_t)planner.min_segment_time); 
    myFysTLcd.FunV03D(planner.min_travel_feedrate_mm_s);
    for(uint8_t i=0;i<4;i++) 
    {
        myFysTLcd.FunV03D(planner.max_jerk[i]);
    }
    myFysTLcd.FunV02E();
#if HAS_HOME_OFFSET
    #if ENABLED(DELTA)
    myFysTLcd.FunV026(8);
    float f=DELTA_HEIGHT + home_offset[Z_AXIS];
    myFysTLcd.FunV03D(f);
    #else
    for(uint8_t i=0;i<3;i++) 
    {
        myFysTLcd.FunV03D(home_offset[i]);
    }
    #endif
#else
    myFysTLcd.FunV026(12);
#endif
#if HAS_MOTOR_CURRENT_PWM
    for (uint8_t q = 0; q<3;q++) 
    {
        myFysTLcd.FunV037((int32_t)stepper.motor_current_setting[q]);
    }
    myFysTLcd.FunV037(MOTOR_CURRENT_PWM_RANGE);
#else
    myFysTLcd.FunV026(16);
#endif
    myFysTLcd.FunV02E();
}
static void FunV040()
{
    myFysTLcd.FunV02C(MACRO_var_V029);
    if (myFysTLcd.FunV027(32))
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            myFysTLcd.FunV047(planner.axis_steps_per_mm[i]);
        }
        planner.refresh_positioning(); 
        for (uint8_t i = 0; i < 4; i++)
        {
            myFysTLcd.FunV047(planner.max_feedrate_mm_s[i]);
        }
    }
    myFysTLcd.ftCmdClear();
    if (myFysTLcd.FunV027(32))
    {
        int32_t t;
        for (uint8_t i = 0; i < 4; i++)
        {
            myFysTLcd.FunV044(t);
            planner.max_acceleration_mm_per_s2[i] = t;
        }
        myFysTLcd.FunV047(planner.acceleration);
        myFysTLcd.FunV047(planner.retract_acceleration);
        myFysTLcd.FunV047(planner.travel_acceleration);
        myFysTLcd.FunV047(planner.min_feedrate_mm_s);
    }
    myFysTLcd.ftCmdClear();
    if (myFysTLcd.FunV027(24))
    {
        int32_t t;
        myFysTLcd.FunV044(t);
        planner.min_segment_time = t;
        myFysTLcd.FunV047(planner.min_travel_feedrate_mm_s);
        myFysTLcd.FunV047(planner.max_jerk[X_AXIS]);
        myFysTLcd.FunV047(planner.max_jerk[Y_AXIS]);
        myFysTLcd.FunV047(planner.max_jerk[Z_AXIS]);
        myFysTLcd.FunV047(planner.max_jerk[E_AXIS]);
    }
    myFysTLcd.ftCmdClear();
    if (myFysTLcd.FunV027(24))
    {
#if HAS_HOME_OFFSET
    #if ENABLED(DELTA)
        myFysTLcd.FunV026(8);
        myFysTLcd.FunV047(home_offset[Z_AXIS]);
        home_offset[Z_AXIS] -= DELTA_HEIGHT;
    #else
        for (uint8_t i = 0; i < 3; i++)
        {
            myFysTLcd.FunV047(home_offset[i]);
        }
    #endif
#else
        myFysTLcd.FunV026(12);
#endif
#if HAS_MOTOR_CURRENT_PWM
        int32_t t;
        for (uint8_t q = 0; q<3; q++) 
        {
            myFysTLcd.FunV044(t);
            stepper.motor_current_setting[q]=t;
        }
#else
        myFysTLcd.FunV026(12);
#endif
    }
}
#if HOTENDS > 1
static void FunV034()
{
    uint8_t e;
    myFysTLcd.FunV02C(MACRO_var_V034);
#if EXTRUDERS==5
    for (e = 1; e < 4; e++)
    {
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.FunV03D(hotend_offset[axis][e]);
        }
    }
    myFysTLcd.FunV02E(); 
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        myFysTLcd.FunV03D(hotend_offset[axis][4]);
    }
    myFysTLcd.FunV02E(); 
#else
    for (e = 1; e < EXTRUDERS; e++)
    {
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.FunV03D(hotend_offset[axis][e]);
        }
    }
    myFysTLcd.FunV02E();
#endif
}
static void FunV033()
{
    myFysTLcd.FunV02C(MACRO_var_V034);
#if EXTRUDERS==5
    if (myFysTLcd.FunV027(36))
    {
        for (uint8_t e = 1; e < 4; e++)
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.FunV047(hotend_offset[axis][e]);
        }
    }
    myFysTLcd.ftCmdClear();
    if (myFysTLcd.FunV027(12))
    {
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.FunV047(hotend_offset[axis][4]);
        }
    }
#else
    if (myFysTLcd.FunV027((EXTRUDERS - 1) * 12))
    {
        for (uint8_t e = 1; e < EXTRUDERS; e++)
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.FunV047(hotend_offset[axis][e]);
        }
    }
#endif
}
static void FunV045()
{
    myFysTLcd.FunV02C(MACRO_var_V036);
#if EXTRUDERS==5
    for (uint8_t e = 1; e < 4; e++)
    {
        myFysTLcd.FunV03D(planner.axis_steps_per_mm[3 + e]);
        myFysTLcd.FunV03D(planner.max_feedrate_mm_s[3 + e]);
        myFysTLcd.FunV03D(planner.max_acceleration_mm_per_s2[3 + e]);
    }
    myFysTLcd.FunV02E();
    myFysTLcd.FunV03D(planner.axis_steps_per_mm[7]);
    myFysTLcd.FunV03D(planner.max_feedrate_mm_s[7]);
    myFysTLcd.FunV03D(planner.max_acceleration_mm_per_s2[7]);
    myFysTLcd.FunV02E();
#else
    for (uint8_t e = 1; e < EXTRUDERS; e++)
    {
        myFysTLcd.FunV03D(planner.axis_steps_per_mm[3 + e]);
        myFysTLcd.FunV03D(planner.max_feedrate_mm_s[3 + e]);
        myFysTLcd.FunV03D(planner.max_acceleration_mm_per_s2[3 + e]);
    }
    myFysTLcd.FunV02E();
#endif
}
static void FunV04B()
{
    int32_t t;
    myFysTLcd.FunV02C(MACRO_var_V036);
#if EXTRUDERS==5
    if (myFysTLcd.FunV027(36))
    {
        for (uint8_t e = 1; e < 4; e++)
        {
            myFysTLcd.FunV047(planner.axis_steps_per_mm[3 + e]);
            myFysTLcd.FunV047(planner.max_feedrate_mm_s[3 + e]);
            myFysTLcd.FunV044(t);
            planner.max_acceleration_mm_per_s2[3 + e] = t;
        }
    }
    myFysTLcd.ftCmdClear();
    if (myFysTLcd.FunV027(12))
    {
        myFysTLcd.FunV047(planner.axis_steps_per_mm[7]);
        myFysTLcd.FunV047(planner.max_feedrate_mm_s[7]);
        myFysTLcd.FunV044(t);
        planner.max_acceleration_mm_per_s2[7] = t;
    }
#else
    if (myFysTLcd.FunV027((EXTRUDERS - 1) * 12))
    {
        for (uint8_t e = 1; e < EXTRUDERS; e++)
        {
            myFysTLcd.FunV047(planner.axis_steps_per_mm[3 + e]);
            myFysTLcd.FunV047(planner.max_feedrate_mm_s[3 + e]);
            myFysTLcd.FunV044(t);
            planner.max_acceleration_mm_per_s2[3 + e] = t;
        }
    }
#endif
}
static void FunV039()
{
    uint8_t e;
    myFysTLcd.FunV02C(MACRO_var_V038);
#if EXTRUDERS>3
    for (e = 1; e < 3; e++)
    {
        myFysTLcd.FunV03D(PID_PARAM(Kp, e));
        myFysTLcd.FunV03D(unscalePID_i(PID_PARAM(Ki, e)));
        myFysTLcd.FunV03D(unscalePID_d(PID_PARAM(Kd, e)));
    #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.FunV03D(PID_PARAM(Kc, e));
    #else
        myFysTLcd.FunV026(4);
    #endif
    }
    myFysTLcd.FunV02E();
    for (; e < EXTRUDERS; e++)
    {
        myFysTLcd.FunV03D(PID_PARAM(Kp, e));
        myFysTLcd.FunV03D(unscalePID_i(PID_PARAM(Ki, e)));
        myFysTLcd.FunV03D(unscalePID_d(PID_PARAM(Kd, e)));
    #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.FunV03D(PID_PARAM(Kc, e));
    #else
        myFysTLcd.FunV026(4);
    #endif
    }
    myFysTLcd.FunV02E();
#else
    for (e = 1; e < EXTRUDERS; e++)
    {
        myFysTLcd.FunV03D(PID_PARAM(Kp, e));
        myFysTLcd.FunV03D(unscalePID_i(PID_PARAM(Ki, e)));
        myFysTLcd.FunV03D(unscalePID_d(PID_PARAM(Kd, e)));
        #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.FunV03D(PID_PARAM(Kc, e));
        #else
        myFysTLcd.FunV026(4);
        #endif
    }
    myFysTLcd.FunV02E();
#endif
}
static void FunV048()
{
    uint8_t e;
    myFysTLcd.FunV02C(MACRO_var_V038);
#if EXTRUDERS>3
    if (myFysTLcd.FunV027(32))
    {
        for (e = 1; e < 3; e++)
        {
            myFysTLcd.FunV047(PID_PARAM(Kp, e));
            myFysTLcd.FunV047(PID_PARAM(Ki, e));
            PID_PARAM(Ki, e) = scalePID_i(PID_PARAM(Ki, e));
            myFysTLcd.FunV047(PID_PARAM(Kd, e));
            PID_PARAM(Kd, e) = scalePID_d(PID_PARAM(Kd, e));
        #if ENABLED(PID_EXTRUSION_SCALING)
            myFysTLcd.FunV047(PID_PARAM(Kc, e));
        #else
            myFysTLcd.FunV026(4);
        #endif
        }
    }
    myFysTLcd.ftCmdClear();
    if (myFysTLcd.FunV027(32))
    {
        for (; e < EXTRUDERS; e++)
        {
            myFysTLcd.FunV047(PID_PARAM(Kp, e));
            myFysTLcd.FunV047(PID_PARAM(Ki, e));
            PID_PARAM(Ki, e) = scalePID_i(PID_PARAM(Ki, e));
            myFysTLcd.FunV047(PID_PARAM(Kd, e));
            PID_PARAM(Kd, e) = scalePID_d(PID_PARAM(Kd, e));
        #if ENABLED(PID_EXTRUSION_SCALING)
            myFysTLcd.FunV047(PID_PARAM(Kc, e));
        #else
            myFysTLcd.FunV026(4);
        #endif
        }
    }
#else
    if (myFysTLcd.FunV027(16*(EXTRUDERS-1)))
    {
        for (e = 1; e < EXTRUDERS; e++)
        {
            myFysTLcd.FunV047(PID_PARAM(Kp, e));
            myFysTLcd.FunV047(PID_PARAM(Ki, e));
            PID_PARAM(Ki, e) = scalePID_i(PID_PARAM(Ki, e));
            myFysTLcd.FunV047(PID_PARAM(Kd, e));
            PID_PARAM(Kd, e) = scalePID_d(PID_PARAM(Kd, e));
        #if ENABLED(PID_EXTRUSION_SCALING)
            myFysTLcd.FunV047(PID_PARAM(Kc, e));
        #else
            myFysTLcd.FunV026(4);
        #endif
        }
    }
#endif
}
#endif
static void FunV038()
{
    myFysTLcd.FunV02C(MACRO_var_V02B);
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    myFysTLcd.FunV03D(planner.z_fade_height);
#elif ABL_PLANAR 
        myFysTLcd.FunV03D(zprobe_zoffset); 
#else
    myFysTLcd.FunV026(4);
#endif
    myFysTLcd.FunV02E();
#if ABL_PLANAR
    for (char i = 0; i < 9; i++)
    {
        myFysTLcd.FunV03D(planner.bed_level_matrix.matrix[i]);
    }
#else
    myFysTLcd.FunV026(36);
#endif
    myFysTLcd.FunV02E();
}
static void FunV04C()
{
    myFysTLcd.FunV02C(MACRO_var_V02B);
    if (myFysTLcd.FunV027(4))
    {
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        myFysTLcd.FunV047(planner.z_fade_height);
#endif
    }
    if (myFysTLcd.FunV027(36))
    {
#if ABL_PLANAR
        for (char i = 0; i < 9; i++)
        {
            myFysTLcd.FunV047(planner.bed_level_matrix.matrix[i]);
        }
#endif
    }
}
static void FunV046()
{
    myFysTLcd.FunV02C(MACRO_var_V02D);
#if ENABLED(PIDTEMP)
    myFysTLcd.FunV03D(PID_PARAM(Kp, 0));
    myFysTLcd.FunV03D(unscalePID_i(PID_PARAM(Ki, 0))); 
    myFysTLcd.FunV03D(unscalePID_d(PID_PARAM(Kd, 0))); 
    #if ENABLED(PID_EXTRUSION_SCALING)
    myFysTLcd.FunV03D(PID_PARAM(Kc, 0));
    #else
    myFysTLcd.FunV026(4);
    #endif
#else
    myFysTLcd.FunV026(16);
#endif 
    myFysTLcd.FunV02E();
#if DISABLED(PIDTEMPBED)
    myFysTLcd.FunV026(16);
#else
    myFysTLcd.FunV03D(thermalManager.bedKp);
    myFysTLcd.FunV03D(unscalePID_i(thermalManager.bedKi));
    myFysTLcd.FunV03D(unscalePID_d(thermalManager.bedKd));
    myFysTLcd.FunV026(4);
#endif
    myFysTLcd.FunV02E();
    for (char i = 0; i < 2; i++)
    {
        myFysTLcd.FunV031(lcd_preheat_hotend_temp[i]);
    }
    for (char i = 0; i < 2; i++)
    {
        myFysTLcd.FunV031(lcd_preheat_bed_temp[i]);
    }
    for (char i = 0; i < 2; i++)
    {
        myFysTLcd.FunV031(lcd_preheat_fan_speed[i]*100/255);
    }
#if ENABLED(PID_EXTRUSION_SCALING)
    myFysTLcd.FunV037((int32_t)lpq_len);
#else
    myFysTLcd.FunV026(4);
#endif
    myFysTLcd.FunV02E();
}
static void FunV03A()
{
    myFysTLcd.FunV02C(MACRO_var_V02D);
    if (myFysTLcd.FunV027(32))
    {
    #if ENABLED(PIDTEMP)
        myFysTLcd.FunV047(PID_PARAM(Kp, 0));
        myFysTLcd.FunV047(PID_PARAM(Ki, 0));
        PID_PARAM(Ki, 0) = scalePID_i(PID_PARAM(Ki, 0));
        myFysTLcd.FunV047(PID_PARAM(Kd, 0));
        PID_PARAM(Kd, 0) = scalePID_d(PID_PARAM(Kd, 0));
        #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.FunV047(PID_PARAM(Kc, 0));
        #else
        myFysTLcd.FunV026(4);
        #endif
    #else
        myFysTLcd.FunV026(16);
    #endif 
    #if DISABLED(PIDTEMPBED)
        myFysTLcd.FunV026(16);
    #else
        myFysTLcd.FunV047(thermalManager.bedKp);
        myFysTLcd.FunV047(thermalManager.bedKi);
        thermalManager.bedKi = scalePID_i(thermalManager.bedKi);
        myFysTLcd.FunV047(thermalManager.bedKd);
        thermalManager.bedKd = scalePID_d(thermalManager.bedKd);
        myFysTLcd.FunV026(4);
    #endif
    }
    myFysTLcd.ftCmdClear();
    thermalManager.updatePID(); 
    if (myFysTLcd.FunV027(20))
    {
        for (char i = 0; i < 2; i++)
        {
            myFysTLcd.FunV03E(lcd_preheat_hotend_temp[i]);
        }
        for (char i = 0; i < 2; i++)
        {
            myFysTLcd.FunV03E(lcd_preheat_bed_temp[i]);
        }
        for (char i = 0; i < 2; i++)
        {
          int16_t s;
          lcd_preheat_fan_speed[i]=s*255/100;
        }
    #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.FunV044((int32_t)lpq_len);
    #else
        myFysTLcd.FunV026(4);
    #endif
    }
}
static void FunV030()
{
    myFysTLcd.FunV02C(MACRO_var_V02F);
#if ENABLED(FWRETRACT)
    myFysTLcd.FunV031(autoretract_enabled);
    myFysTLcd.FunV03D(retract_length);
    #if EXTRUDERS > 1
    myFysTLcd.FunV03D(retract_length_swap);
    #else
    myFysTLcd.FunV026(4);
    #endif
    myFysTLcd.FunV03D(retract_feedrate_mm_s);
    myFysTLcd.FunV03D(retract_zlift);
    myFysTLcd.FunV03D(retract_recover_length);
    #if EXTRUDERS > 1
    myFysTLcd.FunV03D(retract_recover_length_swap);
    #else
    myFysTLcd.FunV026(4);
    #endif
    myFysTLcd.FunV03D(retract_recover_feedrate_mm_s);
#else
    myFysTLcd.FunV026(30);
#endif 
    myFysTLcd.FunV031(volumetric_enabled);
    myFysTLcd.FunV02E();
    uint8_t e;
    for (e = 0; e<EXTRUDERS; e++)myFysTLcd.FunV03D(filament_size[e]);
    for (; e < 5; e++)myFysTLcd.FunV026(4);
#if ENABLED(LIN_ADVANCE)
    myFysTLcd.FunV03D(planner.extruder_advance_k);
    myFysTLcd.FunV03D(planner.advance_ed_ratio);
#else
    myFysTLcd.FunV026(8);
#endif
    myFysTLcd.FunV02E();
}
static void FunV02F()
{
    myFysTLcd.FunV02C(MACRO_var_V02F);
    if(myFysTLcd.FunV027(32))
    {
        int16_t t;
    #if ENABLED(FWRETRACT)
        myFysTLcd.FunV03E(t);
        autoretract_enabled=t;
        myFysTLcd.FunV047(retract_length);
        #if EXTRUDERS > 1
        myFysTLcd.FunV047(retract_length_swap);
        #else
        myFysTLcd.FunV026(4);
        #endif
        myFysTLcd.FunV047(retract_feedrate_mm_s);
        myFysTLcd.FunV047(retract_zlift);
        myFysTLcd.FunV047(retract_recover_length);
        #if EXTRUDERS > 1
        myFysTLcd.FunV047(retract_recover_length_swap);
        #else
        myFysTLcd.FunV026(4);
        #endif
        myFysTLcd.FunV047(retract_recover_feedrate_mm_s);
    #else
        myFysTLcd.FunV026(30);
    #endif 
        myFysTLcd.FunV03E(t);
        volumetric_enabled = t;
    }
    myFysTLcd.ftCmdClear();
    if(myFysTLcd.FunV027(28))
    {
        uint8_t e;
        for (e = 0; e<EXTRUDERS; e++)myFysTLcd.FunV047(filament_size[e]);
        for (; e < 5; e++)myFysTLcd.FunV026(4);
    #if ENABLED(LIN_ADVANCE)
        myFysTLcd.FunV047(planner.extruder_advance_k);
        myFysTLcd.FunV047(planner.advance_ed_ratio);
    #endif
    }
}
static void FunV02D()
{
    int16_t t;
    myFysTLcd.FunV02C(MACRO_var_V031);
#if ENABLED(HAVE_TMC2130)
    #if ENABLED(X_IS_TMC2130)
    t = stepperX.getCurrent();
    myFysTLcd.FunV031(t);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(Y_IS_TMC2130)
    t=stepperY.getCurrent();
    myFysTLcd.FunV031(t);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(Z_IS_TMC2130)
    t=stepperZ.getCurrent();
    myFysTLcd.FunV031(t);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(E0_IS_TMC2130)
    t=stepperE0.getCurrent();
    myFysTLcd.FunV031(t);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(X2_IS_TMC2130)
    t=stepperX2.getCurrent();
    myFysTLcd.FunV031(t);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(Y2_IS_TMC2130)
    t=stepperY2.getCurrent();
    myFysTLcd.FunV031(t);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(Z2_IS_TMC2130)
    t=stepperZ2.getCurrent();
    myFysTLcd.FunV031(t);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(E1_IS_TMC2130)
    t=stepperE1.getCurrent();
    myFysTLcd.FunV031(t);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(E2_IS_TMC2130)
    t=stepperE2.getCurrent();
    myFysTLcd.FunV031(t);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(E3_IS_TMC2130)
    t=stepperE3.getCurrent();
    myFysTLcd.FunV031(t);
    #else
    myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(E4_IS_TMC2130)
    t=stepperE4.getCurrent();
    myFysTLcd.FunV031(t);
    #else
    myFysTLcd.FunV026(2);
    #endif
#else
    myFysTLcd.FunV026(22);
#endif
    myFysTLcd.FunV02E();
}
static void FunV041()
{
    int16_t t;
    myFysTLcd.FunV02C(MACRO_var_V031);
    if (myFysTLcd.FunV027(22))
    {
#if ENABLED(HAVE_TMC2130)
    #if ENABLED(X_IS_TMC2130)
        myFysTLcd.FunV03E(t)
        stepperX.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(Y_IS_TMC2130)
        myFysTLcd.FunV03E(t)
        stepperY.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(Z_IS_TMC2130)
        myFysTLcd.FunV03E(t)
        stepperZ.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(X2_IS_TMC2130)
        myFysTLcd.FunV03E(t)
        stepperX2.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(Y2_IS_TMC2130)
        myFysTLcd.FunV03E(t)
        stepperY2.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(Z2_IS_TMC2130)
        myFysTLcd.FunV03E(t)
        stepperZ2.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(E0_IS_TMC2130)
        myFysTLcd.FunV03E(t)
        stepperE0.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(E1_IS_TMC2130)
        myFysTLcd.FunV03E(t)
        stepperE1.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(E2_IS_TMC2130)
        myFysTLcd.FunV03E(t)
        stepperE2.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(E3_IS_TMC2130)
        myFysTLcd.FunV03E(t)
        stepperE3.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.FunV026(2);
    #endif
    #if ENABLED(E4_IS_TMC2130)
        myFysTLcd.FunV03E(t)
        stepperE4.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.FunV026(2);
    #endif
#else
        myFysTLcd.FunV026(22);
#endif
    }
}
static void FunV054()
{
    myFysTLcd.FunV02C(MACRO_var_V062);
#ifdef FYS_ENERGY_CONSERVE_HEIGHT
    myFysTLcd.FunV032(zEnergyHeight);
#else
    myFysTLcd.FunV026(2);
#endif
#if PIN_EXISTS(PS_ON)&&defined(FYS_ACTIVE_TIME_OVER)
    int16_t t=max_inactive_time/1000UL;
    myFysTLcd.FunV031(t);
#else
    myFysTLcd.FunV026(2);
#endif
    myFysTLcd.FunV02E();
}
static void FunV055()
{
    myFysTLcd.FunV02C(MACRO_var_V062);
    if (myFysTLcd.FunV027(4))
    {
      #ifdef FYS_ENERGY_CONSERVE_HEIGHT
        myFysTLcd.FunV043(zEnergyHeight);
      #else
        myFysTLcd.FunV026(2);
      #endif
      #if PIN_EXISTS(PS_ON)&&defined(FYS_ACTIVE_TIME_OVER)
        int16_t t;
        myFysTLcd.FunV03E(t);
        max_inactive_time = (millis_t)t* 1000UL;
      #else
        myFysTLcd.FunV026(2);
      #endif
    }
}
static void firstScreenData()
{
    movDis = 0.1;
    movFeedrate = 30.0; 
    int16_t tval = movDis*FYSTLCD_DOT_TEN_MUL;
    myFysTLcd.FunV02C(VARADDR_MOVE_DIS_SIGN);
    myFysTLcd.FunV031(tval);
    myFysTLcd.FunV02E();
    tval = movFeedrate*FYSTLCD_DOT_TEN_MUL;
    myFysTLcd.FunV02C(VARADDR_MOVE_SPEED_SIGN);
    myFysTLcd.FunV031(tval);
    myFysTLcd.FunV02E();
    #if ENABLED(FYS_LCD_EVENT)
    GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V004);
    #endif
    FysTLcd::FunV015(VARADDR_VERSION_DATE, __DATE__, ATTACH_STR_LEN);
  #ifdef FYS_WIFI_ESP3D || FYS_WIFI_HLK
    char message[2] = { 0 };
    FysTLcd::FunV015(VARADDR_WIFI_IP, message, ATTACH_STR_LEN);
    FysTLcd::FunV015(VARADDR_WIFI_SSID, message, ATTACH_STR_LEN);
    myFysTLcd.FunV02C(VARADDR_STATUS_WIFI);
    myFysTLcd.FunV031(0);
    myFysTLcd.FunV02E();
    #ifdef FYS_WIFI_ESP3D 
      message[0] = MYSERIAL.activeTxSerial;
      MYSERIAL.setTxActiveSerial(1);   
      SERIAL_ECHOLNPGM("[ESP103]AP"); 
      SERIAL_ECHOLNPGM("[ESP444]RESTART");
      MYSERIAL.setTxActiveSerial(message[0]);
    #endif
  #endif
}
void dwin_popup(const char* msg, char pageChoose, char funid)
{
    char str[INFO_POPUP_LEN + 1], i, j, ch;
    if (msg)
    for (j = 0; j < INFOS_NUM; j++)
    {
        memset(str, 0, INFO_POPUP_LEN);
        i = 0;
        while (ch = pgm_read_byte(msg))
        {
            if (ch == '\n')
            {
                msg++;
                break;
            }
            str[i++] = ch;
            msg++;
            if (i >= INFO_POPUP_LEN)break;
        }
        FysTLcd::FunV015(MACRO_var_V0A0[j], str, INFO_POPUP_LEN);
    }
    switch (pageChoose)
    {
    case 0:
        #if FYSTLCD_PAGE_EXIST(INFO_POPUP)
        lcd_popPage(FTPAGE(INFO_POPUP));
        #endif
        break;
    case 1:
        #if FYSTLCD_PAGE_EXIST(INFO_WAITING)
        lcd_popPage(FTPAGE(INFO_WAITING));
        #endif
        break;
    case 2:
        switch (funid)
        {
        #if ENABLED(ADVANCED_PAUSE_FEATURE)
        case 1:
            optionId = 1;
            advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_WAIT_FOR;
            FysTLcd::ftPutsPGM(VARADDR_QUESTION_LEFT, PSTR("Resume"),8);
            FysTLcd::ftPutsPGM(VARADDR_QUESTION_RIGHT, PSTR("Extrude"),8);
            break;
        #endif
        default:
            break;
        }
        #if FYSTLCD_PAGE_EXIST(INFO_OPTION)
        lcd_popPage(FTPAGE(INFO_OPTION));
        #endif
        break;
    default:
        break;
    }
}
void FunV052()
{
    if (
        #if FYSTLCD_PAGE_EXIST(INFO_POPUP)
        GLOBAL_var_V00C == FTPAGE(INFO_POPUP) || 
        #endif
        #if FYSTLCD_PAGE_EXIST(INFO_WAITING)
        GLOBAL_var_V00C == FTPAGE(INFO_WAITING) ||
        #endif
        #if FYSTLCD_PAGE_EXIST(INFO_OPTION)
        GLOBAL_var_V00C == FTPAGE(INFO_OPTION) ||
        #endif
        0)lcd_popPage(GLOBAL_var_V00D);
}
void FunV029()
{
    lcd_popPage(0x01);
    FysTLcd::FunV028(0x00, 0x02, 0xFF);
    delay(100);
}
void lcd_shutDown()
{
    FysTLcd::FunV028(0x05, 0x02, 0xFF);
    lcd_setPage(0x00);
}
void lcd_setstatus(const char* message, const bool persist)
{
    (void)persist;
  #ifdef FYS_WIFI_ESP3D
    while (*message == ' ')message++;
    bool tf = false;
    const char*t;
    char n, d;
    for (n = 0, d = 0, t = message; (*t >= '0'&&*t <= '9') || *t == '.'; t++)
    {
        if (*t == '.')
        {
            if (tf)
            {
                d++;
                tf = false;
            }
            else break;
        }
        else
        {
            if (!tf)
            {
                tf = true;
                n++;
            }
        }
    }
    if (n == 4 && d == 3)
    {
        FysTLcd::FunV015(VARADDR_WIFI_IP, message, ATTACH_STR_LEN);
        return;
    }
    const char* str = "SSID";
    for (t = message; *t; t++)
    {
        if (*t == str[0])
        {
            const char *r=t;
            for(d=0;*r&&d<4;r++,d++)
            if (*r != str[d])break;
            if (d == 4)
            {
                t = r;
                t++;
                break;
            }
        }
    }
    if (*t)
    {
        FysTLcd::FunV015(VARADDR_WIFI_SSID, t, ATTACH_STR_LEN);
        myFysTLcd.FunV02C(VARADDR_STATUS_WIFI);
        myFysTLcd.FunV031(1);
        myFysTLcd.FunV02E();
        return;
    }
#else
    (void)message;
#endif
}
#if HAS_BED_PROBE 
  void lcd_zstep_zoffset(bool dir) {
    int16_t babystep_increment=0;
    float new_zoffset=0;
    if(dir) { 
      babystep_increment = (int32_t)(movDis /planner.steps_to_mm[Z_AXIS]);
      new_zoffset = zprobe_zoffset + movDis;
    }
    else {
      babystep_increment -= (int32_t)(movDis /planner.steps_to_mm[Z_AXIS]);
      new_zoffset = zprobe_zoffset -movDis;
    }
    if (WITHIN(new_zoffset, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) {
      if(dir) current_position[Z_AXIS] += movDis;
      else current_position[Z_AXIS] -= movDis;
      planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS],  current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
      stepper.synchronize();   
      zprobe_zoffset = new_zoffset;
    }
  }
#endif
#if ENABLED(BABYSTEPPING)
  void _lcd_babystep(const AxisEnum axis, const char* msg) {
    UNUSED(msg);
    const int16_t babystep_increment = (int32_t)10 * (BABYSTEP_MULTIPLICATOR);
    thermalManager.babystep_axis(axis, babystep_increment);
  }
  #if ENABLED(BABYSTEP_XY)
    void _lcd_babystep_x() { _lcd_babystep(X_AXIS, PSTR(MSG_BABYSTEPPING_X)); }
    void _lcd_babystep_y() { _lcd_babystep(Y_AXIS, PSTR(MSG_BABYSTEPPING_Y)); }
    void lcd_babystep_x() { _lcd_babystep_x();}
    void lcd_babystep_y() { _lcd_babystep_y();}
  #endif
  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
    void lcd_babystep_zoffset(bool dir) {
      int16_t babystep_increment=0;
      float new_zoffset=0;
      if(dir) 
      {
          babystep_increment = (int32_t)(movDis /planner.steps_to_mm[Z_AXIS]);
          new_zoffset = zprobe_zoffset + movDis;
      }
      else
      {
          babystep_increment -= (int32_t)(movDis /planner.steps_to_mm[Z_AXIS]);
          new_zoffset = zprobe_zoffset -movDis;
      }
      if (WITHIN(new_zoffset, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) {
              thermalManager.babystep_axis_force(Z_AXIS, babystep_increment); 
              SERIAL_ECHOLNPAIR("bi:",babystep_increment);
              SERIAL_ECHOLNPAIR("btd:",thermalManager.babystepsTodo[Z_AXIS]);
          zprobe_zoffset = new_zoffset; 
      }
    }
  #else 
    void _lcd_babystep_z() { _lcd_babystep(Z_AXIS, PSTR(MSG_BABYSTEPPING_Z)); }
    void lcd_babystep_z() { _lcd_babystep_z();}
  #endif 
#endif 
