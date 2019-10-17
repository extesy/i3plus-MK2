#include "FysTLcd.h"
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
#define     VARADDR_STATUS_SD                       0x1070  
#define     VARADDR_STATUS_AXIS_LOCK                0x1071  
#define     VARADDR_STATUS_FAN                      0x1072  
#define     VARADDR_STATUS_SERVO                    0x1073  
#define     VARADDR_STATUS_WIFI                     0x1074  
#else
#define     VARADDR_STATUS_SD                       0x0070  
#define     VARADDR_STATUS_AXIS_LOCK                0x0071  
#define     VARADDR_STATUS_FAN                      0x0072  
#define     VARADDR_STATUS_SERVO                    0x0073  
#define     VARADDR_STATUS_PID                      0x0074  
#define     VARADDR_STATUS_WIFI                     0x0075  
#endif
#define     INFOS_NUM                               4
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
#define     MACRO_var_V03E                     0x10A0  
#define     MACRO_var_V040                  0x10D0  
#define     MACRO_var_V039                     0x1100  
#define     VARADDR_ZOFFSET_DATA                        0x1110  
#define     MACRO_var_V03A           0x1120  
#if 1 
#define     VARADDR_ACTIVE_EXTRUDER_HOTEND          0x1121
#define     VARADDR_ACTIVE_EXTRUDER_BED             0x112A
#endif
#define     MACRO_var_V03C                      0x1130  
static const uint16_t MACRO_var_V0A0[INFOS_NUM] = { 0x1180, 0x1190, 0x11A0, 0x11B0 };
#elif FYSTLCD_VER==FYSTLCD_VER_2016
#define     MACRO_var_V03E                     0x00A0  
#define     MACRO_var_V040                  0x00D0  
#define     MACRO_var_V039                     0x0100 
#define     VARADDR_ZOFFSET_DATA                    0x0110  
#define     MACRO_var_V03A           0x0120 
#if 1 
#define     VARADDR_ACTIVE_EXTRUDER_HOTEND          0x0121
#define     VARADDR_ACTIVE_EXTRUDER_BED             0x012A
#endif
#define     MACRO_var_V03C                      0x0130  
static const uint16_t MACRO_var_V0A0[INFOS_NUM] = { 0x0180, 0x0190, 0x01A0, 0x01B0 };
#endif
#define     INFO_POPUP_LEN                          0x20
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
#define     MACRO_var_V027                      0x1200  
#define     MACRO_var_V029                     0x1280  
#define     MACRO_var_V02B                  0x1380  
#define     MACRO_var_V02D                      0x1400  
#define     MACRO_var_V02F                  0x1481  
#define     MACRO_var_V031                   0x1500  
#define     MACRO_var_V034          0x1540  
#define     MACRO_var_V036           0x1560  
#define     MACRO_var_V038            0x1580  
#define     MACRO_var_V062                    0x15A0  
#elif FYSTLCD_VER==FYSTLCD_VER_2016
#define     MACRO_var_V027                      0x0200  
#define     MACRO_var_V029                     0x0280  
#define     MACRO_var_V02B                  0x0380  
#define     MACRO_var_V02D                      0x0400  
#define     MACRO_var_V02F                  0x0481  
#define     MACRO_var_V031                   0x0500  
#define     MACRO_var_V034          0x0540  
#define     MACRO_var_V036           0x0560  
#define     MACRO_var_V038            0x0580  
#define     MACRO_var_V062                    0x05A0  
#endif
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
#define     MACRO_var_V068                            0x1700  
#else
#define     MACRO_var_V068                            0x0700  
#endif
#define     MACRO_var_V069              0x0001  
#define     MACRO_var_V06A              0x0002  
#define     MACRO_var_V068B               0x0003  
#define     MACRO_var_V06C                    0x0004  
#define     MACRO_var_V06D                      0x0005  
#define     MACRO_var_V06E                      0x0006  
#define     MACRO_var_V06F                      0x0007  
#define     MACRO_var_V070                    0x0008  
#define     MACRO_var_V071                    0x0009  
#define     MACRO_var_V072                 0x000A  
#define     MACRO_var_V073                 0x000B  
#define     MACRO_var_V074                        0x000C  
#define     MACRO_var_V075             0x000D  
#define     MACRO_var_V076                     0x000E  
#define     MACRO_var_V077                   0x000F  
#define     MACRO_var_V078                  0x0010  
#define     MACRO_var_V079                0x0011  
#define     MACRO_var_V07A                 0x0012  
#define     MACRO_var_V07B                  0x0013  
#define     MACRO_var_V07C             0x0014  
#define     MACRO_var_V07D          0x0015  
#define     MACRO_var_V07E               0x0016  
#define     MACRO_var_V07F         0x0017  
#define     MACRO_var_V080             0x0018  
#define     MACRO_var_V081          0x0019  
#define     MACRO_var_V082          0x001A  
#define     MACRO_var_V083          0x001B  
#define     MACRO_var_V084          0x001C  
#define     MACRO_var_V085       0x001D  
#define     MACRO_var_V086       0x001E  
#define     MACRO_var_V087                  0x0024  
#define     MACRO_var_V088                 0x0025  
#define     MACRO_var_V089                  0x0026  
#define     MACRO_var_V08A                 0x0027  
#define     MACRO_var_V08B                  0x0028  
#define     MACRO_var_V08C                 0x0029  
#define     MACRO_var_V08D                  0x002B  
#define     MACRO_var_V08E                 0x002A  
#define     MACRO_var_V08F              0x002E  
#define     VARVAL_TOOL_ENTER_PAPER_HEIGHT          0x0031
#define     VARVAL_TOOL_RESET                       0x0032  
#define     VARVAL_TOOL_HOME_XY                     0x0033  
#define     VARVAL_TOOL_OPTION_LEFT                 0x0034  
#define     VARVAL_TOOL_OPTION_RIGHT                0x0035  
#define     VARVAL_TOOL_CONNECT_WIFI                0x0036  
#define     VARVAL_TOOL_COOLDOWN_HOTEND             0x0037  
#define     VARVAL_TOOL_COOLDOWN_BED                0x0038  
#define     VARVAL_TOOL_EMERGENCY_STOP_MOTOR        0x0039  
#define     VARVAL_TOOL_WIFI_UPDATE                 0x0040  
#define     VARVAL_TOOL_LEVELING_ZOFFSET_UP_Z       0x0041  
#define     VARVAL_TOOL_LEVELING_ZOFFSET_DOWN_Z     0x0042  
#define     VARVAL_TOOL_LEVELING_ADJUST_ZOFFSET     0x0043  
#define     VARVAL_TOOL_LEVELING_ADJUST_ZOFFSET_CONFIRM 0x0044  
#define     VARVAL_TOOL_LEVELING_ADJUST_ZOFFSET_BACK    0x0045     
#define     VARVAL_TOOL_BABYSTEP_UP_Z               0x0046  
#define     VARVAL_TOOL_BABYSTEP_DOWN_Z             0x0047  
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
#define     MACRO_var_V026                           0x1701  
#elif FYSTLCD_VER==FYSTLCD_VER_2016
#define     MACRO_var_V026                           0x0701  
#endif
#define     MACRO_var_V028                   0x0001  
#define     MACRO_var_V02C          0x0002  
#define     MACRO_var_V02A            0x0003  
static const uint16_t MACRO_var_V02E[] = { 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x0009, 0x000A, 0x000B, 0x000C, 0x000D };
#define     MACRO_var_V030               0x0021  
#define     MACRO_var_V032                      0x0022  
#define     MACRO_var_V033                       0x0023  
#define     MACRO_var_V035                 0x0024  
#define     MACRO_var_V037                 0x0025  
#define     MACRO_var_V03B                   0x0026  
#define     MACRO_var_V03D                     0x0027  
#define     VARVAL_PRINT_CONFIRM                    0x0028  
#define     VARVAL_PRINT_SD_REFRESH                 0x0029  
#define     VARVAL_PRINT_REPRINT                    0x002A  
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
#define     MACRO_var_V044                         0x1702  
#elif FYSTLCD_VER==FYSTLCD_VER_2016
#define     MACRO_var_V044                         0x0702  
#endif
#define     MACRO_var_V045              0x0001  
#define     MACRO_var_V046              0x0002  
#define     MACRO_var_V047               0x0003  
#define     MACRO_var_V048           0x0004  
#define     MACRO_var_V049           0x0005  
#define     MACRO_var_V04A            0x0006  
#define     MACRO_var_V04B               0x0007  
#define     MACRO_var_V04C               0x0008  
#define     MACRO_var_V04D                0x0009  
#define     MACRO_var_V04E           0x000A  
#define     MACRO_var_V04F           0x000B  
#define     MACRO_var_V050            0x000C  
#define     MACRO_var_V051            0x000D  
#define     MACRO_var_V052            0x000E  
#define     MACRO_var_V053             0x000F  
#define     MACRO_var_V054                    0x00C0  
#define     MACRO_var_V055                   0x00C1  
#define     MACRO_var_V063               0x00C2  
#define     MACRO_var_V064                  0x00C3  
#define     MACRO_var_V065            0x00C4  
#define     MACRO_var_V066             0x00C5  
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
#define     MACRO_var_V091                       0x1703  
#elif FYSTLCD_VER==FYSTLCD_VER_2016
#define     MACRO_var_V091                       0x0703  
#endif
#define     MACRO_var_V092           0x0001  
#define     MACRO_var_V093           0x0002  
#define     MACRO_var_V094            0x0003  
#define     MACRO_var_V095            0x0004  
#define     MACRO_var_V096            0x0005  
#define     MACRO_var_V097             0x0006  
#define     MACRO_var_V098             0x0007  
#define     MACRO_var_V099             0x0008  
#define     MACRO_var_V09A              0x0009  
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
#define     MACRO_var_V09C                        0x1704  
#define     MACRO_var_V09D                      0x1705  
#define     MACRO_var_V09E               0x1706  
#define     MACRO_var_V09F            0x1707  
#define     VARADDR_JUMP_PAGE                       0x1710  
#elif FYSTLCD_VER==FYSTLCD_VER_2016
#define     MACRO_var_V09C                        0x0704  
#define     MACRO_var_V09D                      0x0705  
#define     MACRO_var_V09E               0x0706  
#define     MACRO_var_V09F            0x0707  
#define     VARADDR_JUMP_PAGE                       0x0710  
#endif
#if 1 
#define     VARADDR_TUNE_PRINT_PERCENTAGE           0x1711  
#define     VARADDR_TUNE_FAN_SPEED                  0x1712  
#define     VARADDR_TUNE_HOTEND_TEMP                0x1713  
#define     VARADDR_TUNE_BED_TEMP                   0x1714  
#endif
#if FYSTLCD_VER==FYSTLCD_VER_T5V1
#define     VARADDR_MOVE_DIS_SIGN                   0x1721  
#define     VARADDR_MOVE_SPEED_SIGN                 0x1722
#define     VARADDR_QUESTION_LEFT                   0x1730  
#define     VARADDR_QUESTION_RIGHT                  0x1738  
#define     VARADDR_WIFI_SSID                       0x1748  
#define     VARADDR_WIFI_PASSWORD                   0x1750  
#define     VARADDR_WIFI_IP                         0x1758  
#define     VARADDR_WIFI_DEV_MAC                    0x1760  
#define     VARADDR_VERSION_DATE                    0x1768  
#elif FYSTLCD_VER==FYSTLCD_VER_2016
#define     VARADDR_MOVE_DIS_SIGN                   0x0721  
#define     VARADDR_MOVE_SPEED_SIGN                 0x0722
#define     VARADDR_QUESTION_LEFT                   0x0730  
#define     VARADDR_QUESTION_RIGHT                  0x0738  
#define     VARADDR_WIFI_SSID                       0x0748  
#define     VARADDR_WIFI_PASSWORD                   0x0750  
#define     VARADDR_WIFI_IP                         0x0758  
#define     VARADDR_WIFI_DEV_MAC                    0x0760  
#define     VARADDR_VERSION_DATE                    0x0768  
#endif
#define     ATTACH_STR_LEN                          0x10    
  #if FYSTLCD_VER==FYSTLCD_VER_T5V1
    static const uint16_t VARADDR_FILES_NAME[] = { 0x1010, 0x1018, 0x1020, 0x1028 };
  #elif FYSTLCD_VER==FYSTLCD_VER_2016
    static const uint16_t VARADDR_FILES_NAME[] = { 0x0010, 0x0018, 0x0020, 0x0028 };
  #endif
#define     PAGENUM_MAIN                            0x45
#define     PAGENUM_INFO_POPUP                      0x63
#define     PAGENUM_INFO_WAITING                    0x76
#define     PAGENUM_INFO_OPTION                     0x77 
#define     PAGENUM_AUTO_LEVELING_COMPLETE          0x5F
#define     PAGENUM_LEVELING_METHOD         0x72
#define     PAGENUM_TUNE_PID                        0
#define     PAGENUM_PRINT                           0x49
#define     PAGENUM_PRINTFILE_CONFIRM               0
#define     PAGENUM_ACCOMPLISH_PRINT                0
#define     PAGENUM_FILELIST                        0x47
#define     PAGENUM_TUNE                            0x4B
#define     PAGENUM_SETTING_MOTOR                   0x5D
#define     PAGENUM_SETTING_TEMP                    0x5B
#define     PAGENUM_MANUAL_LEVELING                 0x70
#define     PAGENUM_ASK_RESUM_PRINT                 0x61
#define     PAGENUM_POPUP_SHUTDOWN_WARNING          0
#define     PAGENUM_SETTING_EXTRUDERS_OFFSET        0
#define     PAGENUM_SETTING_EXTRUDERS_MOTOR         0
#define     PAGENUM_SETTING_EXTRUDERS_TEMP          0
#define     PAGENUM_SETTING_LEVELING                0
#define     PAGENUM_SETTING_TMC2130                 0
#define     PAGENUM_SETTING_MATERIAL                0
#define     PAGENUM_SETTING_SYSTEM                  0
#define     PAGENUM_TOOL_CALIBRATION                0
#define     PAGENUM_TOOL_PAPERHEIGHT                0
#define     FYSTLCD_PAGE_EXIST(x)   (defined(PAGENUM_##x)&&PAGENUM_##x>0)
#define     FTPAGE(x)   PAGENUM_##x
static const uint8_t FILES_NUM = sizeof(VARADDR_FILES_NAME) / sizeof(VARADDR_FILES_NAME[0]);
