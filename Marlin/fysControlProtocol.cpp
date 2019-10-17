#include "fysControlProtocol.h"
#ifdef FYS_CONTROL_PROTOCOL
#include "language.h"
#include "cardreader.h"
#include "temperature.h"
#include "stepper.h"
#include "configuration_store.h"
#include "endstops.h"
#include "ultralcd.h"
#if ENABLED(USE_WATCHDOG)
#include "watchdog.h"
#endif
#define RX_CMD_BUFFER_LEN   0x200
struct CMDFram
{
    uint16_t sign;
    uint16_t func;
    int32_t size;
    union
    {
        uint16_t devStatus;
        struct
        {
            uint32_t dat;
            uint32_t ord;
        }dev;
        struct
        {
            uint32_t len;
            uint32_t pos;
        }file;
        char dp[8];
    }data;
}cfram;
#define CMDFRAM_FUN_LEN     8 
BufferObject<char>rbuf(RX_CMD_BUFFER_LEN);
uint8_t*cmdp = (uint8_t*)&cfram;
#define CMD_HEAD_SIGN                   0xA00A
#define CMD_REG_APP_ID                  0x0001
#define CMD_REG_DEV_ID                  0x0002
#define CMD_HEART_HIT                   0x0003
#define CMD_GET_FILE_LIST               0x0004
#define CMD_GET_FILE_MODE               0x0005
#define CMD_GET_FILE_CONTENT            0x0006
#define CMD_START_PRINT                 0x0007 
#define CMD_PAUSE_PRINT                 0x0008
#define CMD_STOP_PRINT                  0x0009
#define CMD_CONTINUE_PRINT              0x000A
#define CMD_DEV_STATUS                  0x000B 
#define CMD_ERR_OPEN_FILE_FAIL          0x8001
#define CMD_ERR_NO_FILES                0x8002
#define CMD_ERR_NEED_ID                 0x8003
#define CMD_ERR_BEEN_CONNECTED_BY_OTHER 0x8004
#define CMD_ERR_NEED_APP_APOINT         0x8005
#define CMD_ERR_FILE_END                0x8006
void firstSerlCheck()
{
    devStatus.all = 0;
    card.cardOK = false;
    card.initsd();
    planner.set_position_mm(0, 0, 0, 0);
    planner.buffer_line(30, 30, 10, 0, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
    stepper.synchronize();
    int tempEStart = thermalManager.degHotend(0), tempENow;
    devStatus.detail.temperatureHotendSensorFail = tempEStart < HEATER_0_MINTEMP;
#if HAS_TEMP_BED
    int tempBStart = thermalManager.degBed(), tempBNow;
    devStatus.detail.temperatureBedSensorFail = tempBStart < BED_MINTEMP;
#endif
    if (!devStatus.detail.temperatureHotendSensorFail)thermalManager.setTargetHotend(210, 0);
#if HAS_TEMP_BED
    if (!devStatus.detail.temperatureBedSensorFail)thermalManager.setTargetBed(100);
#endif
#if ENABLED(USE_WATCHDOG)
    watchdog_reset();
#endif
    planner.set_position_mm(0, 0, 0, 0);
    endstops.enable(true); 
    current_position[X_AXIS] = 1.5 * X_MAX_LENGTH * (X_HOME_DIR);
    current_position[Y_AXIS] = 1.5 * Y_MAX_LENGTH * (Y_HOME_DIR);
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], 0, 0, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
    stepper.synchronize();
    current_position[Z_AXIS] = 1.5 * Z_MAX_LENGTH * (Z_HOME_DIR);
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], 0, MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
    stepper.synchronize();
    endstops.hit_on_purpose();
#if X_HOME_DIR<0
    if (!(READ(X_MIN_PIN) ^ X_MIN_ENDSTOP_INVERTING))devStatus.detail.X_moveFail = 1;
#else
    if (!(READ(X_MAX_PIN) ^ X_MAX_ENDSTOP_INVERTING))devStatus.detail.X_moveFail = 1;
#endif
#if Y_HOME_DIR<0
    if (!(READ(Y_MIN_PIN) ^ Y_MIN_ENDSTOP_INVERTING))devStatus.detail.Y_moveFail = 1;
#else
    if (!(READ(Y_MAX_PIN) ^ Y_MAX_ENDSTOP_INVERTING))devStatus.detail.Y_moveFail = 1;
#endif
#if Z_HOME_DIR<0
    if (!(READ(Z_MIN_PIN) ^ Z_MIN_ENDSTOP_INVERTING))devStatus.detail.Z_moveFail = 1;
#else
    if (!(READ(Z_MAX_PIN) ^ Z_MAX_ENDSTOP_INVERTING))devStatus.detail.Z_moveFail = 1;
#endif
    planner.set_position_mm(0, 0, 0, 0);
    planner.buffer_line(0, 0, 10, 0, MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
    stepper.synchronize();
    planner.buffer_line(50, 50, 10, 0, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
    stepper.synchronize();
#if X_HOME_DIR<0
    if (READ(X_MIN_PIN) ^ X_MIN_ENDSTOP_INVERTING)devStatus.detail.X_moveFail = 1;
#else
    if (READ(X_MAX_PIN) ^ X_MAX_ENDSTOP_INVERTING)devStatus.detail.X_moveFail = 1;
#endif
#if Y_HOME_DIR<0
    if (READ(Y_MIN_PIN) ^ Y_MIN_ENDSTOP_INVERTING)devStatus.detail.Y_moveFail = 1;
#else
    if (READ(Y_MAX_PIN) ^ Y_MAX_ENDSTOP_INVERTING)devStatus.detail.Y_moveFail = 1;
#endif
#if Z_HOME_DIR<0
    if (READ(Z_MIN_PIN) ^ Z_MIN_ENDSTOP_INVERTING)devStatus.detail.Z_moveFail = 1;
#else
    if (READ(Z_MAX_PIN) ^ Z_MAX_ENDSTOP_INVERTING)devStatus.detail.Z_moveFail = 1;
#endif
    card.initsd();
    if (!card.cardOK)devStatus.detail.SD_CardFail = 1;
    tempENow = thermalManager.degHotend(0);
    if (tempENow - tempEStart < 3)devStatus.detail.hotHotendFail = 1;
#if HAS_TEMP_BED
    tempBNow = thermalManager.degBed();
    if (tempBNow - tempBStart < 3)devStatus.detail.hotBedFail = 1;
#endif
    cli();
    disable_all_steppers();
    thermalManager.disable_all_heaters();
    delay(1000);
    devStatus.detail.checked = 1;
    thermalManager.disable_all_heaters();
    sei();
    rbuf.clear();
#if 1
    if (devStatus.detail.checked)MYSERIAL.print("Check!\n");
    MYSERIAL.print(tempEStart);
    if (devStatus.detail.hotHotendFail)MYSERIAL.print(" Hotend hot Fail ");
    else MYSERIAL.print(" Hotend hot OK ");
    MYSERIAL.println(tempENow);
    if (devStatus.detail.temperatureHotendSensorFail)MYSERIAL.print("Hotend temperature sensor Fail.\n");
    else MYSERIAL.print("Hotend temperature sensor OK.\n");
#if HAS_TEMP_BED
    MYSERIAL.print(tempBStart);
    if (devStatus.detail.hotBedFail)MYSERIAL.print(" Bed hot Fail ");
    else MYSERIAL.print(" Bed hot OK ");
    MYSERIAL.println(tempBNow);
    if (devStatus.detail.temperatureBedSensorFail)MYSERIAL.print("Bed temperature sensor Fail.\n");
    else MYSERIAL.print("Bed temperature sensor OK.\n");
#endif
    if (devStatus.detail.SD_CardFail)MYSERIAL.print("Sd card Fail.\n");
    else MYSERIAL.print("Sd card OK.\n");
    if (devStatus.detail.X_moveFail)MYSERIAL.print("X move Fail.\n");
    else MYSERIAL.print("X move OK.\n");
    if (devStatus.detail.Y_moveFail)MYSERIAL.print("Y move Fail.\n");
    else MYSERIAL.print("Y move OK.\n");
    if (devStatus.detail.Z_moveFail)MYSERIAL.print("Z move Fail.\n");
    else MYSERIAL.print("Z move OK.\n");
#endif
}
static void uartSend(uint8_t*dt, int size)
{
    MYSERIAL.setTxActiveSerial(1);
    while (size > 0)
    {
        MYSERIAL.write(*dt);
        dt++;
        size--;
    }
    MYSERIAL.setTxActiveSerial(0);
}
void fysControlProtocol()
{
    static char step = 0;
    static int cpos = 0, clen = CMDFRAM_FUN_LEN;
    static millis_t tpass = millis(), tnow = tpass;
    if (tpass > millis())tpass = millis();
    MYSERIAL.setRxActiveSerial(1);
    if (MYSERIAL.available() > 0)
    {
        while (MYSERIAL.available() > 0 && cpos < clen)
        {
            switch (step)
            {
            case 0:
                cmdp[cpos] = uint8_t(MYSERIAL.read());
                break;
            case 1:
              #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
                GLOBAL_var_V004[cpos] = uint8_t(MYSERIAL.read());
              #endif
                break;
            case 2:
                rbuf.store(MYSERIAL.read());
                break;
            }
            _delay_ms(2);
            cpos++;
        }
        tpass = millis();
    }
    MYSERIAL.setRxActiveSerial(0);
    if (cpos == clen || millis() - tpass > 500)
    {
        switch (step)
        {
        case 0:
            if (cfram.sign == CMD_HEAD_SIGN)
                switch (cfram.func)
            {
                case CMD_DEV_STATUS:
                    cfram.size = 2;
                    cfram.data.devStatus = devStatus.all;
                    uartSend(cmdp, cfram.size + sizeof(cfram.data));
                    break;
                case CMD_START_PRINT:
                    clear_command_queue();
                    print_job_timer.start();
                    step = 1;
                    clen = cfram.size > (FILENAME_LENGTH - 1) ? FILENAME_LENGTH - 1 : cfram.size;
                    break;
                case CMD_GET_FILE_CONTENT:
                    step = 2;
                    clen = cfram.size;
                    break;
            }
            memset(cmdp, 0, sizeof(CMDFram));
            break;
        case 1:
            step = 0;
            clen = CMDFRAM_FUN_LEN;
            #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
            GLOBAL_var_V004[cpos] = 0;
            #endif
            MYSERIAL.print("Start print:");
            #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
            MYSERIAL.println(GLOBAL_var_V004);
            #endif
            #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
            lcd_setstatus(GLOBAL_var_V004);
            #endif
            break;
        case 2:
            step = 0;
            clen = CMDFRAM_FUN_LEN;
            break;
        }
        cpos = 0;
        tpass = millis();
    }
    while (MYSERIAL.space() > 0 && rbuf.available() > 0)
    {
        char c = rbuf.Out();
        MYSERIAL.uart0Send(c);
        MYSERIAL.storeCharToUart0(c);
    }
    if (millis()-tnow>1000&&print_job_timer.isRunning() && step == 0 && cpos == 0 && rbuf.space() > 0x20)
    {
        tnow = millis();
        cfram.sign = CMD_HEAD_SIGN;
        cfram.func = CMD_GET_FILE_CONTENT;
        cfram.size = rbuf.space() - 1;
        uartSend(cmdp, CMDFRAM_FUN_LEN);
    }
}
#endif
