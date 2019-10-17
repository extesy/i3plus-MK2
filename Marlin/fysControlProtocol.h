#ifndef FYS_CONTROL_PROTOCOL_H
#define FYS_CONTROL_PROTOCOL_H
#include "Marlin.h"
#ifdef FYS_CONTROL_PROTOCOL
#include "yTemplate.h"
bool _enqueuecommand(const char* cmd, bool say_ok = false);
union DevStatus
{
    struct
    {
        uint8_t checked : 1;
        uint8_t temperatureHotendSensorFail : 1;
        uint8_t hotHotendFail : 1;
        uint8_t ifNoBed : 1;
        uint8_t temperatureBedSensorFail : 1;
        uint8_t hotBedFail : 1;
        uint8_t X_moveFail : 1;
        uint8_t Y_moveFail : 1;
        uint8_t Z_moveFail : 1;
        uint8_t SD_CardFail : 1;
    }detail;
    uint16_t all;
};
extern DevStatus devStatus;
void fysControlProtocol();
void firstSerlCheck();
#endif 
#endif 
