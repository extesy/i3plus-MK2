#include "Marlin.h"
#if ENABLED(USE_WATCHDOG)
#include "watchdog.h"
void watchdog_init() {
  #if ENABLED(WATCHDOG_RESET_MANUAL)
    wdt_reset();
    _WD_CONTROL_REG = _BV(_WD_CHANGE_BIT) | _BV(WDE);
    _WD_CONTROL_REG = _BV(WDIE) | WDTO_4S;
  #else
    wdt_enable(WDTO_4S);
  #endif
}
#if ENABLED(WATCHDOG_RESET_MANUAL)
  ISR(WDT_vect) {
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM("Something is wrong, please turn off the printer.");
    #if ENABLED(FYS_LCD_EXTRA_INFO)
      FunV006("Something is wrong, please turn off the printer.");
    #endif
    kill(PSTR("ERR:Please Reset")); 
    while (1); 
  }
#endif 
#endif 
