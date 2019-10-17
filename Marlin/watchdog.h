#ifndef WATCHDOG_H
#define WATCHDOG_H
#include "Marlin.h"
#include <avr/wdt.h>
void watchdog_init();
inline void watchdog_reset() { wdt_reset(); }
#endif
