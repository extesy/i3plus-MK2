#include "Marlin.h"
#include "printcounter.h"
#include "duration_t.h"
PrintCounter::PrintCounter(): super() {
  this->loadStats();
}
millis_t PrintCounter::deltaDuration() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("deltaDuration"));
  #endif
  millis_t tmp = this->lastDuration;
  this->lastDuration = this->duration();
  return this->lastDuration - tmp;
}
bool PrintCounter::isLoaded() {
  return this->loaded;
}
void PrintCounter::incFilamentUsed(double const &amount) {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("incFilamentUsed"));
  #endif
  if (!this->isLoaded()) return;
  this->data.filamentUsed += amount; 
}
void PrintCounter::initStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("initStats"));
  #endif
  this->loaded = true;
  this->data = { 0, 0, 0, 0, 0.0 };
  this->saveStats();
  eeprom_write_byte((uint8_t *) this->address, 0x16);
}
void PrintCounter::loadStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("loadStats"));
  #endif
  if (eeprom_read_byte((uint8_t *) this->address) != 0x16) this->initStats();
  else eeprom_read_block(&this->data,
    (void *)(this->address + sizeof(uint8_t)), sizeof(printStatistics));
  this->loaded = true;
}
void PrintCounter::saveStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("saveStats"));
  #endif
  if (!this->isLoaded()) return;
  eeprom_update_block(&this->data,
    (void *)(this->address + sizeof(uint8_t)), sizeof(printStatistics));
}
void PrintCounter::showStats() {
  char buffer[21];
  duration_t elapsed;
  SERIAL_PROTOCOLPGM(MSG_STATS);
  SERIAL_ECHOPGM("Prints: ");
  SERIAL_ECHO(this->data.totalPrints);
  SERIAL_ECHOPGM(", Finished: ");
  SERIAL_ECHO(this->data.finishedPrints);
  SERIAL_ECHOPGM(", Failed: "); 
  SERIAL_ECHO(this->data.totalPrints - this->data.finishedPrints
    - ((this->isRunning() || this->isPaused()) ? 1 : 0));
  SERIAL_EOL();
  SERIAL_PROTOCOLPGM(MSG_STATS);
  elapsed = this->data.printTime;
  elapsed.toString(buffer);
  SERIAL_ECHOPGM("Total time: ");
  SERIAL_ECHO(buffer);
  #if ENABLED(DEBUG_PRINTCOUNTER)
    SERIAL_ECHOPGM(" (");
    SERIAL_ECHO(this->data.printTime);
    SERIAL_CHAR(')');
  #endif
  elapsed = this->data.longestPrint;
  elapsed.toString(buffer);
  SERIAL_ECHOPGM(", Longest job: ");
  SERIAL_ECHO(buffer);
  #if ENABLED(DEBUG_PRINTCOUNTER)
    SERIAL_ECHOPGM(" (");
    SERIAL_ECHO(this->data.longestPrint);
    SERIAL_CHAR(')');
  #endif
  SERIAL_EOL();
  SERIAL_PROTOCOLPGM(MSG_STATS);
  SERIAL_ECHOPGM("Filament used: ");
  SERIAL_ECHO(this->data.filamentUsed / 1000);
  SERIAL_ECHOPGM("m");
  SERIAL_EOL();
}
void PrintCounter::tick() {
  if (!this->isRunning()) return;
  static uint32_t update_last = millis(),
                  eeprom_last = millis();
  millis_t now = millis();
  const static uint16_t i = this->updateInterval * 1000;
  if (now - update_last >= i) {
    #if ENABLED(DEBUG_PRINTCOUNTER)
      PrintCounter::debug(PSTR("tick"));
    #endif
    this->data.printTime += this->deltaDuration();
    update_last = now;
  }
  const static millis_t j = this->saveInterval * 1000;
  if (now - eeprom_last >= j) {
    eeprom_last = now;
    this->saveStats();
  }
}
bool PrintCounter::start() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("start"));
  #endif
  bool paused = this->isPaused();
  if (super::start()) {
    if (!paused) {
      this->data.totalPrints++;
      this->lastDuration = 0;
    }
    return true;
  }
  return false;
}
bool PrintCounter::stop() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("stop"));
  #endif
  if (super::stop()) {
    this->data.finishedPrints++;
    this->data.printTime += this->deltaDuration();
    if (this->duration() > this->data.longestPrint)
      this->data.longestPrint = this->duration();
    this->saveStats();
    return true;
  }
  else return false;
}
void PrintCounter::reset() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("stop"));
  #endif
  super::reset();
  this->lastDuration = 0;
}
#if ENABLED(DEBUG_PRINTCOUNTER)
  void PrintCounter::debug(const char func[]) {
    if (DEBUGGING(INFO)) {
      SERIAL_ECHOPGM("PrintCounter::");
      serialprintPGM(func);
      SERIAL_ECHOLNPGM("()");
    }
  }
#endif
