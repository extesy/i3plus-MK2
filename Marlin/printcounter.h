#ifndef PRINTCOUNTER_H
#define PRINTCOUNTER_H
#include "macros.h"
#include "language.h"
#include "stopwatch.h"
#include <avr/eeprom.h>
struct printStatistics {    
  uint16_t totalPrints;     
  uint16_t finishedPrints;  
  uint32_t printTime;       
  uint32_t longestPrint;    
  double   filamentUsed;    
};
class PrintCounter: public Stopwatch {
  private:
    typedef Stopwatch super;
    printStatistics data;
    const uint16_t address = 0x32;
    const uint16_t updateInterval = 10;
    const uint16_t saveInterval = 3600;
    millis_t lastDuration;
    bool loaded = false;
  protected:
    millis_t deltaDuration();
  public:
    PrintCounter();
    bool isLoaded();
    void incFilamentUsed(double const &amount);
    void initStats();
    void loadStats();
    void saveStats();
    void showStats();
    printStatistics getStats() { return this->data; }
    void tick();
    bool start();
    bool stop();
    void reset();
    #if ENABLED(DEBUG_PRINTCOUNTER)
      static void debug(const char func[]);
    #endif
};
#endif 
