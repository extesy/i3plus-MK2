#ifndef STOPWATCH_H
#define STOPWATCH_H
#include "macros.h"
class Stopwatch {
  private:
    enum State {
      STOPPED,
      RUNNING,
      PAUSED
    };
    Stopwatch::State state;
    millis_t accumulator;
    millis_t startTimestamp;
    millis_t stopTimestamp;
  public:
    Stopwatch();
    void resume(millis_t t);
    bool stop();
    bool pause();
    bool start();
    void reset();
    bool isRunning();
    bool isPaused();
    millis_t duration();
    #if ENABLED(DEBUG_STOPWATCH)
      static void debug(const char func[]);
    #endif
};
#endif 
