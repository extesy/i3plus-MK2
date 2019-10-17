#ifndef __BUZZER_H__
#define __BUZZER_H__
#include "types.h"
#include "fastio.h"
#include "circularqueue.h"
#include "temperature.h"
#include "MarlinConfig.h"
#define TONE_QUEUE_LENGTH 4
struct tone_t {
  uint16_t duration;
  uint16_t frequency;
};
class Buzzer {
  private:
    struct state_t {
      tone_t   tone;
      uint32_t endtime;
    } state;
  protected:
    CircularQueue<tone_t, TONE_QUEUE_LENGTH> buffer;
    void invert() {
      TOGGLE(BEEPER_PIN);
    }
    void off() {
      WRITE(BEEPER_PIN, LOW);
    }
    void on() {
      WRITE(BEEPER_PIN, HIGH);
    }
    void reset() {
      this->off();
      this->state.endtime = 0;
    }
  public:
    Buzzer() {
      SET_OUTPUT(BEEPER_PIN);
      this->reset();
    }
    void tone(const uint16_t &duration, const uint16_t &frequency = 0) {
      while (buffer.isFull()) {
        this->tick();
        thermalManager.manage_heater();
      }
      tone_t tone = { duration, frequency };
      this->buffer.enqueue(tone);
    }
    virtual void tick() {
      const millis_t now = millis();
      if (!this->state.endtime) {
        if (this->buffer.isEmpty()) return;
        this->state.tone = this->buffer.dequeue();
        this->state.endtime = now + this->state.tone.duration;
        if (this->state.tone.frequency > 0) {
          #if ENABLED(SPEAKER)
            CRITICAL_SECTION_START;
            ::tone(BEEPER_PIN, this->state.tone.frequency, this->state.tone.duration);
            CRITICAL_SECTION_END;
          #else
            this->on();
          #endif
        }
      }
      else if (ELAPSED(now, this->state.endtime)) this->reset();
    }
};
extern Buzzer buzzer;
#endif
