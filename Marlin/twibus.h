#ifndef TWIBUS_H
#define TWIBUS_H
#include "macros.h"
#include <Wire.h>
typedef void (*twiReceiveFunc_t)(int bytes);
typedef void (*twiRequestFunc_t)();
#define TWIBUS_BUFFER_SIZE 32
class TWIBus {
  private:
    uint8_t buffer_s = 0;
    char buffer[TWIBUS_BUFFER_SIZE];
  public:
    uint8_t addr = 0;
    TWIBus();
    void reset();
    void send();
    void addbyte(const char c);
    void addbytes(char src[], uint8_t bytes);
    void addstring(char str[]);
    void address(const uint8_t adr);
    static void echoprefix(uint8_t bytes, const char prefix[], uint8_t adr);
    static void echodata(uint8_t bytes, const char prefix[], uint8_t adr);
    void echobuffer(const char prefix[], uint8_t adr);
    bool request(const uint8_t bytes);
    uint8_t capture(char *dst, const uint8_t bytes);
    static void flush();
    void relay(const uint8_t bytes);
    #if I2C_SLAVE_ADDRESS > 0
      inline void onReceive(const twiReceiveFunc_t handler) { Wire.onReceive(handler); }
      inline void onRequest(const twiRequestFunc_t handler) { Wire.onRequest(handler); }
      void receive(uint8_t bytes);
      void reply(char str[]=NULL);
      inline void reply(const char str[]) { this->reply((char*)str); }
    #endif
    #if ENABLED(DEBUG_TWIBUS)
      static void prefix(const char func[]);
      static void debug(const char func[], uint32_t adr);
      static void debug(const char func[], char c);
      static void debug(const char func[], char adr[]);
      static inline void debug(const char func[], uint8_t v) { debug(func, (uint32_t)v); }
    #endif
};
#endif 
