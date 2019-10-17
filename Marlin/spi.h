#ifndef __SPI_H__
#define __SPI_H__
#include <stdint.h>
#include "softspi.h"
template<uint8_t MisoPin, uint8_t MosiPin, uint8_t SckPin>
class SPI {
  static SoftSPI<MisoPin, MosiPin, SckPin> softSPI;
  public:
    FORCE_INLINE static void init() { softSPI.begin(); }
    FORCE_INLINE static void send(uint8_t data) { softSPI.send(data); }
    FORCE_INLINE static uint8_t receive() { return softSPI.receive(); }
};
template<>
class SPI<MISO_PIN, MOSI_PIN, SCK_PIN> {
  public:
    FORCE_INLINE static void init() {
        OUT_WRITE(SCK_PIN, LOW);
        OUT_WRITE(MOSI_PIN, HIGH);
        SET_INPUT(MISO_PIN);
        WRITE(MISO_PIN, HIGH);
    }
    FORCE_INLINE static uint8_t receive() {
      SPDR = 0;
      for (;!TEST(SPSR, SPIF););
      return SPDR;
    }
};
#endif 
