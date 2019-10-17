#include "Marlin.h"
#include "utility.h"
#include "temperature.h"
void safe_delay(millis_t ms) {
  while (ms > 50) {
    ms -= 50;
    delay(50);
    thermalManager.manage_heater();
  }
  delay(ms);
  thermalManager.manage_heater(); 
}
#if ENABLED(EEPROM_SETTINGS)
  void crc16(uint16_t *crc, const void * const data, uint16_t cnt) {
    uint8_t *ptr = (uint8_t *)data;
    while (cnt--) {
      *crc = (uint16_t)(*crc ^ (uint16_t)(((uint16_t)*ptr++) << 8));
      for (uint8_t x = 0; x < 8; x++)
        *crc = (uint16_t)((*crc & 0x8000) ? ((uint16_t)(*crc << 1) ^ 0x1021) : (*crc << 1));
    }
  }
#endif 
#if ENABLED(ULTRA_LCD)||ENABLED(FYS_ULTRA_LCD_WANHAO_ONEPLUS) || ENABLED(DOGLCD)
  char conv[8] = { 0 };
  #define DIGIT(n) ('0' + (n))
  #define DIGIMOD(n, f) DIGIT((n)/(f) % 10)
  #define RJDIGIT(n, f) ((n) >= (f) ? DIGIMOD(n, f) : ' ')
  #define MINUSOR(n, alt) (n >= 0 ? (alt) : (n = -n, '-'))
  char* i8tostr3(const uint8_t xx) {
    conv[4] = RJDIGIT(xx, 100);
    conv[5] = RJDIGIT(xx, 10);
    conv[6] = DIGIMOD(xx, 1);
    return &conv[4];
  }
  char* itostr2(const uint8_t& x) {
      int xx = x;
      conv[0] = (xx / 10) % 10 + '0';
      conv[1] = xx % 10 + '0';
      conv[2] = 0;
      return conv;
  }
  char* itostr3(const int x) {
    int xx = x;
    conv[4] = MINUSOR(xx, RJDIGIT(xx, 100));
    conv[5] = RJDIGIT(xx, 10);
    conv[6] = DIGIMOD(xx, 1);
    return &conv[4];
  }
  char* ftostr31(const float& x) {
      int xx = abs(x * 10);
      conv[0] = (x >= 0) ? '+' : '-';
      conv[1] = (xx / 1000) % 10 + '0';
      conv[2] = (xx / 100) % 10 + '0';
      conv[3] = (xx / 10) % 10 + '0';
      conv[4] = '.';
      conv[5] = xx % 10 + '0';
      conv[6] = 0;
      return conv;
  }
  char* itostr3left(const int xx) {
    char *str = &conv[6];
    *str = DIGIMOD(xx, 1);
    if (xx >= 10) {
      *(--str) = DIGIMOD(xx, 10);
      if (xx >= 100)
        *(--str) = DIGIMOD(xx, 100);
    }
    return str;
  }
  char *itostr4sign(const int x) {
    const bool neg = x < 0;
    const int xx = neg ? -x : x;
    if (x >= 1000) {
      conv[3] = DIGIMOD(xx, 1000);
      conv[4] = DIGIMOD(xx, 100);
      conv[5] = DIGIMOD(xx, 10);
    }
    else {
      if (xx >= 100) {
        conv[3] = neg ? '-' : ' ';
        conv[4] = DIGIMOD(xx, 100);
        conv[5] = DIGIMOD(xx, 10);
      }
      else {
        conv[3] = ' ';
        conv[4] = ' ';
        if (xx >= 10) {
          conv[4] = neg ? '-' : ' ';
          conv[5] = DIGIMOD(xx, 10);
        }
        else {
          conv[5] = neg ? '-' : ' ';
        }
      }
    }
    conv[6] = DIGIMOD(xx, 1);
    return &conv[3];
  }
  char* ftostr12ns(const float &x) {
    const long xx = (x < 0 ? -x : x) * 100;
    conv[3] = DIGIMOD(xx, 100);
    conv[4] = '.';
    conv[5] = DIGIMOD(xx, 10);
    conv[6] = DIGIMOD(xx, 1);
    return &conv[3];
  }
  char *ftostr32(const float &x) {
    long xx = x * 100;
    conv[1] = MINUSOR(xx, DIGIMOD(xx, 10000));
    conv[2] = DIGIMOD(xx, 1000);
    conv[3] = DIGIMOD(xx, 100);
    conv[4] = '.';
    conv[5] = DIGIMOD(xx, 10);
    conv[6] = DIGIMOD(xx, 1);
    return &conv[1];
  }
  #if ENABLED(LCD_DECIMAL_SMALL_XY)
    char *ftostr4sign(const float &fx) {
      const int x = fx * 10;
      if (!WITHIN(x, -99, 999)) return itostr4sign((int)fx);
      const bool neg = x < 0;
      const int xx = neg ? -x : x;
      conv[3] = neg ? '-' : (xx >= 100 ? DIGIMOD(xx, 100) : ' ');
      conv[4] = DIGIMOD(xx, 10);
      conv[5] = '.';
      conv[6] = DIGIMOD(xx, 1);
      return &conv[3];
    }
  #endif 
  char* ftostr41sign(const float &x) {
    int xx = x * 10;
    conv[1] = MINUSOR(xx, '+');
    conv[2] = DIGIMOD(xx, 1000);
    conv[3] = DIGIMOD(xx, 100);
    conv[4] = DIGIMOD(xx, 10);
    conv[5] = '.';
    conv[6] = DIGIMOD(xx, 1);
    return &conv[1];
  }
  char* ftostr43sign(const float &x, char plus) {
    long xx = x * 1000;
    conv[1] = xx ? MINUSOR(xx, plus) : ' ';
    conv[2] = DIGIMOD(xx, 1000);
    conv[3] = '.';
    conv[4] = DIGIMOD(xx, 100);
    conv[5] = DIGIMOD(xx, 10);
    conv[6] = DIGIMOD(xx, 1);
    return &conv[1];
  }
  char* ftostr5rj(const float &x) {
    const long xx = x < 0 ? -x : x;
    conv[2] = RJDIGIT(xx, 10000);
    conv[3] = RJDIGIT(xx, 1000);
    conv[4] = RJDIGIT(xx, 100);
    conv[5] = RJDIGIT(xx, 10);
    conv[6] = DIGIMOD(xx, 1);
    return &conv[2];
  }
  char* ftostr51sign(const float &x) {
    long xx = x * 10;
    conv[0] = MINUSOR(xx, '+');
    conv[1] = DIGIMOD(xx, 10000);
    conv[2] = DIGIMOD(xx, 1000);
    conv[3] = DIGIMOD(xx, 100);
    conv[4] = DIGIMOD(xx, 10);
    conv[5] = '.';
    conv[6] = DIGIMOD(xx, 1);
    return conv;
  }
  char* ftostr52sign(const float &x) {
    long xx = x * 100;
    conv[0] = MINUSOR(xx, '+');
    conv[1] = DIGIMOD(xx, 10000);
    conv[2] = DIGIMOD(xx, 1000);
    conv[3] = DIGIMOD(xx, 100);
    conv[4] = '.';
    conv[5] = DIGIMOD(xx, 10);
    conv[6] = DIGIMOD(xx, 1);
    return conv;
  }
  char* ftostr62sign(const float& x) {
      long xx = abs(x * 100);
      conv[0] = MINUSOR(xx, '+');
      conv[1] = DIGIMOD(xx, 100000);
      conv[2] = DIGIMOD(xx, 10000);
      conv[3] = DIGIMOD(xx, 1000);
      conv[4] = DIGIMOD(xx, 100);
      conv[5] = '.';
      conv[6] = DIGIMOD(xx, 10);
      conv[7] = DIGIMOD(xx, 1);
      conv[8] = '\0';
      return conv;
  }
  char* ftostr62rj(const float &x) {
    const long xx = (x < 0 ? -x : x) * 100;
    conv[0] = RJDIGIT(xx, 100000);
    conv[1] = RJDIGIT(xx, 10000);
    conv[2] = RJDIGIT(xx, 1000);
    conv[3] = DIGIMOD(xx, 100);
    conv[4] = '.';
    conv[5] = DIGIMOD(xx, 10);
    conv[6] = DIGIMOD(xx, 1);
    return conv;
  }
  char* ftostr52sp(const float &x) {
    long xx = x * 100;
    uint8_t dig;
    conv[1] = MINUSOR(xx, RJDIGIT(xx, 10000));
    conv[2] = RJDIGIT(xx, 1000);
    conv[3] = DIGIMOD(xx, 100);
    if ((dig = xx % 10)) {          
      conv[4] = '.';
      conv[5] = DIGIMOD(xx, 10);
      conv[6] = DIGIT(dig);
    }
    else {
      if ((dig = (xx / 10) % 10)) { 
        conv[4] = '.';
        conv[5] = DIGIT(dig);
      }
      else                          
        conv[4] = conv[5] = ' ';
      conv[6] = ' ';
    }
    return &conv[1];
  }
#endif 
