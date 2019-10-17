#ifndef __UTILITY_H__
#define __UTILITY_H__
void safe_delay(millis_t ms);
#if ENABLED(EEPROM_SETTINGS)
  void crc16(uint16_t *crc, const void * const data, uint16_t cnt);
#endif
#if ENABLED(ULTRA_LCD)||ENABLED(FYS_ULTRA_LCD_WANHAO_ONEPLUS) 
  char* i8tostr3(const uint8_t x);
  char* itostr2(const uint8_t& x);
  char* itostr3(const int x);
  char* ftostr31(const float& x);
  char* itostr3left(const int xx);
  char *itostr4sign(const int x);
  char* ftostr12ns(const float &x);
  char *ftostr32(const float &x);
  char* ftostr41sign(const float &x);
  char* ftostr43sign(const float &x, char plus=' ');
  char* ftostr5rj(const float &x);
  char* ftostr51sign(const float &x);
  char* ftostr52sp(const float &x);
  char* ftostr52sign(const float &x);
  char* ftostr62sign(const float& x);
  char* ftostr62rj(const float &x);
  FORCE_INLINE char *ftostr3(const float &x) { return itostr3((int)x); }
  #if ENABLED(LCD_DECIMAL_SMALL_XY)
    char *ftostr4sign(const float &fx);
  #else
    FORCE_INLINE char *ftostr4sign(const float &x) { return itostr4sign((int)x); }
  #endif
#endif 
#endif 
