#include "Marlin.h"
#if ENABLED(SDSUPPORT)
#include "SdBaseFile.h"
#include <Print.h>
#ifndef SdFile_h
#define SdFile_h
class SdFile : public SdBaseFile, public Print {
 public:
  SdFile() {}
  SdFile(const char* name, uint8_t oflag);
  #if ARDUINO >= 100
    size_t write(uint8_t b);
  #else
   void write(uint8_t b);
  #endif
  int16_t write(const void* buf, uint16_t nbyte);
  void write(const char* str);
  void write_P(PGM_P str);
  void writeln_P(PGM_P str);
};
#endif  
#endif
