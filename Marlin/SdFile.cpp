#include "Marlin.h"
#if ENABLED(SDSUPPORT)
#include "SdFile.h"
SdFile::SdFile(const char* path, uint8_t oflag) : SdBaseFile(path, oflag) {
}
int16_t SdFile::write(const void* buf, uint16_t nbyte) {
  return SdBaseFile::write(buf, nbyte);
}
#if ARDUINO >= 100
  size_t SdFile::write(uint8_t b) {
    return SdBaseFile::write(&b, 1);
  }
#else
  void SdFile::write(uint8_t b) {
    SdBaseFile::write(&b, 1);
  }
#endif
void SdFile::write(const char* str) {
  SdBaseFile::write(str, strlen(str));
}
void SdFile::write_P(PGM_P str) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) write(c);
}
void SdFile::writeln_P(PGM_P str) {
  write_P(str);
  write_P(PSTR("\r\n"));
}
#endif
