#include "Marlin.h"
#if ENABLED(SDSUPPORT)
#include "SdFatUtil.h"
#ifdef __arm__
extern "C" char* sbrk(int incr);
int SdFatUtil::FreeRam() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}
#else  
extern char* __brkval;
extern char __bss_end;
int SdFatUtil::FreeRam() {
  char top;
  return __brkval ? &top - __brkval : &top - &__bss_end;
}
#endif  
void SdFatUtil::print_P(PGM_P str) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) MYSERIAL.write(c);
}
void SdFatUtil::println_P(PGM_P str) {
  print_P(str);
  MYSERIAL.println();
}
void SdFatUtil::SerialPrint_P(PGM_P str) {
  print_P(str);
}
void SdFatUtil::SerialPrintln_P(PGM_P str) {
  println_P(str);
}
#endif
