#ifndef SdFatUtil_h
#define SdFatUtil_h
#include "Marlin.h"
#if ENABLED(SDSUPPORT)
#define PgmPrint(x) SerialPrint_P(PSTR(x))
#define PgmPrintln(x) SerialPrintln_P(PSTR(x))
namespace SdFatUtil {
  int FreeRam();
  void print_P(PGM_P str);
  void println_P(PGM_P str);
  void SerialPrint_P(PGM_P str);
  void SerialPrintln_P(PGM_P str);
}
using namespace SdFatUtil;  
#endif 
#endif 
