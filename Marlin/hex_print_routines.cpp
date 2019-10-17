#include "Marlin.h"
#include "gcode.h"
#if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(M100_FREE_MEMORY_WATCHER) || ENABLED(DEBUG_GCODE_PARSER)
#include "hex_print_routines.h"
static char _hex[7] = "0x0000";
char* hex_byte(const uint8_t b) {
  _hex[4] = hex_nybble(b >> 4);
  _hex[5] = hex_nybble(b);
  return &_hex[4];
}
char* hex_word(const uint16_t w) {
  _hex[2] = hex_nybble(w >> 12);
  _hex[3] = hex_nybble(w >> 8);
  _hex[4] = hex_nybble(w >> 4);
  _hex[5] = hex_nybble(w);
  return &_hex[2];
}
char* hex_address(const void * const w) {
  (void)hex_word((uint16_t)w);
  return _hex;
}
void print_hex_nybble(const uint8_t n)       { SERIAL_CHAR(hex_nybble(n));  }
void print_hex_byte(const uint8_t b)         { SERIAL_ECHO(hex_byte(b));    }
void print_hex_word(const uint16_t w)        { SERIAL_ECHO(hex_word(w));    }
void print_hex_address(const void * const w) { SERIAL_ECHO(hex_address(w)); }
#endif 
