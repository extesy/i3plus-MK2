#ifndef HEX_PRINT_ROUTINES_H
#define HEX_PRINT_ROUTINES_H
#include "MarlinConfig.h"
#include "gcode.h"
#if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(M100_FREE_MEMORY_WATCHER) || ENABLED(DEBUG_GCODE_PARSER)
inline char hex_nybble(const uint8_t n) {
  return (n & 0xF) + ((n & 0xF) < 10 ? '0' : 'A' - 10);
}
char* hex_byte(const uint8_t b);
char* hex_word(const uint16_t w);
char* hex_address(const void * const w);
void print_hex_nybble(const uint8_t n);
void print_hex_byte(const uint8_t b);
void print_hex_word(const uint16_t w);
void print_hex_address(const void * const w);
#endif 
#endif 
