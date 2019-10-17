#include "gcode.h"
#include "Marlin.h"
#include "language.h"
#if ENABLED(INCH_MODE_SUPPORT)
  float GCodeParser::linear_unit_factor, GCodeParser::volumetric_unit_factor;
#endif
#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  TempUnit GCodeParser::input_temp_units;
#endif
char *GCodeParser::command_ptr,
     *GCodeParser::string_arg,
     *GCodeParser::value_ptr;
char GCodeParser::command_letter;
int GCodeParser::codenum;
#if USE_GCODE_SUBCODES
  int GCodeParser::subcode;
#endif
#if ENABLED(FASTER_GCODE_PARSER)
  byte GCodeParser::codebits[4];   
  uint8_t GCodeParser::param[26];  
#else
  char *GCodeParser::command_args; 
#endif
GCodeParser parser;
void GCodeParser::reset() {
  string_arg = NULL;                    
  command_letter = '?';                 
  codenum = 0;                          
  #if USE_GCODE_SUBCODES
    subcode = 0;                        
  #endif
  #if ENABLED(FASTER_GCODE_PARSER)
    ZERO(codebits);                     
  #endif
}
void GCodeParser::parse(char *p) {
  reset(); 
  while (*p == ' ') ++p;
  if (*p == 'N' && NUMERIC_SIGNED(p[1])) {
    #if ENABLED(FASTER_GCODE_PARSER)
    #endif
    p += 2;                  
    while (NUMERIC(*p)) ++p; 
    while (*p == ' ')   ++p; 
  }
  command_ptr = p;
  const char letter = *p++;
  char *starpos = strchr(p, '*');
  if (starpos) {
    --starpos;                          
    while (*starpos == ' ') --starpos;  
    starpos[1] = '\0';
  }
  switch (letter) { case 'G': case 'M': case 'T': break; default: return; }
  while (*p == ' ') p++;
  if (!NUMERIC(*p)) return;
  command_letter = letter;
  codenum = 0;
  do {
    codenum *= 10, codenum += *p++ - '0';
  } while (NUMERIC(*p));
  #if USE_GCODE_SUBCODES
    if (*p == '.') {
      p++;
      while (NUMERIC(*p))
        subcode *= 10, subcode += *p++ - '0';
    }
  #endif
  while (*p == ' ') p++;
  #if DISABLED(FASTER_GCODE_PARSER)
    command_args = p; 
  #endif
  if (letter == 'M') switch (codenum) { case 23: case 28: case 30: case 117: case 118: case 928: string_arg = p; return; default: break; }
  #if ENABLED(DEBUG_GCODE_PARSER)
    const bool debug = codenum == 800;
  #endif
  string_arg = NULL;
  while (char code = *p++) {                    
    if (code == '!' && letter == 'M' && codenum == 32) {
      string_arg = p;                           
      char * const lb = strchr(p, '#');         
      if (lb) *lb = '\0';                       
      return;
    }
    #if ENABLED(FASTER_GCODE_PARSER)
      #define PARAM_TEST WITHIN(code, 'A', 'Z')
    #else
      #define PARAM_TEST true
    #endif
    if (PARAM_TEST) {
      while (*p == ' ') p++;                    
      const bool has_num = DECIMAL_SIGNED(*p);  
      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) {
          SERIAL_ECHOPAIR("Got letter ", code); 
          SERIAL_ECHOPAIR(" at index ", (int)(p - command_ptr - 1)); 
          if (has_num) SERIAL_ECHOPGM(" (has_num)");
        }
      #endif
      if (!has_num && !string_arg) {            
        string_arg = p - 1;
        #if ENABLED(DEBUG_GCODE_PARSER)
          if (debug) SERIAL_ECHOPAIR(" string_arg: ", hex_address((void*)string_arg)); 
        #endif
      }
      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) SERIAL_EOL();
      #endif
      #if ENABLED(FASTER_GCODE_PARSER)
        set(code, has_num ? p : NULL            
          #if ENABLED(DEBUG_GCODE_PARSER)
            , debug
          #endif
        );
      #endif
    }
    else if (!string_arg) {                     
      string_arg = p - 1;
      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) SERIAL_ECHOPAIR(" string_arg: ", hex_address((void*)string_arg)); 
      #endif
    }
    if (!WITHIN(*p, 'A', 'Z')) {
      while (*p && NUMERIC(*p)) p++;            
      while (*p == ' ') p++;                    
    }
  }
}
void GCodeParser::unknown_command_error() {
  SERIAL_ECHO_START();
  SERIAL_ECHOPAIR(MSG_UNKNOWN_COMMAND, command_ptr);
  SERIAL_CHAR('"');
  SERIAL_EOL();
}
#if ENABLED(DEBUG_GCODE_PARSER)
  void GCodeParser::debug() {
    SERIAL_ECHOPAIR("Command: ", command_ptr);
    SERIAL_ECHOPAIR(" (", command_letter);
    SERIAL_ECHO(codenum);
    SERIAL_ECHOLNPGM(")");
    #if ENABLED(FASTER_GCODE_PARSER)
      SERIAL_ECHO(" args: \"");
      for (char c = 'A'; c <= 'Z'; ++c)
        if (seen(c)) { SERIAL_CHAR(c); SERIAL_CHAR(' '); }
    #else
      SERIAL_ECHOPAIR(" args: \"", command_args);
    #endif
    SERIAL_ECHOPGM("\"");
    if (string_arg) {
      SERIAL_ECHOPGM(" string: \"");
      SERIAL_ECHO(string_arg);
      SERIAL_CHAR('"');
    }
    SERIAL_ECHOPGM("\n\n");
    for (char c = 'A'; c <= 'Z'; ++c) {
      if (seen(c)) {
        SERIAL_ECHOPAIR("Code '", c); SERIAL_ECHOPGM("':");
        if (has_value()) {
          SERIAL_ECHOPAIR("\n    float: ", value_float());
          SERIAL_ECHOPAIR("\n     long: ", value_long());
          SERIAL_ECHOPAIR("\n    ulong: ", value_ulong());
          SERIAL_ECHOPAIR("\n   millis: ", value_millis());
          SERIAL_ECHOPAIR("\n   sec-ms: ", value_millis_from_seconds());
          SERIAL_ECHOPAIR("\n      int: ", value_int());
          SERIAL_ECHOPAIR("\n   ushort: ", value_ushort());
          SERIAL_ECHOPAIR("\n     byte: ", (int)value_byte());
          SERIAL_ECHOPAIR("\n     bool: ", (int)value_bool());
          SERIAL_ECHOPAIR("\n   linear: ", value_linear_units());
          SERIAL_ECHOPAIR("\n  celsius: ", value_celsius());
        }
        else
          SERIAL_ECHOPGM(" (no value)");
        SERIAL_ECHOPGM("\n\n");
      }
    }
  }
#endif 
