#ifndef GCODE_H
#define GCODE_H
#include "enum.h"
#include "types.h"
#include "MarlinConfig.h"
#if ENABLED(DEBUG_GCODE_PARSER)
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    extern char* hex_address(const void * const w);
  #else
    #include "hex_print_routines.h"
  #endif
  #include "serial.h"
#endif
#if ENABLED(INCH_MODE_SUPPORT)
  extern bool volumetric_enabled;
#endif
class GCodeParser {
private:
  static char *value_ptr;           
  #if ENABLED(FASTER_GCODE_PARSER)
    static byte codebits[4];        
    static uint8_t param[26];       
  #else
    static char *command_args;      
  #endif
public:
  #if ENABLED(INCH_MODE_SUPPORT)
    static float linear_unit_factor, volumetric_unit_factor;
  #endif
  #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
    static TempUnit input_temp_units;
  #endif
  static char *command_ptr,               
              *string_arg;                
  static char command_letter;             
  static int codenum;                     
  #if USE_GCODE_SUBCODES
    static int subcode;                   
  #endif
  #if ENABLED(DEBUG_GCODE_PARSER)
    void debug();
  #endif
  static void reset();
  #define PARAM_IND(N)  ((N) >> 3)
  #define PARAM_BIT(N)  ((N) & 0x7)
  #define LETTER_OFF(N) ((N) - 'A')
  #define LETTER_IND(N) PARAM_IND(LETTER_OFF(N))
  #define LETTER_BIT(N) PARAM_BIT(LETTER_OFF(N))
  #if ENABLED(FASTER_GCODE_PARSER)
    static void set(const char c, char * const ptr
      #if ENABLED(DEBUG_GCODE_PARSER)
        , const bool debug=false
      #endif
    ) {
      const uint8_t ind = LETTER_OFF(c);
      if (ind >= COUNT(param)) return;           
      SBI(codebits[PARAM_IND(ind)], PARAM_BIT(ind));        
      param[ind] = ptr ? ptr - command_ptr : 0;  
      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) {
          SERIAL_ECHOPAIR("Set bit ", (int)PARAM_BIT(ind));
          SERIAL_ECHOPAIR(" of index ", (int)PARAM_IND(ind));
          SERIAL_ECHOLNPAIR(" | param = ", (int)param[ind]);
        }
      #endif
    }
    static volatile bool seen(const char c) {
      const uint8_t ind = LETTER_OFF(c);
      if (ind >= COUNT(param)) return false; 
      const bool b = TEST(codebits[PARAM_IND(ind)], PARAM_BIT(ind));
      if (b) value_ptr = param[ind] ? command_ptr + param[ind] : (char*)NULL;
      return b;
    }
    static bool seen_any() { return codebits[3] || codebits[2] || codebits[1] || codebits[0]; }
    #define SEEN_TEST(L) TEST(codebits[LETTER_IND(L)], LETTER_BIT(L))
  #else 
    static volatile bool seen(const char c) {
      const char *p = strchr(command_args, c);
      const bool b = !!p;
      if (b) value_ptr = DECIMAL_SIGNED(p[1]) ? &p[1] : (char*)NULL;
      return b;
    }
    static bool seen_any() { return *command_args == '\0'; }
    #define SEEN_TEST(L) !!strchr(command_args, L)
  #endif 
  static bool seen_axis() {
    return SEEN_TEST('X') || SEEN_TEST('Y') || SEEN_TEST('Z') || SEEN_TEST('E');
  }
  static void parse(char * p);
  FORCE_INLINE static bool has_value() { return value_ptr != NULL; }
  inline static bool seenval(const char c) { return seen(c) && has_value(); }
  inline static float value_float() {
    if (value_ptr) {
      char *e = value_ptr;
      for (;;) {
        const char c = *e;
        if (c == '\0' || c == ' ') break;
        if (c == 'E' || c == 'e') {
          *e = '\0';
          const float ret = strtod(value_ptr, NULL);
          *e = c;
          return ret;
        }
        ++e;
      }
      return strtod(value_ptr, NULL);
    }
    return 0.0;
  }
  inline static int32_t value_long() { return value_ptr ? strtol(value_ptr, NULL, 10) : 0L; }
  inline static uint32_t value_ulong() { return value_ptr ? strtoul(value_ptr, NULL, 10) : 0UL; }
  FORCE_INLINE static millis_t value_millis() { return value_ulong(); }
  FORCE_INLINE static millis_t value_millis_from_seconds() { return value_float() * 1000UL; }
  FORCE_INLINE static int16_t value_int() { return (int16_t)value_long(); }
  FORCE_INLINE static uint16_t value_ushort() { return (uint16_t)value_long(); }
  inline static uint8_t value_byte() { return (uint8_t)constrain(value_long(), 0, 255); }
  inline static bool value_bool() { return !has_value() || value_byte(); }
  #if ENABLED(INCH_MODE_SUPPORT)
    inline static void set_input_linear_units(const LinearUnit units) {
      switch (units) {
        case LINEARUNIT_INCH:
          linear_unit_factor = 25.4;
          break;
        case LINEARUNIT_MM:
        default:
          linear_unit_factor = 1.0;
          break;
      }
      volumetric_unit_factor = POW(linear_unit_factor, 3.0);
    }
    inline static float axis_unit_factor(const AxisEnum axis) {
      return (axis >= E_AXIS && volumetric_enabled ? volumetric_unit_factor : linear_unit_factor);
    }
    inline static float value_linear_units()                     { return value_float() * linear_unit_factor; }
    inline static float value_axis_units(const AxisEnum axis)    { return value_float() * axis_unit_factor(axis); }
    inline static float value_per_axis_unit(const AxisEnum axis) { return value_float() / axis_unit_factor(axis); }
  #else
    FORCE_INLINE static float value_linear_units()                  {            return value_float(); }
    FORCE_INLINE static float value_axis_units(const AxisEnum a)    { UNUSED(a); return value_float(); }
    FORCE_INLINE static float value_per_axis_unit(const AxisEnum a) { UNUSED(a); return value_float(); }
  #endif
  #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
    inline static void set_input_temp_units(TempUnit units) { input_temp_units = units; }
    #if ENABLED(ULTIPANEL) && DISABLED(DISABLE_M503)
      FORCE_INLINE static char temp_units_code() {
        return input_temp_units == TEMPUNIT_K ? 'K' : input_temp_units == TEMPUNIT_F ? 'F' : 'C';
      }
      FORCE_INLINE static char* temp_units_name() {
        return input_temp_units == TEMPUNIT_K ? PSTR("Kelvin") : input_temp_units == TEMPUNIT_F ? PSTR("Fahrenheit") : PSTR("Celsius");
      }
      inline static float to_temp_units(const float &f) {
        switch (input_temp_units) {
          case TEMPUNIT_F:
            return f * 0.5555555556 + 32.0;
          case TEMPUNIT_K:
            return f + 273.15;
          case TEMPUNIT_C:
          default:
            return f;
        }
      }
    #endif 
    inline static float value_celsius() {
      const float f = value_float();
      switch (input_temp_units) {
        case TEMPUNIT_F:
          return (f - 32.0) * 0.5555555556;
        case TEMPUNIT_K:
          return f - 273.15;
        case TEMPUNIT_C:
        default:
          return f;
      }
    }
    inline static float value_celsius_diff() {
      switch (input_temp_units) {
        case TEMPUNIT_F:
          return value_float() * 0.5555555556;
        case TEMPUNIT_C:
        case TEMPUNIT_K:
        default:
          return value_float();
      }
    }
  #else 
    FORCE_INLINE static float value_celsius()      { return value_float(); }
    FORCE_INLINE static float value_celsius_diff() { return value_float(); }
  #endif 
  FORCE_INLINE static float value_feedrate() { return value_linear_units(); }
  void unknown_command_error();
  FORCE_INLINE static float    floatval(const char c, const float dval=0.0)   { return seenval(c) ? value_float()        : dval; }
  FORCE_INLINE static bool     boolval(const char c, const bool dval=false)   { return seen(c)    ? value_bool()         : dval; }
  FORCE_INLINE static uint8_t  byteval(const char c, const uint8_t dval=0)    { return seenval(c) ? value_byte()         : dval; }
  FORCE_INLINE static int16_t  intval(const char c, const int16_t dval=0)     { return seenval(c) ? value_int()          : dval; }
  FORCE_INLINE static uint16_t ushortval(const char c, const uint16_t dval=0) { return seenval(c) ? value_ushort()       : dval; }
  FORCE_INLINE static int32_t  longval(const char c, const int32_t dval=0)    { return seenval(c) ? value_long()         : dval; }
  FORCE_INLINE static uint32_t ulongval(const char c, const uint32_t dval=0)  { return seenval(c) ? value_ulong()        : dval; }
  FORCE_INLINE static float    linearval(const char c, const float dval=0.0)  { return seenval(c) ? value_linear_units() : dval; }
  FORCE_INLINE static float    celsiusval(const char c, const float dval=0.0) { return seenval(c) ? value_celsius()      : dval; }
};
extern GCodeParser parser;
#endif 
