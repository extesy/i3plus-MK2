#ifndef ENDSTOPS_H
#define ENDSTOPS_H
#include "enum.h"
class Endstops {
  public:
    static bool enabled, enabled_globally;
    static volatile char endstop_hit_bits; 
    #if ENABLED(Z_DUAL_ENDSTOPS)
      static uint16_t
    #else
      static byte
    #endif
        current_endstop_bits, old_endstop_bits;
    Endstops() {};
    void init();
    static void update();
    static void report_state(); 
    static void M119();
    static void enable_globally(bool onoff=true) { enabled_globally = enabled = onoff; }
    static void enable(bool onoff=true) { enabled = onoff; }
    static void not_homing() { enabled = enabled_globally; }
    static void hit_on_purpose() { endstop_hit_bits = 0; }
    #if HAS_BED_PROBE
      static volatile bool z_probe_enabled;
      static void enable_z_probe(bool onoff=true) { z_probe_enabled = onoff; }
    #endif
  private:
    #if ENABLED(Z_DUAL_ENDSTOPS)
      static void test_dual_z_endstops(const EndstopEnum es1, const EndstopEnum es2);
    #endif
};
extern Endstops endstops;
#if HAS_BED_PROBE
  #define ENDSTOPS_ENABLED  (endstops.enabled || endstops.z_probe_enabled)
#else
  #define ENDSTOPS_ENABLED  endstops.enabled
#endif
#endif 
