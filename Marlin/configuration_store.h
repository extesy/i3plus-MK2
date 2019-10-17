#ifndef CONFIGURATION_STORE_H
#define CONFIGURATION_STORE_H
#include "MarlinConfig.h"
class MarlinSettings {
  public:
    MarlinSettings() { }
    static void reset();
    static bool save();
  #ifdef FYS_SAFE_PRINT_BREAK
    static void FunV04D(uint32_t& printedTime);
    static void FunV04E(uint32_t& printedTime);
    static bool FunV009();
    static void FunV00D();
    static void FunV004();
  #endif
    #if ENABLED(EEPROM_SETTINGS)
      static bool load();
      #if ENABLED(AUTO_BED_LEVELING_UBL) 
        FORCE_INLINE static int get_start_of_meshes() { return meshes_begin; }
        FORCE_INLINE static int get_end_of_meshes() { return meshes_end; }
        static int calc_num_meshes();
        static void store_mesh(int8_t slot);
        static void load_mesh(int8_t slot, void *into = 0);
      #endif
    #else
      FORCE_INLINE
      static bool load() { reset(); report(); return true; }
    #endif
    #if DISABLED(DISABLE_M503)
      static void report(bool forReplay=false);
    #else
      FORCE_INLINE
      static void report(bool forReplay=false) { UNUSED(forReplay); }
    #endif
  private:
    static void postprocess();
    #if ENABLED(EEPROM_SETTINGS)
      static bool eeprom_error;
      #if ENABLED(AUTO_BED_LEVELING_UBL) 
        static int meshes_begin;
        const static int meshes_end = E2END - 128; 
      #endif
      static void write_data(int &pos, const uint8_t *value, uint16_t size, uint16_t *crc);
      static void read_data(int &pos, uint8_t *value, uint16_t size, uint16_t *crc);
    #endif
};
extern MarlinSettings settings;
#endif 
