#include "Marlin.h"
#include "math.h"
#if ENABLED(AUTO_BED_LEVELING_UBL)
  #include "ubl.h"
  #include "hex_print_routines.h"
  #include "temperature.h"
  extern Planner planner;
  void bit_clear(uint16_t bits[16], uint8_t x, uint8_t y) { CBI(bits[y], x); }
  void bit_set(uint16_t bits[16], uint8_t x, uint8_t y) { SBI(bits[y], x); }
  bool is_bit_set(uint16_t bits[16], uint8_t x, uint8_t y) { return TEST(bits[y], x); }
  uint8_t ubl_cnt = 0;
  void unified_bed_leveling::echo_name() { SERIAL_PROTOCOLPGM("Unified Bed Leveling"); }
  void unified_bed_leveling::report_state() {
    echo_name();
    SERIAL_PROTOCOLPGM(" System v" UBL_VERSION " ");
    if (!state.active) SERIAL_PROTOCOLPGM("in");
    SERIAL_PROTOCOLLNPGM("active.");
    safe_delay(50);
  }
  static void serial_echo_xy(const int16_t x, const int16_t y) {
    SERIAL_CHAR('(');
    SERIAL_ECHO(x);
    SERIAL_CHAR(',');
    SERIAL_ECHO(y);
    SERIAL_CHAR(')');
    safe_delay(10);
  }
  ubl_state unified_bed_leveling::state;
  float unified_bed_leveling::z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y],
        unified_bed_leveling::last_specified_z;
  constexpr float unified_bed_leveling::_mesh_index_to_xpos[16],
                  unified_bed_leveling::_mesh_index_to_ypos[16];
  bool unified_bed_leveling::g26_debug_flag = false,
       unified_bed_leveling::has_control_of_lcd_panel = false;
  volatile int unified_bed_leveling::encoder_diff;
  unified_bed_leveling::unified_bed_leveling() {
    ubl_cnt++;  
    reset();
  }
  void unified_bed_leveling::reset() {
    set_bed_leveling_enabled(false);
    state.z_offset = 0;
    state.storage_slot = -1;
    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      planner.z_fade_height = 10.0;
    #endif
    ZERO(z_values);
    last_specified_z = -999.9;
  }
  void unified_bed_leveling::invalidate() {
    set_bed_leveling_enabled(false);
    state.z_offset = 0;
    set_all_mesh_points_to_value(NAN);
  }
  void unified_bed_leveling::set_all_mesh_points_to_value(float value) {
    for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++) {
      for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++) {
        z_values[x][y] = value;
      }
    }
  }
  void unified_bed_leveling::display_map(const int map_type) {
    constexpr uint8_t spaces = 8 * (GRID_MAX_POINTS_X - 2);
    SERIAL_PROTOCOLPGM("\nBed Topography Report");
    if (map_type == 0) {
      SERIAL_PROTOCOLPGM(":\n\n");
      serial_echo_xy(0, GRID_MAX_POINTS_Y - 1);
      SERIAL_ECHO_SP(spaces + 3);
      serial_echo_xy(GRID_MAX_POINTS_X - 1, GRID_MAX_POINTS_Y - 1);
      SERIAL_EOL();
      serial_echo_xy(UBL_MESH_MIN_X, UBL_MESH_MAX_Y);
      SERIAL_ECHO_SP(spaces);
      serial_echo_xy(UBL_MESH_MAX_X, UBL_MESH_MAX_Y);
      SERIAL_EOL();
    }
    else {
      SERIAL_PROTOCOLPGM(" for ");
      serialprintPGM(map_type == 1 ? PSTR("CSV:\n\n") : PSTR("LCD:\n\n"));
    }
    const float current_xi = get_cell_index_x(current_position[X_AXIS] + (MESH_X_DIST) / 2.0),
                current_yi = get_cell_index_y(current_position[Y_AXIS] + (MESH_Y_DIST) / 2.0);
    for (int8_t j = GRID_MAX_POINTS_Y - 1; j >= 0; j--) {
      for (uint8_t i = 0; i < GRID_MAX_POINTS_X; i++) {
        const bool is_current = i == current_xi && j == current_yi;
        if (map_type == 0) SERIAL_CHAR(is_current ? '[' : ' ');
        const float f = z_values[i][j];
        if (isnan(f)) {
          serialprintPGM(map_type == 0 ? PSTR("    .   ") : PSTR("NAN"));
        }
        else if (map_type <= 1) {
          if (map_type == 0 && f >= 0.0) SERIAL_CHAR(' ');
          SERIAL_PROTOCOL_F(f, 3);
        }
        idle();
        if (map_type == 1 && i < GRID_MAX_POINTS_X - 1) SERIAL_CHAR(',');
        #if TX_BUFFER_SIZE > 0
          MYSERIAL.flushTX();
        #endif
        safe_delay(15);
        if (map_type == 0) {
          SERIAL_CHAR(is_current ? ']' : ' ');
          SERIAL_CHAR(' ');
        }
      }
      SERIAL_EOL();
      if (j && map_type == 0) { 
        SERIAL_CHAR(' ');
        SERIAL_EOL();
      }
    }
    if (map_type == 0) {
      serial_echo_xy(UBL_MESH_MIN_X, UBL_MESH_MIN_Y);
      SERIAL_ECHO_SP(spaces + 4);
      serial_echo_xy(UBL_MESH_MAX_X, UBL_MESH_MIN_Y);
      SERIAL_EOL();
      serial_echo_xy(0, 0);
      SERIAL_ECHO_SP(spaces + 5);
      serial_echo_xy(GRID_MAX_POINTS_X - 1, 0);
      SERIAL_EOL();
    }
  }
  bool unified_bed_leveling::sanity_check() {
    uint8_t error_flag = 0;
    if (settings.calc_num_meshes() < 1) {
      SERIAL_PROTOCOLLNPGM("?Insufficient EEPROM storage for a mesh of this size.");
      error_flag++;
    }
    return !!error_flag;
  }
#endif 
