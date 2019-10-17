#include "MarlinConfig.h"
#if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(UBL_G26_MESH_VALIDATION)
  #include "ubl.h"
  #include "Marlin.h"
  #include "planner.h"
  #include "stepper.h"
  #include "temperature.h"
  #include "ultralcd.h"
  #include "gcode.h"
  #define EXTRUSION_MULTIPLIER 1.0
  #define RETRACTION_MULTIPLIER 1.0
  #define NOZZLE 0.4
  #define FILAMENT 1.75
  #define LAYER_HEIGHT 0.2
  #define PRIME_LENGTH 10.0
  #define BED_TEMP 60.0
  #define HOTEND_TEMP 205.0
  #define OOZE_AMOUNT 0.3
  #define SIZE_OF_INTERSECTION_CIRCLES 5
  #define SIZE_OF_CROSSHAIRS 3
  #if SIZE_OF_CROSSHAIRS >= SIZE_OF_INTERSECTION_CIRCLES
    #error "SIZE_OF_CROSSHAIRS must be less than SIZE_OF_INTERSECTION_CIRCLES."
  #endif
  extern float feedrate_mm_s; 
  extern Planner planner;
  #if ENABLED(ULTRA_LCD)
    extern char lcd_status_message[];
  #endif
  extern float destination[XYZE];
  void set_destination_to_current();
  void prepare_move_to_destination();
  #if AVR_AT90USB1286_FAMILY  
    inline void sync_plan_position_e() { planner.set_e_position_mm(current_position[E_AXIS]); }
    inline void set_current_to_destination() { COPY(current_position, destination); }
  #else
    void sync_plan_position_e();
    void set_current_to_destination();
  #endif
  #if ENABLED(NEWPANEL)
    void lcd_setstatusPGM(const char* const message, const int8_t level);
    void chirp_at_user();
  #endif
  static uint16_t circle_flags[16], horizontal_mesh_line_flags[16], vertical_mesh_line_flags[16];
  float g26_e_axis_feedrate = 0.020,
        random_deviation = 0.0;
  static bool g26_retracted = false; 
  float valid_trig_angle(float);
  float unified_bed_leveling::g26_extrusion_multiplier,
        unified_bed_leveling::g26_retraction_multiplier,
        unified_bed_leveling::g26_nozzle,
        unified_bed_leveling::g26_filament_diameter,
        unified_bed_leveling::g26_layer_height,
        unified_bed_leveling::g26_prime_length,
        unified_bed_leveling::g26_x_pos,
        unified_bed_leveling::g26_y_pos,
        unified_bed_leveling::g26_ooze_amount;
  int16_t unified_bed_leveling::g26_bed_temp,
          unified_bed_leveling::g26_hotend_temp;
  int8_t unified_bed_leveling::g26_prime_flag;
  bool unified_bed_leveling::g26_continue_with_closest,
       unified_bed_leveling::g26_keep_heaters_on;
  int16_t unified_bed_leveling::g26_repeats;
  void unified_bed_leveling::G26_line_to_destination(const float &feed_rate) {
    const float save_feedrate = feedrate_mm_s;
    feedrate_mm_s = feed_rate;      
    prepare_move_to_destination();  
    feedrate_mm_s = save_feedrate;  
  }
  #if ENABLED(NEWPANEL)
    bool user_canceled() {
      if (!ubl_lcd_clicked()) return false;
      safe_delay(10);                       
      #if ENABLED(ULTRA_LCD)
        lcd_setstatusPGM(PSTR("Mesh Validation Stopped."), 99);
        lcd_quick_feedback();
      #endif
      while (!ubl_lcd_clicked()) idle();    
      lcd_setstatusPGM(PSTR("Release button"), 99); 
      while (ubl_lcd_clicked()) idle();             
      lcd_reset_status();
      return true;
    }
  #endif
  void unified_bed_leveling::G26() {
    SERIAL_ECHOLNPGM("G26 command started.  Waiting for heater(s).");
    float tmp, start_angle, end_angle;
    int   i, xi, yi;
    mesh_index_pair location;
    if (axis_unhomed_error() || parse_G26_parameters()) return;
    if (current_position[Z_AXIS] < Z_CLEARANCE_BETWEEN_PROBES) {
      do_blocking_move_to_z(Z_CLEARANCE_BETWEEN_PROBES);
      stepper.synchronize();
      set_current_to_destination();
    }
    if (turn_on_heaters()) goto LEAVE;
    current_position[E_AXIS] = 0.0;
    sync_plan_position_e();
    if (g26_prime_flag && prime_nozzle()) goto LEAVE;
    ZERO(circle_flags);
    ZERO(horizontal_mesh_line_flags);
    ZERO(vertical_mesh_line_flags);
    set_destination_to_current();
    destination[Z_AXIS] = g26_layer_height;
    move_to(destination, 0.0);
    move_to(destination, g26_ooze_amount);
    has_control_of_lcd_panel = true;
    float sin_table[360 / 30 + 1], cos_table[360 / 30 + 1];
    for (i = 0; i <= 360 / 30; i++) {
      cos_table[i] = SIZE_OF_INTERSECTION_CIRCLES * cos(RADIANS(valid_trig_angle(i * 30.0)));
      sin_table[i] = SIZE_OF_INTERSECTION_CIRCLES * sin(RADIANS(valid_trig_angle(i * 30.0)));
    }
    do {
      location = g26_continue_with_closest
        ? find_closest_circle_to_print(current_position[X_AXIS], current_position[Y_AXIS])
        : find_closest_circle_to_print(g26_x_pos, g26_y_pos); 
      if (location.x_index >= 0 && location.y_index >= 0) {
        const float circle_x = mesh_index_to_xpos(location.x_index),
                    circle_y = mesh_index_to_ypos(location.y_index);
        if (!position_is_reachable_raw_xy(circle_x, circle_y)) continue;
        xi = location.x_index;  
        yi = location.y_index;
        if (g26_debug_flag) {
          SERIAL_ECHOPAIR("   Doing circle at: (xi=", xi);
          SERIAL_ECHOPAIR(", yi=", yi);
          SERIAL_CHAR(')');
          SERIAL_EOL();
        }
        start_angle = 0.0;    
        end_angle   = 360.0;
        if (xi == 0) {       
          start_angle = -90.0;
          end_angle   =  90.0;
          if (yi == 0)        
            start_angle = 0.0;
          else if (yi == GRID_MAX_POINTS_Y - 1)
            end_angle = 0.0;
        }
        else if (xi == GRID_MAX_POINTS_X - 1) { 
          start_angle =  90.0;
          end_angle   = 270.0;
          if (yi == 0)                  
            end_angle = 180.0;
          else if (yi == GRID_MAX_POINTS_Y - 1)
            start_angle = 180.0;
        }
        else if (yi == 0) {
          start_angle =   0.0;         
          end_angle   = 180.0;
        }
        else if (yi == GRID_MAX_POINTS_Y - 1) {
          start_angle = 180.0;         
          end_angle   = 360.0;
        }
        for (tmp = start_angle; tmp < end_angle - 0.1; tmp += 30.0) {
          #if ENABLED(NEWPANEL)
            if (user_canceled()) goto LEAVE;              
          #endif
          int tmp_div_30 = tmp / 30.0;
          if (tmp_div_30 < 0) tmp_div_30 += 360 / 30;
          if (tmp_div_30 > 11) tmp_div_30 -= 360 / 30;
          float x = circle_x + cos_table[tmp_div_30],    
                y = circle_y + sin_table[tmp_div_30],
                xe = circle_x + cos_table[tmp_div_30 + 1],
                ye = circle_y + sin_table[tmp_div_30 + 1];
          #if IS_KINEMATIC
            if (!position_is_reachable_raw_xy(x, y) || !position_is_reachable_raw_xy(xe, ye)) continue;
          #else                                              
            x  = constrain(x, X_MIN_POS + 1, X_MAX_POS - 1); 
            y  = constrain(y, Y_MIN_POS + 1, Y_MAX_POS - 1);
            xe = constrain(xe, X_MIN_POS + 1, X_MAX_POS - 1);
            ye = constrain(ye, Y_MIN_POS + 1, Y_MAX_POS - 1);
          #endif
          print_line_from_here_to_there(LOGICAL_X_POSITION(x), LOGICAL_Y_POSITION(y), g26_layer_height, LOGICAL_X_POSITION(xe), LOGICAL_Y_POSITION(ye), g26_layer_height);
        }
        if (look_for_lines_to_connect())
          goto LEAVE;
      }
    } while (--g26_repeats && location.x_index >= 0 && location.y_index >= 0);
    LEAVE:
    lcd_setstatusPGM(PSTR("Leaving G26"), -1);
    retract_filament(destination);
    destination[Z_AXIS] = Z_CLEARANCE_BETWEEN_PROBES;
    move_to(destination, 0); 
    destination[X_AXIS] = g26_x_pos;                                               
    destination[Y_AXIS] = g26_y_pos;
    move_to(destination, 0); 
    has_control_of_lcd_panel = false;     
    if (!g26_keep_heaters_on) {
      #if HAS_TEMP_BED
        thermalManager.setTargetBed(0);
      #endif
      thermalManager.setTargetHotend(0, 0);
    }
  }
  float valid_trig_angle(float d) {
    while (d > 360.0) d -= 360.0;
    while (d < 0.0) d += 360.0;
    return d;
  }
  mesh_index_pair unified_bed_leveling::find_closest_circle_to_print(const float &X, const float &Y) {
    float closest = 99999.99;
    mesh_index_pair return_val;
    return_val.x_index = return_val.y_index = -1;
    for (uint8_t i = 0; i < GRID_MAX_POINTS_X; i++) {
      for (uint8_t j = 0; j < GRID_MAX_POINTS_Y; j++) {
        if (!is_bit_set(circle_flags, i, j)) {
          const float mx = mesh_index_to_xpos(i),  
                      my = mesh_index_to_ypos(j);
          float f = HYPOT(X - mx, Y - my);
          f += HYPOT(g26_x_pos - mx, g26_y_pos - my) / 15.0;
          if (random_deviation > 1.0)
            f += random(0.0, random_deviation);
          if (f < closest) {
            closest = f;              
            return_val.x_index = i;   
            return_val.y_index = j;
            return_val.distance = closest;
          }
        }
      }
    }
    bit_set(circle_flags, return_val.x_index, return_val.y_index);   
    return return_val;
  }
  bool unified_bed_leveling::look_for_lines_to_connect() {
    float sx, sy, ex, ey;
    for (uint8_t i = 0; i < GRID_MAX_POINTS_X; i++) {
      for (uint8_t j = 0; j < GRID_MAX_POINTS_Y; j++) {
        #if ENABLED(NEWPANEL)
          if (user_canceled()) return true;     
        #endif
        if (i < GRID_MAX_POINTS_X) { 
          if (is_bit_set(circle_flags, i, j) && is_bit_set(circle_flags, i + 1, j)) { 
            if (!is_bit_set(horizontal_mesh_line_flags, i, j)) {
              sx = mesh_index_to_xpos(  i  ) + (SIZE_OF_INTERSECTION_CIRCLES - (SIZE_OF_CROSSHAIRS)); 
              ex = mesh_index_to_xpos(i + 1) - (SIZE_OF_INTERSECTION_CIRCLES - (SIZE_OF_CROSSHAIRS)); 
              sx = constrain(sx, X_MIN_POS + 1, X_MAX_POS - 1);
              sy = ey = constrain(mesh_index_to_ypos(j), Y_MIN_POS + 1, Y_MAX_POS - 1);
              ex = constrain(ex, X_MIN_POS + 1, X_MAX_POS - 1);
              if (position_is_reachable_raw_xy(sx, sy) && position_is_reachable_raw_xy(ex, ey)) {
                if (g26_debug_flag) {
                  SERIAL_ECHOPAIR(" Connecting with horizontal line (sx=", sx);
                  SERIAL_ECHOPAIR(", sy=", sy);
                  SERIAL_ECHOPAIR(") -> (ex=", ex);
                  SERIAL_ECHOPAIR(", ey=", ey);
                  SERIAL_CHAR(')');
                  SERIAL_EOL();
                }
                print_line_from_here_to_there(LOGICAL_X_POSITION(sx), LOGICAL_Y_POSITION(sy), g26_layer_height, LOGICAL_X_POSITION(ex), LOGICAL_Y_POSITION(ey), g26_layer_height);
              }
              bit_set(horizontal_mesh_line_flags, i, j);   
            }
          }
          if (j < GRID_MAX_POINTS_Y) { 
            if (is_bit_set(circle_flags, i, j) && is_bit_set(circle_flags, i, j + 1)) { 
              if (!is_bit_set( vertical_mesh_line_flags, i, j)) {
                sy = mesh_index_to_ypos(  j  ) + (SIZE_OF_INTERSECTION_CIRCLES - (SIZE_OF_CROSSHAIRS)); 
                ey = mesh_index_to_ypos(j + 1) - (SIZE_OF_INTERSECTION_CIRCLES - (SIZE_OF_CROSSHAIRS)); 
                sx = ex = constrain(mesh_index_to_xpos(i), X_MIN_POS + 1, X_MAX_POS - 1);
                sy = constrain(sy, Y_MIN_POS + 1, Y_MAX_POS - 1);
                ey = constrain(ey, Y_MIN_POS + 1, Y_MAX_POS - 1);
                if (position_is_reachable_raw_xy(sx, sy) && position_is_reachable_raw_xy(ex, ey)) {
                  if (g26_debug_flag) {
                    SERIAL_ECHOPAIR(" Connecting with vertical line (sx=", sx);
                    SERIAL_ECHOPAIR(", sy=", sy);
                    SERIAL_ECHOPAIR(") -> (ex=", ex);
                    SERIAL_ECHOPAIR(", ey=", ey);
                    SERIAL_CHAR(')');
                    SERIAL_EOL();
                    debug_current_and_destination(PSTR("Connecting vertical line."));
                  }
                  print_line_from_here_to_there(LOGICAL_X_POSITION(sx), LOGICAL_Y_POSITION(sy), g26_layer_height, LOGICAL_X_POSITION(ex), LOGICAL_Y_POSITION(ey), g26_layer_height);
                }
                bit_set(vertical_mesh_line_flags, i, j);   
              }
            }
          }
        }
      }
    }
    return false;
  }
  void unified_bed_leveling::move_to(const float &x, const float &y, const float &z, const float &e_delta) {
    float feed_value;
    static float last_z = -999.99;
    bool has_xy_component = (x != current_position[X_AXIS] || y != current_position[Y_AXIS]); 
    if (z != last_z) {
      last_z = z;
      feed_value = planner.max_feedrate_mm_s[Z_AXIS]/(3.0);  
      destination[X_AXIS] = current_position[X_AXIS];
      destination[Y_AXIS] = current_position[Y_AXIS];
      destination[Z_AXIS] = z;                          
      destination[E_AXIS] = current_position[E_AXIS];
      G26_line_to_destination(feed_value);
      stepper.synchronize();
      set_destination_to_current();
    }
    feed_value = has_xy_component ? PLANNER_XY_FEEDRATE() / 10.0 : planner.max_feedrate_mm_s[E_AXIS] / 1.5;
    if (g26_debug_flag) SERIAL_ECHOLNPAIR("in move_to() feed_value for XY:", feed_value);
    destination[X_AXIS] = x;
    destination[Y_AXIS] = y;
    destination[E_AXIS] += e_delta;
    G26_line_to_destination(feed_value);
    stepper.synchronize();
    set_destination_to_current();
  }
  void unified_bed_leveling::retract_filament(const float where[XYZE]) {
    if (!g26_retracted) { 
      g26_retracted = true;
      move_to(where, -1.0 * g26_retraction_multiplier);
    }
  }
  void unified_bed_leveling::recover_filament(const float where[XYZE]) {
    if (g26_retracted) { 
      move_to(where, 1.2 * g26_retraction_multiplier);
      g26_retracted = false;
    }
  }
  void unified_bed_leveling::print_line_from_here_to_there(const float &sx, const float &sy, const float &sz, const float &ex, const float &ey, const float &ez) {
    const float dx_s = current_position[X_AXIS] - sx,   
                dy_s = current_position[Y_AXIS] - sy,
                dist_start = HYPOT2(dx_s, dy_s),        
                dx_e = current_position[X_AXIS] - ex,   
                dy_e = current_position[Y_AXIS] - ey,
                dist_end = HYPOT2(dx_e, dy_e),
                line_length = HYPOT(ex - sx, ey - sy);
    if (dist_end < dist_start && (SIZE_OF_INTERSECTION_CIRCLES) < FABS(line_length)) {
      return print_line_from_here_to_there(ex, ey, ez, sx, sy, sz);
    }
    if (dist_start > 2.0) {
      retract_filament(destination);
      move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + 0.500, 0.0);  
      move_to(sx, sy, sz + 0.500, 0.0); 
    }
    move_to(sx, sy, sz, 0.0); 
    const float e_pos_delta = line_length * g26_e_axis_feedrate * g26_extrusion_multiplier;
    recover_filament(destination);
    move_to(ex, ey, ez, e_pos_delta);  
  }
  bool unified_bed_leveling::parse_G26_parameters() {
    g26_extrusion_multiplier  = EXTRUSION_MULTIPLIER;
    g26_retraction_multiplier = RETRACTION_MULTIPLIER;
    g26_nozzle                = NOZZLE;
    g26_filament_diameter     = FILAMENT;
    g26_layer_height          = LAYER_HEIGHT;
    g26_prime_length          = PRIME_LENGTH;
    g26_bed_temp              = BED_TEMP;
    g26_hotend_temp           = HOTEND_TEMP;
    g26_prime_flag            = 0;
    g26_ooze_amount           = parser.linearval('O', OOZE_AMOUNT);
    g26_keep_heaters_on       = parser.boolval('K');
    g26_continue_with_closest = parser.boolval('C');
    if (parser.seenval('B')) {
      g26_bed_temp = parser.value_celsius();
      if (!WITHIN(g26_bed_temp, 15, 140)) {
        SERIAL_PROTOCOLLNPGM("?Specified bed temperature not plausible.");
        return UBL_ERR;
      }
    }
    if (parser.seenval('L')) {
      g26_layer_height = parser.value_linear_units();
      if (!WITHIN(g26_layer_height, 0.0, 2.0)) {
        SERIAL_PROTOCOLLNPGM("?Specified layer height not plausible.");
        return UBL_ERR;
      }
    }
    if (parser.seen('Q')) {
      if (parser.has_value()) {
        g26_retraction_multiplier = parser.value_float();
        if (!WITHIN(g26_retraction_multiplier, 0.05, 15.0)) {
          SERIAL_PROTOCOLLNPGM("?Specified Retraction Multiplier not plausible.");
          return UBL_ERR;
        }
      }
      else {
        SERIAL_PROTOCOLLNPGM("?Retraction Multiplier must be specified.");
        return UBL_ERR;
      }
    }
    if (parser.seenval('S')) {
      g26_nozzle = parser.value_float();
      if (!WITHIN(g26_nozzle, 0.1, 1.0)) {
        SERIAL_PROTOCOLLNPGM("?Specified nozzle size not plausible.");
        return UBL_ERR;
      }
    }
    if (parser.seen('P')) {
      if (!parser.has_value()) {
        #if ENABLED(NEWPANEL)
          g26_prime_flag = -1;
        #else
          SERIAL_PROTOCOLLNPGM("?Prime length must be specified when not using an LCD.");
          return UBL_ERR;
        #endif
      }
      else {
        g26_prime_flag++;
        g26_prime_length = parser.value_linear_units();
        if (!WITHIN(g26_prime_length, 0.0, 25.0)) {
          SERIAL_PROTOCOLLNPGM("?Specified prime length not plausible.");
          return UBL_ERR;
        }
      }
    }
    if (parser.seenval('F')) {
      g26_filament_diameter = parser.value_linear_units();
      if (!WITHIN(g26_filament_diameter, 1.0, 4.0)) {
        SERIAL_PROTOCOLLNPGM("?Specified filament size not plausible.");
        return UBL_ERR;
      }
    }
    g26_extrusion_multiplier *= sq(1.75) / sq(g26_filament_diameter); 
    g26_extrusion_multiplier *= g26_filament_diameter * sq(g26_nozzle) / sq(0.3); 
    if (parser.seenval('H')) {
      g26_hotend_temp = parser.value_celsius();
      if (!WITHIN(g26_hotend_temp, 165, 280)) {
        SERIAL_PROTOCOLLNPGM("?Specified nozzle temperature not plausible.");
        return UBL_ERR;
      }
    }
    if (parser.seen('U')) {
      randomSeed(millis());
      random_deviation = parser.has_value() ? parser.value_float() : 50.0;
    }
    #if ENABLED(NEWPANEL)
      g26_repeats = parser.intval('R', GRID_MAX_POINTS + 1);
    #else
      if (!parser.seen('R')) {
        SERIAL_PROTOCOLLNPGM("?(R)epeat must be specified when not using an LCD.");
        return UBL_ERR;
      }
      else
        g26_repeats = parser.has_value() ? parser.value_int() : GRID_MAX_POINTS + 1;
    #endif
    if (g26_repeats < 1) {
      SERIAL_PROTOCOLLNPGM("?(R)epeat value not plausible; must be at least 1.");
      return UBL_ERR;
    }
    g26_x_pos = parser.linearval('X', current_position[X_AXIS]);
    g26_y_pos = parser.linearval('Y', current_position[Y_AXIS]);
    if (!position_is_reachable_xy(g26_x_pos, g26_y_pos)) {
      SERIAL_PROTOCOLLNPGM("?Specified X,Y coordinate out of bounds.");
      return UBL_ERR;
    }
    set_bed_leveling_enabled(!parser.seen('D'));
    return UBL_OK;
  }
  #if ENABLED(NEWPANEL)
    bool unified_bed_leveling::exit_from_g26() {
      lcd_setstatusPGM(PSTR("Leaving G26"), -1);
      while (ubl_lcd_clicked()) idle();
      return UBL_ERR;
    }
  #endif
  bool unified_bed_leveling::turn_on_heaters() {
    millis_t next = millis() + 5000UL;
    #if HAS_TEMP_BED
      #if ENABLED(ULTRA_LCD)
        if (g26_bed_temp > 25) {
          lcd_setstatusPGM(PSTR("G26 Heating Bed."), 99);
          lcd_quick_feedback();
      #endif
          has_control_of_lcd_panel = true;
          thermalManager.setTargetBed(g26_bed_temp);
          while (abs(thermalManager.degBed() - g26_bed_temp) > 3) {
            #if ENABLED(NEWPANEL)
              if (ubl_lcd_clicked()) return exit_from_g26();
            #endif
            if (PENDING(millis(), next)) {
              next = millis() + 5000UL;
              print_heaterstates();
            }
            idle();
          }
      #if ENABLED(ULTRA_LCD)
        }
        lcd_setstatusPGM(PSTR("G26 Heating Nozzle."), 99);
        lcd_quick_feedback();
      #endif
    #endif
    thermalManager.setTargetHotend(g26_hotend_temp, 0);
    while (abs(thermalManager.degHotend(0) - g26_hotend_temp) > 3) {
      #if ENABLED(NEWPANEL)
        if (ubl_lcd_clicked()) return exit_from_g26();
      #endif
      if (PENDING(millis(), next)) {
        next = millis() + 5000UL;
        print_heaterstates();
      }
      idle();
    }
    #if ENABLED(ULTRA_LCD)
      lcd_reset_status();
      lcd_quick_feedback();
    #endif
    return UBL_OK;
  }
  bool unified_bed_leveling::prime_nozzle() {
    #if ENABLED(NEWPANEL)
      float Total_Prime = 0.0;
      if (g26_prime_flag == -1) {  
        has_control_of_lcd_panel = true;
        lcd_setstatusPGM(PSTR("User-Controlled Prime"), 99);
        chirp_at_user();
        set_destination_to_current();
        recover_filament(destination); 
        while (!ubl_lcd_clicked()) {
          chirp_at_user();
          destination[E_AXIS] += 0.25;
          #ifdef PREVENT_LENGTHY_EXTRUDE
            Total_Prime += 0.25;
            if (Total_Prime >= EXTRUDE_MAXLENGTH) return UBL_ERR;
          #endif
          G26_line_to_destination(planner.max_feedrate_mm_s[E_AXIS] / 15.0);
          stepper.synchronize();    
          set_destination_to_current();
          idle();
        }
        while (ubl_lcd_clicked()) idle();           
        #if ENABLED(ULTRA_LCD)
          strcpy_P(lcd_status_message, PSTR("Done Priming")); 
          lcd_setstatusPGM(PSTR("Done Priming"), 99);
          lcd_quick_feedback();
        #endif
        has_control_of_lcd_panel = false;
      }
      else {
    #else
    {
    #endif
      #if ENABLED(ULTRA_LCD)
        lcd_setstatusPGM(PSTR("Fixed Length Prime."), 99);
        lcd_quick_feedback();
      #endif
      set_destination_to_current();
      destination[E_AXIS] += g26_prime_length;
      G26_line_to_destination(planner.max_feedrate_mm_s[E_AXIS] / 15.0);
      stepper.synchronize();
      set_destination_to_current();
      retract_filament(destination);
    }
    return UBL_OK;
  }
#endif 
