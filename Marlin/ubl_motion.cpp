#include "MarlinConfig.h"
#if ENABLED(AUTO_BED_LEVELING_UBL)
  #include "Marlin.h"
  #include "ubl.h"
  #include "planner.h"
  #include "stepper.h"
  #include <avr/io.h>
  #include <math.h>
  extern float destination[XYZE];
  #if AVR_AT90USB1286_FAMILY  
    inline void set_current_to_destination() { COPY(current_position, destination); }
  #else
    extern void set_current_to_destination();
  #endif
#if ENABLED(DELTA)
  extern float delta[ABC],
               endstop_adj[ABC];
  extern float delta_radius,
               delta_tower_angle_trim[2],
               delta_tower[ABC][2],
               delta_diagonal_rod,
               delta_calibration_radius,
               delta_diagonal_rod_2_tower[ABC],
               delta_segments_per_second,
               delta_clip_start_height;
  extern float delta_safe_distance_from_top();
#endif
  static void debug_echo_axis(const AxisEnum axis) {
    if (current_position[axis] == destination[axis])
      SERIAL_ECHOPGM("-------------");
    else
      SERIAL_ECHO_F(destination[X_AXIS], 6);
  }
  void debug_current_and_destination(const char *title) {
    if (*title != '!' && !ubl.g26_debug_flag) return;
    const float de = destination[E_AXIS] - current_position[E_AXIS];
    if (de == 0.0) return; 
    const float dx = destination[X_AXIS] - current_position[X_AXIS],
                dy = destination[Y_AXIS] - current_position[Y_AXIS],
                xy_dist = HYPOT(dx, dy);
    if (xy_dist == 0.0) return;
    SERIAL_ECHOPGM("   fpmm=");
    const float fpmm = de / xy_dist;
    SERIAL_ECHO_F(fpmm, 6);
    SERIAL_ECHOPGM("    current=( ");
    SERIAL_ECHO_F(current_position[X_AXIS], 6);
    SERIAL_ECHOPGM(", ");
    SERIAL_ECHO_F(current_position[Y_AXIS], 6);
    SERIAL_ECHOPGM(", ");
    SERIAL_ECHO_F(current_position[Z_AXIS], 6);
    SERIAL_ECHOPGM(", ");
    SERIAL_ECHO_F(current_position[E_AXIS], 6);
    SERIAL_ECHOPGM(" )   destination=( ");
    debug_echo_axis(X_AXIS);
    SERIAL_ECHOPGM(", ");
    debug_echo_axis(Y_AXIS);
    SERIAL_ECHOPGM(", ");
    debug_echo_axis(Z_AXIS);
    SERIAL_ECHOPGM(", ");
    debug_echo_axis(E_AXIS);
    SERIAL_ECHOPGM(" )   ");
    SERIAL_ECHO(title);
    SERIAL_EOL();
  }
  void unified_bed_leveling::line_to_destination_cartesian(const float &feed_rate, uint8_t extruder) {
    const float start[XYZE] = {
                  current_position[X_AXIS],
                  current_position[Y_AXIS],
                  current_position[Z_AXIS],
                  current_position[E_AXIS]
                },
                end[XYZE] = {
                  destination[X_AXIS],
                  destination[Y_AXIS],
                  destination[Z_AXIS],
                  destination[E_AXIS]
                };
    const int cell_start_xi = get_cell_index_x(RAW_X_POSITION(start[X_AXIS])),
              cell_start_yi = get_cell_index_y(RAW_Y_POSITION(start[Y_AXIS])),
              cell_dest_xi  = get_cell_index_x(RAW_X_POSITION(end[X_AXIS])),
              cell_dest_yi  = get_cell_index_y(RAW_Y_POSITION(end[Y_AXIS]));
    if (g26_debug_flag) {
      SERIAL_ECHOPAIR(" ubl.line_to_destination(xe=", end[X_AXIS]);
      SERIAL_ECHOPAIR(", ye=", end[Y_AXIS]);
      SERIAL_ECHOPAIR(", ze=", end[Z_AXIS]);
      SERIAL_ECHOPAIR(", ee=", end[E_AXIS]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
      debug_current_and_destination(PSTR("Start of ubl.line_to_destination()"));
    }
    if (cell_start_xi == cell_dest_xi && cell_start_yi == cell_dest_yi) { 
      if (!WITHIN(cell_dest_xi, 0, GRID_MAX_POINTS_X - 1) || !WITHIN(cell_dest_yi, 0, GRID_MAX_POINTS_Y - 1)) {
        planner._buffer_line(end[X_AXIS], end[Y_AXIS], end[Z_AXIS] + state.z_offset, end[E_AXIS], feed_rate, extruder);
        set_current_to_destination();
        if (g26_debug_flag)
          debug_current_and_destination(PSTR("out of bounds in ubl.line_to_destination()"));
        return;
      }
      FINAL_MOVE:
      const float xratio = (RAW_X_POSITION(end[X_AXIS]) - mesh_index_to_xpos(cell_dest_xi)) * (1.0 / (MESH_X_DIST)),
                  z1 = z_values[cell_dest_xi    ][cell_dest_yi    ] + xratio *
                      (z_values[cell_dest_xi + 1][cell_dest_yi    ] - z_values[cell_dest_xi][cell_dest_yi    ]),
                  z2 = z_values[cell_dest_xi    ][cell_dest_yi + 1] + xratio *
                      (z_values[cell_dest_xi + 1][cell_dest_yi + 1] - z_values[cell_dest_xi][cell_dest_yi + 1]);
      const float yratio = (RAW_Y_POSITION(end[Y_AXIS]) - mesh_index_to_ypos(cell_dest_yi)) * (1.0 / (MESH_Y_DIST));
      float z0 = z1 + (z2 - z1) * yratio;
      z0 *= fade_scaling_factor_for_z(end[Z_AXIS]);
      if (isnan(z0)) z0 = 0.0;
      planner._buffer_line(end[X_AXIS], end[Y_AXIS], end[Z_AXIS] + z0 + state.z_offset, end[E_AXIS], feed_rate, extruder);
      if (g26_debug_flag)
        debug_current_and_destination(PSTR("FINAL_MOVE in ubl.line_to_destination()"));
      set_current_to_destination();
      return;
    }
    const float dx = end[X_AXIS] - start[X_AXIS],
                dy = end[Y_AXIS] - start[Y_AXIS];
    const int left_flag = dx < 0.0 ? 1 : 0,
              down_flag = dy < 0.0 ? 1 : 0;
    const float adx = left_flag ? -dx : dx,
                ady = down_flag ? -dy : dy;
    const int dxi = cell_start_xi == cell_dest_xi ? 0 : left_flag ? -1 : 1,
              dyi = cell_start_yi == cell_dest_yi ? 0 : down_flag ? -1 : 1;
    const bool use_x_dist = adx > ady;
    float on_axis_distance = use_x_dist ? dx : dy,
          e_position = end[E_AXIS] - start[E_AXIS],
          z_position = end[Z_AXIS] - start[Z_AXIS];
    const float e_normalized_dist = e_position / on_axis_distance,
                z_normalized_dist = z_position / on_axis_distance;
    int current_xi = cell_start_xi,
        current_yi = cell_start_yi;
    const float m = dy / dx,
                c = start[Y_AXIS] - m * start[X_AXIS];
    const bool inf_normalized_flag = (isinf(e_normalized_dist) != 0),
               inf_m_flag = (isinf(m) != 0);
    if (dxi == 0) {       
      current_yi += down_flag;  
      while (current_yi != cell_dest_yi + down_flag) {
        current_yi += dyi;
        const float next_mesh_line_y = LOGICAL_Y_POSITION(mesh_index_to_ypos(current_yi));
        const float x = inf_m_flag ? start[X_AXIS] : (next_mesh_line_y - c) / m;
        float z0 = z_correction_for_x_on_horizontal_mesh_line(x, current_xi, current_yi);
        z0 *= fade_scaling_factor_for_z(end[Z_AXIS]);
        if (isnan(z0)) z0 = 0.0;
        const float y = LOGICAL_Y_POSITION(mesh_index_to_ypos(current_yi));
        if (y != start[Y_AXIS]) {
          if (!inf_normalized_flag) {
            on_axis_distance = use_x_dist ? x - start[X_AXIS] : y - start[Y_AXIS];
            e_position = start[E_AXIS] + on_axis_distance * e_normalized_dist;
            z_position = start[Z_AXIS] + on_axis_distance * z_normalized_dist;
          }
          else {
            e_position = end[E_AXIS];
            z_position = end[Z_AXIS];
          }
          planner._buffer_line(x, y, z_position + z0 + state.z_offset, e_position, feed_rate, extruder);
        } 
      }
      if (g26_debug_flag)
        debug_current_and_destination(PSTR("vertical move done in ubl.line_to_destination()"));
      if (current_position[X_AXIS] != end[X_AXIS] || current_position[Y_AXIS] != end[Y_AXIS])
        goto FINAL_MOVE;
      set_current_to_destination();
      return;
    }
    if (dyi == 0) {             
      current_xi += left_flag;  
      while (current_xi != cell_dest_xi + left_flag) {
        current_xi += dxi;
        const float next_mesh_line_x = LOGICAL_X_POSITION(mesh_index_to_xpos(current_xi)),
                    y = m * next_mesh_line_x + c;   
        float z0 = z_correction_for_y_on_vertical_mesh_line(y, current_xi, current_yi);
        z0 *= fade_scaling_factor_for_z(end[Z_AXIS]);
        if (isnan(z0)) z0 = 0.0;
        const float x = LOGICAL_X_POSITION(mesh_index_to_xpos(current_xi));
        if (x != start[X_AXIS]) {
          if (!inf_normalized_flag) {
            on_axis_distance = use_x_dist ? x - start[X_AXIS] : y - start[Y_AXIS];
            e_position = start[E_AXIS] + on_axis_distance * e_normalized_dist;  
            z_position = start[Z_AXIS] + on_axis_distance * z_normalized_dist;
          }
          else {
            e_position = end[E_AXIS];
            z_position = end[Z_AXIS];
          }
          planner._buffer_line(x, y, z_position + z0 + state.z_offset, e_position, feed_rate, extruder);
        } 
      }
      if (g26_debug_flag)
        debug_current_and_destination(PSTR("horizontal move done in ubl.line_to_destination()"));
      if (current_position[X_AXIS] != end[X_AXIS] || current_position[Y_AXIS] != end[Y_AXIS])
        goto FINAL_MOVE;
      set_current_to_destination();
      return;
    }
    int xi_cnt = cell_start_xi - cell_dest_xi,
        yi_cnt = cell_start_yi - cell_dest_yi;
    if (xi_cnt < 0) xi_cnt = -xi_cnt;
    if (yi_cnt < 0) yi_cnt = -yi_cnt;
    current_xi += left_flag;
    current_yi += down_flag;
    while (xi_cnt > 0 || yi_cnt > 0) {
      const float next_mesh_line_x = LOGICAL_X_POSITION(mesh_index_to_xpos(current_xi + dxi)),
                  next_mesh_line_y = LOGICAL_Y_POSITION(mesh_index_to_ypos(current_yi + dyi)),
                  y = m * next_mesh_line_x + c,   
                  x = (next_mesh_line_y - c) / m; 
      if (left_flag == (x > next_mesh_line_x)) { 
        float z0 = z_correction_for_x_on_horizontal_mesh_line(x, current_xi - left_flag, current_yi + dyi);
        z0 *= fade_scaling_factor_for_z(end[Z_AXIS]);
        if (isnan(z0)) z0 = 0.0;
        if (!inf_normalized_flag) {
          on_axis_distance = use_x_dist ? x - start[X_AXIS] : next_mesh_line_y - start[Y_AXIS];
          e_position = start[E_AXIS] + on_axis_distance * e_normalized_dist;
          z_position = start[Z_AXIS] + on_axis_distance * z_normalized_dist;
        }
        else {
          e_position = end[E_AXIS];
          z_position = end[Z_AXIS];
        }
        planner._buffer_line(x, next_mesh_line_y, z_position + z0 + state.z_offset, e_position, feed_rate, extruder);
        current_yi += dyi;
        yi_cnt--;
      }
      else {
        float z0 = z_correction_for_y_on_vertical_mesh_line(y, current_xi + dxi, current_yi - down_flag);
        z0 *= fade_scaling_factor_for_z(end[Z_AXIS]);
        if (isnan(z0)) z0 = 0.0;
        if (!inf_normalized_flag) {
          on_axis_distance = use_x_dist ? next_mesh_line_x - start[X_AXIS] : y - start[Y_AXIS];
          e_position = start[E_AXIS] + on_axis_distance * e_normalized_dist;
          z_position = start[Z_AXIS] + on_axis_distance * z_normalized_dist;
        }
        else {
          e_position = end[E_AXIS];
          z_position = end[Z_AXIS];
        }
        planner._buffer_line(next_mesh_line_x, y, z_position + z0 + state.z_offset, e_position, feed_rate, extruder);
        current_xi += dxi;
        xi_cnt--;
      }
      if (xi_cnt < 0 || yi_cnt < 0) break; 
    }
    if (g26_debug_flag)
      debug_current_and_destination(PSTR("generic move done in ubl.line_to_destination()"));
    if (current_position[X_AXIS] != end[X_AXIS] || current_position[Y_AXIS] != end[Y_AXIS])
      goto FINAL_MOVE;
    set_current_to_destination();
  }
  #if UBL_DELTA
    #define COPY_XYZE( target, source ) { \
                target[X_AXIS] = source[X_AXIS]; \
                target[Y_AXIS] = source[Y_AXIS]; \
                target[Z_AXIS] = source[Z_AXIS]; \
                target[E_AXIS] = source[E_AXIS]; \
            }
    #if IS_SCARA 
      static float scara_feed_factor, scara_oldA, scara_oldB;
    #endif
    inline void _O2 ubl_buffer_segment_raw( float rx, float ry, float rz, float le, float fr ) {
      #if ENABLED(DELTA)  
        const float delta_A = rz + SQRT( delta_diagonal_rod_2_tower[A_AXIS]
                                         - HYPOT2( delta_tower[A_AXIS][X_AXIS] - rx,
                                                   delta_tower[A_AXIS][Y_AXIS] - ry ));
        const float delta_B = rz + SQRT( delta_diagonal_rod_2_tower[B_AXIS]
                                         - HYPOT2( delta_tower[B_AXIS][X_AXIS] - rx,
                                                   delta_tower[B_AXIS][Y_AXIS] - ry ));
        const float delta_C = rz + SQRT( delta_diagonal_rod_2_tower[C_AXIS]
                                         - HYPOT2( delta_tower[C_AXIS][X_AXIS] - rx,
                                                   delta_tower[C_AXIS][Y_AXIS] - ry ));
        planner._buffer_line(delta_A, delta_B, delta_C, le, fr, active_extruder);
      #elif IS_SCARA  
        const float lseg[XYZ] = { LOGICAL_X_POSITION(rx),
                                  LOGICAL_Y_POSITION(ry),
                                  LOGICAL_Z_POSITION(rz)
                                };
        inverse_kinematics(lseg); 
        const float adiff = FABS(delta[A_AXIS] - scara_oldA),
                    bdiff = FABS(delta[B_AXIS] - scara_oldB);
        scara_oldA = delta[A_AXIS];
        scara_oldB = delta[B_AXIS];
        float s_feedrate = max(adiff, bdiff) * scara_feed_factor;
        planner._buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], le, s_feedrate, active_extruder);
      #else 
        const float lx = LOGICAL_X_POSITION(rx),
                    ly = LOGICAL_Y_POSITION(ry),
                    lz = LOGICAL_Z_POSITION(rz);
        planner._buffer_line(lx, ly, lz, le, fr, active_extruder);
      #endif
    }
    bool _O2 unified_bed_leveling::prepare_segmented_line_to(const float ltarget[XYZE], const float &feedrate) {
      if (!position_is_reachable_xy(ltarget[X_AXIS], ltarget[Y_AXIS]))  
        return true; 
      const float tot_dx = ltarget[X_AXIS] - current_position[X_AXIS],
                  tot_dy = ltarget[Y_AXIS] - current_position[Y_AXIS],
                  tot_dz = ltarget[Z_AXIS] - current_position[Z_AXIS],
                  tot_de = ltarget[E_AXIS] - current_position[E_AXIS];
      const float cartesian_xy_mm = HYPOT(tot_dx, tot_dy);  
      #if IS_KINEMATIC
        const float seconds = cartesian_xy_mm / feedrate;                                  
        uint16_t segments = lroundf(delta_segments_per_second * seconds),                  
                 seglimit = lroundf(cartesian_xy_mm * (1.0 / (DELTA_SEGMENT_MIN_LENGTH))); 
        NOMORE(segments, seglimit);                                                        
      #else
        uint16_t segments = lroundf(cartesian_xy_mm * (1.0 / (DELTA_SEGMENT_MIN_LENGTH))); 
      #endif
      NOLESS(segments, 1);                        
      const float inv_segments = 1.0 / segments;  
      #if IS_SCARA 
        scara_feed_factor = cartesian_xy_mm * inv_segments * feedrate;
        scara_oldA = stepper.get_axis_position_degrees(A_AXIS);
        scara_oldB = stepper.get_axis_position_degrees(B_AXIS);
      #endif
      const float seg_dx = tot_dx * inv_segments,
                  seg_dy = tot_dy * inv_segments,
                  seg_dz = tot_dz * inv_segments,
                  seg_de = tot_de * inv_segments;
      float seg_rx = RAW_X_POSITION(current_position[X_AXIS]),
            seg_ry = RAW_Y_POSITION(current_position[Y_AXIS]),
            seg_rz = RAW_Z_POSITION(current_position[Z_AXIS]),
            seg_le = current_position[E_AXIS];
      const bool above_fade_height = (
        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          planner.z_fade_height != 0 && planner.z_fade_height < RAW_Z_POSITION(ltarget[Z_AXIS])
        #else
          false
        #endif
      );
      if (!state.active || above_fade_height) {   
        const float z_offset = state.active ? state.z_offset : 0.0;
        do {
          if (--segments) {     
            seg_rx += seg_dx;
            seg_ry += seg_dy;
            seg_rz += seg_dz;
            seg_le += seg_de;
          } else {              
            seg_rx = RAW_X_POSITION(ltarget[X_AXIS]);
            seg_ry = RAW_Y_POSITION(ltarget[Y_AXIS]);
            seg_rz = RAW_Z_POSITION(ltarget[Z_AXIS]);
            seg_le = ltarget[E_AXIS];
          }
          ubl_buffer_segment_raw( seg_rx, seg_ry, seg_rz + z_offset, seg_le, feedrate );
        } while (segments);
        return false; 
      }
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        const float fade_scaling_factor = fade_scaling_factor_for_z(ltarget[Z_AXIS]);
      #endif
      seg_rx += seg_dx;
      seg_ry += seg_dy;
      seg_rz += seg_dz;
      seg_le += seg_de;
      for(;;) {  
        int8_t cell_xi = (seg_rx - (UBL_MESH_MIN_X)) * (1.0 / (MESH_X_DIST)),
               cell_yi = (seg_ry - (UBL_MESH_MIN_Y)) * (1.0 / (MESH_X_DIST));
        cell_xi = constrain(cell_xi, 0, (GRID_MAX_POINTS_X) - 1);
        cell_yi = constrain(cell_yi, 0, (GRID_MAX_POINTS_Y) - 1);
        const float x0 = mesh_index_to_xpos(cell_xi),   
                    y0 = mesh_index_to_ypos(cell_yi);
        float z_x0y0 = z_values[cell_xi  ][cell_yi  ],  
              z_x1y0 = z_values[cell_xi+1][cell_yi  ],  
              z_x0y1 = z_values[cell_xi  ][cell_yi+1],  
              z_x1y1 = z_values[cell_xi+1][cell_yi+1];  
        if (isnan(z_x0y0)) z_x0y0 = 0;              
        if (isnan(z_x1y0)) z_x1y0 = 0;              
        if (isnan(z_x0y1)) z_x0y1 = 0;              
        if (isnan(z_x1y1)) z_x1y1 = 0;              
        float cx = seg_rx - x0,   
              cy = seg_ry - y0;
        const float z_xmy0 = (z_x1y0 - z_x0y0) * (1.0 / (MESH_X_DIST)),   
                    z_xmy1 = (z_x1y1 - z_x0y1) * (1.0 / (MESH_X_DIST));   
              float z_cxy0 = z_x0y0 + z_xmy0 * cx;            
        const float z_cxy1 = z_x0y1 + z_xmy1 * cx,            
                    z_cxyd = z_cxy1 - z_cxy0;                 
              float z_cxym = z_cxyd * (1.0 / (MESH_Y_DIST));  
        const float z_sxy0 = z_xmy0 * seg_dx,                                     
                    z_sxym = (z_xmy1 - z_xmy0) * (1.0 / (MESH_Y_DIST)) * seg_dx;  
        for(;;) {  
          float z_cxcy = z_cxy0 + z_cxym * cy;      
          #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
            z_cxcy *= fade_scaling_factor;          
          #endif
          z_cxcy += state.z_offset;                 
          if (--segments == 0) {                    
            seg_rx = RAW_X_POSITION(ltarget[X_AXIS]);
            seg_ry = RAW_Y_POSITION(ltarget[Y_AXIS]);
            seg_rz = RAW_Z_POSITION(ltarget[Z_AXIS]);
            seg_le = ltarget[E_AXIS];
          }
          ubl_buffer_segment_raw( seg_rx, seg_ry, seg_rz + z_cxcy, seg_le, feedrate );
          if (segments == 0 )                       
            return false;                           
          seg_rx += seg_dx;
          seg_ry += seg_dy;
          seg_rz += seg_dz;
          seg_le += seg_de;
          cx += seg_dx;
          cy += seg_dy;
          if (!WITHIN(cx, 0, MESH_X_DIST) || !WITHIN(cy, 0, MESH_Y_DIST)) {  
            break;
          }
          z_cxy0 += z_sxy0;   
          z_cxym += z_sxym;   
        } 
      } 
    }
  #endif 
#endif 
