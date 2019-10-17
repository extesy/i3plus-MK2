#include "nozzle.h"
#include "Marlin.h"
#include "point_t.h"
void Nozzle::stroke(
  _UNUSED point_t const &start,
  _UNUSED point_t const &end,
  _UNUSED uint8_t const &strokes
) {
  #if ENABLED(NOZZLE_CLEAN_FEATURE)
    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      point_t const initial = {
        current_position[X_AXIS],
        current_position[Y_AXIS],
        current_position[Z_AXIS],
        current_position[E_AXIS]
      };
    #endif 
    do_blocking_move_to_xy(start.x, start.y);
    do_blocking_move_to_z(start.z);
    for (uint8_t i = 0; i < (strokes >>1); i++) {
      do_blocking_move_to_xy(end.x, end.y);
      do_blocking_move_to_xy(start.x, start.y);
    }
    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      do_blocking_move_to(initial.x, initial.y, initial.z);
    #endif 
  #endif 
}
void Nozzle::zigzag(
  _UNUSED point_t const &start,
  _UNUSED point_t const &end,
  _UNUSED uint8_t const &strokes,
  _UNUSED uint8_t const &objects
) {
  #if ENABLED(NOZZLE_CLEAN_FEATURE)
    const float A = nozzle_clean_horizontal ? nozzle_clean_height : nozzle_clean_length, 
                P = (nozzle_clean_horizontal ? nozzle_clean_length : nozzle_clean_height) / (objects << 1); 
    if (A <= 0.0f || P <= 0.0f ) return;
    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      point_t const initial = {
        current_position[X_AXIS],
        current_position[Y_AXIS],
        current_position[Z_AXIS],
        current_position[E_AXIS]
      };
    #endif 
    for (uint8_t j = 0; j < strokes; j++) {
      for (uint8_t i = 0; i < (objects << 1); i++) {
        float const x = start.x + ( nozzle_clean_horizontal ? i * P : (A/P) * (P - FABS(FMOD((i*P), (2*P)) - P)) );
        float const y = start.y + (!nozzle_clean_horizontal ? i * P : (A/P) * (P - FABS(FMOD((i*P), (2*P)) - P)) );
        do_blocking_move_to_xy(x, y);
        if (i == 0) do_blocking_move_to_z(start.z);
      }
      for (int i = (objects << 1); i > -1; i--) {
        float const x = start.x + ( nozzle_clean_horizontal ? i * P : (A/P) * (P - FABS(FMOD((i*P), (2*P)) - P)) );
        float const y = start.y + (!nozzle_clean_horizontal ? i * P : (A/P) * (P - FABS(FMOD((i*P), (2*P)) - P)) );
        do_blocking_move_to_xy(x, y);
      }
    }
    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      do_blocking_move_to_z(initial.z);
      do_blocking_move_to_xy(initial.x, initial.y);
    #endif 
  #endif 
}
void Nozzle::circle(
  _UNUSED point_t const &start,
  _UNUSED point_t const &middle,
  _UNUSED uint8_t const &strokes,
  _UNUSED float const &radius
) {
  #if ENABLED(NOZZLE_CLEAN_FEATURE)
    if (strokes == 0) return;
    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      point_t const initial = {
        current_position[X_AXIS],
        current_position[Y_AXIS],
        current_position[Z_AXIS],
        current_position[E_AXIS]
      };
    #endif 
    if (start.z <= current_position[Z_AXIS]) {
      do_blocking_move_to_xy(start.x, start.y);
      do_blocking_move_to_z(start.z);
    }
    else {
      do_blocking_move_to_z(start.z);
      do_blocking_move_to_xy(start.x, start.y);
    }
    float x, y;
    for (uint8_t s = 0; s < strokes; s++) {
      for (uint8_t i = 0; i < NOZZLE_CLEAN_CIRCLE_FN; i++) {
        x = middle.x + sin((M_2_PI / NOZZLE_CLEAN_CIRCLE_FN) * i) * radius;
        y = middle.y + cos((M_2_PI / NOZZLE_CLEAN_CIRCLE_FN) * i) * radius;
        do_blocking_move_to_xy(x, y);
      }
    }
    do_blocking_move_to_xy(start.x, start.y);
    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      if (start.z <= initial.z) {
        do_blocking_move_to_z(initial.z);
        do_blocking_move_to_xy(initial.x, initial.y);
      }
      else {
        do_blocking_move_to_xy(initial.x, initial.y);
        do_blocking_move_to_z(initial.z);
      }
    #endif 
  #endif 
}
void Nozzle::clean(
  _UNUSED uint8_t const &pattern,
  _UNUSED uint8_t const &strokes,
  _UNUSED float const &radius,
  _UNUSED uint8_t const &objects
) {
  #if ENABLED(NOZZLE_CLEAN_FEATURE)
    #if ENABLED(DELTA)
      if (current_position[Z_AXIS] > delta_clip_start_height)
        do_blocking_move_to_z(delta_clip_start_height);
    #endif
    switch (pattern) {
      case 1:
        Nozzle::zigzag(
          NOZZLE_CLEAN_START_POINT,
          NOZZLE_CLEAN_END_POINT, strokes, objects);
        break;
      case 2:
        Nozzle::circle(
          NOZZLE_CLEAN_START_POINT,
          NOZZLE_CLEAN_CIRCLE_MIDDLE, strokes, radius);
        break;
      default:
        Nozzle::stroke(
          NOZZLE_CLEAN_START_POINT,
          NOZZLE_CLEAN_END_POINT, strokes);
    }
  #endif 
}
void Nozzle::park(
  _UNUSED uint8_t const &z_action
) {
  #if ENABLED(NOZZLE_PARK_FEATURE)
    float const z = current_position[Z_AXIS];
    point_t const park = NOZZLE_PARK_POINT;
    switch(z_action) {
      case 1: 
        do_blocking_move_to_z(park.z);
        break;
      case 2: 
        do_blocking_move_to_z(
          (z + park.z > Z_MAX_POS) ? Z_MAX_POS : z + park.z);
        break;
      default: 
        if (current_position[Z_AXIS] < park.z)
          do_blocking_move_to_z(park.z);
    }
    do_blocking_move_to_xy(park.x, park.y);
  #endif 
}
