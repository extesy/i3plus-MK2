#ifndef __NOZZLE_H__
#define __NOZZLE_H__
#include "Marlin.h"
#include "point_t.h"
#if ENABLED(NOZZLE_CLEAN_FEATURE)
  constexpr float nozzle_clean_start_point[4] = NOZZLE_CLEAN_START_POINT,
                  nozzle_clean_end_point[4] = NOZZLE_CLEAN_END_POINT,
                  nozzle_clean_length = FABS(nozzle_clean_start_point[X_AXIS] - nozzle_clean_end_point[X_AXIS]), 
                  nozzle_clean_height = FABS(nozzle_clean_start_point[Y_AXIS] - nozzle_clean_end_point[Y_AXIS]); 
  constexpr bool nozzle_clean_horizontal = nozzle_clean_length >= nozzle_clean_height; 
#endif 
class Nozzle {
  private:
    static void stroke(
      _UNUSED point_t const &start,
      _UNUSED point_t const &end,
      _UNUSED uint8_t const &strokes
    ) _Os;
    static void zigzag(
      _UNUSED point_t const &start,
      _UNUSED point_t const &end,
      _UNUSED uint8_t const &strokes,
      _UNUSED uint8_t const &objects
    ) _Os;
    static void circle(
      _UNUSED point_t const &start,
      _UNUSED point_t const &middle,
      _UNUSED uint8_t const &strokes,
      _UNUSED float const &radius
    ) _Os;
  public:
    static void clean(
      _UNUSED uint8_t const &pattern,
      _UNUSED uint8_t const &strokes,
      _UNUSED float const &radius,
      _UNUSED uint8_t const &objects = 0
    ) _Os;
    static void park(
      _UNUSED uint8_t const &z_action
    ) _Os;
};
#endif
