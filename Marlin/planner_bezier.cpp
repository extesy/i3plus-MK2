#include "Marlin.h"
#if ENABLED(BEZIER_CURVE_SUPPORT)
#include "planner.h"
#include "language.h"
#include "temperature.h"
#define MIN_STEP 0.002
#define MAX_STEP 0.1
#define SIGMA 0.1
inline static float interp(float a, float b, float t) { return (1.0 - t) * a + t * b; }
inline static float eval_bezier(float a, float b, float c, float d, float t) {
  float iab = interp(a, b, t);
  float ibc = interp(b, c, t);
  float icd = interp(c, d, t);
  float iabc = interp(iab, ibc, t);
  float ibcd = interp(ibc, icd, t);
  float iabcd = interp(iabc, ibcd, t);
  return iabcd;
}
inline static float dist1(float x1, float y1, float x2, float y2) { return FABS(x1 - x2) + FABS(y1 - y2); }
void cubic_b_spline(const float position[NUM_AXIS], const float target[NUM_AXIS], const float offset[4], float fr_mm_s, uint8_t extruder) {
  float first0 = position[X_AXIS] + offset[0];
  float first1 = position[Y_AXIS] + offset[1];
  float second0 = target[X_AXIS] + offset[2];
  float second1 = target[Y_AXIS] + offset[3];
  float t = 0.0;
  float bez_target[4];
  bez_target[X_AXIS] = position[X_AXIS];
  bez_target[Y_AXIS] = position[Y_AXIS];
  float step = MAX_STEP;
  millis_t next_idle_ms = millis() + 200UL;
  while (t < 1.0) {
    thermalManager.manage_heater();
    millis_t now = millis();
    if (ELAPSED(now, next_idle_ms)) {
      next_idle_ms = now + 200UL;
      idle();
    }
    bool did_reduce = false;
    float new_t = t + step;
    NOMORE(new_t, 1.0);
    float new_pos0 = eval_bezier(position[X_AXIS], first0, second0, target[X_AXIS], new_t);
    float new_pos1 = eval_bezier(position[Y_AXIS], first1, second1, target[Y_AXIS], new_t);
    for (;;) {
      if (new_t - t < (MIN_STEP)) break;
      float candidate_t = 0.5 * (t + new_t);
      float candidate_pos0 = eval_bezier(position[X_AXIS], first0, second0, target[X_AXIS], candidate_t);
      float candidate_pos1 = eval_bezier(position[Y_AXIS], first1, second1, target[Y_AXIS], candidate_t);
      float interp_pos0 = 0.5 * (bez_target[X_AXIS] + new_pos0);
      float interp_pos1 = 0.5 * (bez_target[Y_AXIS] + new_pos1);
      if (dist1(candidate_pos0, candidate_pos1, interp_pos0, interp_pos1) <= (SIGMA)) break;
      new_t = candidate_t;
      new_pos0 = candidate_pos0;
      new_pos1 = candidate_pos1;
      did_reduce = true;
    }
    if (!did_reduce) for (;;) {
      if (new_t - t > MAX_STEP) break;
      float candidate_t = t + 2.0 * (new_t - t);
      if (candidate_t >= 1.0) break;
      float candidate_pos0 = eval_bezier(position[X_AXIS], first0, second0, target[X_AXIS], candidate_t);
      float candidate_pos1 = eval_bezier(position[Y_AXIS], first1, second1, target[Y_AXIS], candidate_t);
      float interp_pos0 = 0.5 * (bez_target[X_AXIS] + candidate_pos0);
      float interp_pos1 = 0.5 * (bez_target[Y_AXIS] + candidate_pos1);
      if (dist1(new_pos0, new_pos1, interp_pos0, interp_pos1) > (SIGMA)) break;
      new_t = candidate_t;
      new_pos0 = candidate_pos0;
      new_pos1 = candidate_pos1;
    }
    step = new_t - t;
    t = new_t;
    bez_target[X_AXIS] = new_pos0;
    bez_target[Y_AXIS] = new_pos1;
    bez_target[Z_AXIS] = interp(position[Z_AXIS], target[Z_AXIS], t);
    bez_target[E_AXIS] = interp(position[E_AXIS], target[E_AXIS], t);
    clamp_to_software_endstops(bez_target);
    planner.buffer_line_kinematic(bez_target, fr_mm_s, extruder);
  }
}
#endif 
