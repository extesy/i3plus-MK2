#ifndef PLANNER_BEZIER_H
#define PLANNER_BEZIER_H
#include "Marlin.h"
void cubic_b_spline(
              const float position[NUM_AXIS], 
              const float target[NUM_AXIS],   
              const float offset[4],          
              float fr_mm_s,
              uint8_t extruder
            );
#endif 
