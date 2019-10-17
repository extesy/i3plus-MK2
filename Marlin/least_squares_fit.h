#include "MarlinConfig.h"
#if ENABLED(AUTO_BED_LEVELING_UBL)  
#include "Marlin.h"
#include "macros.h"
#include <math.h>
struct linear_fit_data {
  float xbar, ybar, zbar,
        x2bar, y2bar, z2bar,
        xybar, xzbar, yzbar,
        max_absx, max_absy,
        A, B, D, N;
};
void inline incremental_LSF_reset(struct linear_fit_data *lsf) {
  memset(lsf, 0, sizeof(linear_fit_data));
}
void inline incremental_WLSF(struct linear_fit_data *lsf, const float &x, const float &y, const float &z, const float &w) {
  lsf->xbar  += w * x;
  lsf->ybar  += w * y;
  lsf->zbar  += w * z;
  lsf->x2bar += w * x * x;  
  lsf->y2bar += w * y * y;
  lsf->z2bar += w * z * z;
  lsf->xybar += w * x * y;
  lsf->xzbar += w * x * z;
  lsf->yzbar += w * y * z;
  lsf->N     += w;
  lsf->max_absx = max(FABS(w * x), lsf->max_absx);
  lsf->max_absy = max(FABS(w * y), lsf->max_absy);
}
void inline incremental_LSF(struct linear_fit_data *lsf, const float &x, const float &y, const float &z) {
  lsf->xbar += x;
  lsf->ybar += y;
  lsf->zbar += z;
  lsf->x2bar += sq(x);
  lsf->y2bar += sq(y);
  lsf->z2bar += sq(z);
  lsf->xybar += x * y;
  lsf->xzbar += x * z;
  lsf->yzbar += y * z;
  lsf->max_absx = max(FABS(x), lsf->max_absx);
  lsf->max_absy = max(FABS(y), lsf->max_absy);
  lsf->N += 1.0;
}
int finish_incremental_LSF(struct linear_fit_data *);
#endif
